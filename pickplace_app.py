"""
pickplace_app.py
Desktop controller for a pick-and-place machine (X, Y, Z axes).
Talks to a Raspberry Pi Pico W running pickplace_firmware.ino over USB serial.

Protocol (newline-terminated ASCII):
  Send  → firmware:  X±<n>  Y±<n>  Z±<n>  S<µs>  P
  Reply ← firmware:  DONE   PONG   ERR <cmd>   READY

Dependencies: pyserial  (pip install pyserial)
"""

import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import threading
import queue
import time

# ── Easy-to-change constants ───────────────────────────────────────────────
BAUD_RATE     = 9600
DEFAULT_STEP  = 0       # initial jog step size (steps)
DEFAULT_SPEED = 2000    # initial inter-step delay sent to firmware (µs)
POLL_MS       = 50      # how often the main thread drains the reply queue
BUSY_TIMEOUT  = 10.0    # seconds before the busy flag is force-cleared (watchdog)

STEPS_PER_BEAD = 5      # steps per 5 mm bead width

AXIS_LIMITS = {
    "X": (50,   495),
    "Y": (50,   900),
    "Z": (None, None),
}

# ── Application state ──────────────────────────────────────────────────────
# All variables below are read/written only from the main (Tkinter) thread,
# except 'ser', which the reader thread reads as a snapshot (see _reader_loop).
ser            = None   # serial.Serial when connected, else None
busy           = False  # True while awaiting DONE; blocks new commands
busy_since     = None   # time.time() snapshot when busy was set (watchdog)
pending_action = None   # zero-arg callable invoked when DONE arrives

# Per-axis position tracking. Values are absolute machine steps.
axis_pos  = {"X": 0, "Y": 0, "Z": 0}
axis_zero = {"X": 0, "Y": 0, "Z": 0}  # display = axis_pos − axis_zero

# Thread-safe queue: reader thread writes, main thread reads.
reply_q: queue.Queue = queue.Queue()


# ══════════════════════════════════════════════════════════════════════════════
# Serial reader thread
# ══════════════════════════════════════════════════════════════════════════════

def _reader_loop() -> None:
    """
    Daemon thread: reads one line at a time from the serial port and enqueues it.
    Snapshots 'ser' before each readline() to avoid a race with disconnect
    (disconnect sets ser=None; an AttributeError mid-call would silently die).
    timeout=0.1 on the port ensures readline() never blocks more than 100 ms,
    so disconnects are noticed quickly.
    """
    while True:
        local_ser = ser  # snapshot — avoids race with _do_disconnect
        if local_ser is not None and local_ser.is_open:
            try:
                raw = local_ser.readline()
                if raw:
                    line = raw.decode("utf-8", errors="replace").strip()
                    if line:
                        reply_q.put(line)
            except serial.SerialException as exc:
                reply_q.put(f"[serial read error: {exc}]")
                time.sleep(0.2)
        else:
            time.sleep(0.05)


_reader_thread = threading.Thread(target=_reader_loop, daemon=True)
_reader_thread.start()


# ══════════════════════════════════════════════════════════════════════════════
# Command / reply core
# ══════════════════════════════════════════════════════════════════════════════

def send_command(cmd: str, on_done=None) -> None:
    """
    Send one newline-terminated command to the firmware.
    Sets busy=True and disables motion controls until DONE (or ERR) arrives,
    at which point poll_replies() calls on_done() and clears the flag.
    on_done runs on the main thread — safe to touch Tkinter widgets there.
    """
    global busy, busy_since, pending_action
    if ser is None or not ser.is_open:
        _log("not connected")
        return
    if busy:
        _log("busy — command ignored")
        return

    busy = True
    busy_since = time.time()
    pending_action = on_done
    _set_controls("disabled")

    try:
        ser.write((cmd + "\n").encode())
        _log(f"> {cmd}")
    except serial.SerialException as exc:
        _log(f"[write error: {exc}]")
        _clear_busy()


def _clear_busy() -> None:
    global busy, busy_since, pending_action
    busy = False
    busy_since = None
    pending_action = None
    if ser and ser.is_open:
        _set_controls("normal")


def poll_replies() -> None:
    """
    Drains reply_q on the main thread every POLL_MS ms via root.after().
    Reacts to DONE / ERR and runs the pending on_done callback.
    Also runs a watchdog that force-clears busy if DONE never arrives.

    The entire body is wrapped in try/finally so that root.after() is
    ALWAYS rescheduled even if an exception occurs mid-loop.  Without this,
    any uncaught exception silently kills the polling loop, leaving busy=True
    and all buttons permanently disabled.
    """
    global busy
    try:
        while not reply_q.empty():
            line = reply_q.get_nowait()
            _log(f"< {line}")
            if line == "DONE":
                action = pending_action
                _clear_busy()
                if action:
                    try:
                        action()
                    except Exception as exc:
                        _log(f"[callback error: {exc}]")
            elif line.startswith("ERR") or line.startswith("["):
                _clear_busy()

        # Watchdog: release busy if the firmware never replied (serial glitch).
        if busy and busy_since is not None and (time.time() - busy_since) > BUSY_TIMEOUT:
            _log("[watchdog] no DONE received — releasing busy flag")
            _clear_busy()
    except Exception as exc:
        _log(f"[poll error: {exc}]")
    finally:
        root.after(POLL_MS, poll_replies)  # always reschedule, no matter what


# ══════════════════════════════════════════════════════════════════════════════
# Motion helpers
# ══════════════════════════════════════════════════════════════════════════════

def do_move(axis: str, delta: int, after=None) -> None:
    """
    Validate delta against soft limits for the given axis, then send the move.
    Position is updated optimistically inside on_done (after firmware confirms).
    after, if provided, is called once the move completes — used to chain multi-axis moves.
    """
    if delta == 0:
        if after:
            after()
        return

    cur     = axis_pos[axis]
    new_abs = cur + delta
    lo, hi  = AXIS_LIMITS[axis]

    if lo is not None and new_abs < lo:
        _log(f"LIMIT: {axis} {cur}{delta:+d} = {new_abs} < {lo} — move refused")
        return
    if hi is not None and new_abs > hi:
        _log(f"LIMIT: {axis} {cur}{delta:+d} = {new_abs} > {hi} — move refused")
        return

    sign = "+" if delta > 0 else ""

    def on_done(ax=axis, target=new_abs):
        axis_pos[ax] = target
        _refresh_pos_labels()
        if after:
            after()

    send_command(f"{axis}{sign}{delta}", on_done=on_done)


def jog(axis: str, direction: int) -> None:
    try:
        step = int(step_vars[axis].get())
        if step <= 0:
            raise ValueError
    except ValueError:
        _log(f"invalid step size for {axis} — must be a positive integer")
        return
    do_move(axis, direction * step)


def move_by_entry(axis: str) -> None:
    try:
        steps = int(step_vars[axis].get())
    except ValueError:
        _log(f"invalid step count for {axis} — must be an integer")
        return
    do_move(axis, steps)


def _send_multi(moves: list, after=None) -> None:
    """
    Send an M command for a list of (axis, delta) pairs.
    Validates limits, builds the command, and updates positions on DONE.
    after() is called once the move completes — used to chain pattern moves.
    """
    if not moves:
        if after:
            after()
        return

    for axis, delta in moves:
        cur = axis_pos[axis]
        new_abs = cur + delta
        lo, hi = AXIS_LIMITS[axis]
        if lo is not None and new_abs < lo:
            _log(f"LIMIT: {axis} {cur}{delta:+d} = {new_abs} < {lo} — move refused")
            return
        if hi is not None and new_abs > hi:
            _log(f"LIMIT: {axis} {cur}{delta:+d} = {new_abs} > {hi} — move refused")
            return

    parts = [f"{ax}{'+' if d > 0 else ''}{d}" for ax, d in moves]
    targets = {ax: axis_pos[ax] + d for ax, d in moves}

    def on_done():
        for ax, target in targets.items():
            axis_pos[ax] = target
        _refresh_pos_labels()
        if after:
            after()

    send_command("M " + " ".join(parts), on_done=on_done)


def home_all_axes() -> None:
    moves = [(a, axis_zero[a] - axis_pos[a]) for a in ("X", "Y", "Z")
             if axis_pos[a] != axis_zero[a]]
    if not moves:
        _log("all axes already at home")
        return
    _send_multi(moves)


def go_all_axes() -> None:
    moves = []
    for axis in ("X", "Y", "Z"):
        try:
            steps = int(step_vars[axis].get())
        except ValueError:
            _log(f"invalid step count for {axis} — skipping")
            continue
        if steps != 0:
            moves.append((axis, steps))

    if not moves:
        _log("no moves to execute — all step values are 0")
        return
    _send_multi(moves)


def set_zero(axis: str) -> None:
    axis_zero[axis] = axis_pos[axis]
    _refresh_pos_labels()
    _log(f"{axis} zero set at machine position {axis_pos[axis]}")


def go_home(axis: str) -> None:
    delta = axis_zero[axis] - axis_pos[axis]
    if delta == 0:
        _log(f"{axis} already at home")
        return
    do_move(axis, delta)


def set_speed() -> None:
    try:
        us = int(speed_var.get())
        if us <= 0:
            raise ValueError
    except ValueError:
        _log("invalid speed — must be a positive integer (µs)")
        return
    send_command(f"S{us}")




# ── Connection management ──────────────────────────────────────────────────

def connect_disconnect() -> None:
    if ser and ser.is_open:
        _do_disconnect()
    else:
        _do_connect()


def _do_connect() -> None:
    global ser
    port = port_var.get()
    if not port:
        _log("no port selected")
        return
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
        time.sleep(1.5)  # let Pico W reboot and print READY
        status_dot.config(fg="green")
        conn_btn.config(text="Disconnect")
        _set_controls("normal")
        _log(f"connected to {port} at {BAUD_RATE} baud")
    except serial.SerialException as exc:
        _log(f"connection failed: {exc}")
        ser = None


def _do_disconnect() -> None:
    global ser, busy
    busy = False
    _set_controls("disabled")
    old_ser = ser
    ser = None
    time.sleep(0.15)
    if old_ser:
        try:
            old_ser.close()
        except Exception:
            pass
    status_dot.config(fg="red")
    conn_btn.config(text="Connect")
    _log("disconnected")


def refresh_ports() -> None:
    ports = [p.device for p in serial.tools.list_ports.comports()]
    port_menu["values"] = ports
    if ports and port_var.get() not in ports:
        port_var.set(ports[0])
    elif not ports:
        port_var.set("")


def on_close() -> None:
    _do_disconnect()
    root.destroy()


# ── UI helpers ─────────────────────────────────────────────────────────────

def _set_controls(state: str) -> None:
    for w in _motion_controls:
        w.config(state=state)


def _refresh_pos_labels() -> None:
    for ax in ("X", "Y", "Z"):
        pos_vars[ax].set(f"{axis_pos[ax] - axis_zero[ax]} steps")


def _log(msg: str) -> None:
    log_text.config(state="normal")
    log_text.insert("end", msg + "\n")
    log_text.see("end")
    log_text.config(state="disabled")


# ══════════════════════════════════════════════════════════════════════════════
# Build the UI
# ══════════════════════════════════════════════════════════════════════════════

root = tk.Tk()
root.title("Pick-and-Place Controller")
root.resizable(True, False)


def _make_btn(parent, **kwargs) -> tk.Button:
    """
    Drop-in replacement for tk.Button that fixes the macOS click-to-focus problem.

    On macOS, focus_force() does not take effect synchronously — the Button
    class binding checks focus state before our <Button-1> handler's
    focus_force() call has actually been processed, so the press is still
    swallowed.  The reliable fix is to force focus on <Enter> (mouse hover),
    which fires before the click and gives the event loop time to register the
    focus change.  The <Button-1> binding is kept as a secondary fallback for
    fast clicks where Enter and the click arrive in the same event batch.
    """
    b = tk.Button(parent, **kwargs)
    _ff = lambda e: b.winfo_toplevel().focus_force()
    b.bind("<Enter>",    _ff, add="+")   # primary: force focus on hover
    b.bind("<Button-1>", _ff, add="+")   # fallback: force focus on press
    return b


PAD = dict(padx=6, pady=4)

# ── Top-level layout: grid on left, controls on right ─────────────────────
main_frame = tk.Frame(root)
main_frame.pack(fill="both", expand=True)

left_frame = tk.Frame(main_frame)
left_frame.pack(side="left", fill="y", padx=(6, 0), pady=6)

right_frame = tk.Frame(main_frame)
right_frame.pack(side="left", fill="both", expand=True)

# ── Pattern grid ───────────────────────────────────────────────────────────
_GRID_N     = 10
_CELL       = 18
_GAP        = 1
_CANVAS_PX  = _GRID_N * (_CELL + _GAP) + _GAP
_COL_EMPTY  = "#555555"
_COL_FILLED = "#ffffff"
_COL_DONE   = "#00cc44"

# Grid origin: centre the 10×10 bead grid in the usable work area
_X_LO, _X_HI = AXIS_LIMITS["X"]
_Y_LO, _Y_HI = AXIS_LIMITS["Y"]
GRID_X_ORIGIN = (_X_LO + _X_HI) // 2 - (_GRID_N * STEPS_PER_BEAD) // 2
GRID_Y_ORIGIN = (_Y_LO + _Y_HI) // 2 - (_GRID_N * STEPS_PER_BEAD) // 2

grid_state = [[False] * _GRID_N for _ in range(_GRID_N)]

grid_lf = tk.LabelFrame(left_frame, text="Pattern Grid", padx=4, pady=4)
grid_lf.pack(fill="both", expand=True)

grid_canvas = tk.Canvas(grid_lf, bg="#2e2e2e", highlightthickness=0)
grid_canvas.pack(fill="both", expand=True)

_cell_ids = [[None] * _GRID_N for _ in range(_GRID_N)]
for _r in range(_GRID_N):
    for _c in range(_GRID_N):
        _cell_ids[_r][_c] = grid_canvas.create_rectangle(
            0, 0, 1, 1, fill=_COL_EMPTY, outline=""
        )

_live_cell = _CELL  # updated on every Configure event


def _redraw_grid(event=None) -> None:
    global _live_cell
    w = grid_canvas.winfo_width()
    h = grid_canvas.winfo_height()
    size = min(w, h)
    cell = max(1, (size - (_GRID_N + 1) * _GAP) // _GRID_N)
    _live_cell = cell
    for r in range(_GRID_N):
        for c in range(_GRID_N):
            x0 = _GAP + c * (cell + _GAP)
            y0 = _GAP + r * (cell + _GAP)
            grid_canvas.coords(_cell_ids[r][c], x0, y0, x0 + cell, y0 + cell)


grid_canvas.bind("<Configure>", _redraw_grid)


def _set_cell(row, col, filled: bool) -> None:
    grid_state[row][col] = filled
    grid_canvas.itemconfig(_cell_ids[row][col],
                           fill=_COL_FILLED if filled else _COL_EMPTY)


def _cell_at(event):
    col = (event.x - _GAP) // (_live_cell + _GAP)
    row = (event.y - _GAP) // (_live_cell + _GAP)
    if 0 <= row < _GRID_N and 0 <= col < _GRID_N:
        return row, col
    return None, None


_drag_fill = None


def _on_grid_press(event) -> None:
    global _drag_fill
    row, col = _cell_at(event)
    if row is None:
        return
    _drag_fill = not grid_state[row][col]
    _set_cell(row, col, _drag_fill)


def _on_grid_drag(event) -> None:
    row, col = _cell_at(event)
    if row is not None and grid_state[row][col] != _drag_fill:
        _set_cell(row, col, _drag_fill)


grid_canvas.bind("<Button-1>", _on_grid_press)
grid_canvas.bind("<B1-Motion>", _on_grid_drag)


def clear_grid() -> None:
    for r in range(_GRID_N):
        for c in range(_GRID_N):
            if grid_state[r][c]:
                _set_cell(r, c, False)


def run_pattern() -> None:
    cells = [(r, c) for r in range(_GRID_N) for c in range(_GRID_N) if grid_state[r][c]]
    if not cells:
        _log("no cells selected in pattern")
        return

    _log(f"starting pattern: {len(cells)} bead(s)")

    x_center = (_X_LO + _X_HI) // 2
    y_center = (_Y_LO + _Y_HI) // 2

    def return_to_zero():
        dx = -axis_pos["X"]
        dy = -axis_pos["Y"]
        _log(f"returning to zero → X=0 Y=0  (Δx={dx:+d} Δy={dy:+d})")
        parts = [f"{ax}{'+' if d > 0 else ''}{d}" for ax, d in (("X", dx), ("Y", dy)) if d != 0]
        if not parts:
            return
        def on_done():
            axis_pos["X"] = 0
            axis_pos["Y"] = 0
            _refresh_pos_labels()
        send_command("M " + " ".join(parts), on_done=on_done)

    def place_next(remaining):
        if not remaining:
            _log("pattern complete")
            return_to_zero()
            return
        row, col = remaining[0]
        x_target = GRID_X_ORIGIN + col * STEPS_PER_BEAD
        y_target = GRID_Y_ORIGIN + row * STEPS_PER_BEAD
        dx = x_target - axis_pos["X"]
        dy = y_target - axis_pos["Y"]
        _log(f"grid ({row},{col}) → X={x_target} Y={y_target}  (Δx={dx:+d} Δy={dy:+d})")
        moves = [(ax, d) for ax, d in (("X", dx), ("Y", dy)) if d != 0]

        def on_placed():
            grid_canvas.itemconfig(_cell_ids[row][col], fill=_COL_DONE)
            place_next(remaining[1:])

        _send_multi(moves, after=on_placed)

    # Step 1 — move to centre of work area before starting pattern
    dx_c = x_center - axis_pos["X"]
    dy_c = y_center - axis_pos["Y"]
    _log(f"moving to centre → X={x_center} Y={y_center}  (Δx={dx_c:+d} Δy={dy_c:+d})")
    _send_multi([(ax, d) for ax, d in (("X", dx_c), ("Y", dy_c)) if d != 0],
                after=lambda: place_next(cells))


_btn_row = tk.Frame(left_frame)
_btn_row.pack(pady=(6, 2))
_make_btn(_btn_row, text="Run Pattern", command=run_pattern).pack(side="left", padx=4)
_make_btn(_btn_row, text="Clear All",   command=clear_grid).pack(side="left", padx=4)

# ── 1. Connection bar ──────────────────────────────────────────────────────
conn_frame = tk.LabelFrame(right_frame, text="Connection", padx=4, pady=4)
conn_frame.pack(fill="x", **PAD)

port_var = tk.StringVar()
port_menu = ttk.Combobox(conn_frame, textvariable=port_var, width=22, state="readonly")
port_menu.pack(side="left", **PAD)

_make_btn(conn_frame, text="Refresh", command=refresh_ports).pack(side="left", **PAD)
conn_btn = _make_btn(conn_frame, text="Connect", command=connect_disconnect)
conn_btn.pack(side="left", **PAD)
status_dot = tk.Label(conn_frame, text="●", fg="red", font=("Arial", 16))
status_dot.pack(side="left", padx=(2, 6))

# ── 2. Motion settings ─────────────────────────────────────────────────────
motion_frame = tk.LabelFrame(right_frame, text="Motion Settings", padx=4, pady=4)
motion_frame.pack(fill="x", **PAD)

tk.Label(motion_frame, text="Speed (µs):").pack(side="left")
speed_var = tk.StringVar(value=str(DEFAULT_SPEED))
tk.Entry(motion_frame, textvariable=speed_var, width=7).pack(side="left", **PAD)

speed_btn = _make_btn(motion_frame, text="Set Speed", command=set_speed, state="disabled")
speed_btn.pack(side="left", **PAD)

# ── 3. Axes — jog + position in one compact grid ───────────────────────────
#
#        [−axis]    [+axis]    [Home]    position    [Set Zero]
#   X:     …
#   Y:     …
#   Z:     …
#
axes_frame = tk.LabelFrame(right_frame, text="Axes", padx=6, pady=6)
axes_frame.pack(fill="x", **PAD)

pos_vars: dict[str, tk.StringVar] = {}
step_vars: dict[str, tk.StringVar] = {}
_motion_controls: list[tk.Widget] = [speed_btn]  # start with speed_btn; axes buttons added below

axis_max_hint = {"X": "50–495", "Y": "50–900", "Z": ""}

for row, axis in enumerate(("X", "Y", "Z")):
    tk.Label(axes_frame, text=f"{axis}", width=2, anchor="e",
             font=("Arial", 11, "bold")).grid(row=row, column=0, padx=(4, 6), pady=4)

    neg_btn = _make_btn(axes_frame, text=f"◀  −{axis}", width=10, state="disabled",
                        command=lambda a=axis: jog(a, -1))
    neg_btn.grid(row=row, column=1, padx=2)

    pos_btn = _make_btn(axes_frame, text=f"+{axis}  ▶", width=10, state="disabled",
                        command=lambda a=axis: jog(a, +1))
    pos_btn.grid(row=row, column=2, padx=2)

    home_btn = _make_btn(axes_frame, text="Home", width=6, state="disabled",
                         command=lambda a=axis: go_home(a))
    home_btn.grid(row=row, column=3, padx=6)

    sv = tk.StringVar(value=str(DEFAULT_STEP))
    step_vars[axis] = sv
    step_entry = tk.Entry(axes_frame, textvariable=sv, width=7, justify="right")
    step_entry.grid(row=row, column=4, padx=2)
    step_entry.bind("<Return>", lambda e, a=axis: move_by_entry(a))

    tk.Label(axes_frame, text="steps").grid(row=row, column=5, padx=(0, 8))

    pv = tk.StringVar(value="0 steps")
    pos_vars[axis] = pv
    tk.Label(axes_frame, textvariable=pv, width=11, anchor="w",
             font=("Courier", 11, "bold")).grid(row=row, column=6, padx=4)

    # "Set Zero" needs no connection — it's a local reference change.
    _make_btn(axes_frame, text="Set Zero",
              command=lambda a=axis: set_zero(a)).grid(row=row, column=7, padx=2)

    tk.Label(axes_frame, text=axis_max_hint[axis], fg="gray", font=("Arial", 9)
             ).grid(row=row, column=8, padx=(4, 2), sticky="w")

    _motion_controls.extend([neg_btn, pos_btn, home_btn])

btn_row = tk.Frame(axes_frame)
btn_row.grid(row=3, column=0, columnspan=9, pady=(6, 2))

go_all_btn = _make_btn(btn_row, text="Go", width=10, state="disabled",
                       command=go_all_axes)
go_all_btn.pack(side="left", padx=4)

home_all_btn = _make_btn(btn_row, text="Home All", width=10, state="disabled",
                         command=home_all_axes)
home_all_btn.pack(side="left", padx=4)

_motion_controls.extend([go_all_btn, home_all_btn])

# ── 4. Log pane ────────────────────────────────────────────────────────────
log_frame = tk.LabelFrame(right_frame, text="Log", padx=4, pady=4)
log_frame.pack(fill="both", expand=True, **PAD)

log_text = tk.Text(log_frame, height=10, state="disabled", wrap="word",
                   font=("Courier", 10), bg="#1e1e1e", fg="#d4d4d4",
                   insertbackground="white")
log_scroll = tk.Scrollbar(log_frame, command=log_text.yview)
log_text.config(yscrollcommand=log_scroll.set)
log_scroll.pack(side="right", fill="y")
log_text.pack(side="left", fill="both", expand=True)

# ══════════════════════════════════════════════════════════════════════════════
# Bootstrap
# ══════════════════════════════════════════════════════════════════════════════

refresh_ports()
root.after(POLL_MS, poll_replies)
root.protocol("WM_DELETE_WINDOW", on_close)
root.after(100, root.focus_force)
root.mainloop()
