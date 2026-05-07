# Pick-and-Place X-Axis Controller

Two-file project: Arduino firmware for a Raspberry Pi Pico + a Python/Tkinter desktop app.  
Controls one stepper motor (X axis / heated-bed movement) over USB serial.

---

## Wiring

| Signal | Pico GPIO | Stepper Driver Pin |
|--------|-----------|--------------------|
| STEP   | GP16      | STEP (or PUL+)     |
| DIR    | GP17      | DIR  (or DIR+)     |
| GND    | Any GND   | GND  (STEP−/PUL−/DIR−) |

**Power:** The stepper driver must be powered from your motor PSU independently — the Pico's 3.3 V/5 V rails cannot drive a stepper motor.

**Logic level:** Most drivers (A4988, DRV8825, TMC2209) accept 3.3 V logic, which the Pico outputs natively. If yours requires 5 V, add a level shifter on STEP and DIR.

**Direction polarity:** `LOW` on GP17 = forward (+X), `HIGH` = backward (−X). If the motor runs the wrong way, swap `DIR_FORWARD`/`DIR_BACKWARD` in the `.ino` constants section — no re-wiring needed.

---

## Firmware

### Option A — Arduino IDE

1. Install the Pico board package:  
   - Open **File → Preferences** and add this URL to *Additional Boards Manager URLs*:  
     `https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json`
   - Open **Tools → Board → Boards Manager**, search **"rp2040"**, install **Raspberry Pi Pico/RP2040 by Earle F. Philhower, III**.
2. Select **Tools → Board → Raspberry Pi RP2040 Boards → Raspberry Pi Pico**.
3. Select **Tools → Port** — the Pico shows up as a USB serial device.
4. Open `pickplace_firmware.ino` and click **Upload**.

> The Pico resets after upload. Open **Tools → Serial Monitor** at 9600 baud and confirm you see `READY`.

### Option B — PlatformIO

Create `platformio.ini` alongside the `.ino`:

```ini
[env:pico]
platform  = raspberrypi
board     = pico
framework = arduino
monitor_speed = 9600
```

Then run:

```
pio run --target upload
pio device monitor
```

---

## Desktop App

### Requirements

- Python 3.8+
- pyserial

```bash
pip install pyserial
```

Tkinter ships with most Python distributions. On Ubuntu/Debian it may need:

```bash
sudo apt install python3-tk
```

### Running

```bash
python pickplace_app.py
```

### Usage

1. **Connection bar** — select the Pico's COM/tty port from the dropdown, click **Refresh** if it's missing, then **Connect**. The dot turns green and you should see `< READY` in the log.
2. **Motion settings** — set the jog step size (in steps) and the inter-step delay (lower µs = faster). Click **Set Speed** to push the speed to the firmware.
3. **Jog X** — **−X** and **+X** buttons move by one step-size. Buttons are grayed out while a move is in progress.
4. **Position** — tracks cumulative steps. **Set as Zero** redefines the current position as logical 0. **Home** returns to logical position 0.
5. **Saved Positions** — click **Save** on Pickup or Drop to record the current position; click **Go** to move there.
6. **Log** — shows every sent command (`>`) and every firmware reply (`<`).

Soft limits prevent the machine from being commanded outside `[0, 550]` steps (absolute). Change `X_LIMIT_MAX` at the top of `pickplace_app.py` if your travel range differs.

---

## Adding a Second Axis (Future)

The code is structured for easy expansion:

- **Firmware:** add `PIN_Y_STEP`, `PIN_Y_DIR` constants and a `stepY()` function mirroring `stepX()`. Add `Y+<n>` / `Y-<n>` branches to the command parser.
- **App:** add `Y_LIMIT_MIN`/`MAX` constants, a `y_pos` variable, `jog_y()` and `do_move_y()` functions mirroring the X equivalents, and new jog buttons appended to `_motion_controls`. The threading, busy-flag, and reply-queue logic are shared and need no changes.

---

## Constants Reference

### `pickplace_firmware.ino`

| Constant | Default | Meaning |
|----------|---------|---------|
| `PIN_X_STEP` | 16 | GPIO for STEP signal |
| `PIN_X_DIR`  | 17 | GPIO for DIR signal  |
| `DEFAULT_DELAY` | 5000 µs | Inter-step delay (speed) |
| `STEP_PULSE_US` | 4 µs | Step pulse width |
| `BAUD_RATE`  | 9600 | Serial baud rate |

### `pickplace_app.py`

| Constant | Default | Meaning |
|----------|---------|---------|
| `BAUD_RATE`     | 9600    | Must match firmware |
| `X_LIMIT_MIN`   | 0       | Soft lower bound (steps) |
| `X_LIMIT_MAX`   | 550     | Soft upper bound (steps) |
| `DEFAULT_STEP`  | 50      | Initial jog step size |
| `DEFAULT_SPEED` | 5000    | Initial speed sent on connect |
