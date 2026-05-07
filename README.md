# Pick-and-Place Controller

Two-file project: Arduino firmware for a Raspberry Pi Pico W + a Python/Tkinter desktop app.  
Controls up to three stepper motors (X, Y, Z axes) over USB serial.

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
3. **Jog axes** — each axis (X, Y, Z) has **−** and **+** buttons that move by one step-size. Buttons are grayed out while a move is in progress.
4. **Position** — each axis tracks cumulative steps. **Set as Zero** redefines the current position as logical 0. **Home** returns to logical position 0.
5. **Saved Positions** — click **Save** on Pickup or Drop to record the current X position; click **Go** to move there.
6. **Log** — shows every sent command (`>`) and every firmware reply (`<`).

**Soft limits:** Per-axis bounds can be set in `AXIS_LIMITS` at the top of `pickplace_app.py`. Set either bound to `None` to disable it. All axes are currently unlimited.

---

## Serial Protocol

Newline-terminated ASCII commands:

| Command | Example | Effect |
|---------|---------|--------|
| `X±<n>` | `X+100` | Move X axis by n steps |
| `Y±<n>` | `Y-50`  | Move Y axis by n steps |
| `Z±<n>` | `Z+10`  | Move Z axis by n steps |
| `S<µs>` | `S3000` | Set inter-step delay (speed) |
| `P`     | `P`     | Ping |

Firmware replies: `DONE`, `PONG`, `ERR <cmd>`, `READY`

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
| `BAUD_RATE`     | 9600   | Must match firmware |
| `DEFAULT_STEP`  | 50     | Initial jog step size (steps) |
| `DEFAULT_SPEED` | 5000   | Initial inter-step delay sent on connect (µs) |
| `POLL_MS`       | 50     | How often the main thread checks for firmware replies |
| `BUSY_TIMEOUT`  | 10.0 s | Watchdog: force-clears busy flag if firmware never replies |
| `AXIS_LIMITS`   | all `None` | Per-axis soft limits `(min, max)`; `None` disables a bound |
