# HPScan5-Turntable
A turntable plugin and Arduino (RPI2040) driver for HP Scan 5

The dll file was designed to allow HP Scan 5 to talk to arduino based turntables. The original turntable is unobtainable hardware now. 
Tested with HP Scan 5.7 https://support.hp.com/us-en/drivers/hp-3d-structured-light-scanner/14169438
It currently connects to the turntable and can send rotate commands. I haven't tested wiht a full scan ye t.

---
# Building and deploying

### Pico firmware

**Requirements:**
- Arduino IDE
- [Earle Philhower RP2040 Arduino core](https://github.com/earlephilhower/arduino-pico)
- Adafruit TinyUSB library
- AccelStepper library

**Steps:**
1. Open `turntable_pico.ino` in Arduino IDE
2. Select board: `Raspberry Pi Pico`
3. Set **Tools → USB Stack → Adafruit TinyUSB**
4. Flash to the Pico

To enter bootloader mode for reflashing, send `BOOT` via serial monitor (or hold BOOTSEL while plugging in).

**Debug logging** is controlled by a single flag at the top of the sketch:

```cpp
#define DEBUG_ENABLED 1   // on  — enables ?DEBUG / ?CLEAR / ?STATUS commands
#define DEBUG_ENABLED 0   // off — zero RAM and CPU overhead (production)
```

### DLL

**Requirements:**
- MinGW-w64 (64-bit) on Windows

**Build:**
```batch
c:\mingw64\bin\g++ -shared -o motor_plugin1_64_TT1.dll motor_plugin_x64_TT1.cpp  ^
    -static-libgcc -static-libstdc++ ^
    -Wl,-Bstatic -lwinpthread -Wl,-Bdynamic ^
    -lws2_32 -ladvapi32 ^
    -Wl,--kill-at -m64 -O2
```

**Deploy:**
```batch
copy /Y motor_plugin1_64_TT1.dll "C:\Program Files\HP 3D Scan 5\dll64\motor_plugin1_64_TT1.dll"
```

**Debug level** is set at the top of the DLL source:

```cpp
#define DEBUG_LEVEL 0   // errors and key events only (production)
#define DEBUG_LEVEL 1   // + status changes, parameter calls
#define DEBUG_LEVEL 2   // + every serial TX/RX  (default for development)
#define DEBUG_LEVEL 3   // + every getPosition call (very verbose)
```

Log output: `C:\Program Files\HP 3D Scan 5\logs\bypass_plugin.log`

---

## Configuration

### HP XML files MANUAL COM PORT 

`C:\Program Files\HP 3D Scan 5\dll64\motor_plugin1_TT1.xml`:
```xml
<parameters>port:COM9</parameters>
```

Add `COM9` to match the Pico's COM port on your machine.


---
## Hardware

| Component | Details |
|-----------|---------|
| MCU | Raspberry Pi Pico (RP2040) |
| Stepper driver | DRV8825 |
| Microstepping | 32× |
| Motor | 200 steps/rev |
| Gear ratio | 168:52 (~3.23:1) |
| Steps per degree | 57.4359 |
| USB VID/PID | 0x2AD1 / 0x7AB8 (spoofed to match HP's expected device) |

### Pin mapping

| Signal | GPIO |
|--------|------|
| DIR | GP2 |
| STEP | GP5 |
| ENABLE (active LOW) | GP8 |
| SLEEP (active LOW) | GP17 |
| RESET (active LOW) | GP16 |
| M0 | GP19 |
| M1 | GP20 |
| M2 | GP18 |
| LED | GP25 |


---

## How it works


HP 3D Scan 5 loads `motor_plugin1_64_TT1.dll` from its `dll64\` directory and calls exported functions to control the turntable. This project replaces that DLL with a custom one that:

1. Speaks HP's expected plugin interface (vtable layout + 8 exported functions)
2. Translates move commands into serial messages sent to the Pico over USB CDC
3. Handles HP's init sequence, status polling, and position tracking

The Pico firmware receives serial commands, drives the stepper via the DRV8825, and replies with `DONE` or `HOME` when moves complete.

```
HP 3D Scan 5
    │
    │  loads
    ▼
motor_plugin1_64_TT1.dll  (our custom DLL)
    │
    │  USB serial (COM9, 115200 baud)
    ▼
Raspberry Pi Pico (RP2040)
    │
    │  STEP/DIR pulses
    ▼
DRV8825 stepper driver  →  stepper motor  →  turntable
```

---

## Serial protocol

All commands are plain ASCII terminated with `\n`. The DLL sends, the Pico responds.

| Command | Response | Description |
|---------|----------|-------------|
| `steps_per_degree` | `57.4359` | Handshake — returns steps/° |
| `version` | `1.0.0` | Firmware version |
| `m+60.0000` | `DONE` | Move +60° |
| `m-60.0000` | `DONE` | Move −60° |
| `P` | `HOME` | Return to home position |
| `C` | `STOPPED` | Stop motor immediately |
| `+` | `DONE` | Jog +10° (manual testing) |
| `-` | `DONE` | Jog −10° (manual testing) |
| `BOOT` | — | Enter USB bootloader |
| `?DEBUG` | log dump | Print debug log (DEBUG_ENABLED=1 only) |
| `?CLEAR` | `OK` | Clear debug log |
| `?STATUS` | position/moving/cmd | Current state |

---

## HP plugin interface

HP calls these 8 exported functions. All are implemented in the DLL:

| Export | Description |
|--------|-------------|
| `motor_connect(ppPlugin, port, unk, params)` | Opens COM port, does handshake, returns instance |
| `motor_disconnect(inst)` | Closes port, frees instance |
| `motor_move(inst, angle)` | Starts async move, returns immediately |
| `motor_getStatus(inst)` | Returns 0=ready, 1=moving, −4=init, −10=disconnected |
| `motor_getPosition(output, inst)` | Returns current position in degrees |
| `motor_stop(inst)` | Sends stop command |
| `motor_getParameter(output, name, inst)` | Returns tilt / speed / numScans / stepsPerDegree |
| `motor_setParameter(value, name, inst)` | No-op (tilt not supported) |

HP's status polling cycle:
- On connect: polls `getStatus()` expecting `−4` (initializing) for ~1 second, then `0` (ready)
- On rotate: calls `move()`, then polls `getStatus()` waiting for `1` (moving), then waits for `0` (done)

### Important implementation notes

**HP passes a wrapper pointer to move/status calls**, not the original instance pointer returned by `motor_connect`. The DLL handles this by validating every incoming pointer against the known good instance (`g_current_inst`) and falling back to it when there's a mismatch.

**`motor_move` must return immediately** — it runs the serial wait on a background thread. Blocking HP's UI thread causes a watchdog crash.

**DTR must be asserted** after opening the COM port (`EscapeCommFunction(h, SETDTR)`) or TinyUSB CDC will not flush the Pico's TX buffer to the host.
