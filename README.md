# I2CPlayground 🔌

I2CPlayground is an **interactive I²C protocol simulator**: it encodes sensor data as correctly-formatted I²C byte packets with START/STOP conditions, 7-bit device address, R/W bit, and ACK/NACK bits (MSB first), then animates the SDA and SCL signal lines with correct I²C timing on a live plot. A decoder recovers and verifies the original data from the bit stream. It's built for the **BUILDCORED ORCAS — Day 19** challenge.

## How it works

- The **encoder** builds a complete I²C WRITE transaction token by token: START condition → 7-bit address + W bit → ACK → optional register byte → data bytes (each 8 bits MSB-first + ACK) → STOP condition.
- Sensor values (integers or floats) are packed as **16-bit big-endian signed integers** using the same format as the MPU6050 accelerometer — one value becomes two bytes (high byte, low byte).
- The **signal builder** converts the token stream into SCL and SDA sample arrays, respecting the core I²C rule: SDA may only change when SCL is LOW, except during START (SDA falls while SCL is HIGH) and STOP (SDA rises while SCL is HIGH).
- A **matplotlib animation** draws SCL and SDA sample-by-sample, with bit labels fading in as the wave passes each position. Colour coding distinguishes START/STOP (orange/purple), address bits (cyan), R/W bit (amber), data bits (pink), and ACK bits (green).
- The **decoder** reads back the token stream, reconstructs the address, R/W flag, and data bytes, and prints a verification log showing whether the decoded bytes match the originals.

## I²C frame structure

```
[START] [A6 A5 A4 A3 A2 A1 A0 W] [ACK] [REG7..REG0] [ACK] [D7..D0] [ACK] ... [STOP]
         ←── 7-bit address ──→ ↑                              ←── data byte ──→
                               R/W bit (0=write, 1=read)
```

Key rules implemented:
- **MSB first** — bit 7 is transmitted before bit 0
- **ACK = LOW (0)** — receiver pulls SDA low to acknowledge
- **NACK = HIGH (1)** — receiver releases SDA (or it floats high)
- **9th bit** — every byte is followed by one ACK/NACK bit
- SDA changes only when SCL is LOW (setup/hold time)

## Requirements

- Python 3.10.x
- tkinter (bundled with Python on Windows)

## Python packages:

```bash
pip install matplotlib numpy
```

## Setup

1. Install the required Python packages (see above or run:
```
pip install -r requirements.txt
```
after downloading `requirements.txt`)

## Usage

```bash
python i2cplayground.py                            # MPU6050 at 0x68, sample data
python i2cplayground.py --addr 0x48               # TMP102 temperature sensor
python i2cplayground.py --addr 0x68 --data 25 512 -100  # custom sensor values
python i2cplayground.py --register 0x3B           # set register address byte
python i2cplayground.py --speed fast              # 400 kHz Fast-mode label
python i2cplayground.py --no-anim                 # print bit stream only, no plot
```

The animated plot shows three panels:

| Panel | What it shows |
|---|---|
| SCL | Clock line — idles HIGH, pulses LOW for each bit |
| SDA | Data line — bit labels fade in as the wave passes, colour-coded by type |
| Decoder | Transaction summary: address, R/W, data bytes, ACK status |

The terminal prints a full packet summary with the raw bit stream and decoded verification log.

## v2.0 bridge

In v2.0, this same protocol knowledge connects directly to hardware. On a Raspberry Pi Pico reading an MPU6050:

```python
from machine import I2C, Pin
i2c = I2C(0, sda=Pin(4), scl=Pin(5), freq=400_000)
data = i2c.readfrom_mem(0x68, 0x3B, 6)  # read 6 bytes from ACCEL_XOUT_H
```

The `0x68` address, `0x3B` register, 6 data bytes, 400 kHz — all identical to what I2CPlayground simulates. After today you can read any I²C sensor datasheet.

## Common fixes

**Bit order looks wrong** — I²C sends MSB first (bit 7 before bit 0). The encoder does this correctly — if you're comparing to a logic analyser capture, make sure your analyser is also set to MSB-first decoding.

**ACK bit missing in output** — the ACK is the 9th bit after every byte (address or data). It appears green (`A`) in the animation. NACK appears amber (`N`).

**Animation too slow** — run with `--no-anim` to skip the plot and just see the terminal output. The animation speed is controlled by the `step` variable in `make_animation()` — increase it to skip more samples per frame.

**tkinter error on Linux** — run `sudo apt install python3-tk`.

**Address is 8 bits in my datasheet** — most datasheets show the 8-bit form (7-bit address + R/W bit). Strip the lowest bit: `0xD0 >> 1 = 0x68` is the MPU6050's 7-bit address. Pass the 7-bit form to `--addr`.

## Hardware concept

I²C is the protocol that connects sensors to microcontrollers everywhere. The MPU6050 accelerometer, BMP280 pressure sensor, SSD1306 OLED display, DS3231 RTC — all speak I²C. It needs only two wires: **SCL** (clock, driven by master) and **SDA** (data, bidirectional). The master generates the clock and initiates all transactions. The slave pulls SDA low to ACK each byte. The same START/STOP/ACK logic you see animated here runs at up to 5 MHz on real silicon — just much faster.

## Credits

- Visualization & animation: [Matplotlib](https://matplotlib.org/)
- Numerical computing: [NumPy](https://numpy.org/)

Built as part of the **BUILDCORED ORCAS — Day 19: I2CPlayground** challenge.
