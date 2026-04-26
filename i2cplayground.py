"""
I2CPlayground  —  Windows Edition
====================================
Simulates the I2C protocol in Python.
Encodes sensor data as I2C-formatted byte packets with START/STOP conditions,
7-bit device address, R/W bit, ACK/NACK bits (MSB first).
Animates SDA and SCL signal lines with correct timing on a live plot.
Includes a decoder that recovers the original data from the bit stream.

Install dependencies:
    pip install matplotlib numpy

Usage:
    python i2cplayground.py
    python i2cplayground.py --addr 0x68       # MPU6050 I2C address
    python i2cplayground.py --data 25 100 512 # custom sensor values
    python i2cplayground.py --speed fast       # 400 kHz Fast-mode animation
    python i2cplayground.py --no-anim          # print bit stream only, no plot
"""

import argparse
import sys
import numpy as np

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation
from matplotlib.patches import FancyBboxPatch
from matplotlib.ticker import MultipleLocator

# ── I2C Constants ─────────────────────────────────────────────────────────────

# Bit stream tokens
START     = "START"
STOP      = "STOP"
BIT_0     = 0
BIT_1     = 1
ACK       = 0    # LOW = ACK
NACK      = 1    # HIGH = NACK

# I2C mode speeds (for animation timing labels only)
SPEED_MODES = {
    "standard": 100_000,   # 100 kHz
    "fast":     400_000,   # 400 kHz
    "fast+":  1_000_000,   # 1 MHz
}

# ── Colour palette ────────────────────────────────────────────────────────────

BG        = "#0a0a10"
PANEL_BG  = "#0d0d1a"
CARD_BG   = "#12121f"
SCL_COLOR = "#00e5ff"    # clock line — cyan
SDA_COLOR = "#ff4081"    # data line  — pink
ACK_COLOR = "#69ff47"    # ACK bit highlight — green
NACK_C    = "#ffcc00"    # NACK bit highlight — amber
START_C   = "#ff8800"    # START condition — orange
STOP_C    = "#bb44ff"    # STOP condition — purple
ADDR_C    = "#00ccff"    # address bits
RW_C      = "#ffcc00"    # R/W bit
DATA_C    = "#ff4081"    # data bits
DIM       = "#44445a"
TEXT      = "#ccccdd"
GRID      = "#1a1a2e"

# Terminal colours
CYAN  = "\033[96m"; GREEN = "\033[92m"; YELLOW = "\033[93m"
RED   = "\033[91m"; BOLD  = "\033[1m";  DIM_T  = "\033[2m"
RESET = "\033[0m"
def tc(t, k): return f"{k}{t}{RESET}"

# ── I2C Encoder ───────────────────────────────────────────────────────────────

class I2CEncoder:
    """
    Encodes data bytes into a full I2C transaction bit stream.

    I2C frame structure for a WRITE transaction:
      START | ADDR[6:0] | W(0) | ACK | DATA[7:0] | ACK | ... | STOP

    I2C frame structure for a READ transaction:
      START | ADDR[6:0] | R(1) | ACK | DATA[7:0] | ACK/NACK | STOP

    Bit order: MSB first (bit 7 → bit 0) for address and data bytes.
    Clock (SCL): idles HIGH.
    Data (SDA):  changes only when SCL is LOW (except START/STOP).

    START condition: SDA falls while SCL is HIGH.
    STOP condition:  SDA rises while SCL is HIGH.
    ACK:   9th bit, driven LOW by receiver.
    NACK:  9th bit, driven HIGH by receiver (or released/floating).
    """

    def __init__(self, addr: int, read: bool = False):
        self.addr = addr & 0x7F    # 7-bit address
        self.read = read
        self.transactions = []     # list of (label, bits_list)
        self._stream = []          # flat encoded stream

    def _encode_byte(self, byte_val: int, label: str = "DATA",
                      ack: bool = True) -> list:
        """
        Encode one byte (8 bits MSB-first) + ACK/NACK.
        Returns list of token dicts: {type, bit, label}
        """
        tokens = []
        for bit_idx in range(7, -1, -1):
            b = (byte_val >> bit_idx) & 1
            tokens.append({"type": "bit", "bit": b, "label": label,
                            "bit_pos": 7 - bit_idx})
        # 9th bit: ACK (LOW) or NACK (HIGH)
        tokens.append({"type": "ack", "bit": ACK if ack else NACK,
                        "label": "ACK" if ack else "NACK"})
        return tokens

    def write(self, data_bytes: list, register: int = None):
        """
        Build a complete I2C WRITE transaction.
        Optionally include a register address byte before data.
        """
        tokens = []
        tokens.append({"type": "start", "bit": None, "label": "START"})

        # Address byte: [ADDR6:ADDR0 | W=0]
        addr_byte = (self.addr << 1) | 0   # R/W = 0 for write
        for bit_idx in range(7, -1, -1):
            b   = (addr_byte >> bit_idx) & 1
            lbl = "ADDR" if bit_idx >= 1 else "W"
            tokens.append({"type": "bit", "bit": b, "label": lbl,
                            "bit_pos": 7 - bit_idx})
        tokens.append({"type": "ack", "bit": ACK, "label": "ACK"})

        # Optional register byte
        if register is not None:
            tokens.extend(self._encode_byte(register, label="REG"))

        # Data bytes
        for byte_val in data_bytes:
            tokens.extend(self._encode_byte(byte_val, label="DATA", ack=True))

        tokens.append({"type": "stop", "bit": None, "label": "STOP"})
        self._stream = tokens
        return tokens

    def read_transaction(self, n_bytes: int = 1, register: int = None):
        """
        Build a complete I2C READ transaction (with optional register write first).
        For register reads: repeated START after register address write.
        """
        tokens = []

        if register is not None:
            # Write phase: send register address
            tokens.append({"type": "start", "bit": None, "label": "START"})
            addr_w = (self.addr << 1) | 0
            for bit_idx in range(7, -1, -1):
                b   = (addr_w >> bit_idx) & 1
                lbl = "ADDR" if bit_idx >= 1 else "W"
                tokens.append({"type": "bit", "bit": b, "label": lbl,
                                "bit_pos": 7 - bit_idx})
            tokens.append({"type": "ack", "bit": ACK, "label": "ACK"})
            tokens.extend(self._encode_byte(register, label="REG"))
            # Repeated START
            tokens.append({"type": "start", "bit": None,
                            "label": "Sr"})   # Sr = repeated start

        # Read phase — Sr IS the repeated start, no extra START needed
        addr_r = (self.addr << 1) | 1   # R/W = 1 for read
        for bit_idx in range(7, -1, -1):
            b   = (addr_r >> bit_idx) & 1
            lbl = "ADDR" if bit_idx >= 1 else "R"
            tokens.append({"type": "bit", "bit": b, "label": lbl,
                            "bit_pos": 7 - bit_idx})
        tokens.append({"type": "ack", "bit": ACK, "label": "ACK"})

        # Dummy read bytes (all 0xAB for demo)
        for i in range(n_bytes):
            is_last = (i == n_bytes - 1)
            tokens.extend(self._encode_byte(0xAB, label="DATA",
                                            ack=not is_last))

        tokens.append({"type": "stop", "bit": None, "label": "STOP"})
        self._stream = tokens
        return tokens

    def get_stream(self):
        return self._stream


# ── I2C Signal Builder ────────────────────────────────────────────────────────

def tokens_to_signals(tokens: list) -> tuple:
    """
    Convert a token list into SCL and SDA sample arrays.

    I2C timing model (simplified, not cycle-accurate):
    Each data bit occupies SAMPLES_PER_BIT samples:
      [SCL LOW half] | [SCL HIGH half]
    SDA is stable during SCL HIGH, changes during SCL LOW.

    START condition: SCL HIGH, SDA falls (1→0)
    STOP  condition: SCL HIGH, SDA rises (0→1)

    Returns (scl, sda, annotations) where annotations is a list of
    (sample_index, label, color) for plot markers.
    """
    SPB   = 20   # samples per bit
    HALF  = SPB // 2

    scl  = []
    sda  = []
    anns = []   # (x_index, label, color, y_pos)

    # SDA idle level between bytes
    sda_level = 1   # idles HIGH

    def append_bit(sda_val):
        # SCL LOW half: SDA can change
        scl.extend([0] * HALF)
        sda.extend([sda_val] * HALF)
        # SCL HIGH half: SDA must be stable
        scl.extend([1] * HALF)
        sda.extend([sda_val] * HALF)

    for tok in tokens:
        pos = len(scl)

        if tok["type"] == "start":
            # START: SCL HIGH, SDA 1→0
            # Pre-condition: both HIGH (idle)
            scl.extend([1] * SPB)
            sda.extend([1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
            sda_level = 0
            anns.append((pos + 2, tok["label"], START_C, 1.3))

        elif tok["type"] == "stop":
            # STOP: SCL HIGH, SDA 0→1
            scl.extend([1] * SPB)
            sda.extend([0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
                         1, 1, 1, 1, 1, 1, 1, 1, 1, 1])
            sda_level = 1
            anns.append((pos + 2, tok["label"], STOP_C, 1.3))

        elif tok["type"] == "bit":
            b = tok["bit"]
            append_bit(b)
            sda_level = b
            # Colour by label type
            if tok["label"] == "ADDR":
                col = ADDR_C
            elif tok["label"] in ("W", "R"):
                col = RW_C
            elif tok["label"] == "REG":
                col = "#ff8800"
            else:
                col = DATA_C
            anns.append((pos + HALF + HALF // 2, str(b), col, 1.8))

        elif tok["type"] == "ack":
            b = tok["bit"]
            append_bit(b)
            sda_level = b
            col = ACK_COLOR if b == ACK else NACK_C
            lbl = "A" if b == ACK else "N"
            anns.append((pos + HALF + HALF // 2, lbl, col, 1.8))

    # Idle tail
    scl.extend([1] * SPB)
    sda.extend([1] * SPB)

    return np.array(scl), np.array(sda), anns


# ── I2C Decoder ───────────────────────────────────────────────────────────────

class I2CDecoder:
    """
    Decodes a token stream back into the original bytes.
    Validates START/STOP conditions, address, R/W, and ACK bits.
    """

    def __init__(self, tokens: list):
        self.tokens = tokens
        self.pos    = 0
        self.log    = []

    def _next(self):
        if self.pos < len(self.tokens):
            t = self.tokens[self.pos]
            self.pos += 1
            return t
        return None

    def _read_byte(self) -> tuple:
        """Read 8 data bits (MSB first) + ACK bit. Returns (byte_val, ack)."""
        bits = []
        for _ in range(8):
            t = self._next()
            if t and t["type"] == "bit":
                bits.append(t["bit"])
        byte_val = 0
        for b in bits:
            byte_val = (byte_val << 1) | b

        ack_tok = self._next()
        ack     = ack_tok["bit"] if ack_tok and ack_tok["type"] == "ack" else NACK
        return byte_val, ack

    def decode(self) -> dict:
        result = {
            "transactions": [],
            "errors":       [],
            "log":          [],
        }

        while self.pos < len(self.tokens):
            t = self._next()
            if t is None:
                break

            if t["type"] == "start":
                result["log"].append(f"  {'Sr' if t['label']=='Sr' else 'START'} condition detected")
                txn = {"type": None, "addr": None, "rw": None,
                       "bytes": [], "acks": []}

                # Read address byte (7 bits + R/W)
                addr_bits = []
                for _ in range(7):
                    bit_tok = self._next()
                    if bit_tok and bit_tok["type"] == "bit":
                        addr_bits.append(bit_tok["bit"])

                rw_tok = self._next()
                rw     = rw_tok["bit"] if rw_tok else 0

                addr = 0
                for b in addr_bits:
                    addr = (addr << 1) | b

                ack_tok  = self._next()
                addr_ack = ack_tok["bit"] if ack_tok else NACK

                txn["addr"] = addr
                txn["rw"]   = rw
                txn["type"] = "READ" if rw else "WRITE"

                result["log"].append(
                    f"  ADDR=0x{addr:02X}  R/W={'R' if rw else 'W'}  "
                    f"ACK={'ACK' if addr_ack==ACK else 'NACK'}"
                )

                # Read data bytes until STOP
                while self.pos < len(self.tokens):
                    peek = self.tokens[self.pos]
                    if peek["type"] == "stop":
                        self._next()   # consume STOP
                        result["log"].append("  STOP condition")
                        break
                    elif peek["type"] == "start":
                        break          # repeated start — let outer loop handle
                    else:
                        byte_val, ack = self._read_byte()
                        txn["bytes"].append(byte_val)
                        txn["acks"].append(ack)
                        result["log"].append(
                            f"  DATA=0x{byte_val:02X} ({byte_val:3d})  "
                            f"ACK={'ACK' if ack==ACK else 'NACK'}"
                        )

                result["transactions"].append(txn)

        return result


# ── Packet summary printer ────────────────────────────────────────────────────

def print_packet_summary(tokens: list, decoded: dict,
                         original_data: list, addr: int, register: int = None):
    print()
    print(tc("─" * 60, DIM_T))
    print(tc("  I2C Packet Summary", BOLD))
    print(tc("─" * 60, DIM_T))

    # Token-level view
    print(f"\n  {tc('Bit stream:', CYAN)}")
    line = "  "
    for tok in tokens:
        if tok["type"] == "start":
            line += tc(f"[{tok['label']}] ", YELLOW)
        elif tok["type"] == "stop":
            line += tc(f"[{tok['label']}] ", YELLOW)
        elif tok["type"] == "bit":
            col = ADDR_C if tok["label"] == "ADDR" else \
                  RW_C   if tok["label"] in ("W","R") else \
                  "\033[95m" if tok["label"] == "REG" else "\033[96m"
            line += tc(str(tok["bit"]), col)
        elif tok["type"] == "ack":
            col = GREEN if tok["bit"] == ACK else YELLOW
            line += tc("A " if tok["bit"]==ACK else "N ", col)
        if len(line) > 75:
            print(line)
            line = "  "
    if line.strip():
        print(line)

    # Decoded view
    print(f"\n  {tc('Decoded transactions:', GREEN)}")
    for log_line in decoded["log"]:
        print(tc(log_line, DIM_T))

    # Verification
    print(f"\n  {tc('Verification:', BOLD)}")
    for txn in decoded["transactions"]:
        if txn["bytes"]:
            print(f"    Address : 0x{txn['addr']:02X}  "
                  f"({'READ' if txn['rw'] else 'WRITE'})")
            print(f"    Bytes   : {[f'0x{b:02X}' for b in txn['bytes']]}")
            # If the first decoded byte is the register address, skip it
            # for verification — original_data contains only payload bytes
            decoded_payload = txn["bytes"]
            if decoded_payload and register is not None and decoded_payload[0] == register:
                decoded_payload = decoded_payload[1:]
            match = (decoded_payload == original_data[:len(decoded_payload)])
            status = tc("✓ MATCH", GREEN) if match else tc("✗ MISMATCH", RED)
            print(f"    Status  : {status}")

    print()
    print(tc("─" * 60, DIM_T))


# ── Matplotlib animated plot ──────────────────────────────────────────────────

def build_figure(scl: np.ndarray, sda: np.ndarray,
                 anns: list, tokens: list, speed_mode: str,
                 addr: int, data_bytes: list):
    """Build the 3-panel animated figure."""
    fig = plt.figure(figsize=(16, 9), facecolor=BG)
    fig.canvas.manager.set_window_title(
        "I2CPlayground  —  Day 19 | BUILDCORED ORCAS"
    )

    gs = gridspec.GridSpec(
        3, 1, figure=fig,
        left=0.06, right=0.97,
        top=0.90,  bottom=0.06,
        hspace=0.55,
    )

    # ── SCL plot ──────────────────────────────────────────────────────────────
    ax_scl = fig.add_subplot(gs[0])
    ax_scl.set_facecolor(PANEL_BG)
    ax_scl.set_ylim(-0.3, 2.0)
    ax_scl.set_xlim(0, len(scl))
    ax_scl.set_ylabel("SCL", color=SCL_COLOR, fontsize=10, fontweight="bold")
    ax_scl.set_title("I2C Clock (SCL)  —  idles HIGH",
                     color=SCL_COLOR, fontsize=9, pad=5, loc="left")
    ax_scl.tick_params(colors=DIM, labelsize=7)
    ax_scl.set_yticks([0, 1])
    ax_scl.set_yticklabels(["LOW (0)", "HIGH (1)"], color=DIM, fontsize=7)
    ax_scl.spines[:].set_color(DIM)
    ax_scl.grid(True, color=GRID, linewidth=0.4, linestyle="--")
    ax_scl.axhline(0, color=DIM, linewidth=0.5, alpha=0.4)
    ax_scl.axhline(1, color=DIM, linewidth=0.5, alpha=0.4)

    line_scl, = ax_scl.plot([], [], color=SCL_COLOR, linewidth=1.6,
                              drawstyle="steps-post", alpha=0.95)

    # ── SDA plot ──────────────────────────────────────────────────────────────
    ax_sda = fig.add_subplot(gs[1])
    ax_sda.set_facecolor(PANEL_BG)
    ax_sda.set_ylim(-0.3, 2.2)
    ax_sda.set_xlim(0, len(scl))
    ax_sda.set_ylabel("SDA", color=SDA_COLOR, fontsize=10, fontweight="bold")
    ax_sda.set_title("I2C Data (SDA)  —  MSB first, changes on SCL LOW",
                     color=SDA_COLOR, fontsize=9, pad=5, loc="left")
    ax_sda.tick_params(colors=DIM, labelsize=7)
    ax_sda.set_yticks([0, 1])
    ax_sda.set_yticklabels(["LOW (0)", "HIGH (1)"], color=DIM, fontsize=7)
    ax_sda.spines[:].set_color(DIM)
    ax_sda.grid(True, color=GRID, linewidth=0.4, linestyle="--")
    ax_sda.axhline(0, color=DIM, linewidth=0.5, alpha=0.4)
    ax_sda.axhline(1, color=DIM, linewidth=0.5, alpha=0.4)

    line_sda, = ax_sda.plot([], [], color=SDA_COLOR, linewidth=1.6,
                              drawstyle="steps-post", alpha=0.95)

    # SDA fill under
    fill_sda = [None]

    # Bit annotations (labels above SDA line)
    bit_texts = []
    for x_pos, label, color, y_pos in anns:
        txt = ax_sda.text(x_pos, y_pos, label, color=color,
                          fontsize=6.5, ha="center", va="bottom",
                          fontfamily="monospace", alpha=0.0)
        bit_texts.append((txt, x_pos))

    # ── Packet info panel ─────────────────────────────────────────────────────
    ax_info = fig.add_subplot(gs[2])
    ax_info.set_facecolor(PANEL_BG)
    ax_info.set_xlim(0, 1)
    ax_info.set_ylim(0, 1)
    ax_info.axis("off")
    ax_info.set_title("Transaction Decoder",
                      color=TEXT, fontsize=9, pad=5, loc="left")

    # Build info text
    freq = SPEED_MODES.get(speed_mode, 100_000)
    speed_str = f"{freq/1000:.0f} kHz ({speed_mode.capitalize()})"

    info_lines = [
        (f"Device Address : 0x{addr:02X}  ({addr})",  0.92, ADDR_C),
        (f"R/W            : WRITE (0)",                0.82, RW_C),
        (f"Speed mode     : {speed_str}",              0.72, TEXT),
        (f"Data bytes     : {len(data_bytes)}",        0.62, TEXT),
        (f"Total bits     : {len(tokens)} tokens",     0.52, DIM),
    ]
    for text, y_frac, color in info_lines:
        ax_info.text(0.02, y_frac, text, transform=ax_info.transAxes,
                     color=color, fontsize=8.5, va="center",
                     fontfamily="monospace")

    # Data bytes display
    byte_x = 0.02
    ax_info.text(byte_x, 0.38, "Data payload :", transform=ax_info.transAxes,
                 color=DIM, fontsize=8, va="center")
    for i, b in enumerate(data_bytes):
        col = DATA_C if i % 2 == 0 else "#ff8844"
        ax_info.text(byte_x + 0.16 + i * 0.13, 0.38,
                     f"0x{b:02X}", transform=ax_info.transAxes,
                     color=col, fontsize=9, fontweight="bold",
                     fontfamily="monospace", va="center")

    # Legend
    legend_items = [
        ("START/STOP", START_C), ("ADDR bits", ADDR_C),
        ("R/W bit",    RW_C),    ("DATA bits", DATA_C),
        ("ACK (LOW)",  ACK_COLOR), ("NACK (HIGH)", NACK_C),
    ]
    lx = 0.02
    ax_info.text(lx, 0.18, "Legend:", transform=ax_info.transAxes,
                 color=DIM, fontsize=7.5)
    for i, (lbl, col) in enumerate(legend_items):
        x = lx + 0.16 + i * 0.14
        ax_info.add_patch(FancyBboxPatch(
            (x, 0.06), 0.11, 0.08,
            boxstyle="round,pad=0.01",
            linewidth=0.8, edgecolor=col,
            facecolor=col + "22",
            transform=ax_info.transAxes,
        ))
        ax_info.text(x + 0.055, 0.10, lbl, transform=ax_info.transAxes,
                     color=col, fontsize=6, ha="center", va="center")

    # ── Title ─────────────────────────────────────────────────────────────────
    fig.text(0.5, 0.96, "I2CPlayground", ha="center", va="top",
             fontsize=17, fontweight="bold", color="#00e5ff",
             fontfamily="monospace")
    fig.text(0.5, 0.934,
             "I²C Protocol Simulator  ·  START / ADDR / R/W / ACK / DATA / STOP  ·  Day 19 — BUILDCORED ORCAS",
             ha="center", va="top", fontsize=7.5, color=DIM)

    handles = dict(
        fig=fig, ax_scl=ax_scl, ax_sda=ax_sda,
        line_scl=line_scl, line_sda=line_sda,
        fill_sda=fill_sda, bit_texts=bit_texts,
    )
    return handles


def make_animation(handles: dict, scl: np.ndarray, sda: np.ndarray,
                   anns: list, interval_ms: int = 30):
    """
    Animate SCL and SDA being 'drawn' sample by sample.
    The bit labels fade in as the wave passes through them.
    """
    n        = len(scl)
    # Show the wave in chunks (step > 1 = faster animation)
    step     = max(1, n // 300)

    x_all    = np.arange(n)

    def update(frame_idx):
        end = min(frame_idx * step + step, n)
        x   = x_all[:end]

        handles["line_scl"].set_data(x, scl[:end])
        handles["line_sda"].set_data(x, sda[:end])

        # Update fill
        if handles["fill_sda"][0] is not None:
            try:
                handles["fill_sda"][0].remove()
            except Exception:
                pass
        handles["fill_sda"][0] = handles["ax_sda"].fill_between(
            x, sda[:end], alpha=0.08, color=SDA_COLOR, step="post"
        )

        # Fade in bit labels as wave passes
        for txt, x_pos in handles["bit_texts"]:
            if x_pos <= end:
                txt.set_alpha(1.0)

        return (handles["line_scl"], handles["line_sda"])

    n_frames = n // step + 1
    anim = animation.FuncAnimation(
        handles["fig"],
        update,
        frames=n_frames,
        interval=interval_ms,
        blit=False,
        repeat=False,
    )
    return anim


# ── Sensor data encoder ───────────────────────────────────────────────────────

def encode_sensor_data(values: list) -> list:
    """
    Pack sensor values into bytes using struct.
    Supports int, float, and 16-bit signed integers.
    Each value is sent as a 16-bit big-endian signed integer (like MPU6050).
    """
    result = []
    for v in values:
        # Clamp to int16 range
        iv = int(np.clip(v, -32768, 32767))
        # Big-endian 16-bit signed
        high = (iv >> 8) & 0xFF
        low  =  iv       & 0xFF
        result.extend([high, low])
    return result


# ── Banner ────────────────────────────────────────────────────────────────────

def print_banner(addr, data, speed):
    print("\n" + "─" * 60)
    print("  I2CPlayground  ·  I²C Protocol Simulator")
    print("  Day 19 — BUILDCORED ORCAS")
    print("─" * 60)
    print(f"  Device address : 0x{addr:02X}  ({addr})")
    print(f"  Speed mode     : {speed}  ({SPEED_MODES[speed]//1000} kHz)")
    print(f"  Sensor values  : {data}")
    print(f"  Encoded bytes  : {encode_sensor_data(data)}")
    print("─" * 60)
    print("  Close the plot window to exit.")
    print("─" * 60 + "\n")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="I2CPlayground — I2C protocol simulator with animated SDA/SCL"
    )
    parser.add_argument("--addr",    "-a", type=lambda x: int(x, 0),
                        default=0x68,
                        help="7-bit I2C device address (default: 0x68 = MPU6050)")
    parser.add_argument("--data",    "-d", type=float, nargs="+",
                        default=[25.0, 512, -100],
                        help="Sensor values to encode (default: 25.0 512 -100)")
    parser.add_argument("--speed",   "-s", default="standard",
                        choices=list(SPEED_MODES.keys()),
                        help="Speed mode label (default: standard)")
    parser.add_argument("--register","-r", type=lambda x: int(x, 0),
                        default=0x3B,
                        help="Register address byte (default: 0x3B = MPU6050 ACCEL_XOUT_H)")
    parser.add_argument("--no-anim", action="store_true",
                        help="Print packet summary only, skip animation")
    args = parser.parse_args()

    addr        = args.addr & 0x7F
    data_values = args.data
    data_bytes  = encode_sensor_data(data_values)

    print_banner(addr, data_values, args.speed)

    # ── Encode ──────────────────────────────────────────────────────────────
    encoder = I2CEncoder(addr, read=False)
    tokens  = encoder.write(data_bytes, register=args.register)

    # ── Decode + verify ──────────────────────────────────────────────────────
    decoder = I2CDecoder(tokens)
    decoded = decoder.decode()
    print_packet_summary(tokens, decoded, data_bytes, addr, register=args.register)

    if args.no_anim:
        return

    # ── Build signals ────────────────────────────────────────────────────────
    scl, sda, anns = tokens_to_signals(tokens)

    print(f"  Signal length : {len(scl)} samples")
    print(f"  Total tokens  : {len(tokens)}")
    print(f"  Animating...\n")

    # ── Plot + animate ────────────────────────────────────────────────────────
    handles = build_figure(scl, sda, anns, tokens, args.speed,
                           addr, data_bytes)
    anim    = make_animation(handles, scl, sda, anns, interval_ms=25)

    plt.show()
    print(f"\n  I2CPlayground closed.\n")


if __name__ == "__main__":
    main()