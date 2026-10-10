"""Sweep the actuator through f/r at 10..100 % and report.

Run with PlatformIO's python (has pyserial):
  C:\\Users\\anmol\\.platformio\\penv\\Scripts\\python.exe tools\\actuator_sweep.py [hold_ms]

The actuator Teensy is found by probing every Teensy port with the 'a' command.
If the encoder Teensy is also connected (it streams "RAW = ..." lines) its angle
is logged during each move and the motion is analysed.
"""
import re
import sys
import threading
import time
import statistics as st

import serial
from serial.tools import list_ports

HOLD_MS = int(sys.argv[1]) if len(sys.argv) > 1 else 800
SPEEDS = range(10, 101, 10)
PAUSE_S = 1.0

ENC_RE = re.compile(r"RAW = (\d+)")


def teensy_ports():
    return [p.device for p in list_ports.comports() if p.vid == 0x16C0]


def find_boards():
    act = enc = None
    for dev in teensy_ports():
        s = serial.Serial(dev, 115200, timeout=0.4)
        time.sleep(0.4)
        s.reset_input_buffer()
        line = s.readline().decode(errors="ignore")
        if ENC_RE.search(line):
            enc = s
            continue
        s.write(b"a 150\n")
        time.sleep(0.3)
        if "Ramp rate" in s.read(200).decode(errors="ignore"):
            act = s
        else:
            s.close()
    return act, enc


class EncoderLog(threading.Thread):
    def __init__(self, ser):
        super().__init__(daemon=True)
        self.ser, self.samples, self.going = ser, [], True

    def run(self):
        while self.going:
            m = ENC_RE.search(self.ser.readline().decode(errors="ignore"))
            if m:
                self.samples.append((time.time(), int(m[1]) * 360.0 / 2**26))


def unwrap(angles):
    out, off = [angles[0]], 0.0
    for a, b in zip(angles, angles[1:]):
        d = b - a
        if d > 180:
            off -= 360
        elif d < -180:
            off += 360
        out.append(b + off)
    return out


def analyse(samples, t0, t1):
    seg = [(t, a) for t, a in samples if t0 <= t <= t1]
    if len(seg) < 10:
        return None
    ts = [t for t, _ in seg]
    an = unwrap([a for _, a in seg])
    rate = [(an[i + 1] - an[i]) / (ts[i + 1] - ts[i]) for i in range(len(an) - 1)
            if ts[i + 1] > ts[i]]
    mid = rate[len(rate) // 4: 3 * len(rate) // 4] or rate
    return {
        "delta": an[-1] - an[0],
        "peak": max(abs(r) for r in rate),
        "mid_mean": st.mean(mid),
        "mid_std": st.pstdev(mid),
    }


def move(act, enc_log, cmd):
    act.reset_input_buffer()
    t0 = time.time()
    print(f"[{time.strftime('%H:%M:%S')}] {cmd}", flush=True)
    act.write((cmd + "\n").encode())
    done = None
    while time.time() - t0 < HOLD_MS / 1000 + 6:
        line = act.readline().decode(errors="ignore").strip()
        if line == "DONE":
            done = time.time()
            break
    return t0, done


def main():
    act, enc = find_boards()
    if not act:
        sys.exit("Actuator not found (needs the ramp firmware).")
    print(f"actuator: {act.port}   encoder: {enc.port if enc else 'NOT connected'}   hold {HOLD_MS} ms")
    log = None
    if enc:
        log = EncoderLog(enc)
        log.start()
        time.sleep(1)

    rows = []
    try:
        for sp in SPEEDS:
            res = {}
            for name, c in (("f", "f"), ("r", "r")):
                t0, done = move(act, log, f"{c} {sp} {HOLD_MS}")
                res[name] = (t0, done, analyse(log.samples, t0, done + 0.3) if log and done else None)
                time.sleep(PAUSE_S)
            rows.append((sp, res))
    finally:
        act.write(b"x\n")
        if log:
            log.going = False

    print()
    hdr = "speed | fwd time  rev time |"
    if enc:
        hdr += " fwd dAng  rev dAng  asym% | steady-rate fwd/rev (deg/s)  jitter% fwd/rev"
    print(hdr)
    for sp, r in rows:
        tf = (r["f"][1] - r["f"][0]) if r["f"][1] else float("nan")
        tr = (r["r"][1] - r["r"][0]) if r["r"][1] else float("nan")
        line = f"{sp:4d}% | {tf:7.2f}s {tr:7.2f}s |"
        if enc:
            f, b = r["f"][2], r["r"][2]
            if f and b:
                asym = 100 * (abs(f["delta"]) - abs(b["delta"])) / max(abs(f["delta"]), 1e-9)
                jf = 100 * f["mid_std"] / max(abs(f["mid_mean"]), 1e-9)
                jb = 100 * b["mid_std"] / max(abs(b["mid_mean"]), 1e-9)
                line += (f" {f['delta']:8.1f} {b['delta']:8.1f} {asym:6.1f} |"
                         f" {f['mid_mean']:8.1f}/{b['mid_mean']:8.1f}          {jf:5.0f}/{jb:5.0f}")
            else:
                line += " (no encoder data)"
        print(line)


if __name__ == "__main__":
    main()
