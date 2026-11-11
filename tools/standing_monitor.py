"""Record recovery, waveform and reset diagnostics without commanding motion."""

import argparse
import json
import re
import time
from pathlib import Path

import serial
from serial.tools import list_ports


def numeric_fields(line):
    result = {}
    for key, value in re.findall(r"([a-z_]+)=(-?\d+(?:,-?\d+)*)", line):
        values = [int(item) for item in value.split(",")]
        result[key] = values[0] if len(values) == 1 else values
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=120)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--port")
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("seconds must be positive")
    port = args.port
    if port is None:
        matches = [item.device for item in list_ports.comports()
                   if item.serial_number == "6E0C2EEC"]
        if len(matches) != 1:
            parser.error("Connect XIAO 6E0C2EEC, or provide --port")
        port = matches[0]

    args.out.mkdir(parents=True, exist_ok=True)
    commands = (b"state\n", b"gate\n", b"pwm\n", b"health\n", b"config\n")
    frames = []
    events = []
    started = time.monotonic()
    next_query = 0.
    query_index = 0
    last_progress = 0.
    try:
        with serial.Serial(port, 115200, timeout=.1, write_timeout=1) as device:
            with (args.out / "telemetry.txt").open("w") as log:
                while time.monotonic() - started < args.seconds:
                    elapsed = time.monotonic() - started
                    if elapsed >= next_query:
                        device.write(commands[query_index % len(commands)])
                        device.flush()
                        query_index += 1
                        next_query = elapsed + 1.
                    line = device.readline().decode(errors="replace").strip()
                    if not line:
                        continue
                    log.write(line + "\n")
                    if line.startswith("D,"):
                        try:
                            row = [int(value) for value in line.split(",")[1:]]
                            if len(row) == 30:
                                frames.append(row)
                        except ValueError:
                            pass
                    else:
                        event = {"seconds": elapsed, "line": line}
                        if line.startswith(("PWM,", "HEALTH,", "GATE,")):
                            event["fields"] = numeric_fields(line)
                        events.append(event)
                        if line.startswith(("PWM,", "HEALTH,", "OFF:", "REFUSED:")):
                            print(line, flush=True)
                    if elapsed - last_progress >= 15:
                        last_progress = elapsed
                        print(json.dumps({"seconds": round(elapsed, 1),
                                          "frames": len(frames),
                                          "mode": frames[-1][2] if frames else None}), flush=True)
    finally:
        pwm = [event["fields"] for event in events if event["line"].startswith("PWM,")]
        health = [event["fields"] for event in events if event["line"].startswith("HEALTH,")]
        summary = {
            "recording_seconds": time.monotonic() - started,
            "frames_received": len(frames),
            "first_frame": frames[0] if frames else None,
            "last_frame": frames[-1] if frames else None,
            "modes": sorted({row[2] for row in frames}),
            "gate_phases": sorted({row[28] for row in frames}),
            "peak_inference_us": max((row[20] for row in frames), default=0),
            "peak_policy_late_us": max((row[21] for row in frames), default=0),
            "peak_reported_pwm_period_us": max((item["max_period_us"] for item in pwm), default=None),
            "last_pwm": pwm[-1] if pwm else None,
            "last_health": health[-1] if health else None,
            "events": events,
            "motion_commands_sent": False,
        }
        (args.out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        print(json.dumps({key: value for key, value in summary.items() if key != "events"}), flush=True)


if __name__ == "__main__":
    main()
