"""Start/stop only the RViz session created by this tool (Linux/WSL)."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess

workspace = Path(os.environ.get("FRC_SIM_WS", str(Path.home() / "frc-portfolio-sim")))
record = workspace / "session.json"


def alive(info):
    try:
        # PID reuse must not let a stale session record target another process.
        return Path(f"/proc/{info['pid']}/stat").read_text().split()[21] == info["start_ticks"]
    except (FileNotFoundError, ProcessLookupError):
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("start", "stop", "status"))
    args = parser.parse_args()
    info = json.loads(record.read_text()) if record.exists() else None
    if args.action == "start":
        if info and alive(info):
            raise SystemExit("This tool's simulator is already running")
        workspace.mkdir(parents=True, exist_ok=True)
        log = workspace / "live-sim.log"
        with log.open("w") as stream:
            process = subprocess.Popen(["bash", str(Path(__file__).with_name("run_sim.sh"))],
                                       stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        info = {"pid": process.pid, "start_ticks": Path(f"/proc/{process.pid}/stat").read_text().split()[21],
                "log": str(log)}
        record.write_text(json.dumps(info))
        print(json.dumps(info))
    elif args.action == "stop":
        if info and alive(info):
            os.killpg(info["pid"], signal.SIGINT)
            print("Sent SIGINT to this tool's simulator process group")
        else:
            print("This tool's simulator is not running")
    else:
        print(json.dumps({"running": bool(info and alive(info)), "session": info}))


if __name__ == "__main__":
    main()
