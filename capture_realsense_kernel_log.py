"""Save kernel messages for a RealSense diagnostic time window."""

import argparse
from datetime import datetime
from pathlib import Path
import subprocess


def capture_kernel_log(since: str, until: str, boot: int, output: Path) -> Path:
    command = [
        "journalctl", "-k", "-b", str(boot),
        "--since", since, "--until", until, "--no-pager",
    ]
    try:
        result = subprocess.run(command, capture_output=True, text=True, check=False)
    except OSError as error:
        raise RuntimeError(f"Unable to run journalctl: {error}") from error
    if result.returncode != 0:
        raise RuntimeError(f"journalctl failed ({result.returncode}): {result.stderr.strip()}")

    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    header = (
        f"Collected: {datetime.now().astimezone().isoformat(timespec='seconds')}\n"
        f"Boot: {boot}\nSince: {since}\nUntil: {until}\n\n"
    )
    output.write_text(header + result.stdout, encoding="utf-8")
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--since", required=True, help="Start time understood by journalctl")
    parser.add_argument("--until", required=True, help="End time understood by journalctl")
    parser.add_argument("--boot", type=int, default=0, help="0=current boot; -1=previous boot")
    parser.add_argument("--output", type=Path, required=True, help="Destination log file")
    args = parser.parse_args()
    path = capture_kernel_log(args.since, args.until, args.boot, args.output)
    print(path)


if __name__ == "__main__":
    main()
