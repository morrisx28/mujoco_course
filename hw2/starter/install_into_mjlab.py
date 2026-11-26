"""Copy the HW2 starter files into an mjlab v1.6.0 checkout (standard library only).

Usage, from the mjlab checkout:

    python /path/to/mujoco_course/hw2/starter/install_into_mjlab.py --mjlab .

Installs:
  robot/pineapple_v0/{bipedwheel.xml, meshes/} -> src/mjlab/asset_zoo/robots/pineapple_v0/xmls/
  starter/asset_zoo/pineapple_v0/*.py          -> src/mjlab/asset_zoo/robots/pineapple_v0/
  starter/tasks/pineapple_v0/*.py              -> src/mjlab/tasks/velocity/config/pineapple_v0/

The robot files are always refreshed. Your task files are never overwritten unless you pass
--overwrite-task, so re-running the installer cannot erase your work.
"""

import argparse
import re
import shutil
import sys
from pathlib import Path

REQUIRED_VERSION = "1.6.0"
STARTER = Path(__file__).resolve().parent
COURSE = STARTER.parents[1]
ROBOT = COURSE / "robot" / "pineapple_v0"


def mjlab_version(root: Path) -> str:
  text = (root / "pyproject.toml").read_text()
  if not re.search(r'^name\s*=\s*"mjlab"', text, re.M):
    raise SystemExit(f"{root} is not an mjlab checkout (pyproject.toml has no name = \"mjlab\")")
  match = re.search(r'^version\s*=\s*"([^"]+)"', text, re.M)
  return match.group(1) if match else "unknown"


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
  parser.add_argument("--mjlab", type=Path, default=Path.cwd(), help="mjlab checkout (default: cwd)")
  parser.add_argument("--overwrite-task", action="store_true", help="replace existing task files")
  parser.add_argument("--allow-other-version", action="store_true")
  args = parser.parse_args()

  root = args.mjlab.resolve()
  if not (root / "pyproject.toml").exists():
    raise SystemExit(f"{root} has no pyproject.toml; pass --mjlab <path to mjlab checkout>")
  version = mjlab_version(root)
  if version != REQUIRED_VERSION and not args.allow_other_version:
    raise SystemExit(
      f"mjlab version is {version}, HW2 requires {REQUIRED_VERSION}: "
      f"run `git checkout v{REQUIRED_VERSION}` in {root}"
    )

  src = root / "src" / "mjlab"
  asset_dst = src / "asset_zoo" / "robots" / "pineapple_v0"
  task_dst = src / "tasks" / "velocity" / "config" / "pineapple_v0"

  # Robot model and constants: always refreshed (they are given, not edited by students).
  xml_dst = asset_dst / "xmls"
  xml_dst.mkdir(parents=True, exist_ok=True)
  shutil.copy2(ROBOT / "bipedwheel.xml", xml_dst / "bipedwheel.xml")
  shutil.copytree(ROBOT / "meshes", xml_dst / "meshes", dirs_exist_ok=True)
  for f in (STARTER / "asset_zoo" / "pineapple_v0").glob("*.py"):
    shutil.copy2(f, asset_dst / f.name)
  print(f"robot  -> {asset_dst.relative_to(root)}")

  # Task skeleton: never overwrite the student's work by default.
  task_dst.mkdir(parents=True, exist_ok=True)
  for f in sorted((STARTER / "tasks" / "pineapple_v0").glob("*.py")):
    target = task_dst / f.name
    if target.exists() and not args.overwrite_task:
      print(f"keep   {target.relative_to(root)} (already exists)")
      continue
    shutil.copy2(f, target)
    print(f"task   -> {target.relative_to(root)}")

  print("\nNext: uv run python " + str(STARTER / "check_setup.py"))


if __name__ == "__main__":
  sys.exit(main())
