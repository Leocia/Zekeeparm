"""Grasp matching objects until none remain, using the shared SDK route.

Usage:
    python scripts/continuous_grasp.py --target-class "red cube"
    python scripts/continuous_grasp.py --target-class cube
"""

from main import main


if __name__ == "__main__":
    raise SystemExit(main(continuous=True))
