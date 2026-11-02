"""Run without arguments; left-click a detected object to grasp and place it."""

from main import main


if __name__ == "__main__":
    raise SystemExit(main(click_to_grasp=True))
