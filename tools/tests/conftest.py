import sys
from pathlib import Path

# Make `gyrofuse_tools` importable when running `pytest tools/tests` from the repo root.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
