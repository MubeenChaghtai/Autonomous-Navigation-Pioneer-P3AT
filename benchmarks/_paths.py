import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src" / "p3at_navigation"
RESULTS = ROOT / "results"
if str(PACKAGE) not in sys.path:
    sys.path.insert(0, str(PACKAGE))
