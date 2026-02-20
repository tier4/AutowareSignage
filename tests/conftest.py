import sys
from pathlib import Path

# Add source directories to sys.path for module resolution
# (pythonpath in pytest.ini requires pytest >= 7.0)
_base = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_base / "src" / "signage" / "src"))
sys.path.insert(0, str(_base / "src" / "external_signage" / "src"))
