"""Paths for source-tree and installed layouts."""
from pathlib import Path
import shutil
ROOT = Path(__file__).resolve().parents[1]
def backend_path(name):
    for path in (ROOT/'build'/name, ROOT.parent.parent/'bin'/name):
        if path.is_file(): return path
    return Path(shutil.which(name) or ROOT/'build'/name)
