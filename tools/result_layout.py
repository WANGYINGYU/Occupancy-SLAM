"""Canonical result locations; root-level files remain legacy compatibility copies."""
import json
from pathlib import Path
from dataclasses import dataclass
@dataclass(frozen=True)
class ResultLayout:
    root: Path
    initial: Path
    joint: Path
    reintegrated: Path
    @classmethod
    def read(cls,root):
        root=Path(root)
        path=root/'map_outputs.json'
        manifest=json.loads(path.read_text()) if path.exists() else {}
        version=manifest.get('schema_version',1)
        if version not in (1,2):raise ValueError(f'Unsupported result schema: {version}')
        return cls(root,root/manifest.get('initial','initial'),root/manifest.get('joint','.'),root/manifest.get('reintegrated','reintegrated'))
