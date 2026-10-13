"""Timestamp explicit dataset roots as well as repo IDs in the pinned checkout."""

from pathlib import Path

path = Path(__file__).resolve().parents[1] / "vendor/lerobot/src/lerobot/configs/dataset.py"
source = path.read_text()
old = '''        if self.repo_id:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.repo_id = f"{self.repo_id}_{timestamp}"
'''
new = '''        # Use one timestamp for both names, including rapid successive runs.
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        if self.repo_id:
            self.repo_id = f"{self.repo_id}_{timestamp}"
        if self.root is not None:
            root = Path(self.root)
            self.root = root.with_name(f"{root.name}_{timestamp}")
'''
if new in source:
    print("Dataset folder timestamp patch already applied")
elif source.count(old) == 1:
    path.write_text(source.replace(old, new))
    print("Applied dataset folder timestamp patch")
else:
    raise SystemExit("Unexpected dataset configuration source; review before patching")
