from sys import platform
from pathlib import Path
import os

base_dir = Path.cwd()

if platform != "linux" and platform != "linux2":
    dll_path = base_dir / "x64"
    os.add_dll_directory(dll_path)