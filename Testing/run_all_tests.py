import os
import sys
import subprocess

base_dir = os.path.dirname(__file__)
scripts_folder = os.path.join(base_dir, "scripts")

failed = []

for script in sorted(os.listdir(scripts_folder)):
    if script.endswith(".py") and script != "main.py":
        print(f"\n--- Running {script} ---")
        result = subprocess.run(
            [sys.executable, os.path.join(scripts_folder, script)],
            cwd=base_dir,
        )
        if result.returncode != 0:
            failed.append(script)

if failed:
    print(f"\nFailed scripts: {', '.join(failed)}")
else:
    print("\nAll scripts executed successfully.")
