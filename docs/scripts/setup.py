
import subprocess

print("git submodule update --init --recursive")
result = subprocess.run(["git", "submodule", "update", "--init", "--recursive"], capture_output=True, text=True)
print(result.stdout)

print("git update-index --skip-worktree ../Meridian_LITE_for_ESP32/src/keys.h")
result = subprocess.run(["git", "update-index", "--skip-worktree", "../Meridian_LITE_for_ESP32/src/keys.h"], capture_output=True, text=True)
print(result.stdout)
