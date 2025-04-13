import subprocess

print("git update-index --no-skip-worktree ../Meridian_LITE_for_ESP32/src/keys.h")
result = subprocess.run(["git", "update-index", "--no-skip-worktree", "../Meridian_LITE_for_ESP32/src/keys.h"], capture_output=True, text=True)
print(result.stdout)
