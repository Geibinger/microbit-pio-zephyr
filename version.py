import subprocess, os

Import("env")

repo_root = os.getcwd()
src_dir = os.path.join(repo_root, "src")
out_file = os.path.join(src_dir, "version.h")

try:
    tag = (
        subprocess.check_output(
            ["git", "describe", "--tags", "--dirty"],
            cwd=repo_root,
            stderr=subprocess.DEVNULL
        )
        .strip().decode("utf-8")
    )
except subprocess.CalledProcessError:
    tag = (
        subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=repo_root
        )
        .strip().decode("utf-8")
    )

with open(out_file, "w") as f:
    f.write(f"""#ifndef VERSION_H
#define VERSION_H

#define FIRMWARE_VERSION "{tag}"

#endif // VERSION_H
""")