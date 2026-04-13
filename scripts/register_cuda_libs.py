try:
    import nvidia
except ImportError:
    raise SystemExit(
        "ERROR: 'nvidia' package not found. "
        "Install tensorflow[and-cuda] before running this script."
    )

import os

nv = nvidia.__path__[0]
paths = [
    os.path.join(nv, d, "lib")
    for d in os.listdir(nv)
    if os.path.exists(os.path.join(nv, d, "lib"))
]

with open("/etc/ld.so.conf.d/cuda-pip.conf", "w") as f:
    f.write("\n".join(paths) + "\n")

print(f"Registered {len(paths)} CUDA lib paths from {nv}")
