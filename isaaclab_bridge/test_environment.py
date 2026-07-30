"""Package/GPU test that does not launch Isaac Sim or require EULA acceptance."""

import importlib.metadata as metadata

import torch


for package in ("isaacsim", "isaaclab", "unitree_rl_lab", "unitree_sdk2py"):
    try:
        version = metadata.version(package)
    except metadata.PackageNotFoundError:
        version = "NOT_INSTALLED (optional for simulation)"
    print(f"{package}={version}")
print(f"torch={torch.__version__} cuda={torch.version.cuda}")
print(f"gpu={torch.cuda.get_device_name(0)} archs={torch.cuda.get_arch_list()}")
print(f"cuda_tensor={(torch.arange(3).cuda() + 1).cpu().tolist()}")
