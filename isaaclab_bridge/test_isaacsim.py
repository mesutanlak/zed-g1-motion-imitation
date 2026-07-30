"""Minimal headless Isaac Sim smoke test."""

from isaacsim import SimulationApp


app = SimulationApp({"headless": True})
try:
    import torch

    print(f"ISAAC_SIM_STARTED gpu={torch.cuda.get_device_name(0)}")
finally:
    app.close()

