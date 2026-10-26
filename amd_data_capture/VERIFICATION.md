# Verification record

Local development host: Ubuntu 22.04 x86_64, Python 3.10, ROS 2 Humble.
No camera or robot was connected during these checks.

Passed:

- Fresh Python environments installed the capture, camera, and ROS dependencies.
- CPU PyTorch is `2.6.0+cpu`; `torch.version.cuda` is `None`, and no NVIDIA runtime packages are installed in the capture environment.
- `pip check` passes in capture and camera environments. Camera environment pins NumPy `1.26.4`, OpenCV `4.7.0`, and imports the Orbbec SDK.
- Nine no-device tests cover raw writing, validation, failure recovery, auditing, video decode, path portability, calibration equivalence, and preservation of sourced safety files.
- ROS builds all five retained packages. Eight gamepad safe-home tests pass. Controller, SDK, robot model, MoveIt configuration, and three launch argument sets load without starting a node.
- Shell syntax checks pass.

The ROS virtual environment includes Python packages from the host ROS installation. On this development host, `pip check` reports that the system package `PyNaCl` requires `cffi`; this unrelated system metadata warning does not block the ROS imports, build, or tests above.

Not verified: clean-target OS/ROS provisioning, AMD hardware, camera capture, serial connection, or real robot motion. Run `doctor` and `self-check` after installation; supervise `camera-check` and all hardware operation.
