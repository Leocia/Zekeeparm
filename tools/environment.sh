# Set the manually installed Conda root here; an exported value takes precedence.
export ZKEEP_MINIFORGE_DIR="${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}"
# Install the NVIDIA driver and CUDA Toolkit separately; this only sets the path.
export CUDA_HOME="${CUDA_HOME:-/usr/local/cuda-12.8}"
export ZKEEP_VISION_PYTHON="${ZKEEP_VISION_PYTHON:-$ZKEEP_MINIFORGE_DIR/envs/rebotarm/bin/python}"
export EMBODIED_PYTHON="${EMBODIED_PYTHON:-$ZKEEP_MINIFORGE_DIR/envs/embodied/bin/python}"
