# AutoC - Neural Network Aircraft Controller Evolution

AutoC evolves neural network controllers for autonomous RC aircraft flight. It uses evolutionary strategies to optimize NN weights for path-following in simulation, then deploys trained networks to embedded hardware (Seeed XIAO BLE Sense).

## Components

| Component | Description |
|-----------|-------------|
| **autoc** | Main evolution engine — multi-threaded population evaluation |
| **minisim** | Lightweight aircraft physics simulation (training) |
| **crrcsim** | Full flight dynamics model (CRRCSim fork, validation) |
| **renderer** | VTK-based 3D flight path visualization |
| **nnextractor** | Extract NN weights from evolution checkpoints |
| **nn2cpp** | Generate embedded C++ from trained NN weights |
| **xiao/** | PlatformIO project for Seeed XIAO BLE Sense deployment |

## Build

```bash
# Dependencies (Ubuntu 22.04+)
sudo apt-get install -y libeigen3-dev libvtk9-dev libsdl1.2-dev \
  libplib-dev libjpeg-dev g++ cmake gdb libgl-dev \
  libaws-cpp-sdk-core-dev libaws-cpp-sdk-s3-dev

# Debug build (autoc + crrcsim + tests)
bash scripts/rebuild.sh

# Performance build
bash scripts/rebuild-perf.sh

# Incremental rebuild
cd build && make

# Run tests only
cd build && ctest --output-on-failure
```

### Embedded (xiao)

```bash
cd xiao
~/.platformio/penv/bin/pio run -e xiaoblesense_arduinocore_mbed
```

## Configuration

Runtime config via `autoc.ini`:

```ini
PopulationSize=500
NumberOfGenerations=100
SimNumPathsPerGeneration=6
EvalThreads=12
PathGeneratorMethod=random
```

## Repo Layout

```
include/autoc/       # Headers (nn/, eval/, util/, rpc/)
src/                 # Core source (autoc.cc, nn/, eval/, util/)
tools/               # minisim, renderer, nnextractor, nn2cpp
tests/               # Unit + contract tests (GoogleTest)
crrcsim/             # CRRCSim FDM (git submodule)
xiao/                # Embedded target (PlatformIO)
specs/               # Feature specifications
```

## AWS Integration

Evolution checkpoints stored in S3:
- Key format: `autoc-{timestamp}/gen{number}.dmp`
- Configure bucket/profile in `autoc.ini`
