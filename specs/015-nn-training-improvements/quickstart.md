# Quickstart: NN Training Improvements

## Prerequisites

```bash
sudo apt-get install -y libeigen3-dev libvtk9-dev g++ cmake gdb qtbase5-dev
```

## Build

```bash
cd ~/autoc
bash scripts/rebuild.sh
```

## Run (baseline — legacy scalar fitness)

```bash
cd ~/autoc
./build/autoc
```

## Run with Sigma Floor

Edit `autoc.ini`:
```ini
NNSigmaFloor = 0.05
```
Then: `./build/autoc`

Verify: console output shows `sigma >= 0.05` for all individuals.

## Run with Minimax Selection

Edit `autoc.ini`:
```ini
FitnessAggregation = minimax
```
Then: `./build/autoc`

Verify: console output shows per-scenario breakdown and worst-case scenario driving selection.

## Run with Segment Scoring

Edit `autoc.ini`:
```ini
SegmentScoringEnabled = 1
FitnessAggregation = minimax
```
Then: `./build/autoc`

Verify: console output includes per-segment scores with difficulty weights.

## Run Tests

```bash
cd ~/autoc/build
ctest --output-on-failure

# Specific test suites:
ctest -R sigma_floor --output-on-failure
ctest -R fitness_aggregator --output-on-failure
ctest -R segment --output-on-failure
```

## Export to Xiao

```bash
./build/nnextractor -k nn-{timestamp} -o nn_weights.dat -i autoc.ini
./build/nn2cpp -i nn_weights.dat -o xiao/generated/nn_program_generated.cpp
cd xiao && pio run -e xiaoblesense_arduinocore_mbed
```
