# Generated Files

This directory receives output from `nn2cpp` (the NN-to-C++ code generator).

Files placed here are compiled into the xiao firmware build via PlatformIO.

## Usage

```bash
# From autoc root, after training:
# -w = genome (NN01), -i = the ini the RUN used (arena + tracking cone are
# baked from it; there are no CLI overrides).
./build/nn2cpp -w <nn_archive.bin> -i autoc.ini -o xiao/src/generated/nn_program_generated.cpp
```

All files in this directory except this README are git-ignored.
