# Generated Files

This directory receives output from `nn2cpp` (the NN-to-C++ code generator).

Files placed here are compiled into the xiao firmware build via PlatformIO.

## Usage

```bash
# From autoc root, after training:
./build/nn2cpp -i <nn_archive.bin> -o xiao/src/generated/
```

All files in this directory except this README are git-ignored.
