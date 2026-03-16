# Repo Setup Notes (delete after verified)

## Step 1: Create GitHub remote

```bash
# Create repo on GitHub (via web or gh cli)
gh repo create gcmcnutt/autoc --private --source=. --push

# Or manually:
# 1. Create empty repo at github.com/gcmcnutt/autoc
# 2. git remote add origin https://github.com/gcmcnutt/autoc.git
# 3. git push -u origin master
```

## Step 2: Archive old repos

```bash
mkdir -p ~/old
mv ~/GP ~/old/GP
mv ~/xiao-gp ~/old/xiao-gp
mv ~/crsim ~/old/crsim
```

## Step 3: Commit and push

```bash
cd ~/autoc
git add -A
git commit -m "feat: relative paths, unified cmake, xiao integrated"
git push origin master
```

## Step 4: Rename and fresh clone

```bash
mv ~/autoc ~/autoc.orig
git clone --recurse-submodules https://github.com/gcmcnutt/autoc.git ~/autoc
```

## Step 5: Verify builds

```bash
# Autoc + CRRCSim (unified build)
cd ~/autoc
bash rebuild.sh
# Should build autoc, minisim, renderer, nnextractor, nn2cpp, crrcsim
# Tests should run and pass

# xiao (PlatformIO, separate)
cd ~/autoc/xiao
~/.platformio/penv/bin/pio run -e xiaoblesense_arduinocore_mbed

# Quick smoke test
cd ~/autoc
stdbuf -o0 -e0 ./build/autoc 2>&1 | head -20
```

## Step 6: Open in VSCode

```bash
code ~/autoc
# .vscode/ configs should pick up automatically
# CMake Tools extension will find CMakeLists.txt
```

## Step 7: Continue in new session

Start a Claude Code session from ~/autoc. The HANDOFF.md in specs/014-nn-training-signal/
has full context. Next steps:
1. speckit init
2. Migrate 014 spec artifacts from old GP repo
3. Phase 6: sigma floor (first NN improvement)
