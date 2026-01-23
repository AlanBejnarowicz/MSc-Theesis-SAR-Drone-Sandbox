# MarineSIM Project Setup

## Prerequisites

### Required Tools
You need a **GCC** (or alternative C99 compiler), **make**, **git**, and **CMake**.

#### Install on Ubuntu/Debian:
```bash
sudo apt install build-essential git cmake
```

### Required Libraries
Raylib requires several libraries for audio, graphics, and window management.

#### Ubuntu/Debian:
```bash
sudo apt install libasound2-dev libx11-dev libxrandr-dev libxi-dev \
                 libgl1-mesa-dev libglu1-mesa-dev libxcursor-dev \
                 libxinerama-dev libwayland-dev libxkbcommon-dev
```

## Install Raylib with CMake

```bash
git clone https://github.com/raysan5/raylib.git raylib
cd raylib
mkdir build && cd build
cmake -DBUILD_SHARED_LIBS=ON ..
make
sudo make install
sudo ldconfig
```

**Note:** If any dependencies are missing, CMake will inform you.

## Build and Run the Project

```bash
mkdir build
cd build
cmake ..
make
./MarineSIM
```

## Quick One-liner

```bash
mkdir build && cd build && cmake .. && make && ./MarineSIM
```

## Troubleshooting

If you encounter issues:
1. Ensure all dependencies are installed
2. Try a clean build: `rm -rf build/` and start again
3. Check CMake output for specific error messages
```

