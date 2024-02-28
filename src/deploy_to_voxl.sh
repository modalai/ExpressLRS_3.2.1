#!/bin/bash

VERSION=0
BUILD_DIR=".pio/build/MODALAI_Frsky_RX_R9MM_R9MINI_via_UART"
FW_VER="M0139-3.2.1.$VERSION.bin"

# Print out md5sum so we can verify what we push onto target matches what we just built and ecnrypted here
md5sum $BUILD_DIR/$FW_VER

# Push binary on target
adb push $BUILD_DIR/$FW_VER /usr/share/modalai/voxl-elrs/esptool/rx

# Run flash script
# adb shell "echo 2 | voxl-elrs -w"