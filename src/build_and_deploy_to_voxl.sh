#!/bin/bash

VERSION=00
BUILD_DIR=".pio/build/MODALAI_Frsky_RX_R9MM_R9MINI_via_UART"
FW_VER="M0139-3.2.1.$VERSION.bin"

# Build application
platformio run --environment MODALAI_Frsky_RX_R9MM_R9MINI_via_UART

# Encrypt application 
stm32-encrypt $BUILD_DIR/firmware.bin $BUILD_DIR/$FW_VER 0E9B89FC1D97C8B7DC4B2EA4DAB2B3FF02153BCF1ED07E8617E3EFF59BB26806

# Print out md5sum so we can verify what we push onto target matches what we just built and ecnrypted here
md5sum $BUILD_DIR/$FW_VER

# Push binary on target
adb push $BUILD_DIR/$FW_VER /usr/share/modalai/voxl-elrs/esptool/rx

# Run flash script
# adb shell "echo 2 | voxl-elrs -w"
# adb shell voxl-elrs --scan