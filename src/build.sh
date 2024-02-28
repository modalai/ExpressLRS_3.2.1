#!/bin/bash
num_variables=$#
# echo $num_variables
if [ "$num_variables" -gt 0 ]; then 
    ENCRYPT=$1
else 
    ENCRYPT=0
fi
VERSION=0
BUILD_DIR=".pio/build/MODALAI_Frsky_RX_R9MM_R9MINI_via_UART"
FW_VER="FrSky_R9Mini-3.2.1.$VERSION.bin"

# Build application
platformio run --environment MODALAI_Frsky_RX_R9MM_R9MINI_via_UART # -v > build_output.txt


md5sum $BUILD_DIR/firmware.bin

if [ "$ENCRYPT" -eq 1 ]; then 
    # Encrypt application 
    echo "Encrpyting application"
    stm32-encrypt $BUILD_DIR/firmware.bin $BUILD_DIR/$FW_VER 0E9B89FC1D97C8B7DC4B2EA4DAB2B3FF02153BCF1ED07E8617E3EFF59BB26806
else
    echo "Leaving application un-encypted"
    cp $BUILD_DIR/firmware.bin $BUILD_DIR/$FW_VER
fi

# Print out md5sum so we can verify what we push onto target matches what we just built and ecnrypted here
# md5sum FrSky_R9Mini-3.2.1.4.bin
md5sum $BUILD_DIR/$FW_VER

adb push $BUILD_DIR/$FW_VER /usr/share/modalai/voxl-elrs/esptool/rx/

cp $BUILD_DIR/$FW_VER .
cp $BUILD_DIR/$FW_VER /home/jacob/repos/voxl/voxl-elrs/tools/bins

# adb push FrSky_R9Mini-3.2.1.3.bin /usr/share/modalai/voxl-elrs/esptool/rx/