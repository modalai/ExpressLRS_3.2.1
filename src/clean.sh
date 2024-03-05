#!/bin/bash

VERSION=3
# ENV="MODALAI_Frsky_RX_R9MM_R9MINI_via_UART"
ENV="MODALAI_M0139_via_UART"
BUILD_DIR=".pio/build/$ENV"
FW_VER="FrSky_R9Mini-3.2.1.$VERSION.bin"

# Clean application
platformio run --target clean --environment $ENV

