#!/bin/bash

# ENV="MODALAI_Frsky_RX_R9MM_R9MINI_via_UART"
ENV="MODALAI_M0139_via_UART"

# Clean application
platformio run --target clean --environment $ENV

