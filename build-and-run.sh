#!/bin/bash
echo "Building package..."
nrfutil pkg generate --hw-version 52 --sd-req=0x00 \
 --application build/zephyr/zephyr.hex \
 --application-version 1 package.zip

echo "Flashing firmware to device..."
nrfutil dfu usb-serial -pkg package.zip -p /dev/ttyACM0
