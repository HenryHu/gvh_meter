#!/bin/sh

# Full command:
# esptool.py --chip esp32 --port "{serial.port}" --baud {upload.speed}  --before default_reset --after hard_reset write_flash -z --flash_mode {build.flash_mode} --flash_freq {build.flash_freq} --flash_size detect 0xe000 "{runtime.platform.path}/tools/partitions/boot_app0.bin" 0x1000 "{runtime.platform.path}/tools/sdk/bin/bootloader_{build.boot}_{build.flash_freq}.bin" 0x10000 "{build.path}/{build.project_name}.bin" 0x8000 "{build.path}/{build.project_name}.partitions.bin"

#esptool.py write_flash 0x8000 0xc00 /tmp/arduino_out/main.ino.partitions.bin
esptool.py --chip esp32 --port /dev/cuaU1 --baud 921600 write_flash -z --flash_mode dio --flash_freq 80m --flash_size 4MB 0x10000 /tmp/gvh_meter/main.ino.bin
