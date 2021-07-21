esptool.py --chip esp32 --port /dev/cuaU0 --baud 921600 write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x10000 /tmp/arduino_out/main.ino.bin
