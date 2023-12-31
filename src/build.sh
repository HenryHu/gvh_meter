#!/bin/sh

mkdir -p /tmp/arduino_out

ARGS="-prefs build.partitions=huge_app -prefs upload.maximum_size=3145728 -verbose -libraries $HOME/proj/arduino/libraries -hardware $HOME/proj/arduino/hardware -tools /usr/local/arduino/tools -tools /usr/bin -fqbn esp32:esp32:esp32 -build-path /tmp/gvh_meter -build-cache $HOME/proj/arduino/cache main.ino"

arduino-builder -dump-prefs $ARGS
arduino-builder $ARGS
