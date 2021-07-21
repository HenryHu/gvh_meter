#!/bin/sh

mkdir -p /tmp/arduino_out
arduino-builder -prefs build.partitions=huge_app -verbose -libraries $HOME/proj/arduino/libraries -hardware $HOME/proj/arduino/hardware -tools /usr/local/arduino/tools -tools /usr/bin -fqbn esp32:esp32:esp32 -build-path /tmp/arduino_out -build-cache $HOME/proj/arduino/cache main.ino
