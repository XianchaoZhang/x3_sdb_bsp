#!/bin/sh

echo 111 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio111/direction
echo 0 > /sys/class/gpio/gpio111/value
sleep 0.2
echo 1 > /sys/class/gpio/gpio111/value

echo 1 > /sys/class/vps/mipi_host1/param/snrclk_en
echo 37130000 > /sys/class/vps/mipi_host1/param/snrclk_freq
echo 1 >/sys/class/vps/mipi_host1/param/stop_check_instart
