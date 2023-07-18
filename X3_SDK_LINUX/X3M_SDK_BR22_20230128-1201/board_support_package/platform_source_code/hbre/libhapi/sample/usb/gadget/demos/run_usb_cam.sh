#!/bin/sh

echo 1 > /sys/class/vps/mipi_host1/param/snrclk_en
echo 24000000 > /sys/class/vps/mipi_host1/param/snrclk_freq
echo 1 > /sys/class/vps/mipi_host0/param/snrclk_en
echo 24000000 > /sys/class/vps/mipi_host0/param/snrclk_freq
echo 0xc0020000 > /sys/bus/platform/drivers/ddr_monitor/axibus_ctrl/all
echo 0x03120000 > /sys/bus/platform/drivers/ddr_monitor/read_qos_ctrl/all
echo 0x03120000 > /sys/bus/platform/drivers/ddr_monitor/write_qos_ctrl/all
echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor
# echo 1 > /sys/class/vps/mipi_host1/param/stop_check_instart

# bind usb irq num to cpu core 4, otherwise picture tearing issue happens.
# as cpu core 1 is very busy, with 20% interrupts. and usb isoc transfer
# also trigger so many interrupts. 
# (about 125us 1 interrupt, 8000 times per one second)
irq_num=$(cat /proc/interrupts | grep usb | awk -F ':' 'NR==1 {print $1}' | sed 's/^[ /t ]*//g')
orig=$(cat /proc/irq/$irq_num/smp_affinity)
echo 8 > /proc/irq/$irq_num/smp_affinity                # bind usb irq to core 4
new=$(cat /proc/irq/$irq_num/smp_affinity)
echo "usb irq smp_affinity is changed from $orig to $new"

service adbd stop
/etc/init.d/usb-gadget.sh start uvc isoc
./usb_camera_eptz
