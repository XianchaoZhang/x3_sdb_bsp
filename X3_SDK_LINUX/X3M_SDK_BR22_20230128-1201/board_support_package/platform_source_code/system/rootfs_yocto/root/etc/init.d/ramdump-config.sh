#!/bin/sh
# /sys/kernel/hobot-swinfo/boot
# 0:normal                      normal boot, no action
# 1:splonce                     wait in spl once
# 2:ubootonce                   wait in uboot once
# 3:splwait                     always wait in spl for warm reset, won't reset sw_info
# 4:ubootwait                   always wait in uboot for warm reset, won't reset sw_info
# 5:udumptf                     ramdump with tf card
# 6:udumpemmc                   ramdump with emmc
# 7:udumpusb                    ramdump with u disk
# 8:udumpfastboot               ramdump with fastboot

echo "set ubootonce as default panic action"

# only panic will cause reboot into ubootonce.
# not suggest using ubootwait, otherwise every
# warm-boot will cause enter ubootwait mode.
# once is enough!
echo boot=2 > /sys/kernel/hobot-swinfo/panic
