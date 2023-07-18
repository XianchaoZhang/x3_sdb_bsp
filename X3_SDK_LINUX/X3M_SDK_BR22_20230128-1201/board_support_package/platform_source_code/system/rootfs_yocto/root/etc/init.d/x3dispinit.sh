#!/bin/sh

f=$(cat /sys/firmware/devicetree/base/chosen/bootargs)
kernel_ver=$(cat /proc/version | awk '{print $3}')
ko_root="/lib/modules/${kernel_ver}"
#echo $f
lcd="video=hobot:lcd"
m720p="video=hobot:720p"
x3sdb_mipi720p="video=hobot:x3sdb-mipi720p"
x3_mipi720p_h="video=hobot:dsi1280x720"
m1080p="video=hobot:1080p"
hdmi="video=hobot:hdmi"
x3sdb_hdmi1080p="video=hobot:x3sdb-hdmi"
bt656="video=hobot:bt656"
video="video=hobot"

rbt656=$(echo $f | grep "${bt656}")
if [[ "$rbt656" != "" ]]
then
  echo "display panel is BT656 panel"
  echo bt656 > /sys/devices/virtual/graphics/iar_cdev/iar_test_attr
  if [ -f "/etc/iar/iar_xj3_bt656.json" ]
  then
    /usr/bin/x3dispinit /etc/iar/iar_xj3_bt656.json &
#    echo start > /sys/devices/virtual/graphics/iar_cdev/iar_test_attr
  else
    echo "no bt656 configuration file, use default config"
  fi
fi

rhdmi=$(echo $f | grep "${x3sdb_hdmi1080p}")
if [[ "$rhdmi" != "" ]]
then
  echo "display panel is HDMI-1080P"
  if [ -f "/etc/iar/iar_x3sdb_hdmi_1080p.json" ]
  then
    /usr/bin/x3dispinit /etc/iar/iar_x3sdb_hdmi_1080p.json &
#    echo start > /sys/devices/virtual/graphics/iar_cdev/iar_test_attr
    insmod /lib/modules/${kernel_ver}/hobot_hdmi_lt8618.ko
    echo disable0 > /sys/devices/virtual/graphics/iar_cdev/iar_test_attr
  else
    echo "no hdmi 1080p configuration file, use default config"
  fi
fi

rhdmi=$(echo $f | grep "${hdmi}")
if [[ "$rhdmi" != "" ]]
then
  echo "display panel is HDMI"
  if [ -f "/etc/iar/iar_xj3_hdmi_1080p.json" ]
  then
    /usr/bin/x3dispinit /etc/iar/iar_xj3_hdmi_1080p.json &
#    echo start > /sys/devices/virtual/graphics/iar_cdev/iar_test_attr
  else
    echo "no hdmi configuration file, use default config"
  fi
fi


result=$(echo $f | grep "${m720p}")
if [[ "$result" != "" ]]
then
  echo "display panel is MIPI-720P"
  echo dsi720p > /sys/devices/virtual/graphics/iar_cdev/iar_test_attr
  if [ -f "/etc/iar/iar_xj3_mipi720p.json" ]
  then
    /usr/bin/x3dispinit /etc/iar/iar_xj3_mipi720p.json &
#    echo start > /sys/devices/virtual/graphics/iar_cdev/iar_test_attr
  else
    echo "no mipi720p configuration file, use 720p default config"
  fi
fi

result3=$(echo $f | grep "${x3_mipi720p_h}")
if [[ "$result3" != "" ]]
then
  echo "display panel is MIPI-1280x720"
  echo dsi1280x720 > /sys/devices/virtual/graphics/iar_cdev/iar_test_attr
  if [ -f "/etc/iar/iar_xj3_mipi720p_h.json" ]
  then
    /usr/bin/x3dispinit /etc/iar/iar_xj3_mipi720p_h.json &
#    echo start > /sys/devices/virtual/graphics/iar_cdev/iar_test_attr
  else
    echo "no mipi1280x720 configuration file, use default config"
  fi
fi

result0=$(echo $f | grep "${lcd}")
if [[ "$result0" != "" ]]
then
  echo "display panel is 7inch LCD"
  echo lcd > /sys/devices/virtual/graphics/iar_cdev/iar_test_attr
  if [ -f "/etc/iar/iar_xj3_lcd.json" ]
  then
    /usr/bin/x3dispinit /etc/iar/iar_xj3_lcd.json &
#    echo start > /sys/devices/virtual/graphics/iar_cdev/iar_test_attr
  else
    echo "no LCD configuration file, use LCD default config"
  fi
fi

result1=$(echo $f | grep "${m1080p}")
if [[ "$result1" != "" ]]
then
  echo "display panel is MIPI-1080P"
  echo dsi1080 > /sys/devices/virtual/graphics/iar_cdev/iar_test_attr
  if [ -f "/etc/iar/iar_xj3_mipi1080p.json" ]
  then
    /usr/bin/x3dispinit /etc/iar/iar_xj3_mipi1080p.json &
#    echo start > /sys/devices/virtual/graphics/iar_cdev/iar_test_attr
  else
    echo "no mipi1080p configuration file, use 1080p default config"
  fi
fi

result2=$(echo $f | grep "${x3sdb_mipi720p}")
if [[ "$result2" != "" ]]
then
  echo "display panel is MIPI-720P Portrait"
  if [ -f $ko_root/gt9xx.ko ];then
    insmod $ko_root/gt9xx.ko
  fi
  if [ -f "/etc/iar/iar_xj3_mipi720p.json" ]
  then
    /usr/bin/x3dispinit /etc/iar/iar_xj3_mipi720p.json &
    echo dsi720x1280 > /sys/devices/virtual/graphics/iar_cdev/iar_test_attr
    echo disable0 > /sys/devices/virtual/graphics/iar_cdev/iar_test_attr
  else
    echo "no mipi 720p configuration file for 720p portraint, use 1080p default config"
  fi
fi
#echo "@@@@@@disp config end@@@@@@@@@@@@"
