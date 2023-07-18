#!/bin/sh

#virtual PHY(from NXP) seem can not detect PHY errors
#so using peer to check network status

#dynamic get IP List from MCU(later)
IP_J2A=192.168.1.11
IP_J2B=192.168.1.12
IP_J2C=192.168.1.13
IP_J2D=192.168.1.14

#get alive J2 Numbers
for i in $(seq 1 2)
do
ping -q -c 2 -w 1 -A ${IP_J2A} >/dev/null 2>&1
resultA=$?
ping -q -c 2 -w 1 -A ${IP_J2B} >/dev/null 2>&1
resultB=$?
ping -q -c 2 -w 1 -A ${IP_J2C} >/dev/null 2>&1
resultC=$?
ping -q -c 2 -w 1 -A ${IP_J2D} >/dev/null 2>&1
resultD=$?
#Do ETH Monitor & Decisions Actions
#DO once
RESULT=`expr $resultA + $resultB + $resultC + $resultD `
if [ $RESULT -gt 2 ];then
echo "rETH"
echo 1 > /sys/class/gpio/gpio22/value
usleep 100000
echo 0 > /sys/class/gpio/gpio22/value
else
echo "oETH"
exit 0;
fi
done


#DO period(TBD@Future)
