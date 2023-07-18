#!/bin/sh

netlist="eth0 bifeth0"
for net in $netlist; do
	if [ -e /sys/class/net/$net ]; then
		defaultnet=$net
		break
	fi
done
if [ -z "$defaultnet" ]; then
	echo "no default net dev"
	exit 1
fi

macaddr=`hrut_mac g`
if [ "$macaddr" == "ff:ff:ff:ff:ff:ff" ];then
    hrut_mac c
fi

macaddr=`hrut_mac g`
if [ "$macaddr" == "00:00:00:00:00:00" ];then
    addr=$(dd if=/dev/urandom bs=1024 count=1 2>/dev/null|md5sum|sed 's/^\(..\)\(..\)\(..\)\(..\)\(..\).*$/\1:\2:\3:\4:\5/')
    macaddr="00:$addr"
    hrut_mac s $macaddr
fi

hrut_ipfull g
if [ -f /tmp/ip_ip ];then
    IP=`cat /tmp/ip_ip`
else
    IP="192.168.1.10"
    #for QUAD CASE
    SOMNO=`hrut_somid g`
    case $SOMNO in
        1) IP="192.168.1.11"
        ;;
        2) IP="192.168.1.12"
        ;;
        3) IP="192.168.1.13"
        ;;
        4) IP="192.168.1.14"
        ;;
        *) IP="192.168.1.10"
        ;;
    esac
fi


if [ -f /tmp/ip_mask ];then
    MASK=`cat /tmp/ip_mask`
else
    MASK="255.255.255.0"
fi

ifconfig $defaultnet down
ifconfig $defaultnet hw ether $macaddr
ifconfig $defaultnet $IP netmask $MASK
ifconfig $defaultnet up
ifconfig lo 127.0.0.1

if [ -f /tmp/ip_gw ];then
    GATEWAY=`cat /tmp/ip_gw`
    route add default gw $GATEWAY
fi
