#!/bin/sh

echo "Emmc performance test start!"
mkdir -p ../output
loop_num=2000

for i in $(seq 1 $loop_num)
do
	echo "loop_test: ${i}"
    ../bin/iozone -e -I -a -r 4K -r 16K -r 64K -r 256K -r 1M -r 4M -r 16M -s 16K -s 1M -s 16M -s 128M -s 1G -f ../output/iozone_data -Rb ../output/test_iozone_emmc_ext4_performance_${i}.xls
    if [ "$?" != "0" ];then
		echo "Test fail loop ${i} error!" >> ../output/emmc_performance.log
		exit 1
	else
		echo "Test loop ${i} success!" >> ../output/emmc_performance.log
	fi
done