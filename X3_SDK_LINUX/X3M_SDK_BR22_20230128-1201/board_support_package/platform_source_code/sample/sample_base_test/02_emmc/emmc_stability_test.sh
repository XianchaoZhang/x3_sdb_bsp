#!/bin/sh

echo "Emmc stability test start!"
mkdir -p ../output
loop_num=2000

for i in $(seq 1 $loop_num)
do
	echo "loop_test: ${i}"
    ../bin/iozone -e -I -az -n 16m -g 4g -q 16m -f ../output/iozone_data -Rb ../output/test_iozone_emmc_ext4_stability_${i}.xls
    if [ "$?" != "0" ];then
		echo "Test fail loop ${i} error!" >> ../output/emmc_stability.log
		exit 1
	else
		echo "Test loop ${i} success!" >> ../output/emmc_stability.log
	fi
done