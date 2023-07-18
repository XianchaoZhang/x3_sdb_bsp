#!/bin/sh

echo "SPI NandFlash test start!"

mkdir -p ../output

loop_num=2000

for i in $(seq 1 $loop_num)
do
	echo "loop_test: ${i}"

	../bin/iozone -e -+r -o -a -q 1M -s 32M -f ../output/iozone_data -Rb ../output/test_iozone_spi_flash_${i}.xls

	if [ "$?" != "0" ];then
		echo "Test fail loop ${i} error!" >> ../output/spinand.log
		exit 1
	else
		echo "Test loop ${i} success!" >> ../output/spinand.log
	fi
done
