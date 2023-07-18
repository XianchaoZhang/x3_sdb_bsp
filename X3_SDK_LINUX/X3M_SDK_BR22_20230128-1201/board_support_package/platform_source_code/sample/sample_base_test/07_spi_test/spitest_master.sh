devmem 0xA6003174 32 0x00000F7F
chmod +x spidev_tc
size=512
while true
do
        ./spidev_tc -D /dev/spidev2.0 -v -s 20000000 -m 2 -e $size -t 1
        echo "spi master"
        sleep 1
done
