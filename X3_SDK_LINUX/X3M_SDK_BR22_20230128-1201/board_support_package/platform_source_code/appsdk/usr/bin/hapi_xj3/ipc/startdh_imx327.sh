#!/bin/bash
export LD_LIBRARY_PATH=/app/bin/hapi_xj3:$LD_LIBRARY_PATH
echo $1
chmod +x hapi_bpu
echo 120 > /sys/devices/platform/soc/a4001000.sif/hblank
echo 1 > /sys/class/vps/mipi_host1/param/stop_check_instart 
echo 1 > /sys/class/vps/mipi_host0/param/stop_check_instart 
echo 0x10100000 > /sys/bus/platform/drivers/ddr_monitor/axibus_ctrl/all
echo 0x02120000 > /sys/bus/platform/drivers/ddr_monitor/read_qos_ctrl/all
echo 0x03120000 > /sys/bus/platform/drivers/ddr_monitor/write_qos_ctrl/all

echo "start hapi_bpu"
if [ $1 = "i2c5clock" ]; then
        ./hapi_bpu -f detection.hbm -i sample.yuv -o /userdata/ -n detection -a 3 -b 1 -m 1 -G 1 -A 2 -S 1 -X 0  > hapi_bpu.log &
elif [ $1 = "i2c0" ]; then
        ./hapi_bpu -f detection.hbm -i sample.yuv -o /userdata/ -n detection -a 3 -b 1 -m 1 -G 1 -A 2 -S 1 -M 0 -B 0 > hapi_bpu.log &
elif [ $1 = "i2c0clock" ]; then
        ./hapi_bpu -f detection.hbm -i sample.yuv -o /userdata/ -n detection -a 3 -b 1 -m 1 -G 1 -A 2 -S 1 -M 0 -B 0 -X 0 > hapi_bpu.log &
else
        ./hapi_bpu -f detection.hbm -i sample.yuv -o /userdata/ -n detection -a 3 -b 1 -m 1 -G 1 -A 2 -S 1  > hapi_bpu.log &
fi
sleep 10
#nohup /app/scripts/run-portion.sh -b 2 -p 100 > bpu-stress.log &
