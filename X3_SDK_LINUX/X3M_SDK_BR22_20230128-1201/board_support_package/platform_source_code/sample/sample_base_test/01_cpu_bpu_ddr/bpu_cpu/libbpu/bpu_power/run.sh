curPath=$(pwd)
export BMEM_CACHEABLE=true
export LD_LIBRARY_PATH=$curPath/bin
# -b 0: core 0; -b 1: core 1; -b 2: core 0 and core 1
# -t run time(second)

#run the theory max power
$curPath/bin/bpu_power -f reality_max_power/tmp_raw_inst_fc_0.hbinst -s reality_max_power/tmp_raw_inst_fc_1.hbinst -i reality_max_power/snapshot_fc_0_inst_0_sram_with_layout.img -d reality_max_power/dump_sram.hbinst -b 2 -t 10

#run the theory max power
#$curPath/bin/bpu_power -f theory_max_power/tmp_raw_inst_fc_0.hbinst -s theory_max_power/tmp_raw_inst_fc_1.hbinst -i theory_max_power/snapshot_fc_0_inst_0_sram_with_layout.img -d theory_max_power/dump_sram.hbinst -b 2 -t 10








