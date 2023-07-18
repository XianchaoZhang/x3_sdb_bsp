curPath=$(pwd)
export BMEM_CACHEABLE=true
export LD_LIBRARY_PATH=$curPath/bin
OUTPUT_0_0=/tmp/model-i0012_00/
OUTPUT_0_1=/tmp/model-i0012_01/
OUTPUT_1_0=/tmp/model-i0012_10/
OUTPUT_1_1=/tmp/model-i0012_11/
HBM_FILE=$curPath/libbpu/HBDK3_MODEL_I0012/gen_I0012_MobileNet0.5_X2A_1x224x224x3.hbm
SRC_FILE=$curPath/libbpu/HBDK3_MODEL_I0012/input_0_feature_1x224x224x3_ddr_native.bin
MODEL_NAME=I0012_MobileNet0.5_X2A_1x224x224x3
if [ ! -d $OUTPUT_0_0 ];then
        mkdir $OUTPUT_0_0
else
        rm $OUTPUT_0_0 -r
        mkdir $OUTPUT_0_0
fi
if [ ! -d $OUTPUT_0_1 ];then
        mkdir $OUTPUT_0_1
else
        rm $OUTPUT_0_1 -r
        mkdir $OUTPUT_0_1
fi
if [ ! -d $OUTPUT_1_0 ];then
        mkdir $OUTPUT_1_0
else
        rm $OUTPUT_1_0 -r
        mkdir $OUTPUT_1_0
fi
if [ ! -d $OUTPUT_1_1 ];then
        mkdir $OUTPUT_1_1
else
        rm $OUTPUT_1_1 -r
        mkdir $OUTPUT_1_1
fi

if [ $# -eq 0 ]; then
	$curPath/bin/tc_hbdk3 -f $HBM_FILE -i $SRC_FILE -n $MODEL_NAME -o $OUTPUT_0_0,$OUTPUT_0_1,$OUTPUT_1_0,$OUTPUT_1_1 -g 1 -c 2
else
	$curPath/bin/tc_hbdk3 -t $1 -b $2 -f $HBM_FILE -i $SRC_FILE -n $MODEL_NAME -o $OUTPUT_0_0,$OUTPUT_0_1,$OUTPUT_1_0,$OUTPUT_1_1 -g 0 -c 0
fi
