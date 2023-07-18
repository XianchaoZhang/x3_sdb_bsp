# 示例模型介绍

## yolov5_672x672_nv12.bin
公版模型（coco数据集，80个检测分类输出）

性能：
BPU运算，双核单帧 50ms左右
后处理20-30ms

## fcos_512x512_nv12.bin
公版模式（coco数据集，80个检测分类输出）
在BPU上有很优秀的性能表现

## mobilenetv2_224x224_nv12.bin
公版模型（imagenet，1000类分类）
推荐使用

## personMultitask.hbm
地平线自研人体、人脸、人头、人体骨骼点多任务模型