#ifndef __DDR_UMCTL2_REGS_STRUCT_H__
#define __DDR_UMCTL2_REGS_STRUCT_H__
struct	UMCTL_REGS	{
      	volatile uint32    	MSTR;             	//0xd00000
      	volatile uint32    	STAT;             	//0xd00004
      	volatile uint32    	RESERVED_0[2];    	//
      	volatile uint32    	MRCTRL0;          	//0xd00010
      	volatile uint32    	MRCTRL1;          	//0xd00014
      	volatile uint32    	MRSTAT;           	//0xd00018
      	volatile uint32    	MRCTRL2;          	//0xd0001c
      	volatile uint32    	DERATEEN;         	//0xd00020
      	volatile uint32    	DERATEINT;        	//0xd00024
      	volatile uint32    	MSTR2;            	//0xd00028
      	volatile uint32    	DERATECTL;        	//0xd0002c
      	volatile uint32    	PWRCTL;           	//0xd00030
      	volatile uint32    	PWRTMG;           	//0xd00034
      	volatile uint32    	HWLPCTL;          	//0xd00038
      	volatile uint32    	RESERVED_1[5];    	//
      	volatile uint32    	RFSHCTL0;         	//0xd00050
      	volatile uint32    	RFSHCTL1;         	//0xd00054
      	volatile uint32    	RESERVED_2[2];    	//
      	volatile uint32    	RFSHCTL3;         	//0xd00060
      	volatile uint32    	RFSHTMG;          	//0xd00064
      	volatile uint32    	RFSHTMG1;         	//0xd00068
      	volatile uint32    	RESERVED_3[1];    	//
      	volatile uint32    	ECCCFG0;          	//0xd00070
      	volatile uint32    	ECCCFG1;          	//0xd00074
      	volatile uint32    	ECCSTAT;          	//0xd00078
      	volatile uint32    	ECCCTL;           	//0xd0007c
      	volatile uint32    	ECCERRCNT;        	//0xd00080
      	volatile uint32    	ECCCADDR0;        	//0xd00084
      	volatile uint32    	ECCCADDR1;        	//0xd00088
      	volatile uint32    	ECCCSYN0;         	//0xd0008c
      	volatile uint32    	ECCCSYN1;         	//0xd00090
      	volatile uint32    	ECCCSYN2;         	//0xd00094
      	volatile uint32    	ECCBITMASK0;      	//0xd00098
      	volatile uint32    	ECCBITMASK1;      	//0xd0009c
      	volatile uint32    	ECCBITMASK2;      	//0xd000a0
      	volatile uint32    	ECCUADDR0;        	//0xd000a4
      	volatile uint32    	ECCUADDR1;        	//0xd000a8
      	volatile uint32    	ECCUSYN0;         	//0xd000ac
      	volatile uint32    	ECCUSYN1;         	//0xd000b0
      	volatile uint32    	ECCUSYN2;         	//0xd000b4
      	volatile uint32    	ECCPOISONADDR0;   	//0xd000b8
      	volatile uint32    	ECCPOISONADDR1;   	//0xd000bc
      	volatile uint32    	CRCPARCTL0;       	//0xd000c0
      	volatile uint32    	CRCPARCTL1;       	//0xd000c4
      	volatile uint32    	RESERVED_4[1];    	//
      	volatile uint32    	CRCPARSTAT;       	//0xd000cc
      	volatile uint32    	INIT0;            	//0xd000d0
      	volatile uint32    	INIT1;            	//0xd000d4
      	volatile uint32    	INIT2;            	//0xd000d8
      	volatile uint32    	INIT3;            	//0xd000dc
      	volatile uint32    	INIT4;            	//0xd000e0
      	volatile uint32    	INIT5;            	//0xd000e4
      	volatile uint32    	INIT6;            	//0xd000e8
      	volatile uint32    	INIT7;            	//0xd000ec
      	volatile uint32    	DIMMCTL;          	//0xd000f0
      	volatile uint32    	RANKCTL;          	//0xd000f4
      	volatile uint32    	RESERVED_5[2];    	//
      	volatile uint32    	DRAMTMG0;         	//0xd00100
      	volatile uint32    	DRAMTMG1;         	//0xd00104
      	volatile uint32    	DRAMTMG2;         	//0xd00108
      	volatile uint32    	DRAMTMG3;         	//0xd0010c
      	volatile uint32    	DRAMTMG4;         	//0xd00110
      	volatile uint32    	DRAMTMG5;         	//0xd00114
      	volatile uint32    	DRAMTMG6;         	//0xd00118
      	volatile uint32    	DRAMTMG7;         	//0xd0011c
      	volatile uint32    	DRAMTMG8;         	//0xd00120
      	volatile uint32    	DRAMTMG9;         	//0xd00124
      	volatile uint32    	DRAMTMG10;        	//0xd00128
      	volatile uint32    	DRAMTMG11;        	//0xd0012c
      	volatile uint32    	DRAMTMG12;        	//0xd00130
      	volatile uint32    	DRAMTMG13;        	//0xd00134
      	volatile uint32    	DRAMTMG14;        	//0xd00138
      	volatile uint32    	DRAMTMG15;        	//0xd0013c
      	volatile uint32    	RESERVED_6[16];   	//
      	volatile uint32    	ZQCTL0;           	//0xd00180
      	volatile uint32    	ZQCTL1;           	//0xd00184
      	volatile uint32    	ZQCTL2;           	//0xd00188
      	volatile uint32    	ZQSTAT;           	//0xd0018c
      	volatile uint32    	DFITMG0;          	//0xd00190
      	volatile uint32    	DFITMG1;          	//0xd00194
      	volatile uint32    	DFILPCFG0;        	//0xd00198
      	volatile uint32    	DFILPCFG1;        	//0xd0019c
      	volatile uint32    	DFIUPD0;          	//0xd001a0
      	volatile uint32    	DFIUPD1;          	//0xd001a4
      	volatile uint32    	DFIUPD2;          	//0xd001a8
      	volatile uint32    	RESERVED_7[1];    	//
      	volatile uint32    	DFIMISC;          	//0xd001b0
      	volatile uint32    	DFITMG2;          	//0xd001b4
      	volatile uint32    	DFITMG3;          	//0xd001b8
      	volatile uint32    	DFISTAT;          	//0xd001bc
      	volatile uint32    	DBICTL;           	//0xd001c0
      	volatile uint32    	DFIPHYMSTR;       	//0xd001c4
      	volatile uint32    	RESERVED_8[14];   	//
      	volatile uint32    	ADDRMAP0;         	//0xd00200
      	volatile uint32    	ADDRMAP1;         	//0xd00204
      	volatile uint32    	ADDRMAP2;         	//0xd00208
      	volatile uint32    	ADDRMAP3;         	//0xd0020c
      	volatile uint32    	ADDRMAP4;         	//0xd00210
      	volatile uint32    	ADDRMAP5;         	//0xd00214
      	volatile uint32    	ADDRMAP6;         	//0xd00218
      	volatile uint32    	ADDRMAP7;         	//0xd0021c
      	volatile uint32    	ADDRMAP8;         	//0xd00220
      	volatile uint32    	ADDRMAP9;         	//0xd00224
      	volatile uint32    	ADDRMAP10;        	//0xd00228
      	volatile uint32    	ADDRMAP11;        	//0xd0022c
      	volatile uint32    	RESERVED_9[4];    	//
      	volatile uint32    	ODTCFG;           	//0xd00240
      	volatile uint32    	ODTMAP;           	//0xd00244
      	volatile uint32    	RESERVED_10[2];   	//
      	volatile uint32    	SCHED;            	//0xd00250
      	volatile uint32    	SCHED1;           	//0xd00254
      	volatile uint32    	RESERVED_11[1];   	//
      	volatile uint32    	PERFHPR1;         	//0xd0025c
      	volatile uint32    	RESERVED_12[1];   	//
      	volatile uint32    	PERFLPR1;         	//0xd00264
      	volatile uint32    	RESERVED_13[1];   	//
      	volatile uint32    	PERFWR1;          	//0xd0026c
      	volatile uint32    	RESERVED_14[36];  	//
      	volatile uint32    	DBG0;             	//0xd00300
      	volatile uint32    	DBG1;             	//0xd00304
      	volatile uint32    	DBGCAM;           	//0xd00308
      	volatile uint32    	DBGCMD;           	//0xd0030c
      	volatile uint32    	DBGSTAT;          	//0xd00310
      	volatile uint32    	RESERVED_15[1];   	//
      	volatile uint32    	DBGCAM1;          	//0xd00318
      	volatile uint32    	RESERVED_16[1];   	//
      	volatile uint32    	SWCTL;            	//0xd00320
      	volatile uint32    	SWSTAT;           	//0xd00324
      	volatile uint32    	SWCTLSTATIC;      	//0xd00328
      	volatile uint32    	RESERVED_17[16];  	//
      	volatile uint32    	POISONCFG;        	//0xd0036c
      	volatile uint32    	POISONSTAT;       	//0xd00370
      	volatile uint32    	ADVECCINDEX;      	//0xd00374
      	volatile uint32    	RESERVED_18[1];   	//
      	volatile uint32    	ECCPOISONPAT0;    	//0xd0037c
      	volatile uint32    	RESERVED_19[1];   	//
      	volatile uint32    	ECCPOISONPAT2;    	//0xd00384
      	volatile uint32    	ECCAPSTAT;        	//0xd00388
      	volatile uint32    	RESERVED_20[25];  	//
      	volatile uint32    	DERATESTAT;       	//0xd003f0
      	volatile uint32    	RESERVED_21[2];   	//
      	volatile uint32    	PSTAT;            	//0xd003fc
      	volatile uint32    	PCCFG;            	//0xd00400
      	volatile uint32    	PCFGR_0;          	//0xd00404
      	volatile uint32    	PCFGW_0;          	//0xd00408
      	volatile uint32    	RESERVED_22[33];  	//
      	volatile uint32    	PCTRL_0;          	//0xd00490
      	volatile uint32    	PCFGQOS0_0;       	//0xd00494
      	volatile uint32    	PCFGQOS1_0;       	//0xd00498
      	volatile uint32    	PCFGWQOS0_0;      	//0xd0049c
      	volatile uint32    	PCFGWQOS1_0;      	//0xd004a0
      	volatile uint32    	RESERVED_23[4];   	//
      	volatile uint32    	PCFGR_1;          	//0xd004b4
      	volatile uint32    	PCFGW_1;          	//0xd004b8
      	volatile uint32    	RESERVED_24[33];  	//
      	volatile uint32    	PCTRL_1;          	//0xd00540
      	volatile uint32    	PCFGQOS0_1;       	//0xd00544
      	volatile uint32    	PCFGQOS1_1;       	//0xd00548
      	volatile uint32    	PCFGWQOS0_1;      	//0xd0054c
      	volatile uint32    	PCFGWQOS1_1;      	//0xd00550
      	volatile uint32    	RESERVED_25[4];   	//
      	volatile uint32    	PCFGR_2;          	//0xd00564
      	volatile uint32    	PCFGW_2;          	//0xd00568
      	volatile uint32    	RESERVED_26[33];  	//
      	volatile uint32    	PCTRL_2;          	//0xd005f0
      	volatile uint32    	PCFGQOS0_2;       	//0xd005f4
      	volatile uint32    	PCFGQOS1_2;       	//0xd005f8
      	volatile uint32    	PCFGWQOS0_2;      	//0xd005fc
      	volatile uint32    	PCFGWQOS1_2;      	//0xd00600
      	volatile uint32    	RESERVED_27[4];   	//
      	volatile uint32    	PCFGR_3;          	//0xd00614
      	volatile uint32    	PCFGW_3;          	//0xd00618
      	volatile uint32    	RESERVED_28[33];  	//
      	volatile uint32    	PCTRL_3;          	//0xd006a0
      	volatile uint32    	PCFGQOS0_3;       	//0xd006a4
      	volatile uint32    	PCFGQOS1_3;       	//0xd006a8
      	volatile uint32    	PCFGWQOS0_3;      	//0xd006ac
      	volatile uint32    	PCFGWQOS1_3;      	//0xd006b0
      	volatile uint32    	RESERVED_29[4];   	//
      	volatile uint32    	PCFGR_4;          	//0xd006c4
      	volatile uint32    	PCFGW_4;          	//0xd006c8
      	volatile uint32    	RESERVED_30[33];  	//
      	volatile uint32    	PCTRL_4;          	//0xd00750
      	volatile uint32    	PCFGQOS0_4;       	//0xd00754
      	volatile uint32    	PCFGQOS1_4;       	//0xd00758
      	volatile uint32    	PCFGWQOS0_4;      	//0xd0075c
      	volatile uint32    	PCFGWQOS1_4;      	//0xd00760
      	volatile uint32    	RESERVED_31[4];   	//
      	volatile uint32    	PCFGR_5;          	//0xd00774
      	volatile uint32    	PCFGW_5;          	//0xd00778
      	volatile uint32    	RESERVED_32[33];  	//
      	volatile uint32    	PCTRL_5;          	//0xd00800
      	volatile uint32    	PCFGQOS0_5;       	//0xd00804
      	volatile uint32    	PCFGQOS1_5;       	//0xd00808
      	volatile uint32    	PCFGWQOS0_5;      	//0xd0080c
      	volatile uint32    	PCFGWQOS1_5;      	//0xd00810
      	volatile uint32    	RESERVED_33[4];   	//
      	volatile uint32    	PCFGR_6;          	//0xd00824
      	volatile uint32    	PCFGW_6;          	//0xd00828
      	volatile uint32    	RESERVED_34[33];  	//
      	volatile uint32    	PCTRL_6;          	//0xd008b0
      	volatile uint32    	PCFGQOS0_6;       	//0xd008b4
      	volatile uint32    	PCFGQOS1_6;       	//0xd008b8
      	volatile uint32    	PCFGWQOS0_6;      	//0xd008bc
      	volatile uint32    	PCFGWQOS1_6;      	//0xd008c0
      	volatile uint32    	RESERVED_35[4];   	//
      	volatile uint32    	PCFGR_7;          	//0xd008d4
      	volatile uint32    	PCFGW_7;          	//0xd008d8
      	volatile uint32    	RESERVED_36[33];  	//
      	volatile uint32    	PCTRL_7;          	//0xd00960
      	volatile uint32    	PCFGQOS0_7;       	//0xd00964
      	volatile uint32    	PCFGQOS1_7;       	//0xd00968
      	volatile uint32    	PCFGWQOS0_7;      	//0xd0096c
      	volatile uint32    	PCFGWQOS1_7;      	//0xd00970
      	volatile uint32    	RESERVED_37[364]; 	//
      	volatile uint32    	SBRCTL;           	//0xd00f24
      	volatile uint32    	SBRSTAT;          	//0xd00f28
      	volatile uint32    	SBRWDATA0;        	//0xd00f2c
      	volatile uint32    	RESERVED_38[2];   	//
      	volatile uint32    	SBRSTART0;        	//0xd00f38
      	volatile uint32    	SBRSTART1;        	//0xd00f3c
      	volatile uint32    	SBRRANGE0;        	//0xd00f40
      	volatile uint32    	SBRRANGE1;        	//0xd00f44
      	volatile uint32    	RESERVED_39[42];  	//
      	volatile uint32    	UMCTL2_VER_NUMBER;	//0xd00ff0
      	volatile uint32    	UMCTL2_VER_TYPE;  	//0xd00ff4
      	volatile uint32    	RESERVED_40[1034];	//
      	volatile uint32    	FREQ1_DERATEEN;   	//0xd02020
      	volatile uint32    	FREQ1_DERATEINT;  	//0xd02024
      	volatile uint32    	RESERVED_41[3];   	//
      	volatile uint32    	FREQ1_PWRTMG;     	//0xd02034
      	volatile uint32    	RESERVED_42[6];   	//
      	volatile uint32    	FREQ1_RFSHCTL0;   	//0xd02050
      	volatile uint32    	RESERVED_43[4];   	//
      	volatile uint32    	FREQ1_RFSHTMG;    	//0xd02064
      	volatile uint32    	FREQ1_RFSHTMG1;   	//0xd02068
      	volatile uint32    	RESERVED_44[28];  	//
      	volatile uint32    	FREQ1_INIT3;      	//0xd020dc
      	volatile uint32    	FREQ1_INIT4;      	//0xd020e0
      	volatile uint32    	RESERVED_45[1];   	//
      	volatile uint32    	FREQ1_INIT6;      	//0xd020e8
      	volatile uint32    	FREQ1_INIT7;      	//0xd020ec
      	volatile uint32    	RESERVED_46[1];   	//
      	volatile uint32    	FREQ1_RANKCTL;    	//0xd020f4
      	volatile uint32    	RESERVED_47[2];   	//
      	volatile uint32    	FREQ1_DRAMTMG0;   	//0xd02100
      	volatile uint32    	FREQ1_DRAMTMG1;   	//0xd02104
      	volatile uint32    	FREQ1_DRAMTMG2;   	//0xd02108
      	volatile uint32    	FREQ1_DRAMTMG3;   	//0xd0210c
      	volatile uint32    	FREQ1_DRAMTMG4;   	//0xd02110
      	volatile uint32    	FREQ1_DRAMTMG5;   	//0xd02114
      	volatile uint32    	FREQ1_DRAMTMG6;   	//0xd02118
      	volatile uint32    	FREQ1_DRAMTMG7;   	//0xd0211c
      	volatile uint32    	FREQ1_DRAMTMG8;   	//0xd02120
      	volatile uint32    	FREQ1_DRAMTMG9;   	//0xd02124
      	volatile uint32    	FREQ1_DRAMTMG10;  	//0xd02128
      	volatile uint32    	FREQ1_DRAMTMG11;  	//0xd0212c
      	volatile uint32    	FREQ1_DRAMTMG12;  	//0xd02130
      	volatile uint32    	FREQ1_DRAMTMG13;  	//0xd02134
      	volatile uint32    	FREQ1_DRAMTMG14;  	//0xd02138
      	volatile uint32    	FREQ1_DRAMTMG15;  	//0xd0213c
      	volatile uint32    	RESERVED_48[16];  	//
      	volatile uint32    	FREQ1_ZQCTL0;     	//0xd02180
      	volatile uint32    	RESERVED_49[3];   	//
      	volatile uint32    	FREQ1_DFITMG0;    	//0xd02190
      	volatile uint32    	FREQ1_DFITMG1;    	//0xd02194
      	volatile uint32    	RESERVED_50[7];   	//
      	volatile uint32    	FREQ1_DFITMG2;    	//0xd021b4
      	volatile uint32    	FREQ1_DFITMG3;    	//0xd021b8
      	volatile uint32    	RESERVED_51[33];  	//
      	volatile uint32    	FREQ1_ODTCFG;     	//0xd02240
      	volatile uint32    	RESERVED_52[887]; 	//
      	volatile uint32    	FREQ2_DERATEEN;   	//0xd03020
      	volatile uint32    	FREQ2_DERATEINT;  	//0xd03024
      	volatile uint32    	RESERVED_53[3];   	//
      	volatile uint32    	FREQ2_PWRTMG;     	//0xd03034
      	volatile uint32    	RESERVED_54[6];   	//
      	volatile uint32    	FREQ2_RFSHCTL0;   	//0xd03050
      	volatile uint32    	RESERVED_55[4];   	//
      	volatile uint32    	FREQ2_RFSHTMG;    	//0xd03064
      	volatile uint32    	FREQ2_RFSHTMG1;   	//0xd03068
      	volatile uint32    	RESERVED_56[28];  	//
      	volatile uint32    	FREQ2_INIT3;      	//0xd030dc
      	volatile uint32    	FREQ2_INIT4;      	//0xd030e0
      	volatile uint32    	RESERVED_57[1];   	//
      	volatile uint32    	FREQ2_INIT6;      	//0xd030e8
      	volatile uint32    	FREQ2_INIT7;      	//0xd030ec
      	volatile uint32    	RESERVED_58[1];   	//
      	volatile uint32    	FREQ2_RANKCTL;    	//0xd030f4
      	volatile uint32    	RESERVED_59[2];   	//
      	volatile uint32    	FREQ2_DRAMTMG0;   	//0xd03100
      	volatile uint32    	FREQ2_DRAMTMG1;   	//0xd03104
      	volatile uint32    	FREQ2_DRAMTMG2;   	//0xd03108
      	volatile uint32    	FREQ2_DRAMTMG3;   	//0xd0310c
      	volatile uint32    	FREQ2_DRAMTMG4;   	//0xd03110
      	volatile uint32    	FREQ2_DRAMTMG5;   	//0xd03114
      	volatile uint32    	FREQ2_DRAMTMG6;   	//0xd03118
      	volatile uint32    	FREQ2_DRAMTMG7;   	//0xd0311c
      	volatile uint32    	FREQ2_DRAMTMG8;   	//0xd03120
      	volatile uint32    	FREQ2_DRAMTMG9;   	//0xd03124
      	volatile uint32    	FREQ2_DRAMTMG10;  	//0xd03128
      	volatile uint32    	FREQ2_DRAMTMG11;  	//0xd0312c
      	volatile uint32    	FREQ2_DRAMTMG12;  	//0xd03130
      	volatile uint32    	FREQ2_DRAMTMG13;  	//0xd03134
      	volatile uint32    	FREQ2_DRAMTMG14;  	//0xd03138
      	volatile uint32    	FREQ2_DRAMTMG15;  	//0xd0313c
      	volatile uint32    	RESERVED_60[16];  	//
      	volatile uint32    	FREQ2_ZQCTL0;     	//0xd03180
      	volatile uint32    	RESERVED_61[3];   	//
      	volatile uint32    	FREQ2_DFITMG0;    	//0xd03190
      	volatile uint32    	FREQ2_DFITMG1;    	//0xd03194
      	volatile uint32    	RESERVED_62[7];   	//
      	volatile uint32    	FREQ2_DFITMG2;    	//0xd031b4
      	volatile uint32    	FREQ2_DFITMG3;    	//0xd031b8
      	volatile uint32    	RESERVED_63[33];  	//
      	volatile uint32    	FREQ2_ODTCFG;     	//0xd03240
      	volatile uint32    	RESERVED_64[887]; 	//
      	volatile uint32    	FREQ3_DERATEEN;   	//0xd04020
      	volatile uint32    	FREQ3_DERATEINT;  	//0xd04024
      	volatile uint32    	RESERVED_65[3];   	//
      	volatile uint32    	FREQ3_PWRTMG;     	//0xd04034
      	volatile uint32    	RESERVED_66[6];   	//
      	volatile uint32    	FREQ3_RFSHCTL0;   	//0xd04050
      	volatile uint32    	RESERVED_67[4];   	//
      	volatile uint32    	FREQ3_RFSHTMG;    	//0xd04064
      	volatile uint32    	FREQ3_RFSHTMG1;   	//0xd04068
      	volatile uint32    	RESERVED_68[28];  	//
      	volatile uint32    	FREQ3_INIT3;      	//0xd040dc
      	volatile uint32    	FREQ3_INIT4;      	//0xd040e0
      	volatile uint32    	RESERVED_69[1];   	//
      	volatile uint32    	FREQ3_INIT6;      	//0xd040e8
      	volatile uint32    	FREQ3_INIT7;      	//0xd040ec
      	volatile uint32    	RESERVED_70[1];   	//
      	volatile uint32    	FREQ3_RANKCTL;    	//0xd040f4
      	volatile uint32    	RESERVED_71[2];   	//
      	volatile uint32    	FREQ3_DRAMTMG0;   	//0xd04100
      	volatile uint32    	FREQ3_DRAMTMG1;   	//0xd04104
      	volatile uint32    	FREQ3_DRAMTMG2;   	//0xd04108
      	volatile uint32    	FREQ3_DRAMTMG3;   	//0xd0410c
      	volatile uint32    	FREQ3_DRAMTMG4;   	//0xd04110
      	volatile uint32    	FREQ3_DRAMTMG5;   	//0xd04114
      	volatile uint32    	FREQ3_DRAMTMG6;   	//0xd04118
      	volatile uint32    	FREQ3_DRAMTMG7;   	//0xd0411c
      	volatile uint32    	FREQ3_DRAMTMG8;   	//0xd04120
      	volatile uint32    	FREQ3_DRAMTMG9;   	//0xd04124
      	volatile uint32    	FREQ3_DRAMTMG10;  	//0xd04128
      	volatile uint32    	FREQ3_DRAMTMG11;  	//0xd0412c
      	volatile uint32    	FREQ3_DRAMTMG12;  	//0xd04130
      	volatile uint32    	FREQ3_DRAMTMG13;  	//0xd04134
      	volatile uint32    	FREQ3_DRAMTMG14;  	//0xd04138
      	volatile uint32    	FREQ3_DRAMTMG15;  	//0xd0413c
      	volatile uint32    	RESERVED_72[16];  	//
      	volatile uint32    	FREQ3_ZQCTL0;     	//0xd04180
      	volatile uint32    	RESERVED_73[3];   	//
      	volatile uint32    	FREQ3_DFITMG0;    	//0xd04190
      	volatile uint32    	FREQ3_DFITMG1;    	//0xd04194
      	volatile uint32    	RESERVED_74[7];   	//
      	volatile uint32    	FREQ3_DFITMG2;    	//0xd041b4
      	volatile uint32    	FREQ3_DFITMG3;    	//0xd041b8
      	volatile uint32    	RESERVED_75[33];  	//
      	volatile uint32    	FREQ3_ODTCFG;     	//0xd04240
};
#endif
