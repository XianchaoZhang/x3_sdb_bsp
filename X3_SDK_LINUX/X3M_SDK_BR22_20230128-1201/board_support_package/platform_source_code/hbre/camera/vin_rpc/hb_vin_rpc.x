/* RPCGEN code decribing the hb_vin client/server API. */

struct hb_vin_bypass_arg {
	int	port;
	int enable;
};

struct hb_vin_cfg_arg {
	string filename<>;
	int fps;
	int resolution;
	int entry_num;
	string filecont<>;
};

struct hb_vin_snrclk_arg {
	int entry;
	int value;
};

struct hb_vin_pre_arg {
	int entry;
	int type;
	int value;
};

program VINRPC
{
	version VINRPC_V1
	{
		int HB_VIN_INIT(int) = 1;
		int HB_VIN_START(int) = 2;
		int HB_VIN_STOP(int) = 3;
		int HB_VIN_DEINIT(int) = 4;
		int HB_VIN_RESET(int) = 5;
		int HB_VIN_SET_BYPASS(hb_vin_bypass_arg) = 6;
		int HB_CAM_MIPI_PARSE_CFG(hb_vin_cfg_arg) = 7;
		int HB_VIN_SNRCLK_SET_EN(hb_vin_snrclk_arg) = 8;
		int HB_VIN_SNRCLK_SET_FREQ(hb_vin_snrclk_arg) = 9;
		int HB_VIN_PRE_REQUEST(hb_vin_pre_arg) = 10;
		int HB_VIN_PRE_RESULT(hb_vin_pre_arg) = 11;
	} = 1;
} = 0x2076696e;
