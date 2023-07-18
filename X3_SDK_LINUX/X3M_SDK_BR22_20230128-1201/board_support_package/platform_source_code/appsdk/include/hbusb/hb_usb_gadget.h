// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018-2019
 *
 * hbusb transfer
 */
 #ifndef __HBUSB_GADGET_H_
#define __HBUSB_GADGET_H_
#ifdef __cplusplus
extern "C" {
#endif

#define HBUSB_SG_MAX_NUM			16

/* hbusb_init rw parameter */
enum {
	HBUSB_DEV_RW_MODE,
	HBUSB_DEV_RONLY_MODE,
	HBUSB_DEV_WONLY_MODE
};

typedef struct {
        int rw_mode;
} usbdev_info_t;

typedef struct {
        void *virt_addr;
        unsigned long phy_addr;
        int length;
        int actual_length;
} scatterlist_t;

typedef struct {
	scatterlist_t sg[HBUSB_SG_MAX_NUM];
	int num_sgs;
} data_elem_t;

typedef struct {
	void *buf;
	int	size;
	int	actual_size;
} ctrl_elem_t;

typedef struct {
	ctrl_elem_t ctrl_info;
	data_elem_t data_info;	
} elem_t;

typedef struct {
	int sg_max_size[HBUSB_SG_MAX_NUM];;
	int sg_data_size[HBUSB_SG_MAX_NUM];;
	int num_sgs;
} scatterlist_frame_alloc_t;


#if 0
void *hb_usb_init(int rw_mode, scatterlist_frame_alloc_t *tx_scatterlist_info, int tx_alloc_num, 
					scatterlist_frame_alloc_t *rx_scatterlist_info, int rx_alloc_num);
void hb_usb_deinit(void *dev_handle, scatterlist_frame_alloc_t *tx_scatterlist_info, int tx_alloc_num, 
					scatterlist_frame_alloc_t *rx_scatterlist_info, int rx_alloc_num);
#endif

void *hb_usb_init(usbdev_info_t *usbdev_info);
void hb_usb_deinit(void *dev_handle);
int hb_usb_sync_tx_valid_framebuf(void *dev_handle, void *elem);
int hb_usb_sync_rx_valid_framebuf(void *dev_handle, void *elem);

int hb_usb_dev_present(void *dev_handle);

#ifdef __cplusplus
}
#endif
#endif


