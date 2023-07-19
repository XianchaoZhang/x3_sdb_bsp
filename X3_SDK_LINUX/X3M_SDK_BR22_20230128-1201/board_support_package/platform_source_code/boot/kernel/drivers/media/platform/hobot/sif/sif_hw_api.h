/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef __HOBOT_SIF_HW_API__
#define __HOBOT_SIF_HW_API__

#define LINE_BUFFER_SIZE        (2688)

void sif_transfer_ddr_owner(u32 __iomem *base_reg, u32 mux_out_index, u32 buf_index);
u32 sif_get_ddr_addr(u32 __iomem *base_reg, u32 mux_out_index, u32 buf_index);
void sif_set_ddr_output(u32 __iomem *base_reg, sif_output_ddr_t* p_ddr,
		sif_input_splice_t *splice, u32 *enbale);

void sif_set_md_enable(u32 __iomem *base_reg);
void sif_set_md_disable(u32 __iomem *base_reg);

void sif_hw_config(u32 __iomem *base_reg, sif_cfg_t* c);
void sif_hw_mipi_rx_out_select(u32 __iomem *base_reg,
				struct sif_subdev *subdev);
void sif_set_isp_output(u32 __iomem *base_reg,
				sif_output_t *p_out);
void sif_raw_isp_output_config(u32 __iomem *base_reg,
		sif_output_t *p_out);

void sif_hw_disable(u32 __iomem *base_reg);
void sif_hw_enable(u32 __iomem *base_reg);
void sif_hw_post_config(u32 __iomem *base_reg, sif_cfg_t* c);
void sif_disable_ipi(u32 __iomem *base_reg, u8 ipi_channel);

void sif_get_frameid_timestamps(u32 __iomem *base_reg, u32 mux, u32 ipi_index,
	struct frame_id *info, u32 dol_num, u32 instance,
	sif_output_t *output, sif_input_t *inpu, u32 *cnt_shift);
u32 sif_get_current_bufindex(u32 __iomem *base_reg, u32 mux);
bool sif_get_wdma_enable(u32 __iomem *base_reg, u32 mux);
void sif_set_wdma_buf_addr(u32 __iomem *base_reg, u32 mux_index, u32 number, u32 addr);
void sif_config_rdma_fmt(u32 __iomem *base_reg, u32 format, u32 width, u32 height);
void sif_set_rdma_buf_addr(u32 __iomem *base_reg, u32 index, u32 addr);
void sif_set_rdma_buf_stride(u32 __iomem *base_reg, u32 index, u32 stride);

void sif_set_rdma_trigger(u32 __iomem *base_reg, u32 enable);
void sif_set_wdma_enable(u32 __iomem *base_reg, u32 mux_index, bool enable);
void sif_set_rdma_enable(u32 __iomem *base_reg, u32 mux_index, bool enable);
void sif_enable_dma(u32 __iomem *base_reg, u32 cfg);
void sif_hw_enable_bypass(u32 __iomem *base_reg, u32 ch_index, bool enable);

int sif_get_irq_src(u32 __iomem *base_reg, struct sif_irq_src *src, bool clear);
void sif_hw_dump(u32 __iomem *base_reg);
u32 sif_get_frame_intr(void __iomem *base_reg);
void sif_set_isp_performance(u32 __iomem *base_reg, u8 value);
void sif_enable_init_frameid(u32 __iomem *base_reg, u32 index, bool enable);
void sif_print_rx_status(u32 __iomem *base_reg, u32 err_status);
u32 sif_get_buffer_status(u32 __iomem *base_reg);
void sif_set_md_output(u32 __iomem *base_reg, sif_output_md_t *p_md);
void sif_set_pattern_config_framerate(u32 __iomem *base_reg,
				u32 pat_index, sif_data_desc_t* p_data, u32 framerate);
void sif_set_bypass_cfg(u32 __iomem *base_reg, sif_input_bypass_t *cfg);
void sif_enable_frame_intr(void __iomem *base_reg, u32 mux_index,
				bool enable);
void sif_start_pattern_gen(u32 __iomem *base_reg, u32 pat_index);
void sif_hw_disable_ex(u32 __iomem *base_reg);
void sif_disable_isp_out_config(u32 __iomem *base_reg);
void sif_statics_err_clr(u32 __iomem *base_reg);
void sif_transfer_ddr_owner_release(u32 __iomem *base_reg,
	u32 mux_out_index, u32 buf_index);
void sif_set_drop_enable(u32 __iomem *base_reg, bool enable);
u32 sif_get_current_ownerbit(u32 __iomem *base_reg,
		u32 mux, u32 buf_idx);
u32 sif_get_wdma_reg(u32 __iomem *base_reg);
void sif_print_buffer_owner(u32 __iomem *base_reg);

void sif_get_idx_and_owner(u32 __iomem *base_reg, int32_t *idx, int32_t *owner);

void sif_set_rx_ipi_frameid(u32 __iomem *base_reg, u32 index, u32 vc_channel,
	sif_input_mipi_t* p_mipi);

#endif
