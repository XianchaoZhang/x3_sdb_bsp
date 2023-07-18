
//===--- Sample code for beginners to get familiar with hbdk runtime ---*- Cpp -*-===//
//
//                     The HBDK Compiler Sample code
//
// This file is subject to the terms and conditions defined in file
// 'LICENSE.txt', which is part of this source code package.
//
//===-----------------------------------------------------------------------------===//

#include "hbdk_hbrt.h"
#include "plat_cnn.h"

#include <algorithm>
#include <fstream>

#include <getopt.h>

#include <iomanip>
#include <iostream>
#include <string>

#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>

#include <vector>
#include "hb_vio_interface.h"

#include "hb_media_error.h"
#include "hb_media_recorder.h"
#include "hb_mipi_api.h"
#include "sample.h"

#define CORE_MASK_TO_FLAT(mask) ((mask) << 16U)
#define VIO_PIPE_LINE_ID 0
#define LOOP_TIMES 100
#define VIO_ONLINE
#define CAM_DEBUG_ENABLE
#define DEBUG
#define DRAW_BOX
#define VIDEO_RECORDER

#ifdef VIDEO_RECORDER
#define TAG "[VideoRecorder]"
#define MAX_FILE_PATH 256
#define MAX_ENCODER_INSTANCE 4
#endif

#ifndef DEFAULT_CORE_MASK
#define DEFAULT_CORE_MASK (0x3)
#endif

using std::cerr;
using std::cout;
using std::endl;
using std::ifstream;
using std::ofstream;
using std::string;
using std::vector;

typedef struct cmdOptions {
  const char *output_path = nullptr;
  const char *hbm = nullptr;
  vector<string> inputs;
  const char *model_name = nullptr;
  uint32_t core_mask = 0;
  uint32_t yuv_img_size[2] = {0};
  const char *vio_config = nullptr;
  uint32_t disp_config = 0;
  vector<uint32_t> pyramid_down_scale_factor;
  uint32_t yuv_roi_coord[2] = {0};
  uint32_t yuv_roi_size[2] = {0};
#ifdef VIDEO_RECORDER
  uint32_t codec_id; // 0: h265, 1: h264
  uint32_t encoder_rotate; // 0, 90, 180, 270
  int32_t recorder_duration; // unit is s
#endif
  cmdOptions() {
    output_path = nullptr;
    hbm = nullptr;
    inputs.clear();
    model_name = nullptr;
    core_mask = 0;
    yuv_img_size[0] = 0;
    yuv_img_size[1] = 0;
    yuv_roi_coord[0] = 0;
    yuv_roi_coord[1] = 0;
    yuv_roi_size[0] = 0;
    yuv_roi_size[1] = 0;
    vio_config = nullptr;
    disp_config = 0;
    pyramid_down_scale_factor.clear();
#ifdef VIDEO_RECORDER
    codec_id = 0;
    encoder_rotate = 0;
    recorder_duration = -1;
#endif
  }
} cmdOptions;
extern "C" {
extern int HB_VPS_TriggerSnapFrame(int VpsGrp, int VpsChn, uint32_t frameCnt);
}
static void parseCmdOptions(cmdOptions &cmd_options, int argc, char **argv);
static void draw_dot(char *frame, int x, int y, int color);
static void draw_hline(char *frame, int x0, int x1, int y, int color);
static void draw_vline(char *frame, int x, int y0, int y1, int color);
static void draw_rect(char *frame, int x0, int y0, int x1, int y1, int color, int fill);
#undef CHECK_HBRT_ERROR
#undef RETURN_HBRT_ERROR
#define CHECK_HBRT_ERROR(return_val)                                          \
  do {                                                                        \
    hbrt_error_t hbrt_inside_macro_status = (return_val);                     \
    if (hbrt_inside_macro_status != hbrtSuccess) {                            \
	  close(fbfd);															  \
	  munmap(fbp, screen_size);												  \
      hbdk_error_print_to_stderr(hbrtGetErrorName(hbrt_inside_macro_status)); \
      hbdk_error_print_to_stderr(__FILE__);                                   \
      hbdk_int_print_to_stderr(__LINE__);                                     \
      abort();                                                                \
    }                                                                         \
  } while (0)

#define RETURN_HBRT_ERROR(return_val)                                         \
  do {                                                                        \
    hbrt_error_t hbrt_inside_macro_status = (return_val);                     \
    if (hbrt_inside_macro_status) {                                           \
	  close(fbfd);															  \
	  munmap(fbp, screen_size);												  \
      hbdk_error_print_to_stderr(hbrtGetErrorName(hbrt_inside_macro_status)); \
      hbdk_error_print_to_stderr(__FILE__);                                   \
      hbdk_int_print_to_stderr(__LINE__);                                     \
      return hbrt_inside_macro_status;                                        \
    }                                                                         \
  } while (0)

static uint32_t panel_type = 3;
static int iar_draw_width = 800;
static int iar_draw_height = 480;
static int iar_draw_pbyte = 4;
static uint64_t loop = 100000000;
static uint32_t need_bpu = 1;
cmdOptions cmd_options;
void printmode();
void intHandler(int dummy)
{
	g_exit = 1;
	printf("rcv int signal\n");
}

int bpuCalc()
{
	int fbfd = 0;
	char *fbp = nullptr;
	int ret = 0;
	void *out_person_data = nullptr;
	void *out_vehicle_data = nullptr;
	uint32_t output_num;
	const hbrt_feature_handle_t *output_handle;
	uint16_t output_person_box_num;
	uint16_t output_vehicle_box_num;
	float left_point;
	float top_point;
	float right_point;
	float bottom_point;
	uint16_t draw_left;
	uint16_t draw_top;
	uint16_t draw_right;
	uint16_t draw_bottom;
	float vehicle_left_point;
	float vehicle_top_point;
	float vehicle_right_point;
	float vehicle_bottom_point;
	uint16_t vehicle_draw_left;
	uint16_t vehicle_draw_top;
	uint16_t vehicle_draw_right;
	uint16_t vehicle_draw_bottom;
	uint32_t screen_size = 0;

	struct timeval start_time = {0};
	struct timeval pre_run_time = {0};
	struct timeval final_interrupt_time = {0};
	struct timeval load_hbm_time = {0};
	gettimeofday(&start_time, nullptr);

	/* Open bpu cores first */
	if (need_bpu) {
		cnn_core_open(0);
		cnn_core_open(1);
	
	#ifdef DRAW_BOX
		fbfd = open("/dev/fb0", O_RDWR);
		if (!fbfd) {
			cerr << "Error: cannot open framebuffer device." << endl;
			exit(1);
		}
	    screen_size = 1920*1080*4;
		panel_type = 3;
		
		fbp = (char *)mmap(0, screen_size, PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, 0);
		if (fbp == nullptr) {
			cerr << "Error: failed to map framebuffer device to memory." << endl;
			exit(2);
		}
		cerr << "The framebuffer device was mapped to memory successfully." << endl;
	#endif
	}
	/* Capture the HBRT_LOG_LEVEL environment variable for logging if need */
	if (need_bpu) 
		CHECK_HBRT_ERROR(hbrtSetLogLevel());

	/* Set hbrt global configuration */
	hbrt_global_config_t global_config = HBRT_GLOBAL_CONFIG_INITIALIZER;
	global_config.enable_hbrt_memory_pool = false; /* Memory pool is not necessary for this sample code */
	if (need_bpu) {
		CHECK_HBRT_ERROR(hbrtSetGlobalConfig(&global_config));
	}
	/* Since the memory is visible for all cores in bernoulli architecture,
	* simply use memory flags for both core0 and core1
	* */
	uint32_t bpu_mem_alloc_flags = BPU_CORE0 | BPU_CORE1;

	/* Load HBM containing models(s) */
	hbrt_hbm_handle_t hbm_handle;
	uint32_t model_num;
	uint32_t i = 0;
	uint32_t input_feature_num;
	uint32_t output_total_size;
	const hbrt_feature_handle_t *input_feature_handles;
	if (need_bpu) {
		printf("cmd_options.hbm: %s\n", cmd_options.hbm);
		CHECK_HBRT_ERROR(hbrtLoadHBMFromFile(&hbm_handle, cmd_options.hbm));
		gettimeofday(&load_hbm_time, nullptr);
		cerr << "[TIME] Load HBM consume: "
			<< static_cast<double>(load_hbm_time.tv_sec - start_time.tv_sec) * 1000 +
					static_cast<double>(load_hbm_time.tv_usec - start_time.tv_usec) / 1000
			<< "ms" << endl;
	}
	/* Get the model number and names in this HBM */
	const char **model_names;
	if (need_bpu) {
		CHECK_HBRT_ERROR(hbrtGetModelNamesInHBM(&model_names, hbm_handle));
		printf("model_name: %s\n", (char*)model_names);
		CHECK_HBRT_ERROR(hbrtGetModelNumberInHBM(&model_num, hbm_handle));
		/* Find out the model to execute */
		for (i = 0; i < model_num; i++) {
			if (!strcmp(model_names[i], cmd_options.model_name)) {
				break;
			}
		}
		if (i == model_num) {
			cerr << "models in your hbm:" << endl;
			for (uint32_t j = 0; j < model_num; j++) {
			cout << model_names[j] << endl;
			}
			cerr << "hbm [" << cmd_options.hbm << "] does not contain model [" << cmd_options.model_name << "]!" << endl;
			exit(-1);
		}
	}

	/* Get a handle of the model to execute */
	hbrt_model_handle_t model_handle;
	if (need_bpu) {
		CHECK_HBRT_ERROR(hbrtGetModelHandle(&model_handle, hbm_handle, cmd_options.model_name));
		CHECK_HBRT_ERROR(hbrtGetInputFeatureNumber(&input_feature_num, model_handle));
		CHECK_HBRT_ERROR(hbrtGetInputFeatureHandles(&input_feature_handles, model_handle));
		CHECK_HBRT_ERROR(hbrtGetOutputFeatureTotalSize(&output_total_size, model_handle));
	}

	if (need_bpu) {
		/* prepare output/intermediate/heap regions */
		bpu_addr_t output_region = bpu_mem_alloc(output_total_size, bpu_mem_alloc_flags);
		cerr << "output bpu address: " << reinterpret_cast<void *>(output_region) << endl;
		bpu_addr_t output_buffer = bpu_cpumem_alloc(output_total_size, BPU_NON_CACHEABLE);
		memset(reinterpret_cast<void *>(output_buffer), 0, output_total_size);
		bpu_memcpy(output_region, output_buffer, output_total_size, ARM_TO_CNN);
		bpu_cpumem_free(output_buffer);

		cerr << endl << "========= Model Input Info =========" << endl;
		cerr << "Model name: [" << cmd_options.model_name << "]" << endl;
		cerr << "input number: " << input_feature_num << endl;
		cerr << "output region size " << output_total_size << endl;
		int32_t pym_input_index = 0;
		pym_buffer_t out_pym_buf;

		while (loop--) {
			if (g_exit == 1) break;

			bpu_addr_t p3 = 0;
			vector<hbrt_ri_input_info_t> input_infos;
			for (i = 0; i < input_feature_num; i++) {
				const char *input_name;
				hbrt_dimension_t aligned_dim;
				hbrt_dimension_t valid_dim;
				hbrt_element_type_t element_type;
				bool is_big_endian;
				hbrt_input_source_t input_source;
				const char *input_source_name;
				const char *element_type_name;
				const char *layout_name;
				hbrt_layout_type_t layout;
				CHECK_HBRT_ERROR(hbrtGetFeatureName(&input_name, input_feature_handles[i]));
				CHECK_HBRT_ERROR(hbrtGetFeatureAlignedDimension(&aligned_dim, input_feature_handles[i]));
				CHECK_HBRT_ERROR(hbrtGetFeatureValidDimension(&valid_dim, input_feature_handles[i]));
				CHECK_HBRT_ERROR(hbrtGetFeatureElementType(&element_type, input_feature_handles[i]));
				CHECK_HBRT_ERROR(hbrtFeatureIsBigEndian(&is_big_endian, input_feature_handles[i]));
				CHECK_HBRT_ERROR(hbrtGetInputFeatureSource(&input_source, input_feature_handles[i]));
				CHECK_HBRT_ERROR(hbrtGetInputSourceName(&input_source_name, input_source));
				CHECK_HBRT_ERROR(hbrtGetElementTypeName(&element_type_name, element_type));
				CHECK_HBRT_ERROR(hbrtGetFeatureLayoutType(&layout, input_feature_handles[i]));
				CHECK_HBRT_ERROR(hbrtGetLayoutName(&layout_name, layout));
				uint32_t valid_byte_size;
				uint32_t aligned_byte_size;
				CHECK_HBRT_ERROR(hbrtGetFeatureValidTotalByteSize(&valid_byte_size, input_feature_handles[i]));
				CHECK_HBRT_ERROR(hbrtGetFeatureAlignedTotalByteSize(&aligned_byte_size, input_feature_handles[i]));
				uint32_t pym_stride;
				CHECK_HBRT_ERROR(hbrtGetInputPyramidStride(&pym_stride, input_feature_handles[i]));
				if (input_source != INPUT_FROM_PYRAMID) {
					printf("input_source: %d\n", input_source);
				}
				if (input_source == INPUT_FROM_PYRAMID) {
					if (pym_stride != cmd_options.yuv_img_size[1]) {
						if (cmd_options.yuv_img_size[1] == 0) {
							cerr << "warning: yuv_img_size is not provided. Use input shape [" << aligned_dim.h << "x" << aligned_dim.w << "] by default." << pym_stride << endl;
							cmd_options.yuv_img_size[0] = aligned_dim.h;
							cmd_options.yuv_img_size[1] = aligned_dim.w;
						} else {
							cerr << "warning: provided yuv stride is not same as the stride in hbm (" << pym_stride << "). If you are using pyramid API. This may be ignored." << endl;
						}
					}
				}
				input_infos.emplace_back();
				input_infos[i].feature_handle = input_feature_handles[i];
				switch (input_source) {
				case INPUT_FROM_DDR: {
					// 1: load file
					ifstream ifs(cmd_options.inputs[i], std::ifstream::binary);
					ifs.seekg(0, ifstream::end);
					uint32_t file_size = ifs.tellg();
					ifs.seekg(0, ifstream::beg);
					if (file_size != valid_byte_size) {
						cerr << "This input file [" << cmd_options.inputs[i] << "] (" << i << ") size [ " << file_size
							<< "] is different from what is required by model [" << cmd_options.model_name << "] (" <<
							valid_byte_size << ")" << endl;
						exit(-1);
					}
					vector<char> p1(valid_byte_size);
					ifs.read(p1.data(), valid_byte_size);
					ifs.close();

					// 2: add padding
					vector<char> p2(aligned_byte_size);
					CHECK_HBRT_ERROR(hbrtAddPadding(p2.data(), aligned_dim, p1.data(), valid_dim, element_type));

					// 3a: convert layout
					p3 = bpu_cpumem_alloc(aligned_byte_size, BPU_NON_CACHEABLE);
					RETURN_HBRT_ERROR(hbrtConvertLayout((void *)p3, layout, p2.data(), LAYOUT_NHWC_NATIVE, element_type,
														aligned_dim, is_big_endian));

					// 4: DMA to bpu memory
					bpu_addr_t p4 = bpu_mem_alloc(aligned_byte_size, bpu_mem_alloc_flags);
					bpu_memcpy(p4, p3, aligned_byte_size, ARM_TO_CNN);
					bpu_cpumem_free(p3);

					// prepare input info
					input_infos[i].input_source = INPUT_FROM_DDR;
					input_infos[i].feature_ptr = p4;
					break;
				}
				case INPUT_FROM_PYRAMID:
				case INPUT_FROM_RESIZER: {
					// 1. load file
					ifstream ifs(cmd_options.inputs[i], std::ifstream::binary);
					ifs.seekg(0, ifstream::end);
					uint32_t file_size = ifs.tellg();
					ifs.seekg(0, ifstream::beg);
					vector<char> p1(file_size);
					ifs.read(p1.data(), file_size);
					ifs.close();
					void *src_y_data = nullptr;
					void *src_uv_data = nullptr;
					uint32_t src_w = 0;
					uint32_t src_h = 0;

					if (g_bpu_usesample == 0) {
						ret = HB_VPS_GetChnFrame(0, 6, &out_pym_buf, 2000);
						if (ret != 0) {
							printf("HB_VPS_GetChnFrame error!!!\n");
							continue;
						}
						src_y_data = out_pym_buf.pym_roi[0][0].addr[0];
						src_uv_data = out_pym_buf.pym_roi[0][0].addr[1];
						src_h = 704;
						src_w = out_pym_buf.pym_roi[0][0].width;
					} else {
						if (input_source == INPUT_FROM_PYRAMID){
							if (file_size != valid_byte_size / 2) {
								if (file_size < valid_byte_size / 2) {
									cerr << "This input file [" << cmd_options.inputs[i]
									<< "] size [" << file_size
									<< "] is less than what is required by model ["
									<< cmd_options.model_name << "]: " << valid_byte_size / 2
									<< endl;
									exit(-1);
								} else{
									cerr << "warning: input file size is bigger than model input. "
										"Please trim your input if encounter any issue." << endl;
								}
							}
							src_y_data = p1.data();
							src_w = pym_stride;
							src_h = file_size * 2 / 3 / src_w;
							src_uv_data = reinterpret_cast<void *>(reinterpret_cast<uintptr_t>(p1.data()) + src_w * src_h);
							} else {
							if (cmd_options.yuv_img_size[0] == 0 || cmd_options.yuv_img_size[1] == 0){
								cerr << "For resizer input model, please specify the image size by -s HxW" << endl;
								exit(-1);
							}
							if (file_size < cmd_options.yuv_img_size[0] * cmd_options.yuv_img_size[1] * 3 / 2) {
								cerr << "This input file [" << cmd_options.inputs[i] << "] size [" << file_size
									<< "] is less than from yuv_img_size [" << cmd_options.yuv_img_size[0] << "x"
									<< cmd_options.yuv_img_size[1]
									<< "]"  << endl;
								exit(-1);
							}
							src_y_data = p1.data();
							src_w = cmd_options.yuv_img_size[1];
							src_h = file_size * 2 / 3 / src_w;
							src_uv_data = reinterpret_cast<void *>(reinterpret_cast<uintptr_t>(p1.data()) + src_w * src_h);
						}
					}

					// 3. copy vio result to physically consecutive
					bpu_addr_t p2 = bpu_cpumem_alloc(file_size, BPU_NON_CACHEABLE);
					memcpy(reinterpret_cast<void *>(p2), src_y_data, src_w * src_h);
					memcpy(reinterpret_cast<void *>(p2 + src_w * src_h), src_uv_data, src_w * src_h / 2);
					if (g_bpu_usesample == 0) {
						ret = HB_VPS_ReleaseChnFrame(0, 6, &out_pym_buf);
						if (ret) {
							printf("HB_VPS_ReleaseChnFrame error!!!\n");
							g_exit = 1;
							continue;
						}
					}

					// 4. DMA to BPU memory
					p3 = bpu_mem_alloc(file_size, bpu_mem_alloc_flags);
					bpu_memcpy(p3, p2, file_size, ARM_TO_CNN);
					bpu_cpumem_free(p2);

					if (input_source != INPUT_FROM_PYRAMID) {
						printf("input_source: %d\n", input_source);
					}
					input_infos[i].input_source = INPUT_FROM_PYRAMID;
					input_infos[i].y_ptr = p3;
					input_infos[i].uv_ptr = p3 + src_h * src_w;
					input_infos[i].resizer_img_height = src_h;
					input_infos[i].resizer_img_width = src_w;
					input_infos[i].img_stride = src_w;
					input_infos[i].roi.coord.h = cmd_options.yuv_roi_coord[0];
					input_infos[i].roi.size.h = cmd_options.yuv_roi_size[0];
					input_infos[i].roi.coord.w = cmd_options.yuv_roi_coord[1];
					input_infos[i].roi.size.w = cmd_options.yuv_roi_size[1];
					if (input_infos[i].roi.size.h == 0 && input_infos[i].roi.size.w == 0) {
						input_infos[i].roi.coord.h = 0;
						input_infos[i].roi.coord.w = 0;
						input_infos[i].roi.size.h = valid_dim.h;
						input_infos[i].roi.size.w = valid_dim.w;
					}
					pym_input_index++;
					break;
				}
				default:
					cerr << "invalid input source." << endl;
					exit(-1);
				}
			}

			hbrt_ri_config_t config = {0};
			config.core_mask = cmd_options.core_mask;
			config.combined_output_region_ptr = output_region;
			config.combined_output_region_size = output_total_size;

			const uint32_t ri_id = 1;
			const uint32_t interrupt_number = 9;
			uint32_t funccall_number;
			struct timeval before_ri_start_time = {0};
			gettimeofday(&before_ri_start_time, nullptr);
			void *funccall_buffer;
			CHECK_HBRT_ERROR(hbrtRiStart(&funccall_buffer, &funccall_number, model_handle, input_infos.data(), &config, ri_id,
									interrupt_number));

			struct timeval after_ri_start_time = {0};
			gettimeofday(&after_ri_start_time, nullptr);
			gettimeofday(&pre_run_time, nullptr);

			while (funccall_number) {
				struct timeval before_set_fc_time = {0};
				gettimeofday(&before_set_fc_time, nullptr);
				int retval = cnn_core_set_fc(funccall_buffer, funccall_number, DEFAULT_CORE_MASK, nullptr);
				if (retval) {
					exit(retval);
				}
				cnn_core_wait_fc_done(DEFAULT_CORE_MASK, 9999);
				struct timeval after_wait_fc_done_time = {0};
				gettimeofday(&after_wait_fc_done_time, nullptr);

				struct timeval before_ri_continue_time = {0};
				gettimeofday(&before_ri_continue_time, nullptr);
				RETURN_HBRT_ERROR(hbrtRiContinue(&funccall_buffer, &funccall_number, ri_id, interrupt_number));
				struct timeval after_ri_continue_time = {0};
				gettimeofday(&after_ri_continue_time, nullptr);
			}

			gettimeofday(&final_interrupt_time, nullptr);
			CHECK_HBRT_ERROR(hbrtGetOutputFeatureNumber(&output_num, model_handle));
			CHECK_HBRT_ERROR(hbrtGetOutputFeatureHandles(&output_handle, model_handle));
#if 1
			CHECK_HBRT_ERROR(hbrtRiGetOutputData(&out_person_data, ri_id, 3, WORK_BPU_RAW));
			uint32_t output_mem_size = 0;
			CHECK_HBRT_ERROR(hbrtGetFeatureAlignedTotalByteSize(&output_mem_size, output_handle[3]));
			bpu_mem_cache_flush((bpu_addr_t)out_person_data, output_mem_size, BPU_MEM_INVALIDATE);
			size_t item_size = sizeof(bernoulli_hw_detection_post_process_bbox_with_pad_type_t);
			output_person_box_num = *(uint16_t *)(out_person_data) / item_size;
			bernoulli_hw_detection_post_process_bbox_with_pad_type_t *p_box =
			(bernoulli_hw_detection_post_process_bbox_with_pad_type_t *)((char *)(out_person_data) + item_size);

			CHECK_HBRT_ERROR(hbrtRiGetOutputData(&out_vehicle_data, ri_id, 2, WORK_BPU_RAW));
			uint32_t output_vehicle_mem_size = 0;
			CHECK_HBRT_ERROR(hbrtGetFeatureAlignedTotalByteSize(&output_vehicle_mem_size, output_handle[2]));
			bpu_mem_cache_flush((bpu_addr_t)out_vehicle_data, output_vehicle_mem_size, BPU_MEM_INVALIDATE);

			output_vehicle_box_num = *(uint16_t *)(out_vehicle_data) / item_size;

			bernoulli_hw_detection_post_process_bbox_with_pad_type_t *p_box_vehicle =
				(bernoulli_hw_detection_post_process_bbox_with_pad_type_t *)((char *)(out_vehicle_data) + item_size);

			CHECK_HBRT_ERROR(hbrtRiDestroy(ri_id));

#ifdef DRAW_BOX
			memset(fbp, 0x00, screen_size);
			for (uint16_t i = 0; i < output_person_box_num; i++) {
			    left_point = (p_box[i].left / 4)*1920/1280;
			    top_point = (p_box[i].top / 4)*1080/704;
			    right_point = (p_box[i].right / 4)*1920/1280;
			    bottom_point = (p_box[i].bottom / 4)*1080/704;
				draw_left = (uint16_t)(left_point);
				draw_top = (uint16_t)(top_point);
				draw_right = (uint16_t)(right_point);
				draw_bottom = (uint16_t)(bottom_point);
				printf("draw point is(%d, %d), (%d, %d)\n", draw_left, draw_top, draw_right, draw_bottom);
				if (output_person_box_num != 0) {
					draw_rect(fbp, draw_left, draw_top, draw_right, draw_bottom, 0xffff0000, 0);
				}
			}

			for (uint16_t i = 0; i < output_vehicle_box_num; i++) {
			    vehicle_left_point = (p_box_vehicle[i].left / 4)*1920/1280;
			    vehicle_top_point = (p_box_vehicle[i].top / 4)*1080/704;
			    vehicle_right_point = (p_box_vehicle[i].right / 4)*1920/1280;
			    vehicle_bottom_point = (p_box_vehicle[i].bottom / 4)*1080/704;

				vehicle_draw_left = (uint16_t)(vehicle_left_point);
				vehicle_draw_top = (uint16_t)(vehicle_top_point);
				vehicle_draw_right = (uint16_t)(vehicle_right_point);
				vehicle_draw_bottom = (uint16_t)(vehicle_bottom_point);
				printf("draw point is(%d, %d), (%d, %d)\n", vehicle_draw_left, vehicle_draw_top, vehicle_draw_right, vehicle_draw_bottom);
				if (output_vehicle_box_num != 0) {
					draw_rect(fbp, vehicle_draw_left, vehicle_draw_top, vehicle_draw_right, vehicle_draw_bottom, 0xff00ff00, 0);
				}
			}

			bpu_mem_free(p3);
			output_person_box_num = 0;
			output_vehicle_box_num = 0;
#endif
#endif
		}
		bpu_mem_free(output_region);
	}

	close(fbfd);
	munmap(fbp, screen_size);
	return 0;
}

int main(int argc, char **argv)
{
	int ret = 0;

	parseCmdOptions(cmd_options, argc, argv);

	signal(SIGINT, intHandler);

	ret = sample_singlepipe_5venc();
	if (ret != 0) {
		sample_singlepipe_5venc_deinit();
		return -1;
	}
	
	if (g_use_ipu == 0) {
		g_bpu_usesample = 1;
	}

	if (need_bpu == 1) {
		bpuCalc();
	}

	while(loop--) {
		char c;
		if (g_exit == 1) break;
		if (g_use_input) {
			printmode();
			c = getchar();
			// c = get_char();
			if (c == 'q') {
				g_exit = 1;
				break;
			} else if (c == '1') {
				sample_switch_dol2_linear();
			} else if (c == '2') {
				HB_MIPI_SwSensorFps(0, 25);
			} else if (c == '3') {
				HB_MIPI_SwSensorFps(0, 30);
			} else if (c == '4') {
				sample_set_vencfps(0, 30, 12);
			} else if (c == '5') {
				sample_set_vencfps(0, 30, 30);
			} else if (c == '6') {
				sample_set_vencfps(1, 30, 15);
			} else if (c == '7') {
				sample_set_vencfps(1, 30, 30);
			} else if (c == '8') {
				sample_venc_setjpegmode(1, JPEG_ENCODE_SNAP);
			} else if (c == '9') {
				HB_VPS_TriggerSnapFrame(0, 3, 100);
			} else if (c == 'a') {
				sample_venc_setjpegmode(1, JPEG_ENCODE_ALL);
			} else if (c == 'b') {
				sample_venc_setrcattr(0);
			} else if (c == 'c') {
				sample_venc_setrcattr(0);
			} else if (c == 'd') {
				sample_ipu_reconfig();
			} else if (c == 'e') {
				sample_reconfig_roi();
			} else if (c == 'f') {
				sample_reconfig_profilelevel();
			} else if (c == 'g') {
				sample_reconfig_rcmode(0);
			} else if (c == 'h') {
				sample_venc_set_rcmodes(2);
			} else if (c == 'i') {
				need_bpu = 1;
				bpuCalc();
			}
		}
		usleep(1000000);
	}

	g_exit = 1;
	sample_singlepipe_5venc_deinit();
	return 0;
}

void printmode()
{
	printf("select mode:\n");
	printf("1 switch dol2 and linear\n");
	printf("2 set 8a10 25 fps\n");
	printf("3 set 8a10 30 fps\n");
	printf("4 set venc 0 input 30fps, output 12fps\n");
	printf("5 set venc 0 input 30fps, output 30fps\n");
	printf("6 set venc 1 input 30fps, output 15fps\n");
	printf("7 set venc 1 input 30fps, output 30fps\n");
	printf("8 set venc 1 jpeg snap mode\n");
	printf("9 trag vps grp0 chn3 100 snap frames\n");
	printf("a set venc 1 jpeg normal mode\n");
}

static void draw_dot(char *frame, int x, int y, int color)
{
	int pbyte = iar_draw_pbyte;
	if (x >= iar_draw_width || y >= iar_draw_height)
		return;

	frame += ((y * iar_draw_width) + x) * pbyte;
	while (pbyte) {
		pbyte--;
		frame[pbyte] = (color >> (pbyte * 8)) & 0xFF;
	}
}
static void draw_hline(char *frame, int x0, int x1, int y, int color)
{
        int xi, xa;

        xi = (x0 < x1) ? x0 : x1;
        xa = (x0 > x1) ? x0 : x1;
        while (xi <= xa) {
                draw_dot(frame, xi, y, color);
                xi++;
        }
}

static void draw_vline(char *frame, int x, int y0, int y1, int color)
{
        uint32_t yi, ya;

        yi = (y0 < y1) ? y0 : y1;
        ya = (y0 > y1) ? y0 : y1;
        while (yi <= ya) {
                draw_dot(frame, x, yi, color);
                yi++;
        }
}
static void draw_rect(char *frame, int x0, int y0, int x1, int y1,
                                                        int color, int fill)
{
        int xi, xa, yi, ya;
	int i = 0;
	int line_width = 4;

        xi = (x0 < x1) ? x0 : x1;//left
        xa = (x0 > x1) ? x0 : x1;//right
        yi = (y0 < y1) ? y0 : y1;//bottom
        ya = (y0 > y1) ? y0 : y1;//top
        if (fill) {
                while (yi <= ya) {
                        draw_hline(frame, xi, xa, yi, color);
                        yi++;
                }
        } else {
		if (ya < line_width || yi > (iar_draw_height - line_width) ||
                                xi > (iar_draw_width - line_width) || xa > (iar_draw_width - line_width)) {
			cerr << "========point is 0,return========" << endl;
			return;
		}
		for (i = 0; i < line_width; i++) {
			draw_hline(frame, xi, xa, yi + i, color);
			draw_hline(frame, xi, xa, ya - i, color);
			draw_vline(frame, xi + i, yi, ya, color);
			draw_vline(frame, xa + i, yi, ya, color);
		}
        }
}

/*
 * a simple wrapper to check if a given path exists
 */
static bool path_exists(const std::string &fn, bool is_dir) {
  if (is_dir) {
    struct stat myStat = {0};
    if ((stat(fn.c_str(), &myStat) != 0) || !S_ISDIR(myStat.st_mode)) {
      cerr << "Directory [ " << fn << "] does not exist." << endl;
      return false;
    }
    return true;
  } else {
    std::ifstream f(fn.c_str());
    return f.good();
  }
}

void compare_parse(char *optarg)
{
	char *p = optarg, *d;
	int i = 0;
	printf("optarg %s\n", optarg);
	if (optarg == NULL) {
		cmp_param[0] = 1; //w_step
		cmp_param[1] = 1; //h_step
		cmp_param[2] = 30; //y delt threshold
		cmp_param[3] = -1;  //uv delt threshold
		cmp_param[4] = 1600; //y delt cnt
		cmp_param[5] = 800;  //uv delt cnt
	}
	while (p && *p && i < 4) {
		d = strchr(p, ',');
		if (d)
			*d = '\0';
		cmp_param[i] = atoi(p);
		printf("i %d cmp_param[i] %d===========\n", i, cmp_param[i]);
		i++;
		p = (d) ? (d + 1) : NULL;
	}
	return;
}
/*
 * a simple command line option parser
 */
static const char short_options[] = "o:f:i:n:c:s:d:z:v:p:a:l:b:R:E:e:r:t:h:m:G:S:M:B:P:I:C:W:U:L:O:A:D:F:X:T:N:Z::";
static const struct option long_options[] = {{"output-path", required_argument, nullptr, 'o'},
                                             {"hbm", required_argument, nullptr, 'f'},
                                             {"input-binary", required_argument, nullptr, 'i'},
                                             {"model-name", required_argument, nullptr, 'n'},
                                             {"core-mask", required_argument, nullptr, 'c'},
                                             {"yuv-img-size", required_argument, nullptr, 's'},
                                             {"yuv-roi-coord", required_argument, nullptr, 'd'},
                                             {"yuv-roi-size", required_argument, nullptr, 'z'},
                                             {"vio-config", required_argument, nullptr, 'v'},
                                             {"pyramid-ds-factor", required_argument, nullptr, 'p'},
					     {"disp-config", required_argument, nullptr, 'a'},
						 {"chanageRes", required_argument, nullptr, 'r'},
					     {"loop", required_argument, nullptr, 'l'},
					     {"bpu", required_argument, nullptr, 'b'},
						 {"taskbufsize", required_argument, nullptr, 't'},
							{"vin_vps_mode", required_argument, nullptr, 'm'},
							{"is_use_gdc", required_argument, nullptr, 'G'},
							{"sensorId", required_argument, nullptr, 'S'},
							{"mipiIdx", required_argument, nullptr, 'M'},
							{"bus", required_argument, nullptr, 'B'},
							{"port", required_argument, nullptr, 'P'},
							{"sederesIdx", required_argument, nullptr, 'I'},
							{"sedersePort", required_argument, nullptr, 'C'},
							{"osd", required_argument, nullptr, 'O'},
							{"save", required_argument, nullptr, 'R'},
							{"iarenable", required_argument, nullptr, 'W'},
							{"usesample", required_argument, nullptr, 'U'},
							{"useldc", required_argument, nullptr, 'L'},
							{"edit", required_argument, nullptr, 'A'},
							{"bindflag", required_argument, nullptr, 'D'},
							{"ipufeedback", required_argument, nullptr, 'F'},
							{"usex3clock", required_argument, nullptr, 'X'},
							{"compare", optional_argument, nullptr, 'Z'},
                                             {"help", no_argument, nullptr, 'h'},
                                             {nullptr, 0, nullptr, 0}};

int parseInputString(vector<string> &inputs, string &input_string) {
  uint32_t cur_str_index = 0;
  uint32_t cur_input_index = 0;
  while (input_string.find(',', cur_str_index) != std::string::npos) {
    uint32_t comma_index = input_string.find(',', cur_str_index);
    string cur_input_str = input_string.substr(cur_str_index, comma_index - cur_str_index);
    if (!path_exists(cur_input_str, false)) {
      cerr << "input file " << cur_input_str << "does not exist!" << endl;
      exit(-1);
    }
    inputs.push_back(cur_input_str);
    cur_input_index++;
    cur_str_index = comma_index + 1;
  }
  string cur_input_str = input_string.substr(cur_str_index, input_string.length() - cur_str_index);
  if (!path_exists(cur_input_str, false)) {
    cerr << "input file " << cur_input_str << "does not exist!" << endl;
    exit(-1);
  }
  inputs.push_back(cur_input_str);
  return 0;
}

int parsePyramidLayerString(vector<uint32_t> &pyramid_layer, string &input_string) {
  uint32_t cur_str_index = 0;
  uint32_t cur_input_index = 0;
  while (input_string.find(',', cur_str_index) != std::string::npos) {
    uint32_t comma_index = input_string.find(',', cur_str_index);
    string cur_input_str = input_string.substr(cur_str_index, comma_index - cur_str_index);
    uint32_t layer = 0;
    try {
      layer = strtol(cur_input_str.c_str(), nullptr, 10);
    } catch (const char *msg) {
      cerr << "pyramid layer parameter invalid. It is supposed to be numbers separated by comma, like 4,8.";
      exit(-1);
    }
    pyramid_layer.push_back(layer);
    cur_input_index++;
    cur_str_index = comma_index + 1;
  }
  string cur_input_str = input_string.substr(cur_str_index, input_string.length() - cur_str_index);
  uint32_t layer = 0;
  try {
    layer = strtol(cur_input_str.c_str(), nullptr, 10);
  } catch (const char *msg) {
    cerr << "pyramid layer parameter invalid. It is supposed to be numbers separated by comma, like 4,8.";
    exit(-1);
  }
  pyramid_layer.push_back(layer - 1);
  return 0;
}

static void parseCmdOptions(cmdOptions &cmd_options, int argc, char **argv) {
  int cmd_parser_ret = 0;
  cmd_options.inputs.clear();
  while ((cmd_parser_ret = getopt_long(argc, argv, short_options, long_options, nullptr)) != -1) {
    switch (cmd_parser_ret) {
      case 'f': {
        string hbm_str = optarg;
        if (hbm_str.substr(hbm_str.length() - 4, 4) != ".hbm" || !path_exists(hbm_str, false)) {
          cerr << "hbm does not exist or format invalid. hbm file name must end with .hbm" << endl;
          exit(-1);
        }
        cmd_options.hbm = optarg;
        break;
      }
      case 'o': {
        string output_str = optarg;
        if (!path_exists(output_str, true)) {
          cerr << "output path does not exist." << endl;
          exit(-1);
        }
        cmd_options.output_path = optarg;
        break;
      }
	  case 'm': {
		vin_vps_mode = atoi(optarg);
		cerr << "vin_vps_mode: " << vin_vps_mode << endl;
		break;
	  }
	  case 'Z': {
		compare = 1;
		compare_parse(optarg);
		cerr << "compare: " << compare << endl;
		break;
	  }
	  case 'r': {
		g_chanage_res = atoi(optarg);
		cerr << "g_chanage_res: " << g_chanage_res << endl;
		break;  
	  }
	  case 't': {
		g_use_input = atoi(optarg);
		cerr << "g_use_input: " << g_use_input << endl;
		break;  
	  }
	  case 'T': {
		g_temperMode = atoi(optarg);
		if (g_temperMode < 0) g_temperMode = 0;
		else if (g_temperMode > 3) g_temperMode = 0;
		cerr << "g_temperMode: " << g_temperMode << endl;
		break;  
	  }
	  // G:S:M:B:P
	  case 'G': {
		need_grp_rotate = atoi(optarg);
		need_chn_rotate = need_grp_rotate/2;
		need_grp_rotate = need_grp_rotate&1;
		cerr << "need_grp_rotate: " << need_grp_rotate << endl;
		cerr << "need_chn_rotate: " << need_chn_rotate << endl;
		break;
	  }
	  case 'R': {
		g_save_flag = atoi(optarg);
		cerr << "g_save_flag: " << g_save_flag << endl;
		break;
	  }
	  case 'S': {
		sensorId[0] = atoi(optarg);
		cerr << "sensorId[0]: " << sensorId[0] << endl;
		break;
	  }
	  case 'M': {
		mipiIdx[0] = atoi(optarg);
		cerr << "mipiIdx[0]: " << mipiIdx[0] << endl;
		break;
	  }
	  case 'B': {
		bus[0] = atoi(optarg);
		cerr << "bus[0]: " << bus[0] << endl;
		break;
	  }
	  case 'P': {
		port[0] = atoi(optarg);
		cerr << "port[0]: " << port[0] << endl;
		break;
	  }
	  case 'I': {
		serdes_index[0] = atoi(optarg);
		cerr << "serdes_index[0]: " << serdes_index[0] << endl;
		break;
	  }
	  case 'C': {
		serdes_port[0] = atoi(optarg);
		cerr << "serdes_port[0]: " << serdes_port[0] << endl;
		break;
	  }
	  case 'O': {
		g_osd_flag = atoi(optarg);
		cerr << "g_osd_flag: " << g_osd_flag << endl;
		break;
	  }
	  case 'E': {
		g_venc_flag = atoi(optarg);
		cerr << "g_venc_flag: " << g_venc_flag << endl;
		break;
	  }
	  case 'W': {
		g_iar_enable = atoi(optarg);
		cerr << "g_iar_enable: " << g_iar_enable << endl;
		break;
	  }
	  case 'A': {
		g_set_qos = atoi(optarg);
		if (g_set_qos >= 2) {
			g_use_ipu = 1;
		}
		cerr << "g_set_qos: " << g_set_qos << endl;
		cerr << "g_use_ipu: " << g_use_ipu << endl;
		break;
	  }
	  case 'F': {
		g_frame_rate = atoi(optarg);
		cerr << "g_frame_rate: " << g_frame_rate << endl;
		break;
	  }
	  case 'X': {
		g_use_x3clock = atoi(optarg);
		cerr << "g_use_x3clock: " << g_use_x3clock << endl;
		break;
	  }
	  case 'U': {
		g_bpu_usesample = atoi(optarg);
		cerr << "g_bpu_usesample: " << g_bpu_usesample << endl;
		break;
	  }
	  case 'L': {
		g_use_ldc = atoi(optarg);
		cerr << "g_use_ldc: " << g_use_ldc << endl;
		break;
	  }
	  case 'D': {
		g_bindflag = atoi(optarg);
		cerr << "g_bindflag: " << g_bindflag << endl;
		break;
	  }
      case 'i': {
        string input_str = optarg;
        parseInputString(cmd_options.inputs, input_str);
        break;
      }
      case 'n': {
        cmd_options.model_name = optarg;
        break;
      }
      case 'c': {
		g_calib = atoi(optarg);
		cerr << "g_calib: " << g_calib << endl;
        break;
      }
      case 's': {
        string h_str, w_str;
        string str = optarg;
        size_t x_index = str.find('x');
        if (x_index == std::string::npos) {
          cerr << "Illegal yuv image size. size must be in format [HxW]." << endl;
          exit(-1);
        }
        h_str = str.substr(0, x_index);
        w_str = str.substr(x_index + 1, str.length() - x_index - 1);
        cmd_options.yuv_img_size[0] = strtol(h_str.c_str(), nullptr, 10);
        cmd_options.yuv_img_size[1] = strtol(w_str.c_str(), nullptr, 10);
        break;
      }
      case 'd': {
		g_vps_frame_depth = atoi(optarg);
		cerr << "g_vps_frame_depth: " << g_vps_frame_depth << endl;
        break;
      }
      case 'z': {
        string h_str, w_str;
        string str = optarg;
        size_t x_index = str.find('x');
        if (x_index == std::string::npos) {
          cerr << "Illegal yuv ROI size. size must be in format [HxW]." << endl;
          exit(-1);
        }
        h_str = str.substr(0, x_index);
        w_str = str.substr(x_index + 1, str.length() - x_index - 1);
        cmd_options.yuv_roi_size[0] = strtol(h_str.c_str(), nullptr, 10);
        cmd_options.yuv_roi_size[1] = strtol(w_str.c_str(), nullptr, 10);
        break;
      }
      case 'v': {
        cmd_options.vio_config = optarg;
        break;
      }
      case 'a': {
	// cmd_options.disp_config = strtol(optarg, nullptr, 10);
	// cerr << "**********panel type is*************" << cmd_options.disp_config  << endl;
	// panel_type = cmd_options.disp_config;
    // cmd_options.disp_config = optarg;
        break;
      }
      case 'l': {
        loop = strtol(optarg, nullptr, 10);
        cerr << "**********loop cnt is*************" << loop  << endl;
        break;
      }
      case 'b': {
        need_bpu = strtol(optarg, nullptr, 10);
        cerr << "**********need bpu is*************" << need_bpu  << endl;
        break;
      }
      case 'p': {
        // string input_str = optarg;
        // parsePyramidLayerString(cmd_options.pyramid_down_scale_factor, input_str);
		g_inter_qp = atoi(optarg);
		cerr << "g_inter_qp: " << g_inter_qp << endl;
        break;
      }

      case 'h':
      default: {
        cout << "HBDK BPU Simulator. Lite version. ALL RIGHTS RESERVED." << endl;
        cout << "Usage:" << endl;
        cout << "-o/--output-path        path to store output files.                        [OPTIONAL]" << endl;
        cout << "-f/--hbm                hbm containing model to simulate.                  [REQUIRED]" << endl;
        cout << "-i/--input-binary       binary input file(s), separated by comma.          [REQUIRED]" << endl;
        cout << "-n/--model-name         name of model to simulate.                         [REQUIRED]" << endl;
        cout << "-c/--core-mask          bit mask of core(s) to run simulation.             [OPTIONAL]" << endl;
        cout << "-s/--yuv-img-size       size of yuv input image. [HxW]                     [OPTIONAL]" << endl;
        cout << "-d/--yuv-roi-coord      coordinate of resizer ROI. [HxW].                  [OPTIONAL]" << endl;
        cout << "-z/--yuv-roi-size       size of resizer ROI. [HxW].                        [OPTIONAL]" << endl;
        cout << "-v/--vio-config         vio configuration file path.                       [OPTIONAL]" << endl;
        cout << "-p/--pyramid-ds-factor  layer index for pyramid input, separated by comma. [OPTIONAL]" << endl;
#ifdef VIDEO_RECORDER
        cout << "-R/--enable-recorder    enable recorder process.                           [OPTIONAL]" << endl;
        cout << "-E/--enable-4-encoder   enable 4 encoder instance.                         [OPTIONAL]" << endl;
        cout << "-e/--encoder-codec      encoder codec id 0:h265, 1:h264.                   [OPTIONAL]" << endl;
        cout << "-r/--encoder-rotate     encoder rotate degree, 0,90,180,270.               [OPTIONAL]" << endl;
        cout << "-t/--recorder-duration  recorder duration(s).                              [OPTIONAL]" << endl;
#endif
        cout << "-h/--help               print usage of this tool.                                    " << endl;
        exit(0);
      }
    }
  }

  if (cmd_options.core_mask == 0) {
    cmd_options.core_mask = MAX_CORE_MASK - 1;
  }
  if (cmd_options.output_path == nullptr) {
    cmd_options.output_path = "./";
  }
  if (cmd_options.model_name == nullptr) {
    cerr << "model name is not provided!" << endl;
    exit(-1);
  }
  if (cmd_options.inputs.empty()) {
    cerr << "model input is not provided!" << endl;
    exit(-1);
  }
  if (cmd_options.hbm == nullptr) {
    cerr << "hbm is not provided!" << endl;
    exit(-1);
  }

	iar_draw_width = 1920;
	iar_draw_height = 1080;
}
