//
// Created by zhenjiang.wang on 8/11/18.
//

#ifndef HBDK_HBRT_LAYOUT_H
#define HBDK_HBRT_LAYOUT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#if defined(TARGET_M2)
#define CONVERT_ENDIAN 0
#else
#define CONVERT_ENDIAN 1
#endif

// modify <hbrtGetLayoutDescriptor> and this enumerate at the same time
typedef enum {
  LAYOUT_UNKNOWN_UNKNOWN,
  LAYOUT_NHWC_I8_1P1C_S1,
  LAYOUT_NHWC_I8_1P16C_S1,
  LAYOUT_NHWC_I8_4P8C_S1,
  LAYOUT_NHWC_I8_4P8C_S2,
  LAYOUT_NHWC_I8_8P8C_S1,
  LAYOUT_NHWC_I8_8P8C_S2F,
  LAYOUT_NHWC_I8_16P16C_S1,
  LAYOUT_NHWC_I8_16P16C_S2F,
  LAYOUT_NHWC_I8_16P16C_S1_4PE,
  LAYOUT_NHWC_I8_16P16C_S2F_4PE,
  LAYOUT_NHCW_I8_1P32C_S1,
  LAYOUT_NHCW_I8_8P4C_S1,
  LAYOUT_NHCW_I8_8P4C_S2,
  LAYOUT_NHCW_I8_16P16C_S1,
  LAYOUT_NHCW_I8_16P16C_S2,
  LAYOUT_NHWC_I32_1P1C_S1,
  LAYOUT_NHWC_I32_4P4C_S1,
  LAYOUT_NHWC_I32_1P8C_S1,
  LAYOUT_NCHW_I32_1P8C_S1,
  LAYOUT_NHWC_I32_16P4C_S1,
  LAYOUT_NHWC_I32_16P4C_S1_4PE,
  LAYOUT_NHWC_I8_16P1C_S1,
  LAYOUT_NHWC_I8_8P2C_S1,
  LAYOUT_NHWC_I32_2P8C_S1,
  LAYOUT_NHWC_I32_8P8C_S1,
  LAYOUT_NHWC_I8_2C1P_4PE,
  LAYOUT_NHWC_I8_1P2C_2PE,
  LAYOUT_NUMBER,
} hbrt_layout_t;

typedef enum {
  NHWC,
  NHCW,
  NCHW,
} hbrt_layout_order_t;

/*
 * Descriptors for each layout
 */
typedef struct {
  const char *name;
  hbrt_layout_t type;
  hbrt_layout_order_t order;
  uint8_t element_size;  // in bytes
  uint8_t p;
  uint8_t c;
  uint8_t stride;  // 1=s1, 2=s2_outer (coordinate mapping), 3=s2_inner (double
  // inner block size)
  uint8_t pe;
} hbrt_layout_descriptor_t;

extern const hbrt_layout_descriptor_t *hbrtGetLayoutDescriptor(hbrt_layout_t);

/*
 * APIs for layout conversion
 */

// convert the entire feature
extern void hbrtGetFeatureNativeNHWC(const void *source_ptr, void *dest_ptr, hbrt_layout_t source_layout_type,
                                     const uint32_t *source_aligned_dims);

// convert the specified channel, getting H * W * 1 feature
extern void hbrtGetFeatureNativeHW1(const void *source_ptr, void *dest_ptr, hbrt_layout_t source_layout_type,
                                    const uint32_t *source_aligned_dims, /* NHWC */
                                    uint32_t c_index);

// convert the specified pixel, getting 1 * 1 * C
extern void hbrtGetFeatureNative11C(const void *source_ptr, void *dest_ptr, hbrt_layout_t source_layout_type,
                                    const uint32_t *source_aligned_dims, /* NHWC */
                                    uint32_t h_index, uint32_t w_index);

#ifdef __cplusplus
}
#endif

#endif  // HBDK_HBRT_LAYOUT_H
