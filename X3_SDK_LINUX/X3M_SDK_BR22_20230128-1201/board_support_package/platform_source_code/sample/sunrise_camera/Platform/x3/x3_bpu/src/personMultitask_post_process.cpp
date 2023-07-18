// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <utility>
#include <vector>
#include <iostream>
#include <iomanip>
#include <algorithm>

#include "utils/utils_log.h"

#include "personMultitask_post_process.h"

/* output layers
 * 0: bbox
 * 1: kps
 * 2: mask
 * 3: reid
 * 4: lmks2_label
 * 5: lmks2_offset
 * 6: lmks1
 * 7: 3d_pose
 * 8: plate_color
 * 9: plate_row
 * 10: kps_label
 * 11: kps_offset
 */

/**
 * \~Chinese @brief 检测框
 */
template <typename Dtype>
struct BBox_ {
  inline BBox_() {}
  inline BBox_(Dtype x1_, Dtype y1_, Dtype x2_, Dtype y2_, float score_ = 0.0f,
               int32_t id_ = -1, const std::string &category_name_ = "") {
    x1 = x1_;
    y1 = y1_;
    x2 = x2_;
    y2 = y2_;
    id = id_;
    score = score_;
    category_name = category_name_;
  }
  inline Dtype Width() const { return (x2 - x1); }
  inline Dtype Height() const { return (y2 - y1); }
  inline Dtype CenterX() const { return (x1 + (x2 - x1) / 2); }
  inline Dtype CenterY() const { return (y1 + (y2 - y1) / 2); }

  inline friend std::ostream &operator<<(std::ostream &out, BBox_ &bbox) {
    /*out << "( x1: " << bbox.x1 << " y1: " << bbox.y1 << " x2: " << bbox.x2*/
        /*<< " y2: " << bbox.y2 << " score: " << bbox.score << " )";*/
	out << "{" 
       << R"("bbox")"
       << ":" << "[" << bbox.x1 << "," << bbox.y1 << "," << bbox.x2
       << "," << bbox.y2 << "],"
       << R"("score")"
       << ":" << bbox.score
       << "}";
    return out;
  }

  inline friend std::ostream &operator<<(std::ostream &out, const BBox_ &bbox) {
    /*out << "( x1: " << bbox.x1 << " y1: " << bbox.y1 << " x2: " << bbox.x2*/
        /*<< " y2: " << bbox.y2 << " score: " << bbox.score << " )";*/
	out << "{" 
       << R"("bbox")"
       << ":" << "[" << bbox.x1 << "," << bbox.y1 << "," << bbox.x2
       << "," << bbox.y2 << "],"
       << R"("score")"
       << ":" << bbox.score
       << "}";
    return out;
  }

  Dtype x1 = 0;
  Dtype y1 = 0;
  Dtype x2 = 0;
  Dtype y2 = 0;
  float score = 0.0;
  int32_t id = 0;
  float rotation_angle = 0.0;
  std::string category_name = "";
};
typedef BBox_<float> BBox;

typedef struct BPU_BBOX_F32
{
  float left;
  float top;
  float right;
  float bottom;
  float score;
  float class_label;
} BPU_BBOX_F32;


/**
 * \~Chinese @brief 2D坐标点
 */
template <typename Dtype>
struct Point_ {
  inline Point_() {}
  inline Point_(Dtype x_, Dtype y_, float score_ = 0.0)
      : x(x_), y(y_), score(score_) {}

  Dtype x = 0;
  Dtype y = 0;
  float score = 0.0;
};
typedef Point_<float> Point;


/**
 * \~Chinese @brief 2D坐标点集合，可用于存储关键点等结果
 */
struct Points {
	std::vector<Point> values;
	/// \~Chinese 置信度
	float score = 0.0;

	inline friend std::ostream &operator<<(std::ostream &out, Points &points) {
		out << "{" 
		<< R"("kps")" << ":" << "[";
		for (int i = 0; i < points.values.size(); i++) {
			auto point = points.values[i];
			out << point.x << "," << point.y << "," << point.score;
			if (i < points.values.size() - 1)
			out << ",";
		}
		out << "],"
		<< R"("score")"
		<< ":" << points.score
		<< "}";
		return out;
	}
};

typedef Points Landmarks;


// coordinate transform.
// fasterrcnn model's input size maybe not eqaul to origin image size,
// needs coordinate transform for detection result.

void CoordinateBbox(std::vector<BBox> &boxes,
                         int src_image_width, int src_image_height,
                         int model_input_width, int model_input_hight) {
  int x_drift = 0, y_drift = 0;

  for (auto &box : boxes) {
      box.x1 = box.x1 * src_image_width / model_input_width + x_drift;
      box.y1 = box.y1 * src_image_height / model_input_hight + y_drift;
      box.x2 = box.x2 * src_image_width / model_input_width + x_drift;
      box.y2 = box.y2 * src_image_height / model_input_hight + y_drift;
  }
}

void CoordinateLandmarks(std::vector<Landmarks> &landmarks,
                         int src_image_width, int src_image_height,
                         int model_input_width, int model_input_hight) {
	int x_drift = 0, y_drift = 0;

	for (auto &landmark : landmarks) {
		for (auto &point : landmark.values) {
			point.x = point.x * src_image_width / model_input_width + x_drift;
			point.y = point.y * src_image_height / model_input_hight + y_drift;
		}
	}
}

inline float SigMoid(const float &input) {
  return 1 / (1 + std::exp(-1 * input));
}

inline float GetFloatByInt(int32_t value, uint32_t shift) {
  return (static_cast<float>(value)) / (static_cast<float>(1 << shift));
}

static void GetRppRects(
    const hbDNNTensor &tensor,
    std::vector<BBox> &boxes,
    int index) {
  size_t item_size = sizeof(BPU_BBOX_F32);
  float output_byte_size =
      *reinterpret_cast<float *>(tensor.sysMem[0].virAddr);
  float box_num = output_byte_size / item_size;
  auto box_ptr = reinterpret_cast<BPU_BBOX_F32 *>(
      reinterpret_cast<uintptr_t>(tensor.sysMem[0].virAddr) + item_size);
  int nBox = static_cast<int>(box_num);

  for (int i = 0; i < nBox; ++i) {
    BBox box;
    box.x1 = box_ptr[i].left;
    box.y1 = box_ptr[i].top;
    box.x2 = box_ptr[i].right;
    box.y2 = box_ptr[i].bottom;
    box.score = box_ptr[i].score;
    int type = box_ptr[i].class_label;

    boxes.push_back(std::move(box));
  }
}


static char* GetBbox(PersonPostProcessInfo_t *post_info, int index, 	std::vector<BBox> &boxes)
{
	int i;
	GetRppRects(post_info->output_tensor[index], boxes, index);

#if 0
	for (i = 0; i < boxes.size(); i++)
		std::cout << boxes[i] << std::endl;
#endif

	return NULL;
}

void GetKps2(PersonPostProcessInfo_t *post_info,
    std::vector<Landmarks> &kpss,
    void* kps_label_output, void* kps_offset_output,
    const std::vector<BBox> &body_boxes) {
    
  int32_t *label_feature = reinterpret_cast<int32_t *>(kps_label_output);
  int32_t *offset_feature = reinterpret_cast<int32_t *>(kps_offset_output);

  float kps_pos_distance_ = 25;
  int kps_feat_width_ = 16;
  int kps_feat_height_ = 16;
  int kps_points_number_ = 17;
  int kps_feat_stride_ = 16;

  int32_t *aligned_kps_label_dim = post_info->output_tensor[10].properties.alignedShape.dimensionSize;
  int32_t *aligned_kps_offset_dim = post_info->output_tensor[11].properties.alignedShape.dimensionSize;
  uint32_t kps_label_shift_ = post_info->output_tensor[10].properties.shift.shiftData[0];
  uint32_t kps_offset_shift_ = post_info->output_tensor[11].properties.shift.shiftData[0];

  int input_height = kps_feat_height_ * kps_feat_stride_;
  int input_width = kps_feat_width_ * kps_feat_stride_;
  float base_center = (kps_feat_stride_ - 1) / 2.0;

  int label_feature_size = aligned_kps_label_dim[1] *
                           aligned_kps_label_dim[2] *
                           aligned_kps_label_dim[3];
  int label_h_stride = aligned_kps_label_dim[2] *
                       aligned_kps_label_dim[3];
  int label_w_stride = aligned_kps_label_dim[3];

  int offset_feature_size = aligned_kps_offset_dim[1] *
                            aligned_kps_offset_dim[2] *
                            aligned_kps_offset_dim[3];
  int offset_h_stride = aligned_kps_offset_dim[2] *
                        aligned_kps_offset_dim[3];
  int offset_w_stride = aligned_kps_offset_dim[3];

  size_t body_box_num = body_boxes.size();
  for (size_t box_id = 0; box_id < body_box_num; ++box_id) {
    const auto &body_box = body_boxes[box_id];
    float x1 = body_box.x1;
    float y1 = body_box.y1;
    float x2 = body_box.x2;
    float y2 = body_box.y2;
    float w = x2 - x1 + 1;
    float h = y2 - y1 + 1;

    float scale_x = w / input_width;
    float scale_y = h / input_height;
    float scale_pos_x = kps_pos_distance_ * scale_x;
    float scale_pos_y = kps_pos_distance_ * scale_y;

    Landmarks skeleton;
    skeleton.values.resize(kps_points_number_);

    auto *label_feature_begin = label_feature + label_feature_size * box_id;
    for (int kps_id = 0; kps_id < kps_points_number_; ++kps_id) {
      // find the best position
      int max_w = 0;
      int max_h = 0;
      int max_score_before_shift = label_feature_begin[kps_id];
      int32_t *mxnet_out_for_one_point = nullptr;
      for (int hh = 0; hh < kps_feat_height_; ++hh) {
        for (int ww = 0; ww < kps_feat_width_; ++ww) {
          mxnet_out_for_one_point =
              label_feature_begin + hh * label_h_stride + ww * label_w_stride;
          if (mxnet_out_for_one_point[kps_id] > max_score_before_shift) {
            max_w = ww;
            max_h = hh;
            max_score_before_shift = mxnet_out_for_one_point[kps_id];
          }
        }
      }
      float max_score = GetFloatByInt(max_score_before_shift, kps_label_shift_);

      float base_x = (max_w * kps_feat_stride_ + base_center) * scale_x + x1;
      float base_y = (max_h * kps_feat_stride_ + base_center) * scale_y + y1;

	  /*std::cout << base_x  << ", " << base_y << ", " << SigMoid(max_score) << ",";*/

      // get delta
      int32_t *offset_feature_begin =
          offset_feature + offset_feature_size * box_id;
      mxnet_out_for_one_point = offset_feature_begin + max_h * offset_h_stride +
                                max_w * offset_w_stride;

      const auto x_delta = mxnet_out_for_one_point[2 * kps_id];
      float fp_delta_x = GetFloatByInt(x_delta, kps_offset_shift_);

      const auto y_delta = mxnet_out_for_one_point[2 * kps_id + 1];
      float fp_delta_y = GetFloatByInt(y_delta, kps_offset_shift_);

      Point point;
      point.x = base_x + fp_delta_x * scale_pos_x;
      point.y = base_y + fp_delta_y * scale_pos_y;
      point.score = SigMoid(max_score);
      skeleton.values[kps_id] = point;
	  /*std::cout << point.x  << ", " << point.y << ", " << point.score << ",";*/
    }
    kpss.push_back(std::move(skeleton));
  }
}

char* PersonPostProcess(PersonPostProcessInfo_t *post_info)
{
	int i = 0;
	char *str_dets;
	std::vector<BBox> body_boxes;
	std::vector<BBox> face_boxes;
	std::vector<BBox> head_boxes;
	// 获取人体 bbox
	GetBbox(post_info, 1, body_boxes);
	// 获取人脸 bbox
	GetBbox(post_info, 5, face_boxes);
	// 获取人头 bbox
	GetBbox(post_info, 3, head_boxes);
	// 获取人体骨骼关键点 17个点
	std::vector<Landmarks> body_kps;
	void* kps_label_out_put = post_info->output_tensor[10].sysMem[0].virAddr;
	void* kps_offset_out_put = post_info->output_tensor[11].sysMem[0].virAddr;

	GetKps2(post_info, body_kps, kps_label_out_put, kps_offset_out_put, body_boxes);

	// 根据实际图片分辨率和算法计算用的图片分辨率信息，把算法结果的数据修正到实际图片上的坐标上
	CoordinateBbox(body_boxes, post_info->ori_width, post_info->ori_height,
                      post_info->width, post_info->height);
	CoordinateBbox(face_boxes, post_info->ori_width, post_info->ori_height,
                      post_info->width, post_info->height);
	CoordinateBbox(head_boxes, post_info->ori_width, post_info->ori_height,
                      post_info->width, post_info->height);
	CoordinateLandmarks(body_kps, post_info->ori_width, post_info->ori_height,
                      post_info->width, post_info->height);

#if 0
		for (int i = 0; i < body_kps.size(); i++) {
			auto kps = body_kps[i];
			std::cout << kps << std::endl;
		}
#endif


	std::stringstream out_string;
	// 这里就把给web的头添加上了
	out_string << "\"person_result\": {";
	out_string << "\"body\": [";
	for (i = 0; i < body_boxes.size(); i++) {
		auto box = body_boxes[i];
	/*std::cout << det_ret << std::endl;*/
		out_string << box;
		if (i < body_boxes.size() - 1)
		out_string << ",";
	}
	out_string << "],";
	out_string << "\"face\": [";
	for (i = 0; i < face_boxes.size(); i++) {
		auto box = face_boxes[i];
	/*std::cout << det_ret << std::endl;*/
		out_string << box;
		if (i < face_boxes.size() - 1)
		out_string << ",";
	}
	out_string << "],";
	out_string << "\"head\": [";
	for (i = 0; i < head_boxes.size(); i++) {
		auto box = head_boxes[i];
	/*std::cout << det_ret << std::endl;*/
		out_string << box;
		if (i < head_boxes.size() - 1)
		out_string << ",";
	}
	out_string << "],";
	out_string << "\"kps\": [";
	for (i = 0; i < body_kps.size(); i++) {
		auto kps = body_kps[i];
		out_string << kps;
		if (i < body_kps.size() - 1)
		out_string << ",";
	}
	out_string << "]";
	out_string << "}" << std::endl;

	str_dets = (char *)malloc(out_string.str().length() + 1);
	str_dets[out_string.str().length()] = '\0';
	snprintf(str_dets, out_string.str().length(), "%s", out_string.str().c_str());
	/*printf("%s\n", str_dets);*/
	return str_dets;
}

