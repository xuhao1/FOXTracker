#ifndef FSANET_H
#define FSANET_H

#include <Eigen/Eigen>
#include <onnxruntime_cxx_api.h>
#include <opencv2/core.hpp>

class FSANet {
    Ort::Env env;
    Ort::Session * session = nullptr;
public:
    FSANet();

    Eigen::Vector3d inference(const cv::Mat & image);
private:
    std::array<float, 64*64*3> input_image;
    Ort::Value output_tensor_{nullptr};
    Ort::Value input_tensor_{nullptr};
    std::array<float, 3> results_{};
    std::vector<const char*> input_node_names{"input_1:0"};
    std::vector<const char*> output_node_names{"pred_pose/mul_24:0"};
};
#endif // FSANET_H
