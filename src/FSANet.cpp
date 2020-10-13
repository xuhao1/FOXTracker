#include "FSANet.h"
#include "FlightAgxSettings.h"
#include "fagx_datatype.h"

FSANet::FSANet():env(ORT_LOGGING_LEVEL_WARNING, "test") {
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    session_options.SetInterOpNumThreads(1);
    //session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    printf("Using Onnxruntime C++ API\n");
    std::wstring unicode(settings->fsanet_model.begin(), settings->fsanet_model.end());
    session = new Ort::Session(env, unicode.c_str(), session_options);


    Ort::AllocatorWithDefaultOptions allocator;
    size_t num_input_nodes = session->GetInputCount();

    std::vector<int64_t> input_node_dims{1, 64, 64, 3};  // simplify... this model has only 1 input node {1, 3, 224, 224}.
                                         // Otherwise need vector<vector<>>
    std::vector<int64_t> output_shape_{1, 3};

    printf("Number of inputs = %zu\n", num_input_nodes);

    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    input_tensor_ = Ort::Value::CreateTensor<float>(memory_info,
               input_image.data(), input_image.size(), input_node_dims.data(), 4);
    output_tensor_ = Ort::Value::CreateTensor<float>(memory_info, results_.data(), results_.size(), output_shape_.data(), output_shape_.size());

}


Eigen::Vector3d FSANet::inference(const cv::Mat &image) {
    cv::Mat data, data_F;

    cv::resize(image, data, cv::Size(64, 64));
    cv::normalize(data, data, 0, 255, cv::NORM_MINMAX);
    //std::cout << "Size" << data.size() << "type" << data.type() << std::endl;
    //data.convertTo(data_F, CV_32FC3, 1/255.0);
    data.convertTo(data_F, CV_32FC3, 1.0);

    memcpy(input_image.data(), data_F.data, 64*64*3*sizeof(float));
    TicToc t;
    auto output_tensors = session->Run(Ort::RunOptions{nullptr}, input_node_names.data(), &input_tensor_, 1, output_node_names.data(), 1);
    float* floatarr = output_tensors.front().GetTensorMutableData<float>();
    //Yaw Pitch Roll
    //qDebug() << "FSA T" << t.toc() << "Y " << -floatarr[0] << "P" << floatarr[1] << "R" << floatarr[2];

    return Eigen::Vector3d(floatarr[0], floatarr[1], floatarr[2])*3.1415926/180;
}
