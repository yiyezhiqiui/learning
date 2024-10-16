#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>

namespace rune {
class NeuralNetwork {
public:
    struct RuneObject {
        cv::Point2f vertices[5]; //符叶的五个顶点
        cv::Rect_<float> rect;
        int cls;
        int color;
        float prob;
        std::vector<cv::Point2f> pts;
        //cv::Point2f symbol;
    };

    NeuralNetwork();
    ~NeuralNetwork();
    bool Init(std::string path); //模型路径
    bool Detect(cv::Mat& src, std::vector<RuneObject>& objects);

private:
    float* SyncImageDetect(cv::Mat& frame);
    ov::Core core;
    std::shared_ptr<ov::Model> model; // 网络
    ov::CompiledModel compiled_model; // 可执行网络
    ov::InferRequest infer_request;   // 推理请求
    ov::Tensor input_tensor;

    std::string input_name;
    std::string output_name;

    Eigen::Matrix<float, 3, 3> transfrom_matrix;
};
} // namespace rune