#include <iostream>
#include <opencv2/core/types.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "Metrics/Metrics.h"
#include "Metrics/BlurEstimation.h"

using namespace std;

int main() {
    string camera_image_path_lidar6 = "../../data/metrics_examples/lidar6/main_part.png";
    string rendered_image_path_lidar6_1 = "../../data/metrics_examples/lidar6/BF_ct_part.png";
    string rendered_image_path_lidar6_2 = "../../data/metrics_examples/lidar6/BF_part.png";
    string rendered_image_path_lidar6_3 = "../../data/metrics_examples/lidar6/PCL_ct_part.png";
    string rendered_image_path_lidar6_4 = "../../data/metrics_examples/lidar6/PCL_part.png";

    string camera_image_path_lidar10 = "../../data/metrics_examples/lidar10/main_part.png";
    string rendered_image_path_lidar10_1 = "../../data/metrics_examples/lidar10/BF_ct_part.png";
    string rendered_image_path_lidar10_2 = "../../data/metrics_examples/lidar10/BF_part.png";
    string rendered_image_path_lidar10_3 = "../../data/metrics_examples/lidar10/PCL_ct_part.png";
    string rendered_image_path_lidar10_4 = "../../data/metrics_examples/lidar10/PCL_part.png";

    string camera_image_path_lidar15 = "../../data/metrics_examples/lidar15/main_part.png";
    string rendered_image_path_lidar15_1 = "../../data/metrics_examples/lidar15/BF_ct_part.png";
    string rendered_image_path_lidar15_2 = "../../data/metrics_examples/lidar15/BF_part.png";
    string rendered_image_path_lidar15_3 = "../../data/metrics_examples/lidar15/PCL_ct_part.png";
    string rendered_image_path_lidar15_4 = "../../data/metrics_examples/lidar15/PCL_part.png";

    cv::Mat camera_image;
    cv::Mat rendered_image1;
    cv::Mat rendered_image2;
    cv::Mat rendered_image3;
    cv::Mat rendered_image4;
    camera_image = cv::imread(camera_image_path_lidar15, cv::IMREAD_COLOR);
    rendered_image1 = cv::imread(rendered_image_path_lidar15_1, cv::IMREAD_COLOR);
    rendered_image2 = cv::imread(rendered_image_path_lidar15_2, cv::IMREAD_COLOR);
    rendered_image3 = cv::imread(rendered_image_path_lidar15_3, cv::IMREAD_COLOR);
    rendered_image4 = cv::imread(rendered_image_path_lidar15_4, cv::IMREAD_COLOR);

    cout << "Значения метрик для lidar15: \n";

    // Считаем ssim метрику для одного и того же изображения
    cout << "ssim метрика для одного и того же изображения: " << Metrics::SSIM(camera_image, camera_image, 8) << endl;
    // Считаем ssim метрику для изображения, полученного с камеры и для срендеренного изображения
    cout << "ssim метрика main_part и BF_ct_part: " << Metrics::SSIM(camera_image, rendered_image1, 8) << endl;
    cout << "ssim метрика main_part и BF_part: " << Metrics::SSIM(camera_image, rendered_image2, 8) << endl;
    cout << "ssim метрика main_part и PCL_ct_part: " << Metrics::SSIM(camera_image, rendered_image3, 8) << endl;
    cout << "ssim метрика main_part и PCL_part: " << Metrics::SSIM(camera_image, rendered_image4, 8) << endl;

    // Считаем psnr метрику для одного и того же изображения
    cout << "psnr метрика для одного и того же изображения: " << Metrics::PSNR(camera_image, camera_image) << endl;
    // Считаем psnr метрику для изображения, полученного с камеры и для срендеренного изображения
    cout << "psnr метрика main_part и BF_ct_part: " << Metrics::PSNR(camera_image, rendered_image1) << endl;
    cout << "psnr метрика main_part и BF_part: " << Metrics::PSNR(camera_image, rendered_image2) << endl;
    cout << "psnr метрика main_part и PCL_ct_part: " << Metrics::PSNR(camera_image, rendered_image3) << endl;
    cout << "psnr метрика main_part и PCL_part: " << Metrics::PSNR(camera_image, rendered_image4) << endl;

//    // Считаем The Blur Effect метрику для изображения с камеры
//    BlurEstimation estimater_camera_image(camera_image);
//    cout << "The Blur Effect метрика для изображения с камеры: " << estimater_camera_image.estimate() << endl;
//    // Считаем The Blur Effect метрику для срендеренного изображения
//    BlurEstimation estimater_rendered_image(rendered_image);
//    cout << "The Blur Effect метрика для срендеренного изображения: " << estimater_rendered_image.estimate() << endl;

    return 0;
}
