#include <iostream>
#include <opencv2/core/types.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string>

#include "Metrics/Metrics.h"
#include "Metrics/BlurEstimation.h"

using namespace std;

int main() {
    std::ofstream file_results("../../data/metrics_examples/metrics_results.txt");

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

    // -------------------------------------

    string camera_full_image_path_lidar2 = "../../data/metrics_examples/lidar2/main.jpg";
    string rendered_full_image_path_lidar2_1 = "../../data/metrics_examples/lidar2/r39_BF_ct.png";
    string rendered_full_image_path_lidar2_2 = "../../data/metrics_examples/lidar2/r39_BF.png";
    string rendered_full_image_path_lidar2_3 = "../../data/metrics_examples/lidar2/r39_PCL.png";
    string rendered_full_image_path_lidar2_4 = "../../data/metrics_examples/lidar2/r39_PCL_ct.png";

    string camera_full_image_path_lidar6 = "../../data/metrics_examples/lidar6/main.jpg";
    string rendered_full_image_path_lidar6_1 = "../../data/metrics_examples/lidar6/r376_BF_ct.png";
    string rendered_full_image_path_lidar6_2 = "../../data/metrics_examples/lidar6/r376_BF.png";
    string rendered_full_image_path_lidar6_3 = "../../data/metrics_examples/lidar6/r376_PCL.png";
    string rendered_full_image_path_lidar6_4 = "../../data/metrics_examples/lidar6/r376_PCL_ct.png";

    string camera_full_image_path_lidar10 = "../../data/metrics_examples/lidar10/main.jpg";
    string rendered_full_image_path_lidar10_1 = "../../data/metrics_examples/lidar10/r73_BF_ct.png";
    string rendered_full_image_path_lidar10_2 = "../../data/metrics_examples/lidar10/r73_BF.png";
    string rendered_full_image_path_lidar10_3 = "../../data/metrics_examples/lidar10/r73_PCL.png";
    string rendered_full_image_path_lidar10_4 = "../../data/metrics_examples/lidar10/r73_PCL_ct.png";

    string camera_full_image_path_lidar15 = "../../data/metrics_examples/lidar15/main.jpg";
    string rendered_full_image_path_lidar15_1 = "../../data/metrics_examples/lidar15/r173_BF_ct.png";
    string rendered_full_image_path_lidar15_2 = "../../data/metrics_examples/lidar15/r173_BF.png";
    string rendered_full_image_path_lidar15_3 = "../../data/metrics_examples/lidar15/r173_PCL.png";
    string rendered_full_image_path_lidar15_4 = "../../data/metrics_examples/lidar15/r173_PCL_ct.png";

    // -------------------------------------

    string rendered_full_path_lidar2_1 = "../../data/metrics_examples/lidar2/rfull_BF_ct.png";
    string rendered_full_path_lidar2_2 = "../../data/metrics_examples/lidar2/rfull_BF.png";
    string rendered_full_path_lidar2_3 = "../../data/metrics_examples/lidar2/rfull_PCL_ct.png";
    string rendered_full_path_lidar2_4 = "../../data/metrics_examples/lidar2/rfull_PCL.png";

    string rendered_full_path_lidar6_1 = "../../data/metrics_examples/lidar6/rfull_BF_ct.png";
    string rendered_full_path_lidar6_2 = "../../data/metrics_examples/lidar6/rfull_BF.png";
    string rendered_full_path_lidar6_3 = "../../data/metrics_examples/lidar6/rfull_PCL_ct.png";
    string rendered_full_path_lidar6_4 = "../../data/metrics_examples/lidar6/rfull_PCL.png";

    string rendered_full_path_lidar10_1 = "../../data/metrics_examples/lidar10/rfull_BF_ct.png";
    string rendered_full_path_lidar10_2 = "../../data/metrics_examples/lidar10/rfull_BF.png";
    string rendered_full_path_lidar10_3 = "../../data/metrics_examples/lidar10/rfull_PCL_ct.png";
    string rendered_full_path_lidar10_4 = "../../data/metrics_examples/lidar10/rfull_PCL.png";

    string rendered_full_path_lidar15_1 = "../../data/metrics_examples/lidar15/rfull_BF_ct.png";
    string rendered_full_path_lidar15_2 = "../../data/metrics_examples/lidar15/rfull_BF.png";
    string rendered_full_path_lidar15_3 = "../../data/metrics_examples/lidar15/rfull_PCL_ct.png";
    string rendered_full_path_lidar15_4 = "../../data/metrics_examples/lidar15/rfull_PCL.png";

    cv::Mat camera_image;
    cv::Mat rendered_image1, rendered_image2, rendered_image3, rendered_image4;

    camera_image = cv::imread(camera_image_path_lidar6, cv::IMREAD_COLOR);
    rendered_image1 = cv::imread(rendered_image_path_lidar6_1, cv::IMREAD_COLOR);
    rendered_image2 = cv::imread(rendered_image_path_lidar6_2, cv::IMREAD_COLOR);
    rendered_image3 = cv::imread(rendered_image_path_lidar6_3, cv::IMREAD_COLOR);
    rendered_image4 = cv::imread(rendered_image_path_lidar6_4, cv::IMREAD_COLOR);

    file_results << "Значения метрик для lidar6: \n";

    // Считаем ssim метрику для одного и того же изображения
    file_results << "ssim метрика для одного и того же изображения: " << Metrics::SSIM(camera_image, camera_image, 8)
                 << endl;
    // Считаем ssim метрику для изображения, полученного с камеры и для срендеренного изображения
    file_results << "ssim метрика main_part и BF_ct_part: " << Metrics::SSIM(camera_image, rendered_image1, 8) << endl;
    file_results << "ssim метрика main_part и BF_part: " << Metrics::SSIM(camera_image, rendered_image2, 8) << endl;
    file_results << "ssim метрика main_part и PCL_ct_part: " << Metrics::SSIM(camera_image, rendered_image3, 8) << endl;
    file_results << "ssim метрика main_part и PCL_part: " << Metrics::SSIM(camera_image, rendered_image4, 8) << endl;

    // Считаем psnr метрику для одного и того же изображения
    file_results << "psnr метрика для одного и того же изображения: " << Metrics::PSNR(camera_image, camera_image)
                 << endl;
    // Считаем psnr метрику для изображения, полученного с камеры и для срендеренного изображения
    file_results << "psnr метрика main_part и BF_ct_part: " << Metrics::PSNR(camera_image, rendered_image1) << endl;
    file_results << "psnr метрика main_part и BF_part: " << Metrics::PSNR(camera_image, rendered_image2) << endl;
    file_results << "psnr метрика main_part и PCL_ct_part: " << Metrics::PSNR(camera_image, rendered_image3) << endl;
    file_results << "psnr метрика main_part и PCL_part: " << Metrics::PSNR(camera_image, rendered_image4) << endl;

    camera_image.deallocate();
    rendered_image1.deallocate();
    rendered_image2.deallocate();
    rendered_image3.deallocate();
    rendered_image4.deallocate();

    camera_image = cv::imread(camera_image_path_lidar10, cv::IMREAD_COLOR);
    rendered_image1 = cv::imread(rendered_image_path_lidar10_1, cv::IMREAD_COLOR);
    rendered_image2 = cv::imread(rendered_image_path_lidar10_2, cv::IMREAD_COLOR);
    rendered_image3 = cv::imread(rendered_image_path_lidar10_3, cv::IMREAD_COLOR);
    rendered_image4 = cv::imread(rendered_image_path_lidar10_4, cv::IMREAD_COLOR);

    file_results << "Значения метрик для lidar10: \n";

    // Считаем ssim метрику для одного и того же изображения
    file_results << "ssim метрика для одного и того же изображения: " << Metrics::SSIM(camera_image, camera_image, 8)
                 << endl;
    // Считаем ssim метрику для изображения, полученного с камеры и для срендеренного изображения
    file_results << "ssim метрика main_part и BF_ct_part: " << Metrics::SSIM(camera_image, rendered_image1, 8) << endl;
    file_results << "ssim метрика main_part и BF_part: " << Metrics::SSIM(camera_image, rendered_image2, 8) << endl;
    file_results << "ssim метрика main_part и PCL_ct_part: " << Metrics::SSIM(camera_image, rendered_image3, 8) << endl;
    file_results << "ssim метрика main_part и PCL_part: " << Metrics::SSIM(camera_image, rendered_image4, 8) << endl;

    // Считаем psnr метрику для одного и того же изображения
    file_results << "psnr метрика для одного и того же изображения: " << Metrics::PSNR(camera_image, camera_image)
                 << endl;
    // Считаем psnr метрику для изображения, полученного с камеры и для срендеренного изображения
    file_results << "psnr метрика main_part и BF_ct_part: " << Metrics::PSNR(camera_image, rendered_image1) << endl;
    file_results << "psnr метрика main_part и BF_part: " << Metrics::PSNR(camera_image, rendered_image2) << endl;
    file_results << "psnr метрика main_part и PCL_ct_part: " << Metrics::PSNR(camera_image, rendered_image3) << endl;
    file_results << "psnr метрика main_part и PCL_part: " << Metrics::PSNR(camera_image, rendered_image4) << endl;

    camera_image.deallocate();
    rendered_image1.deallocate();
    rendered_image2.deallocate();
    rendered_image3.deallocate();
    rendered_image4.deallocate();

    camera_image = cv::imread(camera_image_path_lidar15, cv::IMREAD_COLOR);
    rendered_image1 = cv::imread(rendered_image_path_lidar15_1, cv::IMREAD_COLOR);
    rendered_image2 = cv::imread(rendered_image_path_lidar15_2, cv::IMREAD_COLOR);
    rendered_image3 = cv::imread(rendered_image_path_lidar15_3, cv::IMREAD_COLOR);
    rendered_image4 = cv::imread(rendered_image_path_lidar15_4, cv::IMREAD_COLOR);

    file_results << "Значения метрик для lidar15: \n";

    // Считаем ssim метрику для одного и того же изображения
    file_results << "ssim метрика для одного и того же изображения: " << Metrics::SSIM(camera_image, camera_image, 8)
                 << endl;
    // Считаем ssim метрику для изображения, полученного с камеры и для срендеренного изображения
    file_results << "ssim метрика main_part и BF_ct_part: " << Metrics::SSIM(camera_image, rendered_image1, 8) << endl;
    file_results << "ssim метрика main_part и BF_part: " << Metrics::SSIM(camera_image, rendered_image2, 8) << endl;
    file_results << "ssim метрика main_part и PCL_ct_part: " << Metrics::SSIM(camera_image, rendered_image3, 8) << endl;
    file_results << "ssim метрика main_part и PCL_part: " << Metrics::SSIM(camera_image, rendered_image4, 8) << endl;

    // Считаем psnr метрику для одного и того же изображения
    file_results << "psnr метрика для одного и того же изображения: " << Metrics::PSNR(camera_image, camera_image)
                 << endl;
    // Считаем psnr метрику для изображения, полученного с камеры и для срендеренного изображения
    file_results << "psnr метрика main_part и BF_ct_part: " << Metrics::PSNR(camera_image, rendered_image1) << endl;
    file_results << "psnr метрика main_part и BF_part: " << Metrics::PSNR(camera_image, rendered_image2) << endl;
    file_results << "psnr метрика main_part и PCL_ct_part: " << Metrics::PSNR(camera_image, rendered_image3) << endl;
    file_results << "psnr метрика main_part и PCL_part: " << Metrics::PSNR(camera_image, rendered_image4) << endl;

    camera_image.deallocate();
    rendered_image1.deallocate();
    rendered_image2.deallocate();
    rendered_image3.deallocate();
    rendered_image4.deallocate();

    // ---------------------------------------

    camera_image = cv::imread(camera_full_image_path_lidar6, cv::IMREAD_COLOR);
    rendered_image1 = cv::imread(rendered_full_image_path_lidar6_1, cv::IMREAD_COLOR);
    rendered_image2 = cv::imread(rendered_full_image_path_lidar6_2, cv::IMREAD_COLOR);
    rendered_image3 = cv::imread(rendered_full_image_path_lidar6_3, cv::IMREAD_COLOR);
    rendered_image4 = cv::imread(rendered_full_image_path_lidar6_4, cv::IMREAD_COLOR);

    file_results << "Значения метрик для lidar6: \n";

    // Считаем ssim метрику для одного и того же изображения
    file_results << "ssim метрика для одного и того же изображения: " << Metrics::SSIM(camera_image, camera_image, 8)
                 << endl;
    // Считаем ssim метрику для изображения, полученного с камеры и для срендеренного изображения
    file_results << "ssim метрика main_part и BF_ct: " << Metrics::SSIM(camera_image, rendered_image1, 8) << endl;
    file_results << "ssim метрика main_part и BF: " << Metrics::SSIM(camera_image, rendered_image2, 8) << endl;
    file_results << "ssim метрика main_part и PCL_ct: " << Metrics::SSIM(camera_image, rendered_image3, 8) << endl;
    file_results << "ssim метрика main_part и PCL: " << Metrics::SSIM(camera_image, rendered_image4, 8) << endl;

    // Считаем psnr метрику для одного и того же изображения
    file_results << "psnr метрика для одного и того же изображения: " << Metrics::PSNR(camera_image, camera_image)
                 << endl;
    // Считаем psnr метрику для изображения, полученного с камеры и для срендеренного изображения
    file_results << "psnr метрика main_part и BF_ct: " << Metrics::PSNR(camera_image, rendered_image1) << endl;
    file_results << "psnr метрика main_part и BF: " << Metrics::PSNR(camera_image, rendered_image2) << endl;
    file_results << "psnr метрика main_part и PCL_ct: " << Metrics::PSNR(camera_image, rendered_image3) << endl;
    file_results << "psnr метрика main_part и PCL: " << Metrics::PSNR(camera_image, rendered_image4) << endl;

    camera_image.deallocate();
    rendered_image1.deallocate();
    rendered_image2.deallocate();
    rendered_image3.deallocate();
    rendered_image4.deallocate();

    camera_image = cv::imread(camera_full_image_path_lidar10, cv::IMREAD_COLOR);
    rendered_image1 = cv::imread(rendered_full_image_path_lidar10_1, cv::IMREAD_COLOR);
    rendered_image2 = cv::imread(rendered_full_image_path_lidar10_2, cv::IMREAD_COLOR);
    rendered_image3 = cv::imread(rendered_full_image_path_lidar10_3, cv::IMREAD_COLOR);
    rendered_image4 = cv::imread(rendered_full_image_path_lidar10_4, cv::IMREAD_COLOR);

    file_results << "Значения метрик для lidar10: \n";

    // Считаем ssim метрику для одного и того же изображения
    file_results << "ssim метрика для одного и того же изображения: " << Metrics::SSIM(camera_image, camera_image, 8)
                 << endl;
    // Считаем ssim метрику для изображения, полученного с камеры и для срендеренного изображения
    file_results << "ssim метрика main_part и BF_ct: " << Metrics::SSIM(camera_image, rendered_image1, 8) << endl;
    file_results << "ssim метрика main_part и BF: " << Metrics::SSIM(camera_image, rendered_image2, 8) << endl;
    file_results << "ssim метрика main_part и PCL_ct: " << Metrics::SSIM(camera_image, rendered_image3, 8) << endl;
    file_results << "ssim метрика main_part и PCL: " << Metrics::SSIM(camera_image, rendered_image4, 8) << endl;

    // Считаем psnr метрику для одного и того же изображения
    file_results << "psnr метрика для одного и того же изображения: " << Metrics::PSNR(camera_image, camera_image)
                 << endl;
    // Считаем psnr метрику для изображения, полученного с камеры и для срендеренного изображения
    file_results << "psnr метрика main_part и BF_ct: " << Metrics::PSNR(camera_image, rendered_image1) << endl;
    file_results << "psnr метрика main_part и BF: " << Metrics::PSNR(camera_image, rendered_image2) << endl;
    file_results << "psnr метрика main_part и PCL_ct: " << Metrics::PSNR(camera_image, rendered_image3) << endl;
    file_results << "psnr метрика main_part и PCL: " << Metrics::PSNR(camera_image, rendered_image4) << endl;

    camera_image.deallocate();
    rendered_image1.deallocate();
    rendered_image2.deallocate();
    rendered_image3.deallocate();
    rendered_image4.deallocate();

    camera_image = cv::imread(camera_full_image_path_lidar15, cv::IMREAD_COLOR);
    rendered_image1 = cv::imread(rendered_full_image_path_lidar15_1, cv::IMREAD_COLOR);
    rendered_image2 = cv::imread(rendered_full_image_path_lidar15_2, cv::IMREAD_COLOR);
    rendered_image3 = cv::imread(rendered_full_image_path_lidar15_3, cv::IMREAD_COLOR);
    rendered_image4 = cv::imread(rendered_full_image_path_lidar15_4, cv::IMREAD_COLOR);

    file_results << "Значения метрик для lidar15: \n";

    // Считаем ssim метрику для одного и того же изображения
    file_results << "ssim метрика для одного и того же изображения: " << Metrics::SSIM(camera_image, camera_image, 8)
                 << endl;
    // Считаем ssim метрику для изображения, полученного с камеры и для срендеренного изображения
    file_results << "ssim метрика main_part и BF_ct: " << Metrics::SSIM(camera_image, rendered_image1, 8) << endl;
    file_results << "ssim метрика main_part и BF: " << Metrics::SSIM(camera_image, rendered_image2, 8) << endl;
    file_results << "ssim метрика main_part и PCL_ct: " << Metrics::SSIM(camera_image, rendered_image3, 8) << endl;
    file_results << "ssim метрика main_part и PCL: " << Metrics::SSIM(camera_image, rendered_image4, 8) << endl;

    // Считаем psnr метрику для одного и того же изображения
    file_results << "psnr метрика для одного и того же изображения: " << Metrics::PSNR(camera_image, camera_image)
                 << endl;
    // Считаем psnr метрику для изображения, полученного с камеры и для срендеренного изображения
    file_results << "psnr метрика main_part и BF_ct: " << Metrics::PSNR(camera_image, rendered_image1) << endl;
    file_results << "psnr метрика main_part и BF: " << Metrics::PSNR(camera_image, rendered_image2) << endl;
    file_results << "psnr метрика main_part и PCL_ct: " << Metrics::PSNR(camera_image, rendered_image3) << endl;
    file_results << "psnr метрика main_part и PCL: " << Metrics::PSNR(camera_image, rendered_image4) << endl;

    camera_image.deallocate();
    rendered_image1.deallocate();
    rendered_image2.deallocate();
    rendered_image3.deallocate();
    rendered_image4.deallocate();

    // ---------------------------------------

    camera_image = cv::imread(camera_full_image_path_lidar2, cv::IMREAD_COLOR);
    rendered_image1 = cv::imread(rendered_full_image_path_lidar2_1, cv::IMREAD_COLOR);
    rendered_image2 = cv::imread(rendered_full_image_path_lidar2_2, cv::IMREAD_COLOR);
    rendered_image3 = cv::imread(rendered_full_image_path_lidar2_3, cv::IMREAD_COLOR);
    rendered_image4 = cv::imread(rendered_full_image_path_lidar2_4, cv::IMREAD_COLOR);

    file_results << "Значения метрики Крита для lidar2: \n";
    // Считаем метрику Крита для исходного изображения
    BlurEstimation estimater_lidar2_image(camera_image);
    file_results << "метрику Крита для main: " << estimater_lidar2_image.estimate() << endl;
    // Считаем метрику Крита для срендеренного изображения
    BlurEstimation estimater_rendered_lidar2_image1(rendered_image1);
    BlurEstimation estimater_rendered_lidar2_image2(rendered_image2);
    BlurEstimation estimater_rendered_lidar2_image3(rendered_image3);
    BlurEstimation estimater_rendered_lidar2_image4(rendered_image4);
    file_results << "метрику Крита для BF_ct: " << estimater_rendered_lidar2_image1.estimate() << endl;
    file_results << "метрику Крита для BF: " << estimater_rendered_lidar2_image2.estimate() << endl;
    file_results << "метрику Крита для PCL_ct: " << estimater_rendered_lidar2_image3.estimate() << endl;
    file_results << "метрику Крита для PCL: " << estimater_rendered_lidar2_image4.estimate() << endl;

    camera_image.deallocate();
    rendered_image1.deallocate();
    rendered_image2.deallocate();
    rendered_image3.deallocate();
    rendered_image4.deallocate();

    camera_image = cv::imread(camera_full_image_path_lidar6, cv::IMREAD_COLOR);
    rendered_image1 = cv::imread(rendered_full_image_path_lidar6_1, cv::IMREAD_COLOR);
    rendered_image2 = cv::imread(rendered_full_image_path_lidar6_2, cv::IMREAD_COLOR);
    rendered_image3 = cv::imread(rendered_full_image_path_lidar6_3, cv::IMREAD_COLOR);
    rendered_image4 = cv::imread(rendered_full_image_path_lidar6_4, cv::IMREAD_COLOR);

    file_results << "Значения метрики Крита для lidar6: \n";
    // Считаем метрику Крита для исходного изображения
    BlurEstimation estimater_lidar6_image(camera_image);
    file_results << "метрику Крита для main: " << estimater_lidar6_image.estimate() << endl;
    // Считаем метрику Крита для срендеренного изображения
    BlurEstimation estimater_rendered_lidar6_image1(rendered_image1);
    BlurEstimation estimater_rendered_lidar6_image2(rendered_image2);
    BlurEstimation estimater_rendered_lidar6_image3(rendered_image3);
    BlurEstimation estimater_rendered_lidar6_image4(rendered_image4);
    file_results << "метрику Крита для BF_ct: " << estimater_rendered_lidar6_image1.estimate() << endl;
    file_results << "метрику Крита для BF: " << estimater_rendered_lidar6_image2.estimate() << endl;
    file_results << "метрику Крита для PCL_ct: " << estimater_rendered_lidar6_image3.estimate() << endl;
    file_results << "метрику Крита для PCL: " << estimater_rendered_lidar6_image4.estimate() << endl;

    camera_image.deallocate();
    rendered_image1.deallocate();
    rendered_image2.deallocate();
    rendered_image3.deallocate();
    rendered_image4.deallocate();

    camera_image = cv::imread(camera_full_image_path_lidar10, cv::IMREAD_COLOR);
    rendered_image1 = cv::imread(rendered_full_image_path_lidar10_1, cv::IMREAD_COLOR);
    rendered_image2 = cv::imread(rendered_full_image_path_lidar10_2, cv::IMREAD_COLOR);
    rendered_image3 = cv::imread(rendered_full_image_path_lidar10_3, cv::IMREAD_COLOR);
    rendered_image4 = cv::imread(rendered_full_image_path_lidar10_4, cv::IMREAD_COLOR);

    file_results << "Значения метрики Крита для lidar10: \n";
    // Считаем метрику Крита для исходного изображения
    BlurEstimation estimater_lidar10_image(camera_image);
    file_results << "метрику Крита для main: " << estimater_lidar10_image.estimate() << endl;
    // Считаем метрику Крита для срендеренного изображения
    BlurEstimation estimater_rendered_lidar10_image1(rendered_image1);
    BlurEstimation estimater_rendered_lidar10_image2(rendered_image2);
    BlurEstimation estimater_rendered_lidar10_image3(rendered_image3);
    BlurEstimation estimater_rendered_lidar10_image4(rendered_image4);
    file_results << "метрику Крита для BF_ct: " << estimater_rendered_lidar10_image1.estimate() << endl;
    file_results << "метрику Крита для BF: " << estimater_rendered_lidar10_image2.estimate() << endl;
    file_results << "метрику Крита для PCL_ct: " << estimater_rendered_lidar10_image3.estimate() << endl;
    file_results << "метрику Крита для PCL: " << estimater_rendered_lidar10_image4.estimate() << endl;

    camera_image.deallocate();
    rendered_image1.deallocate();
    rendered_image2.deallocate();
    rendered_image3.deallocate();
    rendered_image4.deallocate();

    camera_image = cv::imread(camera_full_image_path_lidar15, cv::IMREAD_COLOR);
    rendered_image1 = cv::imread(rendered_full_image_path_lidar15_1, cv::IMREAD_COLOR);
    rendered_image2 = cv::imread(rendered_full_image_path_lidar15_2, cv::IMREAD_COLOR);
    rendered_image3 = cv::imread(rendered_full_image_path_lidar15_3, cv::IMREAD_COLOR);
    rendered_image4 = cv::imread(rendered_full_image_path_lidar15_4, cv::IMREAD_COLOR);

    file_results << "Значения метрики Крита для lidar15: \n";
    // Считаем метрику Крита для исходного изображения
    BlurEstimation estimater_lidar15_image(camera_image);
    file_results << "метрику Крита для main: " << estimater_lidar15_image.estimate() << endl;
    // Считаем метрику Крита для срендеренного изображения
    BlurEstimation estimater_rendered_lidar15_image1(rendered_image1);
    BlurEstimation estimater_rendered_lidar15_image2(rendered_image2);
    BlurEstimation estimater_rendered_lidar15_image3(rendered_image3);
    BlurEstimation estimater_rendered_lidar15_image4(rendered_image4);
    file_results << "метрику Крита для BF_ct: " << estimater_rendered_lidar15_image1.estimate() << endl;
    file_results << "метрику Крита для BF: " << estimater_rendered_lidar15_image2.estimate() << endl;
    file_results << "метрику Крита для PCL_ct: " << estimater_rendered_lidar15_image3.estimate() << endl;
    file_results << "метрику Крита для PCL: " << estimater_rendered_lidar15_image4.estimate() << endl;

    camera_image.deallocate();
    rendered_image1.deallocate();
    rendered_image2.deallocate();
    rendered_image3.deallocate();
    rendered_image4.deallocate();

    file_results.close();

    return 0;
}
