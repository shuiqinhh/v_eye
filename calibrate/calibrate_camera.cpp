#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <opencv2/opencv.hpp>

#include "tools/img_tools.hpp"

const std::string keys =
  "{help h usage ? |                          | 输出命令行参数说明}"
  "{config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{@input-folder  | assets/img_with_q        | 输入文件夹路径   }";

std::vector<cv::Point3f> centers_3d(const cv::Size & pattern_size, const float center_distance)
{
  std::vector<cv::Point3f> centers_3d;

  for (int i = 0; i < pattern_size.height; i++)
    for (int j = 0; j < pattern_size.width; j++)
      centers_3d.push_back({j * center_distance, i * center_distance, 0});

  return centers_3d;
}

void load(
  const std::string & input_folder, const std::string & config_path, cv::Size & img_size,
  std::vector<std::vector<cv::Point3f>> & obj_points,
  std::vector<std::vector<cv::Point2f>> & img_points)
{
  // 读取yaml参数
  auto yaml = YAML::LoadFile(config_path);
  auto pattern_cols = yaml["pattern_cols"].as<int>();
  auto pattern_rows = yaml["pattern_rows"].as<int>();
  auto center_distance_mm = yaml["center_distance_mm"].as<double>();
  cv::Size pattern_size(pattern_cols, pattern_rows);

  for (int i = 1; true; i++) {
    // 读取图片
    auto img_path = fmt::format("{}/{}.jpg", input_folder, i);
    auto img = cv::imread(img_path);
    if (img.empty()) break;

    // 设置图片尺寸
    img_size = img.size();

    // 识别标定板
    std::vector<cv::Point2f> centers_2d;
    auto success = cv::findCirclesGrid(img, pattern_size, centers_2d, cv::CALIB_CB_SYMMETRIC_GRID);

    // 显示识别结果
    auto drawing = img.clone();
    cv::drawChessboardCorners(drawing, pattern_size, centers_2d, success);
    cv::resize(drawing, drawing, {}, 0.5, 0.5);  // 缩小图片尺寸便于显示完全
    cv::imshow("Press any to continue", drawing);
    cv::waitKey(0);

    // 输出识别结果
    fmt::print("[{}] {}\n", success ? "success" : "failure", img_path);
    if (!success) continue;

    // 记录所需的数据
    img_points.emplace_back(centers_2d);
    obj_points.emplace_back(centers_3d(pattern_size, center_distance_mm));
  }
}

// 将 img_size 作为参数传入，并且添加了默认的保存路径参数
void print_yaml(
  const cv::Mat & camera_matrix, const cv::Mat & distort_coeffs, double error,
  const cv::Size & img_size, const std::string & save_path = "../params.yaml")
{
  // 1. 从 cv::Mat 中解构出独立的相机内参和畸变参数
  double fx = camera_matrix.at<double>(0, 0);
  double fy = camera_matrix.at<double>(1, 1);
  double cx = camera_matrix.at<double>(0, 2);
  double cy = camera_matrix.at<double>(1, 2);

  double k1 = distort_coeffs.at<double>(0);
  double k2 = distort_coeffs.at<double>(1);
  double p1 = distort_coeffs.at<double>(2);
  double p2 = distort_coeffs.at<double>(3);
  // 由于你在标定时使用了 cv::CALIB_FIX_K3，所以 k3 为 0
  double k3 = 0.0;

  YAML::Emitter result;
  result << YAML::BeginMap;
  result << YAML::Comment(fmt::format("Reprojection Error: {:.4f}px", error));

  // 2. ORB-SLAM3 相机基础参数定义
  result << YAML::Key << "File.version" << YAML::Value << "1.0";
  result << YAML::Key << "Camera.type" << YAML::Value << "PinHole";
  result << YAML::Key << "Camera.setup" << YAML::Value << "monocular";

  // 相机内参
  result << YAML::Key << "Camera1.fx" << YAML::Value << fx;
  result << YAML::Key << "Camera1.fy" << YAML::Value << fy;
  result << YAML::Key << "Camera1.cx" << YAML::Value << cx;
  result << YAML::Key << "Camera1.cy" << YAML::Value << cy;

  // 畸变系数
  result << YAML::Key << "Camera1.k1" << YAML::Value << k1;
  result << YAML::Key << "Camera1.k2" << YAML::Value << k2;
  result << YAML::Key << "Camera1.p1" << YAML::Value << p1;
  result << YAML::Key << "Camera1.p2" << YAML::Value << p2;
  result << YAML::Key << "Camera1.k3" << YAML::Value << k3;

  // 图像尺寸与帧率
  result << YAML::Key << "Camera.width" << YAML::Value << img_size.width;
  result << YAML::Key << "Camera.height" << YAML::Value << img_size.height;
  result << YAML::Key << "Camera.fps" << YAML::Value << 30;  // 若你的相机不是30帧，请修改
  result << YAML::Key << "Camera.RGB" << YAML::Value << 1;

  // 3. ORB-SLAM3 特征提取必需参数 (给出一组标准默认值)
  result << YAML::Key << "ORBextractor.nFeatures" << YAML::Value << 1000;
  result << YAML::Key << "ORBextractor.scaleFactor" << YAML::Value << 1.2;
  result << YAML::Key << "ORBextractor.nLevels" << YAML::Value << 8;
  result << YAML::Key << "ORBextractor.iniThFAST" << YAML::Value << 20;
  result << YAML::Key << "ORBextractor.minThFAST" << YAML::Value << 7;

  // 4. Viewer 窗口显示参数 (ORB-SLAM3 运行时生成 GUI 必需)
  result << YAML::Key << "Viewer.KeyFrameSize" << YAML::Value << 0.05;
  result << YAML::Key << "Viewer.KeyFrameLineWidth" << YAML::Value << 1;
  result << YAML::Key << "Viewer.GraphLineWidth" << YAML::Value << 0.9;
  result << YAML::Key << "Viewer.PointSize" << YAML::Value << 2;
  result << YAML::Key << "Viewer.CameraSize" << YAML::Value << 0.08;
  result << YAML::Key << "Viewer.CameraLineWidth" << YAML::Value << 3;
  result << YAML::Key << "Viewer.ViewpointX" << YAML::Value << 0;
  result << YAML::Key << "Viewer.ViewpointY" << YAML::Value << -0.7;
  result << YAML::Key << "Viewer.ViewpointZ" << YAML::Value << -1.8;
  result << YAML::Key << "Viewer.ViewpointF" << YAML::Value << 500;

  result << YAML::EndMap;

  // 5. 组装最终字符串，首行强行插入 %YAML:1.0 供 OpenCV 识别
  std::string final_yaml_str = "%YAML:1.0\n" + std::string(result.c_str());

  // 打印输出到终端
  fmt::print("\n{}\n", final_yaml_str);

  // 自动将配置保存为本地文件
  std::ofstream fout(save_path);
  if (fout.is_open()) {
    fout << final_yaml_str;
    fout.close();
    fmt::print("ORB-SLAM3 配置文件已成功保存至: {}\n", save_path);
  } else {
    fmt::print(stderr, "文件保存失败: {}\n", save_path);
  }
}

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto input_folder = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");

  // 从输入文件夹中加载标定所需的数据
  cv::Size img_size;
  std::vector<std::vector<cv::Point3f>> obj_points;
  std::vector<std::vector<cv::Point2f>> img_points;
  load(input_folder, config_path, img_size, obj_points, img_points);

  // 相机标定
  cv::Mat camera_matrix, distort_coeffs;
  std::vector<cv::Mat> rvecs, tvecs;
  auto criteria = cv::TermCriteria(
    cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100,
    DBL_EPSILON);  // 默认迭代次数(30)有时会导致结果发散，故设为100
  cv::calibrateCamera(
    obj_points, img_points, img_size, camera_matrix, distort_coeffs, rvecs, tvecs, cv::CALIB_FIX_K3,
    criteria);  // 由于视场角较小，不需要考虑k3

  // 重投影误差
  double error_sum = 0;
  size_t total_points = 0;
  for (size_t i = 0; i < obj_points.size(); i++) {
    std::vector<cv::Point2f> reprojected_points;
    cv::projectPoints(
      obj_points[i], rvecs[i], tvecs[i], camera_matrix, distort_coeffs, reprojected_points);

    total_points += reprojected_points.size();
    for (size_t j = 0; j < reprojected_points.size(); j++)
      error_sum += cv::norm(img_points[i][j] - reprojected_points[j]);
  }
  auto error = error_sum / total_points;

  // 输出yaml
  // 输出生成并保存兼容 ORB-SLAM3 的 yaml (新代码)
  print_yaml(camera_matrix, distort_coeffs, error, img_size, "../params.yaml");
}
