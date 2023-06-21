#include <cmath>
#include <cstdio>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <sys/stat.h>
#include <dirent.h>
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <gflags/gflags.h>
#include <string>

// TODO: Consider forcing the user to enter all the fields incase he/she may have forgotten to enter one. Perhaps tell user which fields he did not enter as a warning.

// Definitions for the gflags
DEFINE_double(fx, 900, "Used to set an initial estimate for the fx, camera intrinsic");
DEFINE_double(fy, 900, "Used to set an initial estimate for the fy, camera intrinsic");
DEFINE_double(cx, 320, "Used to set an initial estimate for the cx, camera intrinsic");
DEFINE_double(cy, 240, "Used to set an initial estimate for the cy, camera intrinsic");
DEFINE_uint64(internal_corners_x, 7, "Used to set the number of internal number of corners for the columns of the calibration board");
DEFINE_uint64(internal_corners_y, 6, "Used to set the number of internal number of corners for the rows of the calibration board");
DEFINE_double(block_dimension, 0.02, "Used to set the dimension of a single side of a block from the calibration board in meters");
DEFINE_string(read_directory, "../Calibration_images_numbered/", "Used to set the directory of the folder containing the calibration images");
DEFINE_string(write_directory, "../Processed_calibration_images_numbered/", "Used to set the directory of the folder to write the processed calibration images to");
DEFINE_string(read_image, "image%02d.jpg", "Used to set the naming convention of the images to be read from disk");
DEFINE_string(write_image, "image%02d.jpg", "Used to set the naming convention of the images to be written to disk");


struct ReProjectionResidual
{ 
  ReProjectionResidual(const double *pixel_points, const double *single_calibration_board_point)
  {
    // Initialise the pixel points
    observed_pixel_points[0] = pixel_points[0]; // u
    observed_pixel_points[1] = pixel_points[1]; // v

    // Initialise the calibration board point
    calibration_board_point[0] = single_calibration_board_point[0]; // X
    calibration_board_point[1] = single_calibration_board_point[1]; // Y
    calibration_board_point[2] = single_calibration_board_point[2]; // Z
  }

  template <typename T>
  bool operator()(const T* const cam_T_board, const T* const camera_intrinsics, T* residuals)
  const
  {
    // compute projective coordinates: p = RX + t.
    // cam_T_board[0, 1, 2]: axis-angle
    // cam_T_board[3, 4, 5]: translation
    const T R_angle_axis[3] = {T(cam_T_board[0]), T(cam_T_board[1]), T(cam_T_board[2])};
    const T point[3] = {T(calibration_board_point[0]), T(calibration_board_point[1]), T(calibration_board_point[2])};
    T p[3];

    // AngleAxisRotatePoint used to rotate the calibration board point about the axis of rotation which is set 
    // as the R component of the camera extrinsic matric, after the rotation matrix to angle axis conversion
    ceres::AngleAxisRotatePoint(R_angle_axis, point, p);

    // AngleAxisRotatePoint gives the "RX" therefore it must be translated by "t" (from camera extrinsics) to give "p".
    p[0] += cam_T_board[3]; // X component of camera to calibration point
    p[1] += cam_T_board[4]; // Y component of camera to calibration point
    p[2] += cam_T_board[5]; // Z component of camera to calibration point

    // The projected pixel coordinates are computed here.
    T up = p[0] / p[2];
    T vp = p[1] / p[2];

    // The projected uv pixel values are calculated here based on the current camera intrinsics and extrinsics
    T projected_u = up * camera_intrinsics[0] + camera_intrinsics[2];
    T projected_v = vp * camera_intrinsics[1] + camera_intrinsics[3];

    // The residuals are calculated here
    residuals[0] = projected_u - T(observed_pixel_points[0]);
    residuals[1] = projected_v - T(observed_pixel_points[1]);

    return true;
  }

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction *Create(const double *pixel_points, const double *calibration_board_point)
  {
    return (new ceres::AutoDiffCostFunction<ReProjectionResidual, 2, 6, 4>(
      new ReProjectionResidual(pixel_points, calibration_board_point)));
  }

  // Declare struct variables
  private:
    double observed_pixel_points[2];
    double calibration_board_point[3];
};

int main(int argc, char **argv)
{
  gflags::ParseCommandLineFlags(&argc, &argv, false);

  // Definitions for the initial estimates for the parameters desired to be optimised.
  const double kInitial_fx = FLAGS_fx;
  const double kInitial_fy = FLAGS_fy;
  const double kInitial_cx = FLAGS_cx;
  const double kInitial_cy = FLAGS_cy;

  // Definitions for the dimensions of the calibration board.
  const int kNumber_of_internal_corners_x = FLAGS_internal_corners_x;
  const int kNumber_of_internal_corners_y = FLAGS_internal_corners_y;
  const double kBlock_dimension = FLAGS_block_dimension;

  // Naming convention for the images to be written to and from the disk.
  std::string image_name_read = FLAGS_read_image;
  std::string image_name_write = FLAGS_write_image;

  // Point to the folder to read images
  cv::VideoCapture cap(FLAGS_read_directory + FLAGS_read_image);

  // Point to the folder to write images
  std::string directory = FLAGS_write_directory;

  ///////////////////////////////////// Input data processing code implementation begin here /////////////////////////////////////

  unsigned int num_of_calibration_points = kNumber_of_internal_corners_x * kNumber_of_internal_corners_y;
  unsigned int num_of_images = 0;
  unsigned int total_chessboards_detected = 0;
 
  cv::Size patternsize = cv::Size(kNumber_of_internal_corners_x, kNumber_of_internal_corners_y); // (width, height) or (columns, rows) or (X, Y)
  std::vector<cv::Mat> images;
  std::vector<std::vector<cv::Point2f>> chessboard_corners;

  // This loop goes into the folder defined by FLAGS_read_directory and reads the images one by one.
  // The files MUST be of the same type and have the same name.
  // The files MUST have the same name and have a consecutive numbering scheme
  // For example: image01.jpg, image02.jpg, image03.jpg ...
  while(cap.isOpened())
  {
    // The image is read into the OpenCV matrix vector here
    cv::Mat img;
    cap.read(img);
    if(img.empty())
    {
      break;
    }
    
    // The recently read image is processed here
    std::vector<cv::Point2f> corners; //This will be filled by the detected corners
    bool chessboard_found = cv::findChessboardCorners(img, patternsize, corners);

    if(chessboard_found)
    {
      total_chessboards_detected++;
      images.push_back(img); // Only store the images where the Chessboards were found.
      chessboard_corners.push_back(corners); // Store the calibration points from the image but only for the chessboards that were found.
    }
    
    num_of_images++;
  }

  std::cout << "The total number of images found: " << num_of_images << "\r\n";
  std::cout << "The number of images with Chess boards detected: " << total_chessboards_detected << "\r\n\n";

  // The calibration board is generated here
  std::vector<std::array<double, 3>> true_calibration_board_points;
 
  for (int y = 0; y != kNumber_of_internal_corners_y; y++)
  {
    for (int x = 0; x != kNumber_of_internal_corners_x; x++)
    {
      std::array<double, 3> true_calibration_board_points_builder;

      true_calibration_board_points_builder[0] = x * kBlock_dimension;
      true_calibration_board_points_builder[1] = y * kBlock_dimension;
      true_calibration_board_points_builder[2] = 0 * kBlock_dimension;

      true_calibration_board_points.push_back(true_calibration_board_points_builder);
    }
  }

  // The solvePnP function is used here to determine the initial estimates for the cam_T_board for each image.
  double initial_estimates[3][3] = {{kInitial_fx, 0, kInitial_cx}, {0, kInitial_fy, kInitial_cy}, {0, 0, 1} };
  cv::Mat initial_estimates_matrix = cv::Mat(3, 3, CV_64FC1, initial_estimates);
  std::vector<std::array<double, 6>> cam_T_board;
  cv::Mat cam_r_board;
  cv::Mat cam_t_board;
  
  // Convert the true_calibration_board_points into an opencv true_calibration_board_points_matrix to be used with solvePnP
  cv::Mat true_calibration_board_points_matrix(num_of_calibration_points, 3, CV_64FC1);

  for (int y = 0; y != true_calibration_board_points_matrix.rows; y++)
  {
    for (int x = 0; x != true_calibration_board_points_matrix.cols; x++)
    {
      true_calibration_board_points_matrix.at<double>(y, x) = true_calibration_board_points.at(y).at(x);
    }
  }
  
  for (int z = 0; z != total_chessboards_detected; z++)
  {
    cv::solvePnP(true_calibration_board_points_matrix, chessboard_corners[z], initial_estimates_matrix, cv::Mat(4,1,CV_64FC1,cv::Scalar(0)), cam_r_board, cam_t_board, false);

    std::array<double, 6> cam_T_board_builder;

    cam_T_board_builder[0] = cam_r_board.at<double>(0, 0); // axis-angle x
    cam_T_board_builder[1] = cam_r_board.at<double>(0, 1); // axis-angle y
    cam_T_board_builder[2] = cam_r_board.at<double>(0, 2); // axis-angle z
    cam_T_board_builder[3] = cam_t_board.at<double>(0, 0); // t1
    cam_T_board_builder[4] = cam_t_board.at<double>(0, 1); // t2
    cam_T_board_builder[5] = cam_t_board.at<double>(0, 2); // t3

    cam_T_board.push_back(cam_T_board_builder); 
  }

  ///////////////////////////////////// Optimisation code begin here /////////////////////////////////////

  // Set the initial values for the mutable parameters. 
  double camera_intrinsics[4] = {kInitial_fx, kInitial_fy, kInitial_cx, kInitial_cy};

  // Begin building the problem
  ceres::Problem problem;

  for (int z = 0; z != chessboard_corners.size(); z++)
  {
    for (int i = 0; i != chessboard_corners[z].size(); i++)
    {
      double calibration_board_point[3] = {true_calibration_board_points[i][0], true_calibration_board_points[i][1], true_calibration_board_points[i][2]}; //This is the calibration point of the format: X,Y,Z
      double image_pixels[2] = {chessboard_corners[z][i].x, chessboard_corners[z][i].y}; //This is the image point of the format: u, v.

      ceres::CostFunction* cost_function = ReProjectionResidual::Create(image_pixels, calibration_board_point);
      problem.AddResidualBlock(cost_function, NULL, &cam_T_board[z][0], camera_intrinsics);
    }
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 10000;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << "\r\n";

  // Camera Intrinsics results
  std::cout << "CAMERA INTRINSICS RESULTS: \r\n\n";
  std::cout << "Initial fx: " << kInitial_fx << "\r\n";
  std::cout << "Final   fx: " << camera_intrinsics[0]  << "\r\n\n";

  std::cout << "Initial fy: " << kInitial_fy << "\r\n";
  std::cout << "Final   fy: " << camera_intrinsics[1]  << "\r\n\n";

  std::cout << "Initial cx: " << kInitial_cx << "\r\n";
  std::cout << "Final   cx: " << camera_intrinsics[2]  << "\r\n\n";

  std::cout << "Initial cy: " << kInitial_cy << "\r\n";
  std::cout << "Final   cy: " << camera_intrinsics[3]  << "\r\n\n";

  // Camera Extrinsics results
  std::cout << "CAMERA EXTRINSICS RESULTS:  \r\n\n";
  std::cout << "Format of results: axis-angle x, axis-angle y, axis-angle z, t1, t2, t3.  \r\n";
  for (int z = 0; z != total_chessboards_detected; z++)
  {
    std::cout << "Camera extrinsics for image " << z << ": ";
    std::cout << cam_T_board[z][0] << ", ";
    std::cout << cam_T_board[z][1] << ", ";
    std::cout << cam_T_board[z][2] << ", ";
    std::cout << cam_T_board[z][3] << ", ";
    std::cout << cam_T_board[z][4] << ", ";
    std::cout << cam_T_board[z][5]  << " \r\n";
  }

  ///////////////////////////////////// Code for analysing optimised camera intrinsics and extrinsics begin here /////////////////////////////////////

  // Create the camera intrinsics matrix using the optimised values and convert it to an opencv matrix
  double optimised_camera_intrinsics[3][3] = {{camera_intrinsics[0], 0, camera_intrinsics[2]}, {0, camera_intrinsics[1], camera_intrinsics[3]}, {0, 0, 1} };
  cv::Mat optimised_camera_intrinsics_matrix = cv::Mat(3, 3, CV_64FC1, optimised_camera_intrinsics);

  // Create the 3D vector of cam_T_boards matrices using the optimised values
  std::vector<std::array<std::array<double, 4>, 3>> optimised_cam_T_board;

  for (int z = 0; z != total_chessboards_detected; z++)
  {
    std::array<std::array<double, 4>, 3> optimised_cam_T_board_builder;

    double angle_axis[3] = {cam_T_board[z][0], cam_T_board[z][1],  cam_T_board[z][2]};
    double rotation_matrix[9];
    ceres::AngleAxisToRotationMatrix(&angle_axis[0], &rotation_matrix[0]);

    optimised_cam_T_board_builder[0][0] = rotation_matrix[0];
    optimised_cam_T_board_builder[1][0] = rotation_matrix[1];
    optimised_cam_T_board_builder[2][0] = rotation_matrix[2];
    optimised_cam_T_board_builder[0][1] = rotation_matrix[3];
    optimised_cam_T_board_builder[1][1] = rotation_matrix[4];
    optimised_cam_T_board_builder[2][1] = rotation_matrix[5];
    optimised_cam_T_board_builder[0][2] = rotation_matrix[6];
    optimised_cam_T_board_builder[1][2] = rotation_matrix[7];
    optimised_cam_T_board_builder[2][2] = rotation_matrix[8]; 
    optimised_cam_T_board_builder[0][3] = cam_T_board[z][3];
    optimised_cam_T_board_builder[1][3] = cam_T_board[z][4];
    optimised_cam_T_board_builder[2][3] = cam_T_board[z][5];

    // The cam_T_board matrix is stored here
    optimised_cam_T_board.push_back(optimised_cam_T_board_builder);
  }

  // Convert the 3D vector of cam_T_boards into a vector of opencv cam_T_board matrices
  std::vector<cv::Mat> optimised_cam_T_board_matrix;

  for (int z = 0; z != total_chessboards_detected; z++)
  {
    // Create a temporary matrix to build the opencv matrix for a camera extrinsic.
    cv::Mat optimised_cam_T_board_matrix_builder(3, 4, CV_64FC1);

    // Build the temporary matrix
    for (int y = 0; y != optimised_cam_T_board_matrix_builder.rows; y++)
    {
      for (int x = 0; x != optimised_cam_T_board_matrix_builder.cols; x++)
      {
        optimised_cam_T_board_matrix_builder.at<double>(y, x) = optimised_cam_T_board[z].at(y).at(x);
      }
    }

    // Push the temporary matrix in the vector of opencv matrices
    optimised_cam_T_board_matrix.push_back(optimised_cam_T_board_matrix_builder);
  }

  // The calibration board matrix that was generated previously must be transposed in order to matrix multiply
  cv::Mat true_calibration_board_points_matrix_transpose = true_calibration_board_points_matrix.t();

  // The current dimension of the true_calibration_board_points_matrix_transpose is a 3x42 but in order to be multiplied in the equation
  // it must be a 4x42. A row of ones would be inserted as the last row.
  cv::Mat row = cv::Mat::ones(1, num_of_calibration_points, CV_64FC1); // This is a row of ones
  true_calibration_board_points_matrix_transpose.push_back(row);

  // The new image pixels based on the optimised camera intrinsics and cam_T_boards would be computed here
  std::vector<cv::Mat> Optimised_image_pixels;

  for (int z = 0; z != total_chessboards_detected; z++)
  {
    cv::Mat Optimised_image_pixels_builder;
    Optimised_image_pixels_builder = optimised_camera_intrinsics_matrix * optimised_cam_T_board_matrix[z] * true_calibration_board_points_matrix_transpose; 

    Optimised_image_pixels.push_back(Optimised_image_pixels_builder);
  }
  
  // Create the processed chess board corners (from the optimised camera intrinsics and cam_T_boards).
  // This step is necessary to get rid of the scaling factor, s.
  std::vector<std::vector<cv::Point2f>> processed_chessboard_corners; 

  for (int z = 0; z != total_chessboards_detected; z++)
  {
    std::vector<cv::Point2f> processed_chessboard_corners_builder;
    for (int x = 0; x != num_of_calibration_points; x++)
    {
      double U = Optimised_image_pixels[z].at<double>(0, x) / Optimised_image_pixels[z].at<double>(2, x); // (s * u) / s
      double V = Optimised_image_pixels[z].at<double>(1, x) / Optimised_image_pixels[z].at<double>(2, x); // (s * v) / s
      processed_chessboard_corners_builder.push_back(cv::Point2f(U, V));
    }	
    processed_chessboard_corners.push_back(processed_chessboard_corners_builder);
  }

  // Loops used to compute some results comparing the true chessboard corners to the optimised chessboard corners
  std::vector<double> max_pixel_error;                        // Store the actual calculated max error
  std::vector<cv::Point2f> max_pixel_error_calibration_point; // Store coordinates of the calibration point where the max pixel error occurred
  std::vector<double> min_pixel_error;                        // Store the actual calculated min error
  std::vector<cv::Point2f> min_pixel_error_calibration_point; // Store coordinates of the calibration point where the min pixel error occurred
  std::vector<double> rms_error;

  // The code finds the reprojected error distance for each calibration point by finding the square root of the sum of squares for the U and V errors.
  // The code then determines which calibration point had the max and min reprojected error distance. After this is done,
  // the rms error for each image is found by finding the sum of squares of all the U and V errors (for all calibration points),
  // then dividing by the number of calibration points and then finding the square root.

  for (int z = 0; z != total_chessboards_detected; z++)
  {
    double image_max_pixel_error = 0;
    double image_min_pixel_error = 0;
    double error_accumulator = 0;
    cv::Point2f max_pixel_error_calibration_point_coordinate;
    cv::Point2f min_pixel_error_calibration_point_coordinate;

    for (int i = 0; i != num_of_calibration_points; i++)
    {
      double error_u = chessboard_corners[z][i].x - processed_chessboard_corners[z][i].x;
      double error_v = chessboard_corners[z][i].y - processed_chessboard_corners[z][i].y;

      double error_u_sq = pow(error_u, 2);
      double error_v_sq = pow(error_v, 2);

      double reprojected_error_distance = sqrt(error_u_sq + error_v_sq);
      error_accumulator = error_v_sq + error_v_sq + error_accumulator;

      // Assume the first pixel read has the highest and lowest error.
      if (i == 0)
      {
        image_max_pixel_error = reprojected_error_distance;
        image_min_pixel_error = reprojected_error_distance;

        max_pixel_error_calibration_point_coordinate.x = true_calibration_board_points[i][0];
        max_pixel_error_calibration_point_coordinate.y = true_calibration_board_points[i][1];

        min_pixel_error_calibration_point_coordinate.x = true_calibration_board_points[i][0];
        min_pixel_error_calibration_point_coordinate.y = true_calibration_board_points[i][1];
      }

      if (reprojected_error_distance > image_max_pixel_error)
      {
        image_max_pixel_error = reprojected_error_distance;
        max_pixel_error_calibration_point_coordinate.x = true_calibration_board_points[i][0];
        max_pixel_error_calibration_point_coordinate.y = true_calibration_board_points[i][1];
      }

      if (reprojected_error_distance < image_min_pixel_error)
      {
        image_min_pixel_error = reprojected_error_distance;
        min_pixel_error_calibration_point_coordinate.x = true_calibration_board_points[i][0];
        min_pixel_error_calibration_point_coordinate.y = true_calibration_board_points[i][1];
      }
    }
    
    double error_accumulator_avg = error_accumulator / num_of_calibration_points;
    double rms_error_image = sqrt(error_accumulator_avg);

    max_pixel_error.push_back(image_max_pixel_error);
    min_pixel_error.push_back(image_min_pixel_error);
    rms_error.push_back(rms_error_image);
    max_pixel_error_calibration_point.push_back(max_pixel_error_calibration_point_coordinate);
    min_pixel_error_calibration_point.push_back(min_pixel_error_calibration_point_coordinate);
  }
  
  // Display the results to the user
  std::cout << "\r\nERROR RESULTS: \r\n\n";
  for (int z = 0; z != total_chessboards_detected; z++)
  {
    std::cout << "Image " << z << ": \r\n";
    std::cout << "Max reprojected error distance: " << max_pixel_error[z] << "\r\n";
    std::cout << "Max reprojected error distance location (u, v): " << max_pixel_error_calibration_point[z].x << ", " << max_pixel_error_calibration_point[z].y << "\r\n";
    std::cout << "Min reprojected error distance: " << min_pixel_error[z] << "\r\n";
    std::cout << "Min reprojected error distance location (u, v): " << min_pixel_error_calibration_point[z].x << ", " << min_pixel_error_calibration_point[z].y << "\r\n";
    std::cout << "Image error rms: " << rms_error[z] << "\r\n\n";
  }

  // Make the directory if it doesn't exist
  DIR *pDir = opendir(directory.c_str());
  if (pDir == NULL)
  {
    int isDirCreated = mkdir(directory.c_str(), 0777);
    if (isDirCreated != 0)
    {
      std::cout << "Failed to create directory: " << directory << std::endl;
    }
  }

  // Superimpose the processed chessboard corners on the calibration images and write images to disk
  for (int z = 0; z != total_chessboards_detected; z++)
  {
    char filename[100];
    sprintf(filename, image_name_write.c_str(), z); 
    cv::drawChessboardCorners(images[z], patternsize, processed_chessboard_corners[z], true);
    std::string full_directory = directory + filename;
    
    imwrite(full_directory.c_str(), images[z]); //TODO: Make the full directory before writing images. imwrite cannot do it. cv2.imwrite(os.path.join(dirname, face_file_name), image)
  }

  return 0;
}
