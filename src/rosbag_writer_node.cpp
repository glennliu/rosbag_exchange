#include <string>
#include <iostream>
#include <dirent.h>
#include <list>
#include <array>
// #include <filesystem>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2_msgs/TFMessage.h>
#include <rosgraph_msgs/Clock.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rosbag/bag.h>

// static const std::string OPENCV_WINDOW = "Image window";
using namespace std;
// typedef std::string string;

vector<string> read_scans(const string split_dir)
{
  cout<<"reading scans from: "<<split_dir<<endl;
  vector<string> scan_names;

  ifstream split_file(split_dir);
  if (split_file.is_open()){
    string line;
    while (getline(split_file, line)){
      scan_names.push_back(line);
    }
  }
  else{
    ROS_ERROR("split file not exist!");
  }

  return scan_names;
}

class DataBuilder
{
  public:
  DataBuilder(const std::string &scan_dir)
  {
    scan_dir_ = scan_dir;
  }

  sensor_msgs::ImagePtr load_color_img(const string& dir,const float& timestamp,const int& frame_id)
  {
      // cv::Mat raw_img = cv::imread(dir, cv::IMREAD_COLOR);
      cv_bridge::CvImage cv_img;
      cv::Mat gray_img = cv::imread(dir, cv::IMREAD_GRAYSCALE);

      cv_img.header.stamp.fromSec(timestamp);
      cv_img.header.seq = frame_id;
      cv_img.header.frame_id = "left_cam";
      cv_img.encoding = "mono8";
      cv_img.image = gray_img;

      return cv_img.toImageMsg();
  }

  sensor_msgs::ImagePtr load_depth(const string& dir,const float& timestamp,const int& frame_id)
  {
      float DEPTH_SCALE = 1000.0;
      cv::Mat raw_depth = cv::imread(dir, cv::IMREAD_ANYDEPTH);
      cv::Mat scaled_depth;
      raw_depth.convertTo(scaled_depth, CV_32FC1, 1/DEPTH_SCALE);
      // depth_img = depth_img / DEPTH_SCALE;
      cv_bridge::CvImage cv_depth;
      cv_depth.header.stamp.fromSec(timestamp);
      cv_depth.header.seq = frame_id;
      cv_depth.header.frame_id = "left_cam";
      cv_depth.encoding = "32FC1";
      cv_depth.image = scaled_depth;

      return cv_depth.toImageMsg();
  }

  sensor_msgs::ImagePtr load_semantic_image(const string &dir, const float &timestamp, const int &frame_id)
  {
    cv::Mat raw_semantic = cv::imread(dir, cv::IMREAD_COLOR);
    cv::cvtColor(raw_semantic, raw_semantic, cv::COLOR_BGR2RGB);

    cv_bridge::CvImage cv_semantic;
    cv_semantic.header.stamp.fromSec(timestamp);
    cv_semantic.header.seq = frame_id;
    cv_semantic.header.frame_id = "left_cam";
    cv_semantic.encoding = "rgb8";
    cv_semantic.image = raw_semantic;
    return cv_semantic.toImageMsg();
  }

  sensor_msgs::CameraInfo load_camera_info(const float &timestamp, const int &frame_id)
  {
    sensor_msgs::CameraInfo cam_info;
    cam_info.header.stamp.fromSec(timestamp);
    cam_info.header.seq = frame_id;
    cam_info.header.frame_id = "left_cam";
    cam_info.height = 480;
    cam_info.width = 640;
    cam_info.distortion_model = "radial-tangential";
    cam_info.D = {0.0, 0.0, 0.0, 0.0, 0.0};
    cam_info.K = {575.66, 0.0, 320.28, 0.0, 578.05, 240.13, 0.0, 0.0, 1.0};
    cam_info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0,0.0, 0.0, 1.0};
    cam_info.P = {575.66, 0.0, 320.28, 0.0, 0.0, 578.05, 240.13, 0.0, 0.0, 0.0, 1.0, 0.0};
    return cam_info;
  }

  bool load_tf(const string &dir, const float &timestamp, const int &frame_id,
    tf2_msgs::TFMessage &tf_msg, geometry_msgs::TransformStamped &tf, nav_msgs::Path &path)
  {
    // tf2_msgs::TFMessage tf_msg;
    // geometry_msgs::TransformStamped tf;
    geometry_msgs::PoseStamped pose_msg;
    tf.header.stamp.fromSec(timestamp);
    tf.header.seq = frame_id;
    tf.header.frame_id = "world";
    tf.child_frame_id = "base_link_gt";

    pose_msg.header.stamp.fromSec(timestamp);
    pose_msg.header.seq = frame_id;
    pose_msg.header.frame_id = "world";

    path.header.stamp.fromSec(timestamp);
    path.header.seq = frame_id;
    path.header.frame_id = "world";

    // odom.header.stamp.fromSec(timestamp);
    // odom.header.seq = frame_id;
    // odom.header.frame_id = "world";

    // read pose 
    ifstream pose_file(dir);
    if (pose_file.is_open()){
      string line;
      array<float,16> pose;

      // cout<<"---\n";
      int idx = 0;
      while (getline(pose_file, line)){
        stringstream ss(line);
        string item;
        if (line.find('inf')!=string::npos){
          cout<<"inf pose found!"<<endl;
          return false;
        }
        while (getline(ss, item, ' ')){
          // pose.push_back(stof(item));
          pose[idx] = stof(item);
          idx++;
        }
      }
      Eigen::Matrix4f T_wc;
      T_wc<<pose[0], pose[1], pose[2], pose[3],
            pose[4], pose[5], pose[6], pose[7],
            pose[8], pose[9], pose[10], pose[11],
            pose[12], pose[13], pose[14], pose[15];
      
      Eigen::Matrix3f rotation = T_wc.topLeftCorner<3,3>();
      Eigen::Vector3f translation = T_wc.block<3,1>(0,3);
      Eigen::Quaternionf q(rotation);

      // cout<<T_wc<<"\n";
      tf.transform.translation.x = translation.x();
      tf.transform.translation.y = translation.y();
      tf.transform.translation.z = translation.z();
      tf.transform.rotation.x = q.x();
      tf.transform.rotation.y = q.y();
      tf.transform.rotation.z = q.z();
      tf.transform.rotation.w = q.w();

      // odom.pose.pose.position.x = translation.x();
      // odom.pose.pose.position.y = translation.y();
      // odom.pose.pose.position.z = translation.z();
      // odom.pose.pose.orientation.x = q.x();
      // odom.pose.pose.orientation.y = q.y();
      // odom.pose.pose.orientation.z = q.z();
      // odom.pose.pose.orientation.w = q.w();

      pose_msg.pose.position.x = translation.x();
      pose_msg.pose.position.y = translation.y();
      pose_msg.pose.position.z = translation.z();
      pose_msg.pose.orientation.x = q.x();
      pose_msg.pose.orientation.y = q.y();
      pose_msg.pose.orientation.z = q.z();
      pose_msg.pose.orientation.w = q.w();

      tf_msg.transforms.push_back(tf);
      path.poses.push_back(pose_msg);
      return true;
    }
    else{
      ROS_WARN("Reach sequence end!");
      return false;
    }
    
    // return tf_msg;

  }

  tf2_msgs::TFMessage create_static_tf()
  {
    tf2_msgs::TFMessage tf_msg;
    geometry_msgs::TransformStamped tf;
    tf.header.stamp.fromSec(0);
    tf.header.seq = 0;
    tf.header.frame_id = "base_link_gt";
    tf.child_frame_id = "left_cam";

    // read pose 
    tf.transform.translation.x = -0.2;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = 0.0;
    tf.transform.rotation.w = 1.0;

    tf_msg.transforms.push_back(tf);
    
    return tf_msg;
  }
  
  bool write_rosbag(const string prediction_folder, const int frame_gap)
  {
    // rosbag
    rosbag::Bag bag;
    string pred_name;
    if (prediction_folder.find("maskrcnn") != string::npos){
      pred_name = "maskrcnn";
    }
    else{
      pred_name = "gsam";
    }

    bag.open(scan_dir_+"/"+pred_name+"_data.bag", rosbag::bagmode::Write);

    int frame_id = 0;
    float time_shift = 10.0;
    tf2_msgs::TFMessage tfs;
    nav_msgs::Path path_msg;

    auto static_tf = create_static_tf();
    bag.write("/tf_static", ros::Time(time_shift), static_tf);

    // while ((ptr = readdir(dir)) != NULL)
    while (frame_id<9999)
    {
      stringstream frame_name;
      frame_name<<"frame-"<<std::setw(6)<<std::setfill('0')<<frame_id;
      cout <<"processing frame "<< frame_name.str() << endl;

      float timestamp = frame_id * 0.05 + time_shift;

      string rgb_dir = scan_dir_ + "/color/" + frame_name.str()+".jpg";
      string depth_dir = scan_dir_ + "/depth/" + frame_name.str()+".png";
      string pred_dir = scan_dir_ + "/"+prediction_folder+"/" + frame_name.str()+".png";
      string pose_dir = scan_dir_ + "/pose/" + frame_name.str()+".txt";

      if (!ifstream(depth_dir) or !ifstream(pose_dir)){
        cout<<"Reached the end of the sequence"<<endl;
        break;
      }

      nav_msgs::Odometry odom_msg;
      geometry_msgs::TransformStamped tf;
      tfs.transforms.clear();
      auto color_msg = load_color_img(rgb_dir, timestamp, frame_id);
      auto depth_msg = load_depth(depth_dir, timestamp, frame_id);
      auto camera_info = load_camera_info(timestamp, frame_id);
      bool ret = load_tf(pose_dir, timestamp, frame_id, tfs, tf, path_msg);
      if (!ret){
        cout<<"skip frame due to invalid pose"<<endl;
        frame_id += frame_gap;
        continue;
      }

      rosgraph_msgs::Clock clock_msg;
      clock_msg.clock.fromSec(timestamp);

      bag.write("/tesse/left_cam/image_raw", ros::Time(timestamp), color_msg);
      bag.write("/tesse/depth/image_raw", ros::Time(timestamp), depth_msg);
      bag.write("/tesse/left_cam/camera_info", ros::Time(timestamp), camera_info);
      bag.write("/tf", ros::Time(timestamp), tfs);
      bag.write("/transform",ros::Time(timestamp), tf);
      // bag.write("/odom", ros::Time(timestamp), odom_msg);
      bag.write("/path", ros::Time(timestamp), path_msg);
      // bag.write('/tf_static', ros::Time(timestamp), static_tf);
      bag.write("/clock", ros::Time(timestamp), clock_msg);
      if (ifstream(pred_dir)){ 
        auto semantic_msg = load_semantic_image(pred_dir, timestamp, frame_id);
        bag.write("/tesse/segmentation/image_raw", ros::Time(timestamp), semantic_msg);
      }
      else{
        cv::Mat zero_semantic = cv::Mat::zeros(480, 640, CV_8UC3);
        cv_bridge::CvImage cv_semantic;
        cv_semantic.header.stamp.fromSec(timestamp);
        cv_semantic.header.seq = frame_id;
        cv_semantic.header.frame_id = "left_cam";
        cv_semantic.encoding = "rgb8";
        cv_semantic.image = zero_semantic;
        auto semantic_msg = cv_semantic.toImageMsg();
        bag.write("/tesse/segmentation/image_raw", ros::Time(timestamp), semantic_msg);
      }

      frame_id += frame_gap;

    }

    bag.close();
    cout<<"finished reading: "<<scan_dir_<<endl;

    return true;
  }

  private:
    std::string scan_dir_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "data_builder");
  string data_root = "/media/lch/SeagateExp/dataset_/ScanNet";
  // string scan_name;
  int frame_gap;
  string prediction_folder;

  if (argc > 2) {
    prediction_folder = argv[1];
    frame_gap = atoi(argv[2]);
    cout<<"prediction_folder: "<<prediction_folder<<endl;
    cout<<"frame_gap: "<<frame_gap<<endl;
  }
  else{
    cout<< "set prediction folder first!"<<endl;
    return 0;
  }

  auto scans = read_scans(data_root+"/splits/val_clean.txt");
  cout<<"scans size: "<<scans.size()<<endl;

  for (string scan:scans)
  {
    cout<<"scan_name: "<<scan<<", frame gap: "<< frame_gap<<endl;
    DataBuilder build_rosbag(data_root+"/target_scans/"+scan);
    bool ret = build_rosbag.write_rosbag(prediction_folder,frame_gap);
    // break;
  }

  ros::spinOnce();
  return 0;
}