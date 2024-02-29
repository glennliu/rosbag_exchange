#include <vector>
#include <sys/stat.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <eigen3/Eigen/Dense>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

class RosbagReader
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private;

    string rosbag_path;
    string rosbag_folder;
    string scan_name;
    string rgb_topic_name;
    string depth_topic_name;
    string camera_info_topic_name;
    string camera_pose_topic_name;

    vector<sensor_msgs::ImageConstPtr> rgb_msgs;
    vector<sensor_msgs::ImageConstPtr> depth_msgs;
    vector<nav_msgs::OdometryConstPtr> camera_pose_msgs;

  public:
    RosbagReader():
      nh_(),
      nh_private("~")
    {
      // bag.open(bag_dir, rosbag::bagmode::Read);
      assert(nh_private.getParam("rosbag_path", rosbag_path));
      assert(nh_private.getParam("rgb_topic_name", rgb_topic_name));
      assert(nh_private.getParam("depth_topic_name", depth_topic_name));
      assert(nh_private.getParam("camera_info_topic_name", camera_info_topic_name));
      assert(nh_private.getParam("camera_pose_topic_name", camera_pose_topic_name));

      rosbag_folder = rosbag_path.substr(0, rosbag_path.find_last_of("/"));
      scan_name = rosbag_path.substr(rosbag_path.find_last_of("/")+1, rosbag_path.find_last_of(".")-rosbag_path.find_last_of("/")-1);

      ROS_INFO("rosbag_path: %s", rosbag_folder.c_str());

    }

    virtual ~RosbagReader()= default;

    void create_scan_folder()
    {
      string scan_folder = rosbag_folder + "/" + scan_name;
      string rgb_folder = scan_folder + "/rgb";
      string depth_folder = scan_folder + "/depth";
      string camera_pose_folder = scan_folder + "/pose";

      string cmd;
      struct stat buffer;

      if (stat(scan_folder.c_str(), &buffer) != 0) mkdir(scan_folder.c_str(), 0777);
      if (stat(rgb_folder.c_str(), &buffer) != 0) mkdir(rgb_folder.c_str(), 0777);
      if (stat(depth_folder.c_str(), &buffer) != 0) mkdir(depth_folder.c_str(), 0777);
      if (stat(camera_pose_folder.c_str(), &buffer) != 0) mkdir(camera_pose_folder.c_str(), 0777);

    }

    bool image_cb(const string &output_folder, const sensor_msgs::ImageConstPtr img_msg, 
      const int &frame_id, const string &encoding_type)
    {
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
      cv_ptr = cv_bridge::toCvCopy(img_msg, encoding_type);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
      }

      auto img = cv_ptr->image;
      int seq = img_msg->header.seq;
      stringstream frame_dir;
      frame_dir<<output_folder<<"/frame-"<<setfill('0')<<setw(6)<<frame_id<<".png";

      cv::imwrite(frame_dir.str(),img);
      return true;
    }

    bool camera_pose_cb(const string &output_folder, const nav_msgs::OdometryConstPtr pose_msg, 
      const int &frame_id, Eigen::Matrix4f &pose_mat, std_msgs::Header &ts)
    {
      stringstream pose_dir;
      // int seq = pose_msg->header.seq;
      pose_dir<<output_folder<<"/frame-"<<setfill('0')<<setw(6)<<frame_id<<".txt";
      ofstream pose_file(pose_dir.str());
      Eigen::Vector3f t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
      Eigen::Quaternionf q(pose_msg->pose.pose.orientation.w, pose_msg->pose.pose.orientation.x, pose_msg->pose.pose.orientation.y, pose_msg->pose.pose.orientation.z);
      // Eigen::Matrix4f pose_mat;
      pose_mat<<q.toRotationMatrix(), t, 0, 0, 0, 1;

      pose_file<<pose_mat<<endl;

      pose_file.close();

      ts = pose_msg->header;
      return true;
    }

    bool parse_bag()
    {
      rosbag::Bag bag;
      bag.open(rosbag_path, rosbag::bagmode::Read);
      if(!bag.isOpen()){
        ROS_ERROR("rosbag is not open");
        return false;
      }

      create_scan_folder();

      std::vector<std::string> topics;
      topics.push_back(rgb_topic_name);
      topics.push_back(depth_topic_name);
      topics.push_back(camera_info_topic_name);
      topics.push_back(camera_pose_topic_name);

      rosbag::View view(bag, rosbag::TopicQuery(topics));

      for(const rosbag::MessageInstance &msg: view)
      {
        sensor_msgs::ImageConstPtr img_msg = msg.instantiate<sensor_msgs::Image>();
        const string &msg_topic = msg.getTopic();
        // const int seq = img_msg->header.seq;

        if (img_msg != nullptr){
          if (msg_topic==depth_topic_name){
            depth_msgs.push_back(img_msg);
          }
          else if (msg_topic==rgb_topic_name){
            rgb_msgs.push_back(img_msg);
          }
        }

        nav_msgs::OdometryConstPtr cm_pose_msg = msg.instantiate<nav_msgs::Odometry>();
        if (cm_pose_msg != nullptr && msg_topic==camera_pose_topic_name){
          camera_pose_msgs.push_back(cm_pose_msg);
        }

      }
      bag.close();

      ROS_WARN("read %d rgb images", rgb_msgs.size());
      ROS_WARN("read %d depth images", depth_msgs.size());
      ROS_WARN("read %d camera pose", camera_pose_msgs.size());

      return true;
    }

    bool extract_messages()
    {
      int rgb_ts = 0;
      int frame_id = 0;
      vector<int> rgbd_pose_list;
      vector<double> rgbd_timestamp;
      vector<Eigen::Matrix4f> pose_list;

      // Extract rgb-d and pose
      for (int i=0; i<camera_pose_msgs.size(); i++){
        std_msgs::Header ts;
        Eigen::Matrix4f pose_mat;
        camera_pose_cb(rosbag_folder + "/" + scan_name + "/pose", camera_pose_msgs[i], frame_id, pose_mat, ts);

        // Find rgb
        bool find_rgb = false;
        for (int j=rgb_ts; j<rgb_msgs.size(); j++){
          double delta_t = abs(rgb_msgs[j]->header.stamp.toSec() - ts.stamp.toSec());
          if (delta_t < 0.01){
            find_rgb = image_cb(rosbag_folder + "/" + scan_name + "/rgb", rgb_msgs[j], frame_id, "bgr8");
            rgb_ts = j;
            // ROS_INFO("find rgb image for pose at %d", i);
            break;
          }
        }

        // Find depth
        bool find_depth = false;
        for (int j=0; j<depth_msgs.size(); j++){
          double delta_t = abs(depth_msgs[j]->header.stamp.toSec() - ts.stamp.toSec());
          if (delta_t < 0.01){
            find_depth = image_cb(rosbag_folder + "/" + scan_name + "/depth", depth_msgs[j], frame_id, "");
            break;
          }
        }

        if (find_rgb && find_depth){
          rgbd_pose_list.push_back(frame_id);
          rgbd_timestamp.push_back(ts.stamp.toSec());
          pose_list.push_back(pose_mat);
          ROS_INFO("find rgb-d image for pose %d", i);
        }
        else{
          ROS_WARN("CAN NOT find rgb-d image for pose %d", i);
        }

        frame_id++;

      }

      // Save data association 
      ofstream pose_list_file(rosbag_folder + "/" + scan_name + "/data_association.txt");
      for(int valid_frame_id:rgbd_pose_list){
        stringstream frame_name;
        frame_name<<"frame-"<<setfill('0')<<setw(6)<<valid_frame_id;
        pose_list_file<<"depth/"<<frame_name.str()<<".png "
          <<"rgb/"<<frame_name.str()<<".png"<<endl;
      }
      pose_list_file.close();

      // Save timestamp
      ofstream timestamp_file(rosbag_folder + "/" + scan_name + "/timestamp.txt");
      ofstream trajectory_file(rosbag_folder + "/" + scan_name + "/trajectory.log");

      for(int idx=0;idx<rgbd_timestamp.size();idx++){
        timestamp_file<<"frame-"<<setfill('0')<<setw(6)<<rgbd_pose_list[idx]<<" "
                  <<setprecision(16)<<rgbd_timestamp[idx]
                  <<endl;

        trajectory_file<< idx << " "<<idx<<" "<<idx+1<<"\n"
          <<setprecision(8)<<pose_list[idx]
                  <<endl;
      }
      timestamp_file.close();
      trajectory_file.close();

      ROS_WARN("Export %d aligned rgb-d frames", rgbd_pose_list.size());
      return true;
    }


/*
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    auto img = cv_ptr->image;
    std::cout<<img.size() <<std::endl;
    std::cout<<img.channels()<<std::endl;
    cv::imwrite("/home/lch/mit_ws/catkin_new_ws/src/kimera_semantics/test.png", img);


    // image_pub_.publish(cv_ptr->toImageMsg());
  }

  void read_depth_cb(const sensor_msgs::ImageConstPtr &depth_msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(depth_msg);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    ROS_INFO("reading depth frame %d", depth_msg->header.seq);
    cv::Mat img = cv_ptr->image;
    cv::imwrite("/home/lch/mit_ws/catkin_new_ws/src/kimera_semantics/depth.png", img);


  }
*/

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rosbag_reader_node");
  ros::NodeHandle nh; 
  ros::NodeHandle nh_private("~");


  RosbagReader reader;
  bool loaded = reader.parse_bag();
  if(loaded){
    reader.extract_messages();
  }
  
  ROS_WARN("finished read rosbag");
  ros::spinOnce();
  return 0;
}