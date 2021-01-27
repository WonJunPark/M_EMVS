#include <mapper_emvs/data_loading.hpp>
#include <mapper_emvs/mapper_emvs.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/pcd_io.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <chrono>
#include <string>
#include <fstream>
#include <iostream>

// Input parameters
DEFINE_string(bag_filename, "input.bag", "Path to the rosbag");
DEFINE_string(event_topic, "/dvs/events", "Name of the event topic (default: /dvs/events)"); //이벤트 값(), frame당 1000~3000 point, 0.04~0.05초당 한 frame 723
DEFINE_string(pose_topic, "/optitrack/davis", "Name of the pose topic (default: /optitrack/davis)"); // gt값 (timestamp,ax,ay,az,gx,gy,gz)
DEFINE_string(camera_info_topic, "/dvs/camera_info", "Name of the camera info topic (default: /dvs/camera_info)"); // camera 파라미터 (fx,fy,cx,cy,k1,k2,p1,p2,k3)
DEFINE_double(start_time_s, 0.0, "Start time in seconds (default: 0.0)");
DEFINE_double(stop_time_s, 1000.0, "Stop time in seconds (default: 1000.0)"); // 약 16분
DEFINE_double(end_time_s, 288.0, "end time in seconds (default: 1000.0)");

// Disparity Space Image (DSI) parameters. Section 5.2 in the IJCV paper.
DEFINE_int32(dimX, 0, "X dimension of the voxel grid (if 0, will use the X dim of the event camera) (default: 0)");
DEFINE_int32(dimY, 0, "Y dimension of the voxel grid (if 0, will use the Y dim of the event camera) (default: 0)");
DEFINE_int32(dimZ, 100, "Z dimension of the voxel grid (default: 100) must be <= 256");
DEFINE_double(fov_deg, 0.0, "Field of view of the DSI, in degrees (if < 10, will use the FoV of the event camera) (default: 0.0)");
DEFINE_double(min_depth, 0.3, "Min depth, in meters (default: 0.3)");
DEFINE_double(max_depth, 5.0, "Max depth, in meters (default: 5.0)");

// Depth map parameters (selection and noise removal). Section 5.2.3 in the IJCV paper.
DEFINE_int32(adaptive_threshold_kernel_size, 5, "Size of the Gaussian kernel used for adaptive thresholding. (default: 5)");
DEFINE_double(adaptive_threshold_c, 5., "A value in [0, 255]. The smaller the noisier and more dense reconstruction (default: 5.)");
DEFINE_int32(median_filter_size, 5, "Size of the median filter used to clean the depth map. (default: 5)");

// Point cloud parameters (noise removal). Section 5.2.4 in the IJCV paper.
DEFINE_double(radius_search, 0.05, "Size of the radius filter. (default: 0.05)");
DEFINE_int32(min_num_neighbors, 3, "Minimum number of points for the radius filter. (default: 3)");


int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  // 21.01.26 add
  const std::string &rosbag = FLAGS_bag_filename;
  std::vector<dvs_msgs::Event> events_;
  std::map<ros::Time, geometry_utils::Transformation> poses_;
  sensor_msgs::CameraInfo camera_info_msg;
  const std::string& event_topic = FLAGS_event_topic;
  const std::string& camera_info_topic = FLAGS_camera_info_topic;
  const std::string& pose_topic = FLAGS_pose_topic;

  std::vector<std::string> topics;
  //

  topics.push_back(event_topic);
  topics.push_back(camera_info_topic);
  topics.push_back(pose_topic);

  rosbag::Bag  bag(rosbag, rosbag::bagmode::Read);
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // int count;
  // double start,end;
  // double stamp = FLAGS_stop_time_s;
  std::string path, num, file;
  bool continue_looping_through_bag = true;
  bool got_initial_stamp = false;
  ros::Time initial_timestamp;
  
  double et1,et2;
  int count = 0;

  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    events_.clear();

    const std::string& topic_name = m.getTopic();
    VLOG(2) << topic_name;
    
    // Events
    if (topic_name == topics[0])
    {
      
      dvs_msgs::EventArray::ConstPtr msg = m.instantiate<dvs_msgs::EventArray>();
      // LOG(INFO) << "event array: " << msg;

      if (msg != NULL)
      {
        if(msg->events.empty())
        {
          continue;
        }
        
        const ros::Time& stamp = msg->events[0].ts;

        LOG(INFO) << "stamp: " <<  stamp;

        if(!got_initial_stamp)
        {
          initial_timestamp = stamp;
          LOG(INFO) << "initial stamp: " << stamp;
          got_initial_stamp = true;
        }

        et1 = (msg->events[0].ts).toSec();
        et2 = (msg->events[msg->events.size()-1].ts).toSec();

        for (size_t i = 0; i < msg->events.size(); ++i)
        {
          const double rel_stamp = (msg->events[i].ts - initial_timestamp).toSec();

          if(rel_stamp < et1)
          {
            continue;
          }
          if(rel_stamp > et2)
          {
            continue_looping_through_bag = false;
          }

          dvs_msgs::Event ev_modified(msg->events[i]);
          ev_modified.ts = ros::Time(ev_modified.ts.toSec() - initial_timestamp.toSec());
          events_.push_back(ev_modified); // event total number
        }

        LOG(INFO) << "events_: " << events_.size();
        LOG(INFO) << "poses_: " << poses_.size();

        image_geometry::PinholeCameraModel cam;
        cam.fromCameraInfo(camera_info_msg);

        // // Use linear interpolation to compute the camera pose for each event
        LinearTrajectory trajectory = LinearTrajectory(poses_);
        // LOG(INFO) << "poses : " << poses.size();

        // // Set the position of the reference view in the middle of the trajectory 
        geometry_utils::Transformation T0_, T1_;
        ros::Time t0_, t1_;

        trajectory.getFirstControlPose(&T0_, &t0_);
        trajectory.getLastControlPose(&T1_, &t1_);
        geometry_utils::Transformation T_w_rv;

        // // LOG(INFO) << "T_w_rv : " << T_w_rv; // 4*4 unit matrix

        trajectory.getPoseAt(ros::Time(0.5 * (t0_.toSec() + t1_.toSec())), T_w_rv);
        geometry_utils::Transformation T_rv_w = T_w_rv.inverse();

        // LOG(INFO) << "t0_ : " << t0_; // 0.043637276
        // LOG(INFO) << "t1_ : " << t1_; // 2.003728390

        // // LOG(INFO) << "T_rv_w : " << T_rv_w;

        // // Initialize the DSI
        CHECK_LE(FLAGS_dimZ, 256) << "Number of depth planes should be <= 256";
        EMVS::ShapeDSI dsi_shape(FLAGS_dimX, FLAGS_dimY, FLAGS_dimZ,
                                FLAGS_min_depth, FLAGS_max_depth,
                                FLAGS_fov_deg);

        // // Initialize mapper
        EMVS::MapperEMVS mapper(cam, dsi_shape);

        // // 1. Back-project events into the DSI
        std::chrono::high_resolution_clock::time_point t_start_dsi = std::chrono::high_resolution_clock::now();

        mapper.evaluateDSI(events_, trajectory, T_rv_w);
        std::chrono::high_resolution_clock::time_point t_end_dsi = std::chrono::high_resolution_clock::now();
        auto duration_dsi = std::chrono::duration_cast<std::chrono::milliseconds>(t_end_dsi - t_start_dsi ).count();

        // LOG(INFO) << "Time to evaluate DSI: " << duration_dsi << " milliseconds";
        // LOG(INFO) << "Number of events processed: " << events.size() << " events";
        // LOG(INFO) << "Number of events processed per second: " << static_cast<float>(events.size()) / (1000.f * static_cast<float>(duration_dsi)) << " Mev/s";
        // LOG(INFO) << "Mean square = " << mapper.dsi_.computeMeanSquare();

        // // Write the DSI (3D voxel grid) to disk

        path = "./result/dsi";
        num = std::to_string(count);
        file=".npy";

        path += num;
        path += file;
        // mapper.dsi_.writeGridNpy("./result/dsi.npy");

        mapper.dsi_.writeGridNpy(path.c_str());

        // event
        path = "./result/events";
        num = std::to_string(count);
        file=".txt";

        path += num;
        path += file;
        

        std::ofstream writeFile(path.data());
        
        for(int i=0;i<events_.size();i++){
          if(writeFile.is_open()){
            writeFile << events_[i];
          }
        }
        writeFile.close();

        EMVS::OptionsDepthMap opts_depth_map;
        opts_depth_map.adaptive_threshold_kernel_size_ = FLAGS_adaptive_threshold_kernel_size;
        opts_depth_map.adaptive_threshold_c_ = FLAGS_adaptive_threshold_c;
        opts_depth_map.median_filter_size_ = FLAGS_median_filter_size;
        cv::Mat depth_map, confidence_map, semidense_mask;
        mapper.getDepthMapFromDSI(depth_map, confidence_map, semidense_mask, opts_depth_map);
        
        //Save confidence map as an 8-bit image
        path = "./result/confidence_map";
        num = std::to_string(count);
        file=".png";
        path += num;
        path += file;

        cv::Mat confidence_map_255;
        cv::normalize(confidence_map, confidence_map_255, 0, 255.0, cv::NORM_MINMAX, CV_32FC1);
        cv::imwrite(path.c_str(), confidence_map_255);

        poses_.clear();
        count += 1;
      }
    }

    // Camera Info
    if (topic_name == topics[1])
    {
      camera_info_msg = *(m.instantiate<sensor_msgs::CameraInfo>());
    }

    // Pose
    if (topic_name == topics[2])
    {
      const geometry_msgs::PoseStamped pose_msg = *(m.instantiate<geometry_msgs::PoseStamped>());
      const ros::Time& stamp = pose_msg.header.stamp;

      // LOG(INFO) << "got_initial_stamp: " << got_initial_stamp; //1

      if(!got_initial_stamp)
      {
        initial_timestamp = stamp;
        got_initial_stamp = true;
      }
      const double rel_stamp = (stamp - initial_timestamp).toSec();

      if(rel_stamp < et1)
      {
        continue;
      }
      if(rel_stamp > et2)
      {
        continue_looping_through_bag = false;
      }

      const Eigen::Vector3d position(pose_msg.pose.position.x,
                                     pose_msg.pose.position.y,
                                     pose_msg.pose.position.z);
      const Eigen::Quaterniond quat(pose_msg.pose.orientation.w,
                                    pose_msg.pose.orientation.x,
                                    pose_msg.pose.orientation.y,
                                    pose_msg.pose.orientation.z);
      geometry_utils::Transformation T(position, quat);
      poses_.insert(std::pair<ros::Time, geometry_utils::Transformation>(ros::Time(pose_msg.header.stamp.toSec() - initial_timestamp.toSec()), T));
    }

    std::sort(events_.begin(), events_.end(),
            [](const dvs_msgs::Event& a, const dvs_msgs::Event& b) -> bool
    {
      return a.ts < b.ts;
    });
  }
  return 0;
}
