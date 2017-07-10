#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <cv_bridge/cv_bridge.h>
#include <raw_video_stream/ProcessFileConfig.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/console.h>

#include <iostream>
#include <sstream>


namespace raw_video_stream
{

class ProcessFileNodelet : public nodelet::Nodelet
{
  // ROS communication
  boost::shared_ptr<image_transport::ImageTransport> it_out_;

  boost::mutex connect_mutex_;

  image_transport::CameraPublisher image_pub_;

  sensor_msgs::ImagePtr msg_;
  sensor_msgs::CameraInfo cam_info_msg_;
  std_msgs::Header header_;

  FILE * fp_;
  int frame_size_;
  int frame_count_;

  std::string video_stream_provider_;
  std::string camera_name_;
  int fps_;
  std::string frame_id_;
  std::string camera_info_url_;
  bool flip_horizontal_;
  bool flip_vertical_;
  int width_;
  int height_;
  bool flip_image_;
  int flip_value_;

  // dynamic reconfigure
  // boost::recursive_mutex config_mutex_;
  // typedef raw_video_stream::ProcessFileConfig Config;
  // typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  // boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  // Config config_;

  virtual void onInit();

  void connectCb();

  void publish();

  // void configCb(Config &config,uint32_t level);

};

void ProcessFileNodelet::onInit()
{
  ros::NodeHandle & nh = getNodeHandle();
  ros::NodeHandle _nh("~"); // to get the private params
  // ros::NodeHandle & _nh = getPrivateNodeHandle();
  it_out_.reset(new image_transport::ImageTransport(nh));

  fp_ = NULL;

  // read parameters

  if (_nh.getParam("video_stream_provider", video_stream_provider_))
  {
    ROS_INFO_STREAM("Resource video_stream_provider: " << video_stream_provider_);
    ROS_INFO_STREAM("Getting video from provider: " << video_stream_provider_);
    const char * c = video_stream_provider_.c_str();
    fp_ = fopen(c,"rb");
  }
  else
  {
    ROS_ERROR("Failed to get param 'video_stream_provider'");
    return;
  }

  _nh.param("camera_name", camera_name_, std::string("camera"));
  ROS_INFO_STREAM("Camera name: " << camera_name_);

  _nh.param("fps", fps_, 240);
  ROS_INFO_STREAM("Throttling to fps: " << fps_);

  _nh.param("frame_id", frame_id_, std::string("camera"));
  ROS_INFO_STREAM("Publishing with frame_id: " << frame_id_);

  _nh.param("camera_info_url", camera_info_url_, std::string(""));
  ROS_INFO_STREAM("Provided camera_info_url: '" << camera_info_url_ << "'");

  _nh.param("flip_horizontal", flip_horizontal_, false);
  ROS_INFO_STREAM("Flip horizontal image is: " << ((flip_horizontal_)?"true":"false"));

  _nh.param("flip_vertical", flip_vertical_, false);
  ROS_INFO_STREAM("Flip vertical image is: " << ((flip_vertical_)?"true":"false"));

  _nh.param("width", width_, 0);
  _nh.param("height", height_, 0);
  ROS_INFO_STREAM("Image size: " << width_ << "x" << height_);
  frame_size_ = width_ * height_;

  _nh.param("frame_count", frame_count_, 0);
  ROS_INFO_STREAM("Frame count: " << frame_count_);

  // From http://docs.opencv.org/modules/core/doc/operations_on_arrays.html#void flip(InputArray src, OutputArray dst, int flipCode)
  // FLIP_HORIZONTAL == 1, FLIP_VERTICAL == 0 or FLIP_BOTH == -1
  flip_image_ = true;
  if (flip_horizontal_ && flip_vertical_)
    flip_value_ = 0; // flip both, horizontal and vertical
  else if (flip_horizontal_)
    flip_value_ = 1;
  else if (flip_vertical_)
    flip_value_ = -1;
  else
    flip_image_ = false;

  // set up dynamic reconfigure
  // reconfigure_server_.reset(new ReconfigureServer(config_mutex_,private_nh));
  // ReconfigureServer::CallbackType f = boost::bind(&ProcessFileNodelet::configCb,this,_1,_2);
  // reconfigure_server_->setCallback(f);

  // monitor whether anyone is subscribed to the output
  image_transport::SubscriberStatusCallback connect_cb = boost::bind(&ProcessFileNodelet::connectCb,this);
  ros::SubscriberStatusCallback connect_cb_info = boost::bind(&ProcessFileNodelet::connectCb,this);
  // make sure we don't enter connectCb() between advertising and assigning to image_pub_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  image_pub_ = it_out_->advertiseCamera("image_raw",1,connect_cb,connect_cb,connect_cb_info,connect_cb_info);
}

// handles (un)subscribing when clients (un)subscribe
void ProcessFileNodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (image_pub_.getNumSubscribers() == 0)
  {
  }
  else
  {
    publish();
  }
}

void ProcessFileNodelet::publish()
{
  // Config config;
  // {
  //   boost::lock_guard<boost::recursive_mutex> lock(config_mutex_);
  //   config = config_;
  // }
  // int morph_kernel_size = config.morph_kernel_size;
  // int center_marker_radius = config.center_marker_radius;
  // int drawn_line_thickness = config.drawn_line_thickness;
  // bool draw_crosshairs = config.draw_crosshairs;

  // header.frame_id = frame_id;
  // camera_info_manager::CameraInfoManager cam_info_manager(nh, camera_name, camera_info_url);
  // Get the saved camera info if any
  // cam_info_msg = cam_info_manager.getCameraInfo();

  ros::NodeHandle & nh = getNodeHandle();

  cv::Mat frame_in;
  frame_in.create(height_,width_,CV_8UC1);

  cv::Mat frame_out;

  //Memory allocation for bayer image data buffer.
  unsigned char * image_data = (unsigned char *) malloc (sizeof(unsigned char) * frame_size_);

  ros::Rate r(fps_);
  for (int frame_n=0; frame_n<frame_count_; ++frame_n)
  {
    if (!nh.ok())
    {
      return;
    }
    //Read image data and store in buffer.
    fread(image_data,sizeof(unsigned char),frame_size_,fp_);

    memcpy(frame_in.data,image_data,frame_size_);

    cv::cvtColor(frame_in,frame_out,CV_GRAY2BGR);

    msg_ = cv_bridge::CvImage(header_, "bgr8", frame_out).toImageMsg();
    // cam_info_msg = get_default_camera_info_from_image(msg);
    image_pub_.publish(*msg_, cam_info_msg_, ros::Time::now());

    ros::spinOnce();

    r.sleep();
  }
}

// void ProcessFileNodelet::configCb(Config &config,uint32_t level)
// {
//   config_ = config;
// }

} // namespace raw_video_stream

// register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( raw_video_stream::ProcessFileNodelet,nodelet::Nodelet)
