/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>
#include<queue>
#include<thread>
#include<mutex>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Imu.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"../include/ImuTypes.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include "nav_msgs/Odometry.h"


using namespace std;

/*static ros::Publisher pub1;
static ros::Publisher pub2;

class CallBack{
  public:
  CallBack(){};
  
  void callback(const geometry_msgs::PoseStamped& pose){
    geometry_msgs::PoseStamped pub_pose;
    pub_pose.header.stamp = ros::Time::now();
    pub_pose.header.frame_id = "world";
    pub_pose.pose.position.x = pose.pose.position.x;
    pub_pose.pose.position.y = pose.pose.position.y;
    pub_pose.pose.position.z = pose.pose.position.z;
    pub_pose.pose.orientation.x = pose.pose.orientation.x;
    pub_pose.pose.orientation.y = pose.pose.orientation.y;
    pub_pose.pose.orientation.z = pose.pose.orientation.z;
    pub_pose.pose.orientation.w = pose.pose.orientation.w;
    pub1.publish(pub_pose);
    //std::cout<<pub_pose.pose.position.x<<std::endl;
  }

  void callback2(const geometry_msgs::PoseStamped& odometry){
    geometry_msgs::PoseStamped pub_odometry;
    pub_odometry.header.stamp = ros::Time::now();
    pub_odometry.header.frame_id = "world";
    pub_odometry.pose.position.x = odometry.pose.position.x;
    pub_odometry.pose.position.y = odometry.pose.position.y;
    pub_odometry.pose.position.z = odometry.pose.position.z;
    pub_odometry.pose.orientation.x = odometry.pose.orientation.x;
    pub_odometry.pose.orientation.y = odometry.pose.orientation.y;
    pub_odometry.pose.orientation.z = odometry.pose.orientation.z;
    pub_odometry.pose.orientation.w = odometry.pose.orientation.w;
    pub2.publish(pub_odometry);
    //std::cout<<pub_pose.pose.position.x<<std::endl;
  }

};*/

class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);
    void GrabImuPix(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf, imuPixBuf;
    std::mutex mBufMutex;
    std::mutex mPixBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb, const bool bRect, const bool bClahe): mpSLAM(pSLAM), mpImuGb(pImuGb), do_rectify(bRect), mbClahe(bClahe){}

    void GrabImageLeft(const sensor_msgs::ImageConstPtr& msg);
    void GrabImageRight(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();
    
    Eigen::Vector3f twc;//以imu为中心
    Eigen::Quaternionf Qwb;
    Eigen::Vector3f Vwc;
    Eigen::Vector3f Angvel;

    queue<sensor_msgs::ImageConstPtr> imgLeftBuf, imgRightBuf;
    std::mutex mBufMutexLeft,mBufMutexRight;
   
    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;

    const bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "Stereo_Inertial");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  bool bEqual = false;
  if(argc < 4 || argc > 5)
  {
    cerr << endl << "Usage: rosrun ORB_SLAM3 Stereo_Inertial path_to_vocabulary path_to_settings do_rectify [do_equalize]" << endl;
    ros::shutdown();
    return 1;
  }

  std::string sbRect(argv[3]);
  if(argc==5)
  {
    std::string sbEqual(argv[4]);
    if(sbEqual == "true")
      bEqual = true;
  }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_STEREO,false);//turn off viewer thread

  ImuGrabber imugb;
  ImageGrabber igb(&SLAM,&imugb,sbRect == "true",bEqual);

  if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

  // Maximum delay, 5 seconds
  //ros::Subscriber sub_imu = n.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb);
  //ros::Subscriber sub_img_left = n.subscribe("/camera/left/image_raw", 100, &ImageGrabber::GrabImageLeft,&igb);
  //ros::Subscriber sub_img_right = n.subscribe("/camera/right/image_raw", 100, &ImageGrabber::GrabImageRight,&igb);
  
  //D435i
  //ros::Subscriber sub_imu = n.subscribe("/mavros/imu/data", 1000, &ImuGrabber::GrabImu, &imugb);
  ros::Subscriber sub_imu = n.subscribe("/camera/imu", 1000, &ImuGrabber::GrabImu, &imugb);
  //ros::Subscriber sub_imu_pix = n.subscribe("/mavros/imu/data", 1000, &ImuGrabber::GrabImuPix, &imugb);  
  ros::Subscriber sub_img_left = n.subscribe("/camera/infra1/image_rect_raw", 100, &ImageGrabber::GrabImageLeft,&igb);
  ros::Subscriber sub_img_right = n.subscribe("/camera/infra2/image_rect_raw", 100, &ImageGrabber::GrabImageRight,&igb);
  
  //Euroc
  //ros::Subscriber sub_imu = n.subscribe("/imu0", 1000, &ImuGrabber::GrabImu, &imugb); 
  //ros::Subscriber sub_img_left = n.subscribe("/cam0/image_raw", 100, &ImageGrabber::GrabImageLeft,&igb);
  //ros::Subscriber sub_img_right = n.subscribe("/cam1/image_raw", 100, &ImageGrabber::GrabImageRight,&igb);
  
  std::thread sync_thread(&ImageGrabber::SyncWithImu,&igb);
  
  //ros::Publisher pub_pos = n.advertise<geometry_msgs::PoseStamped> ("pos", 1);
  //ros::Publisher pub_vel = n.advertise<geometry_msgs::TwistStamped> ("vel", 1);
  ros::Publisher pub_odo = n.advertise<nav_msgs::Odometry> ("odometry", 1000);
  ros::Rate loop_rate(120);
  //geometry_msgs::PoseStamped msgp;
  //geometry_msgs::TwistStamped msgv;
  nav_msgs::Odometry msgodo;
  
  int pub_count =0;
  while(ros::ok()){
  
  // Odometry
  msgodo.header.stamp = ros::Time::now();
  msgodo.header.frame_id = "world";
  msgodo.child_frame_id = "world";
  msgodo.pose.pose.position.x = igb.twc(0);
  msgodo.pose.pose.position.y = igb.twc(1);
  msgodo.pose.pose.position.z = igb.twc(2);
  msgodo.pose.pose.orientation.x = igb.Qwb.x();
  msgodo.pose.pose.orientation.y = igb.Qwb.y();
  msgodo.pose.pose.orientation.z = igb.Qwb.z();
  msgodo.pose.pose.orientation.w = igb.Qwb.w();
  msgodo.twist.twist.linear.x = igb.Vwc(0);
  msgodo.twist.twist.linear.y = igb.Vwc(1);
  msgodo.twist.twist.linear.z = igb.Vwc(2);
  msgodo.twist.twist.angular.x = igb.Angvel(0);
  msgodo.twist.twist.angular.y = igb.Angvel(1);
  msgodo.twist.twist.angular.z = igb.Angvel(2);
   

  pub_odo.publish(msgodo);

  std::cout << "UAVposition " << msgodo.pose.pose.position.x <<" , "<< msgodo.pose.pose.position.y <<" , "<< msgodo.pose.pose.position.z << endl;
  std::cout << "UAVQuaternion " << msgodo.pose.pose.orientation.x  <<" , "<< msgodo.pose.pose.orientation.y <<" , "<< msgodo.pose.pose.orientation.z << " , " << msgodo.pose.pose.orientation.w << endl;
  std::cout << "UAVVelocity " << msgodo.twist.twist.linear.x <<" , "<< msgodo.twist.twist.linear.y <<" , "<< msgodo.twist.twist.linear.z << endl;
  std::cout << "UAVAngvel " << msgodo.twist.twist.angular.x <<" , "<< msgodo.twist.twist.angular.y <<" , "<< msgodo.twist.twist.angular.z << endl;
  std::cout << " " << endl;

  ros::spinOnce();
  loop_rate.sleep();
  ++pub_count;
  }

  ros::spin();

  return 0;
}



void ImageGrabber::GrabImageLeft(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutexLeft.lock();
  if (!imgLeftBuf.empty())
    imgLeftBuf.pop();
  imgLeftBuf.push(img_msg);
  mBufMutexLeft.unlock();
}

void ImageGrabber::GrabImageRight(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutexRight.lock();
  if (!imgRightBuf.empty())
    imgRightBuf.pop();
  imgRightBuf.push(img_msg);
  mBufMutexRight.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  
  if(cv_ptr->image.type()==0)
  {
    return cv_ptr->image.clone();
  }
  else
  {
    std::cout << "Error type" << std::endl;
    return cv_ptr->image.clone();
  }
}

void ImageGrabber::SyncWithImu()
{
  const double maxTimeDiff = 0.01;
  while(1)
  {
    cv::Mat imLeft, imRight;
    double tImLeft = 0, tImRight = 0;
    if (!imgLeftBuf.empty()&&!imgRightBuf.empty()&&!mpImuGb->imuBuf.empty())
    {
      tImLeft = imgLeftBuf.front()->header.stamp.toSec();
      tImRight = imgRightBuf.front()->header.stamp.toSec();

      this->mBufMutexRight.lock();
      while((tImLeft-tImRight)>maxTimeDiff && imgRightBuf.size()>1)
      {
        imgRightBuf.pop();
        tImRight = imgRightBuf.front()->header.stamp.toSec();
      }
      this->mBufMutexRight.unlock();

      this->mBufMutexLeft.lock();
      while((tImRight-tImLeft)>maxTimeDiff && imgLeftBuf.size()>1)
      {
        imgLeftBuf.pop();
        tImLeft = imgLeftBuf.front()->header.stamp.toSec();
      }
      this->mBufMutexLeft.unlock();

      if((tImLeft-tImRight)>maxTimeDiff || (tImRight-tImLeft)>maxTimeDiff)
      {
        // std::cout << "big time difference" << std::endl;
        continue;
      }
      if(tImLeft>mpImuGb->imuBuf.back()->header.stamp.toSec())
          continue;

      this->mBufMutexLeft.lock();
      imLeft = GetImage(imgLeftBuf.front());
      imgLeftBuf.pop();
      this->mBufMutexLeft.unlock();

      this->mBufMutexRight.lock();
      imRight = GetImage(imgRightBuf.front());
      imgRightBuf.pop();
      this->mBufMutexRight.unlock();

      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      mpImuGb->mBufMutex.lock();
      if(!mpImuGb->imuBuf.empty())
      {
        // Load imu measurements from buffer
        vImuMeas.clear();
        
        // origin ORB3 piximu
         /*while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=tImLeft)
         {
           double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
           cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
           cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
           vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
          
           mpImuGb->imuBuf.pop();
         }*/
        
        // for D435i imu
        while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=tImLeft)
        {
          double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
          cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.z, -1 * mpImuGb->imuBuf.front()->linear_acceleration.x, -1 * mpImuGb->imuBuf.front()->linear_acceleration.y);
          cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.z, -1 * mpImuGb->imuBuf.front()->angular_velocity.x, -1 * mpImuGb->imuBuf.front()->angular_velocity.y);
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
          
          mpImuGb->imuBuf.pop();
        }

      }
      mpImuGb->mBufMutex.unlock();
      if(mbClahe)
      {
        mClahe->apply(imLeft,imLeft);
        mClahe->apply(imRight,imRight);
      }

      if(do_rectify)
      {
        cv::remap(imLeft,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(imRight,imRight,M1r,M2r,cv::INTER_LINEAR);
      }

      mpSLAM->TrackStereo(imLeft,imRight,tImLeft,vImuMeas);
      
      Eigen::Matrix<float,3,3> Tsp;//从SLAM到pix
      Tsp(0,0) = 0.0;
      Tsp(0,1) = 1.0;
      Tsp(0,2) = 0.0;
      Tsp(1,0) = -1.0;
      Tsp(1,1) = 0.0;
      Tsp(1,2) = 0.0;
      Tsp(2,0) = 0.0;
      Tsp(2,1) = 0.0;
      Tsp(2,2) = 1.0;

      Eigen::Matrix<float,3,3> Tcs;//从cameraimu到SLAM
      Tcs(0,0) = 1.0;
      Tcs(0,1) = 0.0;
      Tcs(0,2) = 0.0;
      Tcs(1,0) = 0.0;
      Tcs(1,1) = 0.0;
      Tcs(1,2) = 1.0;
      Tcs(2,0) = 0.0;
      Tcs(2,1) = -1.0;
      Tcs(2,2) = 0.0;

      Eigen::Matrix<float,3,3> Tpp;//从姿态输出转到SLAM坐标系
      Tpp(0,0) = 1.0;
      Tpp(0,1) = 0.0;
      Tpp(0,2) = 0.0;
      Tpp(1,0) = 0.0;
      Tpp(1,1) = -1.0;
      Tpp(1,2) = 0.0;
      Tpp(2,0) = 0.0;
      Tpp(2,1) = 1.0;
      Tpp(2,2) = 0.0;

      /*Eigen::Quaterniond Qsp;
      Qsp.w() = 0.7071068;
      Qsp.x() = 0.0;
      Qsp.y() = 0.0;
      Qsp.z() = -0.7071068;*/
            
      twc = Tsp * mpSLAM->UAVPosition();//以imu为中心
      Qwb = Tsp * mpSLAM->UAVRotation();
      Qwb = Qwb.normalized();//不需要normalize也会是normalize
      Vwc = Tsp * mpSLAM->UAVVelocity();
      Angvel = mpSLAM->UAVAngvel();
      
      //Angvel = Tsp * Tcs * mpSLAM->UAVAngvel(); //translation and rotation for camera imu, but seems useless
      
            
      //std::cout << "Quaternion" <<endl <<Qwb.coeffs()<<endl;
            
      /*Eigen::Vector3f kkk;
      UAVAngvel(Eigen::Vector3f& kkk);//不用return这样传速度更快*/
      
      /*cout << "UAVposition" << twc(0)<<","<<twc(1)<<","<<twc(2) << endl;
      cout << "UAVQuaternion" << Qwb.w() <<"," << Qwb.x() <<"," << Qwb.y() <<"," << Qwb.z() << endl;//need to change
      cout << "UAVVelocity" << Vwc << endl;
      cout << "UAVAngvel" << Angvel << endl;*/
      //cout << "UAVAngvel" << mpImuGb->imuBuf.front()->angular_velocity << endl;


      std::chrono::milliseconds tSleep(1);
      std::this_thread::sleep_for(tSleep);
    }
  }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
  mBufMutex.lock();
  imuBuf.push(imu_msg);
  mBufMutex.unlock();
  return;
}

void ImuGrabber::GrabImuPix(const sensor_msgs::ImuConstPtr &imu_msg)
{
  mPixBufMutex.lock();
  imuPixBuf.push(imu_msg);
  mPixBufMutex.unlock();
  return;
}


