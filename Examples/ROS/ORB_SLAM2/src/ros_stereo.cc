/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/CompressedImage.h>
#include <nav_msgs/Odometry.h>
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/NavSatFix.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::CompressedImageConstPtr& msgLeft,const sensor_msgs::CompressedImageConstPtr& msgRight);
    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
    ros::Publisher odom_pub;
    ros::Publisher odom_gps;
    tf::TransformBroadcaster br;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    ImageGrabber igb(&SLAM);

    stringstream ss(argv[3]);
	ss >> boolalpha >> igb.do_rectify;

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

    ros::NodeHandle nh;

    igb.odom_pub = nh.advertise<nav_msgs::Odometry>("orbslam/odom", 50);
    igb.odom_gps = nh.advertise<sensor_msgs::NavSatFix>("orbslam/gps", 50);

    message_filters::Subscriber<sensor_msgs::CompressedImage> left_sub(nh, "/camera/left/image_raw/compressed", 1);
    message_filters::Subscriber<sensor_msgs::CompressedImage> right_sub(nh, "/camera/right/image_raw/compressed", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::CompressedImageConstPtr& msgLeft,const sensor_msgs::CompressedImageConstPtr& msgRight)
{
    ros::Time current_time = ros::Time::now();
    // Copy the ros image message to cv::Mat.
    //cv_bridge::CvImageConstPtr cv_ptrLeft;
    cv::Mat cv_imageLeft;
    try
    {
        //cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
        cv_imageLeft = cv::imdecode(cv::Mat(msgLeft->data), CV_LOAD_IMAGE_UNCHANGED);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //cv_bridge::CvImageConstPtr cv_ptrRight;
    cv::Mat cv_imageRight;
    try
    {
        //cv_ptrRight = cv_bridge::toCvShare(msgRight);
        cv_imageRight = cv::imdecode(cv::Mat(msgRight->data), CV_LOAD_IMAGE_UNCHANGED);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    //cout<<"hi"<<endl;
    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::Mat resizeLeft, resizeRight;
        cv::resize(cv_imageLeft,resizeLeft,cv::Size(320,180));
        cv::resize(cv_imageRight,resizeRight,cv::Size(320,180));
        cv::remap(resizeLeft,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(resizeRight,imRight,M1r,M2r,cv::INTER_LINEAR);
        cv::Mat pose = mpSLAM->TrackStereo(imLeft,imRight,msgLeft->header.stamp.toSec());
        
        if (pose.empty())
        return;

        /* global left handed coordinate system */
        static cv::Mat pose_prev = cv::Mat::eye(4,4, CV_32F);
        static cv::Mat world_lh = cv::Mat::eye(4,4, CV_32F);
        // matrix to flip signs of sinus in rotation matrix, not sure why we need to do that
        static const cv::Mat flipSign = (cv::Mat_<float>(4,4) <<   1,-1,-1, 1,
                                                                   -1, 1,-1, 1,
                                                                   -1,-1, 1, 1,
                                                                    1, 1, 1, 1);

        //prev_pose * T = pose
        cv::Mat translation =  (pose * pose_prev.inv()).mul(flipSign);
        world_lh = world_lh * translation;
        pose_prev = pose.clone();


        /* transform into global right handed coordinate system, publish in ROS*/
        tf::Matrix3x3 cameraRotation_rh(  - world_lh.at<float>(0,0),   world_lh.at<float>(0,1),   world_lh.at<float>(0,2),
                                      - world_lh.at<float>(1,0),   world_lh.at<float>(1,1),   world_lh.at<float>(1,2),
                                        world_lh.at<float>(2,0), - world_lh.at<float>(2,1), - world_lh.at<float>(2,2));

        tf::Vector3 cameraTranslation_rh( world_lh.at<float>(0,3),world_lh.at<float>(1,3), - world_lh.at<float>(2,3) );

        //rotate 270deg about x and 270deg about x to get ENU: x forward, y left, z up
        const tf::Matrix3x3 rotation270degXZ(   0, 1, 0,
                                                0, 0, 1,
                                                1, 0, 0);


        //tf::Matrix3x3 globalRotation_rh = cameraRotation_rh * rotation270degXZ;
        tf::Vector3 globalTranslation_rh = cameraTranslation_rh * rotation270degXZ;
        //tf::Transform transform = tf::Transform(globalRotation_rh, globalTranslation_rh);
        //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_link", "camera_pose"));

        //publish odometry
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);

        //set the position
        odom.pose.pose.position.x = globalTranslation_rh[0];
        odom.pose.pose.position.y = globalTranslation_rh[1];
        odom.pose.pose.position.z = 0.5;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = 0;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = 0;

        //publish the message
        odom_pub.publish(odom);

        sensor_msgs::NavSatFix gps_msg;
        gps_msg.header.stamp = current_time;
        gps_msg.header.frame_id = "orb_gps";
        gps_msg.status.status = 0;
        gps_msg.status.service = 1;
        float pi = 3.14;
        gps_msg.latitude = 22.3191537 + (180/pi)*(odom.pose.pose.position.y/6378.137);
        gps_msg.longitude = 87.3027312 + (180/pi)*(odom.pose.pose.position.x/6378.137)/cos(22.3191537);
        gps_msg.altitude = 47.89;
        gps_msg.position_covariance = {3.0276000331878663, 0.0, 0.0, 0.0, 3.0276000331878663, 0.0, 0.0, 0.0, 12.110400132751465};
        gps_msg.position_covariance_type = 1;
        odom_gps.publish(gps_msg);

    }
    else
    {
         mpSLAM->TrackStereo(cv_imageLeft,cv_imageRight,msgLeft->header.stamp.toSec());
    }

}


