#include "ros/ros.h"
#include "ros/console.h"
#include<iostream>
#include<fstream>
#include<ros/package.h>
#include<boost/thread.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#define foreach BOOST_FOREACH

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "split_bag");
	ros::start();

	ros::NodeHandle n;
	
	string bagname;
	vector<string> topics;
	if(!ros::param::get("~bagname",bagname))
	{
		ROS_ERROR("Can not find bag name");
		exit(0);
	}
	
	bool save_laser, save_image_left, save_image_right, save_imu;
	
	string lasermsgname;
	if(!ros::param::get("~laser_msg_name",lasermsgname))
	{
		ROS_ERROR("Skip laser msg name");
		save_laser = false;
	}else{
		topics.push_back(lasermsgname);
		save_laser = true;
	}
	
	string imumsgname;
	if(!ros::param::get("~imu_msg_name",imumsgname))
	{
		ROS_ERROR("Skip imu msg name");
		save_imu = false;
		
	}else{
		cout <<"Skip imu msg "<< imumsgname<<endl;
		topics.push_back(imumsgname);
		save_imu = true;
	}
	
	string odommsgname;
	if(!ros::param::get("~odom_msg_name",odommsgname))
	{
		ROS_ERROR("Skip odom msg name");
	}else{
		topics.push_back(odommsgname);
	}
	
	string leftimagemsgname;
	if(!ros::param::get("~leftimage_msg_name",leftimagemsgname))
	{
		ROS_ERROR("Skip leftimage msg name");
		save_image_left = false;
	}else{
		topics.push_back(leftimagemsgname);
		save_image_left = true;
	}
	
	string rightimagemsgname;
	if(!ros::param::get("~rightimage_msg_name",rightimagemsgname))
	{
		ROS_ERROR("Skip rightimage msg name");
		save_image_right = false;
	}else{
		topics.push_back(rightimagemsgname);
		save_image_right = true;
	}
	
	string savingdirname;
	if(!ros::param::get("~saving_dir",savingdirname))
	{
		ROS_ERROR("Can not find saving dir name");
		exit(1);
	}
	

	
	string laserDirname = savingdirname + "/laser";
	string laserTimestampName = savingdirname + "/laserTimestamp.txt";
	string baseimagedirname = savingdirname + "/image";
	string leftimageDirname = baseimagedirname + "/left";
	string rightimageDirname = baseimagedirname + "/right";
	string leftimageTimestampname = savingdirname + "/leftImageTimestamp.txt";
	string rightimageTimestampname = savingdirname + "/rightImageTimestamp.txt";
	string imuDirname = savingdirname + "/imu.txt";
	string imuTimestampDirname = savingdirname + "/imuTimestamp.txt";
	
	system(("mkdir " + laserDirname).c_str());
	system(("mkdir "+baseimagedirname ).c_str());
	system(("mkdir " + leftimageDirname).c_str());
	system(("mkdir " + rightimageDirname).c_str());
	
	
	ofstream laserTimestampFile;
	if(save_laser){laserTimestampFile.open(laserTimestampName);}
	ofstream leftimageTimestampFile;
	if(save_image_left) leftimageTimestampFile.open(leftimageTimestampname);
	ofstream rightimageTimestampFile;
	if(save_image_right) rightimageTimestampFile.open(rightimageTimestampname);
	ofstream imuDataFile;
	if(save_imu) imuDataFile.open(imuDirname);
	ofstream imuTimestampFile;
	if(save_imu) imuTimestampFile.open(imuTimestampDirname);
	
    rosbag::Bag databag;
    databag.open(bagname, rosbag::bagmode::Read);
	ROS_INFO ( "bag opened"); 
	cout << topics.size()<<endl;
	
	rosbag::View view(databag, rosbag::TopicQuery(topics));
	cout<<view.size()<<endl;
	
	int lasercnt = 0, leftimagecnt = 0, rightimagecnt = 0;

	
	foreach(rosbag::MessageInstance const m, view){
	
		if(!ros::ok())
			break;

		sensor_msgs::PointCloud2::Ptr lidarData = m.instantiate<sensor_msgs::PointCloud2>();

		
		sensor_msgs::Image::Ptr imageData = m.instantiate<sensor_msgs::Image>();
		
		if(imageData){
			
			cv_bridge::CvImageConstPtr cv_ptr_image;
			try
			{
				cv_ptr_image = cv_bridge::toCvShare(imageData);
				if(m.getTopic() == leftimagemsgname){
					cout << "read "<< leftimagecnt << " left image " << imageData->header.stamp<<endl;
					cv::imwrite(leftimageDirname + "/" + to_string(leftimagecnt++) + ".png", cv_ptr_image->image);
					// leftimageTimestampFile << imageData->header.stamp << endl;
				}
				
				if(m.getTopic() == rightimagemsgname){
					cout << "read " << rightimagecnt <<" right image " << imageData->header.stamp<<endl;
					cv::imwrite(rightimageDirname + "/" + to_string(rightimagecnt++) + ".png", cv_ptr_image->image);
					// rightimageTimestampFile << imageData->header.stamp << endl;
				}
				
				
			}
			catch (cv_bridge::Exception& e)
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
				break;
			}
			
		}
		
		sensor_msgs::Imu::Ptr ImuData = m.instantiate<sensor_msgs::Imu>();
		if(ImuData){
			
			stringstream s;
			s<< ImuData->header.stamp<<endl;
			cout <<s.str()<<endl;
			
			imuTimestampFile << s.str() ;
		}

	}
	
	leftimageTimestampFile.close();
	rightimageTimestampFile.close();
	imuTimestampFile.close();
}
