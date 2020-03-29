#include  <ros/ros.h>
#include  <cherry/Status.h>
#include  <cherry/INSRMS.h>
#include <cherry/INS.h>

#include<tf/transform_broadcaster.h>



double origin_longitude;
double origin_latitude;
double origin_altitude;

bool first_flag;

void insCallback(const cherry::INSConstPtr& msg)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;

    if (first_flag)
    {
        origin_altitude = msg->altitude;
        origin_latitude = msg->latitude;
        origin_longitude = msg->longitude;
        first_flag = false;
    }
    transform.setOrigin( tf::Vector3(msg->longitude,msg->latitude,msg->altitude) );
    tf::Quaternion q ;
    
    q.setRPY(0, 0, 0);
    
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, msg->header.stamp,"world","lidar_center"));

}

int main(int argc, char **argv) {
    ros::init(argc,argv,"LidarInsTf");
    ros::NodeHandle nh;
    first_flag = true;
    ros::Subscriber sub = nh.subscribe("/ins/data" , 100, &insCallback);
    ros::spin();
    return 0;
}