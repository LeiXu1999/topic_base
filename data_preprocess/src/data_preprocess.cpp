#include<iostream>
#include<vector>
#include<boost/array.hpp>
#include<ros/ros.h>
#include<sensor_msgs/Imu.h>
#include<nav_msgs/Odometry.h>
#include<nav_msgs/OccupancyGrid.h>
#include<geometry_msgs/PoseWithCovariance.h>
#include<geometry_msgs/TwistWithCovariance.h>
#include <tf/transform_datatypes.h>
#include<vector>
#include<cmath>
class map_database{
    public:
    map_database();
    unsigned int map_width, map_height;
    double scale,dx,dy,rz;
    map_database(nav_msgs::OccupancyGrid msg);
};
map_database::map_database(){

}
map_database::map_database(nav_msgs::OccupancyGrid msg){
    map_width = msg.info.width;
    map_height = msg.info.height;
    scale = msg.info.resolution;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.info.origin.orientation, quat);
    dx = msg.info.origin.position.x;
    dy = msg.info.origin.position.y;
    double roll,pitch,yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    rz = yaw;
    std::cout << " the origin of the maper from /map is x: "<< dx << ", y: "<<dy <<", rz: "<<rz<<std::endl;   
}
class data_preprocess{
    public:
    data_preprocess();
    private:
    ros::NodeHandle nh;
    ros::NodeHandle ph;

    ros::Subscriber sub_imu;
    ros::Subscriber sub_odom;
    ros::Subscriber sub_pose;
    ros::Subscriber sub_twist;
    ros::Subscriber sub_map;
    ros::Subscriber sub_fusion_odom;
    //todo  需要一个订阅地图的回调函数 来触发model_flag的切换;

    ros::Publisher pub_imu;
    ros::Publisher pub_odom;
    ros::Publisher pub_pose;
    ros::Publisher pub_twist;

    std::string imu_topic_name;
    std::string odom_topic_name;
    std::string pose_topic_name;
    std::string twist_topic_name;
    std::string map_topic_name;
    std::string fusion_odom_topic_name;
    

    std::vector<std::vector<float>> weight_model;
    int model_flag = 0;
    //int model_num;
    map_database map_info;
    nav_msgs::OccupancyGrid grid_map;
    

    void imu_callback(const sensor_msgs::ImuConstPtr & msg);
    void odom_callback(const nav_msgs::OdometryConstPtr & msg);
    void pose_callback(const geometry_msgs::PoseWithCovarianceConstPtr & msg);
    void twist_callback(const geometry_msgs::TwistWithCovarianceConstPtr & msg);
    
    void fusion_odom_callback(const nav_msgs::OdometryConstPtr & msg);
    void map_callback(const nav_msgs::OccupancyGridConstPtr & msg);
    void init_param();

    boost::array<double, 9> process_weight_9 ( boost::array<double, 9> arr, float weight);
    boost::array<double, 36> process_weight_36 ( boost::array<double, 36> arr, float weight);


};
data_preprocess::data_preprocess()
:ph("~")
{
    init_param();
    sub_imu = nh.subscribe(imu_topic_name,1000,&data_preprocess::imu_callback,this);
    sub_odom = nh.subscribe(odom_topic_name,1000,&data_preprocess::odom_callback,this);
    sub_pose = nh.subscribe(pose_topic_name,1000,&data_preprocess::pose_callback,this);
    sub_twist = nh.subscribe(twist_topic_name,1000,&data_preprocess::twist_callback,this);
    sub_map = nh.subscribe(map_topic_name,1000,&data_preprocess::map_callback,this);
    sub_fusion_odom = nh.subscribe(fusion_odom_topic_name,1000,&data_preprocess::fusion_odom_callback,this);

    pub_imu = nh.advertise<sensor_msgs::Imu>(imu_topic_name+"/processed",1000);
    pub_odom = nh.advertise<nav_msgs::Odometry>(odom_topic_name+"/processed",1000);
    pub_pose = nh.advertise<geometry_msgs::PoseWithCovariance>(pose_topic_name+"/processed",1000);
    pub_twist = nh.advertise<geometry_msgs::TwistWithCovariance>(twist_topic_name+"/processed",1000);
}
boost::array<double, 9> data_preprocess::process_weight_9 ( boost::array<double, 9> arr, float weight){
    boost::array<double, 9> res ;
    for ( int i = 0; i < arr.size(); i++ ){
        res[i] = arr[i] / weight;
    }
    return  res;
}
boost::array<double, 36> data_preprocess::process_weight_36 ( boost::array<double, 36> arr, float weight){
    boost::array<double, 36> res ;
    for ( int i = 0; i < arr.size(); i++ ){
        res[i] = arr[i] / weight;
    }
    return  res;
}
void data_preprocess::init_param()
{
    ph.param("imu_topic_name",imu_topic_name,std::string("/imu"));
    ph.param("odom_topic_name",odom_topic_name,std::string("/odom"));
    ph.param("pose_topic_name",pose_topic_name,std::string("/pose"));
    ph.param("twist_topic_name",twist_topic_name,std::string("/twist"));
    ph.param("map_topic_name",map_topic_name,std::string("/map"));
    ph.param("fusion_odom_topic_name",fusion_odom_topic_name,std::string("/fusion_odom"));

    std::vector<float> model0 = { 1.0, 1.0, 1.0, 1.0 }; //不变
    std::vector<float> model1 = { 4.0, 1.0, 2.0, 3.0 }; //更相信 IMU 
    std::vector<float> model2 = { 3.0, 4.0, 1.0, 2.0 }; //更相信 odom
    std::vector<float> model3 = { 2.0, 3.0, 4.0, 1.0 }; //更相信 pose
    std::vector<float> model4 = { 1.0, 2.0, 3.0, 4.0 }; //更相信 twist

    weight_model = {model1,model2,model3,model4};
}
void data_preprocess::imu_callback(const sensor_msgs::ImuConstPtr & msg)
{   
    sensor_msgs::Imu _msg = *msg;
    auto r_c = _msg.orientation_covariance;
    auto w_c = _msg.angular_velocity_covariance;
    auto a_c = _msg.linear_acceleration_covariance;

    _msg.orientation_covariance = process_weight_9(r_c,weight_model[model_flag][1]);
    _msg.angular_velocity_covariance = process_weight_9(w_c,weight_model[model_flag][1]);
    _msg.linear_acceleration_covariance = process_weight_9(a_c,weight_model[model_flag][1]);

    pub_imu.publish(_msg);
}
void data_preprocess::odom_callback(const nav_msgs::OdometryConstPtr & msg)
{ 
    nav_msgs::Odometry _msg =  *msg;
    auto p_c = _msg.pose.covariance;
    _msg.pose.covariance = process_weight_36(p_c,weight_model[model_flag][2]);
    auto t_c = _msg.twist.covariance;
    _msg.twist.covariance = process_weight_36(t_c,weight_model[model_flag][2]);
    pub_odom.publish(_msg);
}
void data_preprocess::pose_callback(const geometry_msgs::PoseWithCovarianceConstPtr & msg)
{
    geometry_msgs::PoseWithCovariance _msg = *msg;
    auto p_c = _msg.covariance;
    _msg.covariance = process_weight_36(p_c,weight_model[model_flag][3]);
    pub_pose.publish(_msg);
}
void data_preprocess::twist_callback(const geometry_msgs::TwistWithCovarianceConstPtr &msg)
{
    geometry_msgs::TwistWithCovariance _msg = *msg;
    auto t_c = _msg.covariance;
    _msg.covariance = process_weight_36(t_c,weight_model[model_flag][4]);
    pub_twist.publish(_msg);
}
void data_preprocess::fusion_odom_callback(const nav_msgs::OdometryConstPtr & msg)
{
    nav_msgs::Odometry _msg =  *msg;
    if(_msg.header.frame_id != "/map"){
        std::cout << "the frame_id of fusion_odom is not /map!!"<< std::endl;
    }
    else{
        auto x = _msg.pose.pose.position.x;
        auto y = _msg.pose.pose.position.y;
        int id_x,id_y;
        id_x = int (( x*cos(map_info.rz) + y*sin(map_info.rz) - map_info.dx )/ map_info.scale);
        id_y = int (( y*cos(map_info.rz) - x*sin(map_info.rz) - map_info.dy )/ map_info.scale);    
        if( id_x >=0 && id_x <= map_info.map_width && id_y >=0 && id_y <= map_info.map_height )
        {
            int data = int (grid_map.data[id_x+id_y*map_info.map_height]);
            switch (data)
            {
                case 21:
                    //all
                    model_flag = 0;
                    std::cout<<"model_flag change: "<<model_flag << std::endl;
                    break;
                case 41:
                    //imu
                    model_flag = 1;
                    std::cout<<"model_flag change: "<<model_flag << std::endl; 
                    break;
                case 60:
                    //odom
                    model_flag = 2; 
                    std::cout<<"model_flag change: "<<model_flag << std::endl;
                    break;
                case 80:
                    //pose
                    model_flag = 3; 
                    std::cout<<"model_flag change: "<<model_flag << std::endl;
                    break;
                case 100:
                    //twist
                    model_flag = 4; 
                    std::cout<<"model_flag change: "<<model_flag << std::endl;
                    break;
                default:
                    std::cout << " new grey in the map which is not record" << std::endl << " model_flag does not change"  << std::endl;
                    break;
                }
        }
        else   {
            std::cout << " odom is over the map" << std::endl << " model_flag change to 0 (all)" << std::endl;
            model_flag = 0;
        } 
    }




}
void data_preprocess::map_callback(const nav_msgs::OccupancyGridConstPtr & msg)
{
    grid_map = *msg;
    map_database tmp(grid_map);
    map_info = tmp;
    //grid_map_data.data = _msg.data;
}
int main(int argc, char ** argv){

    ros::init(argc, argv,"data_preprocess");
    data_preprocess processer;
    ros::spin();

    //std::cout << "hello " << std::endl;
    return 0;
}
