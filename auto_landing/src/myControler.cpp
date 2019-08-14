//
// Created by lucius on 19-8-5.
//
/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

class MyControler {
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> MySyncPolicy;

    ros::NodeHandle nh;
    ros::Subscriber state_sub;
    ros::Publisher local_pos_pub;
    ros::Publisher image_pos_pub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;


    mavros_msgs::State current_state;

    geometry_msgs::PoseStamped pose;

    ros::Subscriber control_sub;
    ros::Subscriber control_x_sub;
    ros::Subscriber control_y_sub;

    geometry_msgs::PoseStamped local_pose;
    std::string con_msg;
    float con_x;
    float con_y;
    geometry_msgs::PoseStamped vision_pose;
    geometry_msgs::PoseStamped image_local_pose_pub;

    message_filters::Subscriber<geometry_msgs::PoseStamped> local_pose_sub;
    message_filters::Subscriber<geometry_msgs::PoseStamped> vision_pose_sub;
    message_filters::Synchronizer<MySyncPolicy> sync;
public:
    MyControler(): state_sub(nh.subscribe<mavros_msgs::State>("uav0/mavros/state", 10, &MyControler::state_cb, this)),
                    local_pos_pub(nh.advertise<geometry_msgs::PoseStamped>("uav0/mavros/setpoint_position/local", 10)),
                    image_pos_pub(nh.advertise<geometry_msgs::PoseStamped>("/MyDetector/pose", 10)),
                    arming_client(nh.serviceClient<mavros_msgs::CommandBool>("uav0/mavros/cmd/arming")),
                    set_mode_client(nh.serviceClient<mavros_msgs::SetMode>("uav0/mavros/set_mode")),
                    control_sub(nh.subscribe<std_msgs::String>("/MyControler/cmd_vel", 10, &MyControler::cmd_vel_cb, this)),
                    con_x(0.0),
                    con_y(0.0),
                    control_x_sub(nh.subscribe<std_msgs::Float32>("/MyControler/cmd_x", 10, &MyControler::con_x_cb, this)),
                    control_y_sub(nh.subscribe<std_msgs::Float32>("/MyControler/cmd_y", 10, &MyControler::con_y_cb, this)),
                    local_pose_sub(nh, "/uav0/mavros/local_position/pose", 1),
                    vision_pose_sub(nh, "/visp_auto_tracker/object_position", 1),
                    sync(MySyncPolicy(10), local_pose_sub, vision_pose_sub)
    {
      sync.registerCallback(boost::bind(&MyControler::time_sync_cb, this, _1, _2));
    }

    void time_sync_cb(const geometry_msgs::PoseStamped::ConstPtr& local_msg, const geometry_msgs::PoseStamped::ConstPtr& vision_msg)
    {
      local_pose = *local_msg;
      vision_pose = *vision_msg;
    }
    void con_x_cb(const std_msgs::Float32::ConstPtr& msg){
      con_x = msg->data;
    }
    void con_y_cb(const std_msgs::Float32::ConstPtr& msg){
      con_y = msg->data;
    }

    void cmd_vel_cb(const std_msgs::String::ConstPtr& msg){
      con_msg = msg->data;
    }

    void state_cb(const mavros_msgs::State::ConstPtr& msg){
      current_state = *msg;
    }

    void spin()
    {
      static tf::TransformBroadcaster br;
      tf::TransformListener listener;
      tf::StampedTransform transform;
      geometry_msgs::PoseStamped base_point;
      geometry_msgs::PoseStamped base_point_ned;

      //the setpoint publishing rate MUST be faster than 2Hz
      ros::Rate rate(20.0);

      // wait for FCU connection
      while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
      }

      pose.pose.position.x = 0;
      pose.pose.position.y = 0;
      pose.pose.position.z = 3;
      pose.pose.orientation.z = 0;
      pose.pose.orientation.w = 1;
      //send a few setpoints before starting
      for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
      }

      mavros_msgs::SetMode offb_set_mode;
      offb_set_mode.request.custom_mode = "OFFBOARD";

      mavros_msgs::CommandBool arm_cmd;
      arm_cmd.request.value = true;

      ros::Time last_request = ros::Time::now();

      float delta = 0;
      while(ros::ok()){
        transform.setOrigin( tf::Vector3(local_pose.pose.position.x, local_pose.pose.position.y, local_pose.pose.position.z) );
        tf::Quaternion q;
        q.setX(local_pose.pose.orientation.x);
        q.setY(local_pose.pose.orientation.y);
        q.setZ(local_pose.pose.orientation.z);
        q.setW(local_pose.pose.orientation.w);

        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local_origin_ned", "camera_frame"));

        base_point.pose.position.x = vision_pose.pose.position.x/0.153;
        base_point.pose.position.y = vision_pose.pose.position.y/0.153;
        base_point.pose.position.z = vision_pose.pose.position.z/0.153;
        base_point.pose.orientation = vision_pose.pose.orientation;
        base_point.header.frame_id = "camera_frame";

        try {
          listener.transformPose("local_origin_ned", base_point, base_point_ned);
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
        }
        image_pos_pub.publish(base_point_ned);

        if( current_state.mode != "OFFBOARD" ){
          if( set_mode_client.call(offb_set_mode) &&
              offb_set_mode.response.mode_sent){
            ROS_INFO("Offboard enabled");
          }
        } else if( !current_state.armed){
          if( arming_client.call(arm_cmd) &&
              arm_cmd.response.success){
            ROS_INFO("Vehicle armed");
          }
        }
        else if(con_msg == "land"){
          pose.pose.position.x = con_x;
          pose.pose.position.y = con_y;
          pose.pose.position.z = 3;
        } else {
          pose.pose.position.x = -base_point_ned.pose.position.x;
          pose.pose.position.y = base_point_ned.pose.position.y;
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
      }
    }
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "myControler");
  MyControler mc;
  mc.spin();
  return 0;
}
