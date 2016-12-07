#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <vicon_bridge/Markers.h>


#include "pid.hpp"
#define PUB
double get(
    const ros::NodeHandle& n,
    const std::string& name) {
    double value;
    n.getParam(name, value);
    return value;
}

class Controller2
{
public:

    Controller2(
        const std::string& worldFrame,
        const std::string& frame,
        const ros::NodeHandle& n)
        : m_worldFrame(worldFrame)
        , m_frame(frame)
        , m_pubNav()
        , m_listener()
        , m_pidX(
            get(n, "PIDs/X/kp"),
            get(n, "PIDs/X/kd"),
            get(n, "PIDs/X/ki"),
            get(n, "PIDs/X/minOutput"),
            get(n, "PIDs/X/maxOutput"),
            get(n, "PIDs/X/integratorMin"),
            get(n, "PIDs/X/integratorMax"),
            "x")
        , m_pidY(
            get(n, "PIDs/Y/kp"),
            get(n, "PIDs/Y/kd"),
            get(n, "PIDs/Y/ki"),
            get(n, "PIDs/Y/minOutput"),
            get(n, "PIDs/Y/maxOutput"),
            get(n, "PIDs/Y/integratorMin"),
            get(n, "PIDs/Y/integratorMax"),
            "y")
        , m_pidZ(
            get(n, "PIDs/Z/kp"),
            get(n, "PIDs/Z/kd"),
            get(n, "PIDs/Z/ki"),
            get(n, "PIDs/Z/minOutput"),
            get(n, "PIDs/Z/maxOutput"),
            get(n, "PIDs/Z/integratorMin"),
            get(n, "PIDs/Z/integratorMax"),
            "z")
        , m_pidYaw(
            get(n, "PIDs/Yaw/kp"),
            get(n, "PIDs/Yaw/kd"),
            get(n, "PIDs/Yaw/ki"),
            get(n, "PIDs/Yaw/minOutput"),
            get(n, "PIDs/Yaw/maxOutput"),
            get(n, "PIDs/Yaw/integratorMin"),
            get(n, "PIDs/Yaw/integratorMax"),
            "yaw")
        , m_state(Idle)
        , m_goal()
        , m_subscribeGoal()
        , m_subsrcibeLeaderFollower()
        , m_subscribeMarkers()
        , m_serviceTakeoff()
        , m_serviceLand()
        , m_thrust(0)
        , m_startZ(0)
    {
        ros::NodeHandle nh;
        m_listener.waitForTransform(m_worldFrame, m_frame, ros::Time(0), ros::Duration(10.0)); 
        m_pubNav = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        m_subscribeGoal = nh.subscribe("goal", 1, &Controller2::goalChanged, this);
        m_subsrcibeLeaderFollower = nh.subscribe("leaderfollower/cmd_vel", 1, &Controller2::lfCallback, this);
        m_subscribeMarkers = nh.subscribe("/vicon/markers", 1, &Controller2::markerCallback, this);
        m_serviceTakeoff = nh.advertiseService("takeoff", &Controller2::takeoff, this);
        m_serviceLand = nh.advertiseService("land", &Controller2::land, this);
        //shouldPublish = true;
    }

    void run(double frequency)
    {
        ros::NodeHandle node;
        ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &Controller2::iteration, this);
        ros::spin();
    }

private:
    void goalChanged(
        const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        m_goal = *msg;
    }

    void lfCallback
    (const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
        if (abs(msg->twist.linear.x) > 0.001 && abs(msg->twist.linear.y) > 0.001){
                    //lf_cmd.push_back(*msg);
                    last_cmd = *msg;
         ROS_INFO("receiving cmd from LF");
        }
        else ROS_INFO("not adding 0,0!");


    }   
    void markerCallback(const vicon_bridge::Markers::ConstPtr& msg){
      // ROS_INFO("marker callback with %i markers", msg->markers.size()); 
        int count = 0;
    for (int i = 0; i<msg->markers.size(); i++){
        if (msg->markers[i].marker_name == ""){
            count++;
        }
    }
     if(count>1){
        ROS_INFO("MORE THAN ONE MARKER!!");
          }
    }
    bool takeoff(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
    {
        ROS_INFO("Takeoff requested!");
        m_state = TakingOff;

        tf::StampedTransform transform;
        m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
        m_startZ = transform.getOrigin().z();

        return true;
    }

    bool land(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
    {
        ROS_INFO("Landing requested!");
        m_state = Landing;

        return true;
    }

    void getTransform(
        const std::string& sourceFrame,
        const std::string& targetFrame,
        tf::StampedTransform& result)
    {
        m_listener.lookupTransform(sourceFrame, targetFrame, ros::Time(0), result);
    }

    void pidReset()
    {
        m_pidX.reset();
        m_pidZ.reset();
        m_pidZ.reset();
        m_pidYaw.reset();
    }

    void iteration(const ros::TimerEvent& e)
    {
        float dt = e.current_real.toSec() - e.last_real.toSec();

        switch(m_state)
        {
        case TakingOff:
            {
                tf::StampedTransform transform;
                //ROS_INFO("worldframe: %s, frame: %s", m_worldFrame.c_str(), m_frame.c_str());
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);

                #ifdef PUB
                 ROS_INFO("Target drone: %f, %f, %f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
                #endif
                // //
                //ROS_INFO("m_startz = %f", m_startZ);
                if (transform.getOrigin().z() > m_startZ + 0.05 || m_thrust > 50000)
                {
                    pidReset();
                    m_pidZ.setIntegral(m_thrust / m_pidZ.ki());
                    m_state = Automatic;
                    m_thrust = 0;
                }
                else
                {
                    m_thrust += 10000 * dt;
                    geometry_msgs::Twist msg;
                    msg.linear.z = m_thrust;
                   #ifdef PUB
                    m_pubNav.publish(msg);
                    #endif
                }

            }
            break;
        case Landing:
            {
                m_goal.pose.position.z = m_startZ + 0.05;
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
                if (transform.getOrigin().z() <= m_startZ + 0.05) {
                    m_state = Idle;
                    geometry_msgs::Twist msg;
                    #ifdef PUB
                    m_pubNav.publish(msg);
                    #endif
                }
            }
            // intentional fall-thru
        case Automatic:
            {
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);

                geometry_msgs::PoseStamped targetWorld;
                targetWorld.header.stamp = transform.stamp_;
                targetWorld.header.frame_id = m_worldFrame;
                targetWorld.pose = m_goal.pose;
                geometry_msgs::PoseStamped targetDrone = targetWorld;
                //m_listener.transformPose(m_frame, targetWorld, targetDrone);

                tfScalar roll, pitch, yaw;
                tf::Matrix3x3(
                    tf::Quaternion(
                        targetDrone.pose.orientation.x,
                        targetDrone.pose.orientation.y,
                        targetDrone.pose.orientation.z,
                        targetDrone.pose.orientation.w
                    )).getRPY(roll, pitch, yaw);

                geometry_msgs::Twist msg;
              msg.linear.x = -m_pidX.update(transform.getOrigin().x(), targetDrone.pose.position.x);
              msg.linear.y = m_pidY.update(transform.getOrigin().y(), targetDrone.pose.position.y);
            /* if (abs(last_cmd.twist.linear.x) > 0.001 && abs(last_cmd.twist.linear.y) > 0.001){
                msg = last_cmd.twist;
            }*/

               //msg.linear.x *= (-1);
                msg.linear.z = m_pidZ.update(transform.getOrigin().z(), targetDrone.pose.position.z);
                //msg.angular.z = m_pidYaw.update(0.0, yaw);
                //msg.angular.z = 0.0;
                #ifdef PUB
                m_pubNav.publish(msg);
                #endif
                ROS_INFO("Current drone: %f, %f, %f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());

                ROS_INFO("sending cmd_vel: %f, %f, %f", msg.linear.x, msg.linear.y, msg.linear.z);
                 //ROS_INFO("Target drone: %f, %f, %f", trans, form.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());


            }
            break;
        case Idle:
            {
                geometry_msgs::Twist msg;
                m_pubNav.publish(msg);

                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);

                geometry_msgs::PoseStamped targetWorld;
                targetWorld.header.stamp = transform.stamp_;
                targetWorld.header.frame_id = m_worldFrame;
                targetWorld.pose = m_goal.pose;
                geometry_msgs::PoseStamped targetDrone = targetWorld;
                //m_listener.transformPose(m_frame, targetWorld, targetDrone);

                tfScalar roll, pitch, yaw;
                tf::Matrix3x3(
                    tf::Quaternion(
                        targetDrone.pose.orientation.x,
                        targetDrone.pose.orientation.y,
                        targetDrone.pose.orientation.z,
                        targetDrone.pose.orientation.w
                    )).getRPY(roll, pitch, yaw);

                ROS_INFO("Current drone: %f, %f, %f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());


        /* tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);

                geometry_msgs::PoseStamped targetWorld;
                targetWorld.header.stamp = transform.stamp_;
                targetWorld.header.frame_id = m_worldFrame;
                targetWorld.pose = m_goal.pose;

                geometry_msgs::PoseStamped targetDrone;
                m_listener.transformPose(m_frame, targetWorld, targetDrone);

                tfScalar roll, pitch, yaw;
                tf::Matrix3x3(
                    tf::Quaternion(
                        targetDrone.pose.orientation.x,
                        targetDrone.pose.orientation.y,
                        targetDrone.pose.orientation.z,
                        targetDrone.pose.orientation.w
                    )).getRPY(roll, pitch, yaw);*/
        //ROS_INFO("Target drone: %f, %f, %f, %f", targetDrone.pose.position.x, targetDrone.pose.position.ys,
                            // targetDrone.pose.position.z, yaw);
        //sROS_INFO("From markers: %f, %f, %f, %f", 

            }
            break;
        }
    }

private:

    enum State
    {
        Idle = 0,
        Automatic = 1,
        TakingOff = 2,
        Landing = 3,
    };

private:
    std::string m_worldFrame;
    std::string m_frame;
    ros::Publisher m_pubNav;
    tf::TransformListener m_listener;
    PID m_pidX;
    PID m_pidY;
    PID m_pidZ;
    PID m_pidYaw;
    State m_state;
    geometry_msgs::PoseStamped m_goal;
    ros::Subscriber m_subscribeGoal;
    ros::Subscriber m_subsrcibeLeaderFollower;
    ros::Subscriber m_subscribeMarkers;
    ros::ServiceServer m_serviceTakeoff;
    ros::ServiceServer m_serviceLand;
    float m_thrust;
    float m_startZ;
    std::vector<geometry_msgs::TwistStamped> lf_cmd;
    geometry_msgs::TwistStamped last_cmd;
    bool shouldPublish;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller2");

  // Read parameters
  ros::NodeHandle n("~");
  std::string worldFrame;
  n.param<std::string>("worldFrame", worldFrame, "/world");
  std::string frame;
  n.getParam("frame", frame);
  double frequency;
  n.param("frequency", frequency, 50.0);

  Controller2 controller2(worldFrame, frame, n);
  controller2.run(frequency);

  return 0;
}
