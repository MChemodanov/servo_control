#include <string>
#include "ros/ros.h"
#include "servo_control/Servo.h"
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>

#include <boost/thread/thread.hpp>
#include <atomic>

extern "C"
{
#include "rc_usefulincludes.h"
#include <roboticscape.h>
}

const int CHANNELS = 8;

class Imu
{
public:
  Imu(ros::NodeHandle& nh) : _node("~")
  {
    rc_imu_config_t conf = rc_default_imu_config();

    // now set up the imu for dmp interrupt operation
    if(rc_initialize_imu_dmp(&_data, conf))
    {
      ROS_FATAL("rc_imu_initialize_failed");
      return;
    }

    ROS_INFO("Imu initialized");
    _imu_publisher = nh.advertise<sensor_msgs::Imu>("imu", 1);
  }

  void publishImu()
  {
    sensor_msgs::Imu msg;
    msg.header.frame_id = "robot";

    msg.orientation.w = _data.dmp_quat[QUAT_W];
    msg.orientation.x = _data.dmp_quat[QUAT_X];
    msg.orientation.y = _data.dmp_quat[QUAT_Y];
    msg.orientation.z = _data.dmp_quat[QUAT_Z];

    msg.angular_velocity.x = _data.gyro[0];
    msg.angular_velocity.y = _data.gyro[1];
    msg.angular_velocity.z = _data.gyro[2];

    _imu_publisher.publish(msg);
  }

private:
  rc_imu_data_t _data;
  ros::NodeHandle _node;
  ros::Publisher _imu_publisher;
};

void signal_handler(__attribute__ ((unused)) int dummy)
{
  ros::shutdown();
  return;
}

int frequency;
std::atomic<int> data[8];


void callback(const servo_control::Servo msg)
{
    for (int i=0; i < CHANNELS; i++)
        data[i] = msg.channels[i];
}


void update_servos()
{
  while(ros::ok()){
    for (int i=0; i < CHANNELS; i++)
        rc_send_servo_pulse_us(i+1, data[i]);
    rc_usleep(1000000/50);
  }
}


int main(int argc, char **argv)
{
  signal(SIGINT, signal_handler);

  for (int i=0; i < CHANNELS; i++)
  {
      data[i] = 0;
  }

  ros::init(argc, argv, "servo_control_node");
  ros::start();


  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pnh.param("frequency", frequency, int(20));


  ros::Subscriber sub;
  sub = nh.subscribe("servos", 1,  &callback);


  if(rc_initialize()<0)
  {
    ROS_INFO("ERROR: failed to initialize cape.");
    return -1;
  }

  Imu imu(nh);

  rc_set_led(RED,1);
  rc_set_led(GREEN,0);
  rc_set_state(UNINITIALIZED);


  if(rc_enable_servo_power_rail()){
    fprintf(stderr,"failed to enable power rail\n");
    return -1;
  } 

  boost::thread thread_servo(update_servos);

  ROS_INFO("Begin loop");
  int counter = 0;


  ros::Rate loop_rate(10);
   while(ros::ok()){
    imu.publishImu();
    ros::spinOnce();
    loop_rate.sleep();
  }


  rc_set_state(EXITING);
  rc_power_off_imu();
  rc_disable_servo_power_rail();
  rc_cleanup();
  ros::shutdown();
  
  thread_servo.join();

  return EXIT_SUCCESS;
}
