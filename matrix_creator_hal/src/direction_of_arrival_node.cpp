#include <matrix_hal/wishbone_bus.h>
#include <matrix_hal/microphone_array.h>
#include <matrix_hal/direction_of_arrival.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <matrix_creator_msgs/DirectionOfArrival.h>

matrix_creator_msgs::DirectionOfArrival g_msg;
ros::Publisher g_pub;

int main(int argc, char** argv) 
{
  //! Set-up Matrix HAL iface
  matrix_hal::WishboneBus bus;
  bus.SpiInit();

  matrix_hal::MicrophoneArray mics;
  mics.Setup(&bus);

  matrix_hal::DirectionOfArrival doa(mics);

  //! Set-up ROS output iface
  ros::init(argc, argv, "direction_of_arrival");
  ros::NodeHandle nh;
  g_pub = nh.advertise<matrix_creator_msgs::DirectionOfArrival>("direction_of_arrival", 1);

  ros::NodeHandle local_nh;
  std::string frame_id = local_nh.param("frame_id", std::string("map"));
  g_msg.header.frame_id = frame_id;

  while (true) 
  {
    // Read microphones and calculate DOA
    mics.Read(); /* Reading 8-mics buffer from de FPGA */
    doa.Calculate();

    // Publish result
    g_msg.header.stamp = ros::Time::now();
    g_msg.azimuth_angle = doa.GetAzimutalAngle();
    g_msg.polar_angle = doa.GetPolarAngle();
    g_pub.publish(g_msg);
  }
  return 0;
}
