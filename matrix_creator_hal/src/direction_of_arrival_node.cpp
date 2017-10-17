#include <matrix_hal/wishbone_bus.h>
#include <matrix_hal/microphone_array.h>
#include <matrix_hal/direction_of_arrival.h>
#include <matrix_hal/everloop.h>
#include <matrix_hal/everloop_image.h>
#include <matrix_hal/microphone_array_location.h>
#include <iomanip>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::PoseStamped g_msg;
ros::Publisher g_pub;

int main(int argc, char** argv) 
{
  //! Set-up Matrix HAL iface
  matrix_hal::WishboneBus bus;
  bus.SpiInit();

  matrix_hal::MicrophoneArray mics;
  mics.Setup(&bus);

  matrix_hal::EverloopImage image1d;
  matrix_hal::Everloop everloop;
  mics.SetGain(8);
  everloop.Setup(&bus);

  matrix_hal::DirectionOfArrival doa(mics);

  //! Set-up ROS output iface
  ros::init(argc, argv, "direction_of_arrival");
  ros::NodeHandle nh;
  g_pub = nh.advertise<geometry_msgs::PoseStamped>("direction_of_arrival", 1);

  ros::NodeHandle local_nh("~");
  std::string frame_id = local_nh.param("frame_id", std::string("map"));
  double frequency = local_nh.param("frequency", 50.0);
  int min_count = local_nh.param("min_count", 5);
  g_msg.header.frame_id = frame_id;

  ros::Time last_update = ros::Time::now();
  std::map<int, size_t> count_map;

  uint64_t instantE = 0;
  uint64_t avgEnergy = 0;
  size_t buffer_length = 30;
  std::valarray<uint64_t> localAverage (buffer_length);
  localAverage = 0;

  int j = 0;
  while (ros::ok()) 
  {
    // Read microphones and calculate DOA
    mics.Read(); /* Reading 8-mics buffer from de FPGA */
    doa.Calculate();

    int mic = doa.GetNearestMicrophone();

    instantE  = 0;
    for (uint32_t s = 0; s < mics.NumberOfSamples(); s++) {
      instantE = instantE + (mics.At(s, 0))*(mics.At(s, 0));
    }

    localAverage[j%buffer_length] = instantE;
    avgEnergy = 0;
    for(auto& data : localAverage){
      avgEnergy = (avgEnergy + data);
    }

    avgEnergy = avgEnergy/buffer_length;
    //std::cout << std::setfill(' ') << std::setw(10) << avgEnergy << std::endl;

    if (mic >= 0 && avgEnergy > 10024919)
	    count_map[mic] += 1;

    if ((ros::Time::now() - last_update).toSec() < 1./frequency) {
        continue;
    }

    int max_count = 0;
    for (auto& item : count_map) {
        if (item.second > max_count) {
           mic = item.first;
           max_count = item.second; 
        }
    }
    count_map.clear();
    last_update = ros::Time::now();

    ROS_INFO("Max count = %d", max_count);

    if (max_count < min_count) continue;

    // Get the result (spherical, can be represented as RPY with roll = 0)

    double yaw = atan2(matrix_hal::micarray_location[mic][1],
		    matrix_hal::micarray_location[mic][0]);

    double pitch = 0;

    int led_offset[] = {23, 27, 32, 1, 6, 10, 14, 19};
    int lut[] = {1, 2, 10, 200, 10, 2, 1};
    for (matrix_hal::LedValue& led : image1d.leds) {
      led.blue = 0;
    }

    for (int i = led_offset[mic] - 3, j = 0; i < led_offset[mic] + 3;
         ++i, ++j) {
      if (i < 0) {
        image1d.leds[image1d.leds.size() + i].blue = lut[j];
      } else {
        image1d.leds[i % image1d.leds.size()].blue = lut[j];
      }

      everloop.Write(&image1d);
    }

    // Fill the message
    g_msg.pose.orientation.x = - sin(pitch) * sin(yaw);
    g_msg.pose.orientation.y = sin(pitch) * cos(yaw);
    g_msg.pose.orientation.z = cos(pitch) * sin(yaw / 2.0);
    g_msg.pose.orientation.w = cos(pitch) * cos(yaw / 2.0);

    // Publish result
    g_msg.header.stamp = ros::Time::now();
    g_pub.publish(g_msg);
  }
  return 0;
}
