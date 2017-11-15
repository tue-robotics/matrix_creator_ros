#include <matrix_hal/wishbone_bus.h>
#include <matrix_hal/microphone_array.h>
#include <matrix_hal/direction_of_arrival.h>
#include <matrix_hal/everloop.h>
#include <matrix_hal/everloop_image.h>
#include <matrix_hal/microphone_array_location.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <geometry_msgs/PoseStamped.h>

/**
 * Compute the average value of the  entire buffer.
 * @param buffer Values to average.
 * @return Average of all the values.
 */
static uint64_t get_average(const std::valarray<uint64_t>& buffer) {
  uint64_t avg = 0;
  for(auto& e : buffer)
    avg += e;
  return (avg / buffer.size());
}

/**
 * Write the direction of a sound into the leds.
 * @param everloop Hardware access to the leds.
 * @param image1d led status.
 */
static void write_leds(matrix_hal::Everloop &everloop, matrix_hal::EverloopImage &image1d, int mic) {
  static const int led_offset[] = {23, 27, 32, 1, 6, 10, 14, 19};
  static const int lut[] = {1, 2, 10, 200, 10, 2, 1};

  for (matrix_hal::LedValue& led : image1d.leds) {
    led.blue = 0;
  }

  int j;
  for (int i = led_offset[mic] - 3, j = 0; i < led_offset[mic] + 3;
       ++i, ++j) {
    if (i < 0) {
      image1d.leds[image1d.leds.size() + i].blue = lut[j];
    } else {
      image1d.leds[i % image1d.leds.size()].blue = lut[j];
    }

    everloop.Write(&image1d);
  }
}

geometry_msgs::PoseStamped g_msg;
ros::Publisher g_pub;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "direction_of_arrival");
  ros::NodeHandle local_nh("~");

  //! Set-up Matrix HAL iface
  matrix_hal::WishboneBus bus;
  bus.SpiInit();

  matrix_hal::MicrophoneArray mics;
  mics.Setup(&bus);

  matrix_hal::EverloopImage image1d;
  matrix_hal::Everloop everloop;
  mics.SetGain(local_nh.param("gain", 2));
  everloop.Setup(&bus);
  matrix_hal::DirectionOfArrival doa(mics);
  doa.Init();

  //! Set-up ROS output iface
  ros::NodeHandle nh;
  g_pub = nh.advertise<geometry_msgs::PoseStamped>("direction_of_arrival", 1);

  std::string frame_id = local_nh.param("frame_id", std::string("map"));
  double average_energy_threshold = local_nh.param("average_energy_threshold", 4000000);
  int buffer_length = local_nh.param("buffer_length", 15);
  g_msg.header.frame_id = frame_id;

  ros::Time last_update = ros::Time::now();
  std::map<int, size_t> count_map;

  uint64_t instantE = 0;
  uint64_t avgEnergy = 0;
  std::valarray<uint64_t> buffer (buffer_length);
  buffer = 0;

  int next_free = 0; // Next free element in the buffer.
  int last_mic = -1; // Direction of the last heard sound.
  while (ros::ok())
  {
    // Read microphones and calculate DOA
    mics.Read(); /* Reading 8-mics buffer from de FPGA */
    doa.Calculate();

    // Reset buffer if we receive sound from another direction
    int mic = doa.GetNearestMicrophone();
    if (last_mic >= 0 && mic != last_mic) {
      next_free = 0;
    }
    last_mic = mic;

    // Store the energy in the buffer
    buffer[next_free] = mics.At(mic, 0)*mics.At(mic, 0);
    next_free++;

    // If the buffer is full, check if the average is high enough
    if (next_free == buffer_length) {
      next_free = 0;

      // Check if the average energy level is high enough
      uint64_t avg_energy = get_average(buffer);

      // ROS LOGGING gives a segfault. Could be because of different boost versions
      std::cout << "Sound detected! Average energy: " << avg_energy << std::endl;

      if (avg_energy > average_energy_threshold) {
        //ROS_INFO("Detected directional sound, average energy ok!");
        write_leds(everloop, image1d, mic);

        // Fill the message
        double yaw = atan2(matrix_hal::micarray_location[mic][1],
                           matrix_hal::micarray_location[mic][0]);

        double pitch = 0;

        g_msg.pose.orientation.x = - sin(pitch) * sin(yaw);
        g_msg.pose.orientation.y = sin(pitch) * cos(yaw);
        g_msg.pose.orientation.z = cos(pitch) * sin(yaw / 2.0);
        g_msg.pose.orientation.w = cos(pitch) * cos(yaw / 2.0);

        // Publish result
        g_msg.header.stamp = ros::Time::now();
        g_pub.publish(g_msg);
      }
      else
      {
        //ROS_WARN("Detected directional sound, average energy too low!");
      }
    }

  }
  return 0;
}
