# matrix_creator_hal

ROS Wrapper for the [Matrix Creator HAL](https://github.com/matrix-io/matrix-creator-hal/) functionalities.

## Direction of arrival (DOA)

    rosrun matrix_creator_hal direction_of_arrival_node 

This nodes gets the DOA from the Matrix Creator HAL API and buffers the energy of the specific microphone. The buffer
only stores values if consecutive measurements are from the same microphone the DOA API provides. If the DOA API provides
a measurement from a different microphone, the buffer is cleared. If the buffer is full, a `geometry_msgs/PoseStamped` is
published if the average energy in the buffer is larget than the threshold.

### Topics

- direction_of_arrival `geometry_msgs/PoseStamped` 

### Parameters

- ~gain (microphone gain, default=2) 
- ~frame_id (default=map)
- ~buffer_length (Length of the buffer. default=15)
- ~average_energy_threshold (The average energy in the buffer should be larger than this value. default=4000000)
