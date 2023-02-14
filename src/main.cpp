/*
  Tarot protocol message
  UART / Serial
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <diagnostic_msgs/KeyValue.h>
#include <std_srvs/SetBool.h>
#include <serial/serial.h>

serial::Serial ser;
std::string PORT_NAME;
int BAUD_RATE;
bool VERBOSE;
int ANGLE_LIMIT[6];

uint8_t gimbal_input_phase = 211;
uint8_t gimbal_initialize_step_1[64] = {0x5D, 0x6F, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00,
                                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x31};
uint8_t gimbal_initialize_step_2[64] = {0x5D, 0x6F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x00, 0x00, 0x00,
                                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x58, 0x14};
uint8_t gimbal_reset_init_angle[64] = {0x5D, 0x6F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00,
                                       0x01, 0x00, 0xC8, 0xC8, 0xC8, 0x00, 0x00, 0x2D, 0x83, 0x2D, 0xD3, 0x00, 0x00, 0x00, 0x23, 0x23,
                                       0x23, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00,
                                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6F, 0x60};
uint8_t gimbal_control_angle[64] = {0x5D, 0x6F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00,
                                    0x01, 0x00, 0xC8, 0xC8, 0xC8, 0x00, 0x00, 0x2D, 0x83, 0x2D, 0xD3, 0x00, 0x00, 0x00, 0x23, 0x23,
                                    0x23, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6F, 0x60};
uint8_t gimbal_start_motor[64] = {0x5D, 0x6F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8C, 0x02};
uint8_t gimbal_stop_motor[64] = {0x5D, 0x6F, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x00, 0x00, 0x00,
                                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x51, 0xF2};
uint8_t gimbal_get_data[64] = {0x5D, 0x6F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x00, 0x00, 0x00,
                               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x37, 0x4D};
uint8_t buffer[64];
uint8_t byte_read[1];

const uint16_t CRC_TABLE[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0};

uint16_t crc16_ccitt(uint8_t *data, uint32_t size)
{
  uint32_t counter;
  uint16_t crc = 0;
  for (counter = 0; counter < size; counter++)
  {
    crc = (crc << 8) ^ CRC_TABLE[((crc >> 8) ^ data[counter]) & 0x00FF];
  }
  return crc;
}

constexpr char hexmap[] = {'0', '1', '2', '3', '4', '5', '6', '7',
                           '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

void outcomingCommandAttitudeEncoder(uint8_t *data, float roll, float pitch, float yaw)
{
  if (roll > ANGLE_LIMIT[0])roll=ANGLE_LIMIT[0];
  else if (roll < ANGLE_LIMIT[1])roll = ANGLE_LIMIT[1];
  if (pitch > ANGLE_LIMIT[2])pitch=ANGLE_LIMIT[2];
  else if (pitch < ANGLE_LIMIT[3])pitch = ANGLE_LIMIT[3];
  if (yaw > ANGLE_LIMIT[4])yaw=ANGLE_LIMIT[4];
  else if (yaw < ANGLE_LIMIT[5])yaw = ANGLE_LIMIT[5];
  
  data[27] = (int8_t)roll;
  data[28] = (int8_t)pitch;
  data[29] = (int8_t)yaw;
  uint16_t crc16_xmodem = crc16_ccitt(data, 62);
  uint8_t checksum_high, checksum_low;
  checksum_high = (crc16_xmodem >> 8);
  checksum_low = crc16_xmodem & 0xFF;
  data[62] = checksum_high;
  data[63] = checksum_low;
}

geometry_msgs::PoseStamped gimbal_to_camera_current_attitude;
tf2::Quaternion gimbal_to_camera_current_quaternion;
geometry_msgs::PointStamped world_to_camera_current_angle;
bool incomingMessageDecode(uint8_t *data)
{
  if (data[0] != 0x6F || data[1] != 0x80)
  {
    if (VERBOSE)ROS_ERROR("Data error");
    return false;
  }
  // TODO: checksum
  int16_t roll_camera, pitch_camera, yaw_camera;
  roll_camera = data[7] << 8 | data[6];
  pitch_camera = data[9] << 8 | data[8];
  yaw_camera = data[11] << 8 | data[10];
  int16_t roll_platform, pitch_platform, yaw_platform;
  roll_platform = data[27] << 8 | data[26];
  pitch_platform = data[29] << 8 | data[28];
  yaw_platform = data[31] << 8 | data[30];
  float roll_relative, pitch_relative, yaw_relative;
  roll_relative = (roll_camera - roll_platform)/100.0f;
  pitch_relative = (pitch_camera - pitch_platform)/100.0f;
  /* Fix yaw limitation of not going over 327.67 deg */
  float temp_yaw_camera, temp_yaw_platform;
  if (yaw_camera < 0)temp_yaw_camera = (yaw_camera + 32768 + 32767)/100.0f;
  else temp_yaw_camera = yaw_camera / 100.0f;
  if (yaw_platform < 0)temp_yaw_platform = (yaw_platform + 32768 + 32767)/100.0f;
  else temp_yaw_platform = yaw_platform / 100.0f;
  yaw_relative = temp_yaw_camera - temp_yaw_platform;
  if (yaw_relative > 180)yaw_relative -= 360;
  else if (yaw_relative < -180)yaw_relative += 360;

  gimbal_to_camera_current_quaternion.setRPY(roll_relative/180*M_PI, pitch_relative/180*M_PI, yaw_relative/180*M_PI);
  gimbal_to_camera_current_attitude.header.stamp = ros::Time::now();
  gimbal_to_camera_current_attitude.pose.position.x = 0.0f;
  gimbal_to_camera_current_attitude.pose.position.y = 0.0f;
  gimbal_to_camera_current_attitude.pose.position.z = 0.0f;
  gimbal_to_camera_current_attitude.pose.orientation.w = gimbal_to_camera_current_quaternion.w();
  gimbal_to_camera_current_attitude.pose.orientation.x = gimbal_to_camera_current_quaternion.x();
  gimbal_to_camera_current_attitude.pose.orientation.y = gimbal_to_camera_current_quaternion.y();
  gimbal_to_camera_current_attitude.pose.orientation.z = gimbal_to_camera_current_quaternion.z();

  world_to_camera_current_angle.header.stamp = ros::Time::now();
  world_to_camera_current_angle.point.x = (float)roll_camera/100.0f/180*M_PI;
  world_to_camera_current_angle.point.y = (float)pitch_camera/100.0f/180*M_PI;
  world_to_camera_current_angle.point.z = yaw_relative/180*M_PI;

  return true;
}

geometry_msgs::PointStamped gimbal_command_attitude;
void gimbalCommandAttitudeCallback(const geometry_msgs::PointStampedConstPtr &msg)
{
  gimbal_command_attitude = *msg;
  outcomingCommandAttitudeEncoder(gimbal_control_angle,
    gimbal_command_attitude.point.x/M_PI*180,
    gimbal_command_attitude.point.y/M_PI*180,
    gimbal_command_attitude.point.z/M_PI*180);
  gimbal_input_phase = 111;
}

geometry_msgs::PointStamped gimbal_command_delta_attitude;
void gimbalCommandDeltaAttitudeCallback(const geometry_msgs::PointStampedConstPtr &msg)
{
  gimbal_command_delta_attitude = *msg;
  float roll, pitch, yaw;
  yaw = gimbal_command_delta_attitude.point.z + world_to_camera_current_angle.point.z;
  if (yaw > M_PI)yaw -= 2*M_PI;
  else if (yaw < -M_PI)yaw += 2*M_PI;
  pitch = gimbal_command_delta_attitude.point.y + world_to_camera_current_angle.point.y;
  roll = gimbal_command_delta_attitude.point.x + world_to_camera_current_angle.point.x;
  outcomingCommandAttitudeEncoder(gimbal_control_angle,
    roll/M_PI*180,
    pitch/M_PI*180,
    yaw/M_PI*180);
  gimbal_input_phase = 111;
}

bool gimbalCommandStatusCallback(std_srvs::SetBool::Request &req,
                                 std_srvs::SetBool::Response &res)
{
  // TODO: True service server
  // TODO: Service server for calibrate sensors and calibrate gimbal angles
  if (req.data)
  {
    if (VERBOSE)ROS_INFO("Request to turn on gimbal motor");
    res.message = "Request to turn on gimbal motor";
    gimbal_input_phase = 1;
    // gimbal_input_phase = 121;
  }
  else
  {
    if (VERBOSE)ROS_INFO("Request to turn off gimbal motor");
    res.message = "Request to turn off gimbal motor";
    gimbal_input_phase = 221;
  }

  res.success = true;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tarot_gimbal_ros_driver");
  ros::NodeHandle nh;
  ros::Rate rate(30.0);

  nh.param<bool>("gimbal/verbose", VERBOSE, false);
  nh.param<std::string>("gimbal/port_name", PORT_NAME, "/dev/gimbal");
  nh.param<int>("gimbal/baud_rate", BAUD_RATE, 115200);
  nh.param<int>("gimbal/roll_angle_max", ANGLE_LIMIT[0], 40);
  nh.param<int>("gimbal/roll_angle_min", ANGLE_LIMIT[1], -40);
  nh.param<int>("gimbal/pitch_angle_max", ANGLE_LIMIT[2], 45);
  nh.param<int>("gimbal/pitch_angle_min", ANGLE_LIMIT[3], -125);
  nh.param<int>("gimbal/yaw_angle_max", ANGLE_LIMIT[4], 45);
  nh.param<int>("gimbal/yaw_angle_min", ANGLE_LIMIT[5], 45);

  ros::Subscriber gimbal_command_attitude_sub = nh.subscribe
    <geometry_msgs::PointStamped>("gimbal/command_attitude", 10, gimbalCommandAttitudeCallback);
  ros::Subscriber gimbal_command_delta_attitude_sub = nh.subscribe
    <geometry_msgs::PointStamped>("gimbal/command_delta_attitude", 10, gimbalCommandDeltaAttitudeCallback);

  ros::Publisher gimbal_current_attitude_pub = nh.advertise
    <geometry_msgs::PoseStamped>("gimbal/current_attitude", 10);
  ros::Publisher gimbal_current_absolute_attitude_pub = nh.advertise
    <geometry_msgs::PointStamped>("gimbal/current_absolute_attitude", 10);
  ros::Publisher gimbal_current_status_pub = nh.advertise
    <diagnostic_msgs::KeyValue>("gimbal/current_status", 10);

  ros::ServiceServer gimbal_command_status_service = nh.advertiseService
    ("gimbal/command_status", gimbalCommandStatusCallback);

  try
  {
    ser.setPort(PORT_NAME);
    ser.setBaudrate(BAUD_RATE);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);
    ser.open();
  }
  catch (serial::IOException &e)
  {
    if (VERBOSE)ROS_ERROR("Unable to open port");
    return -1;
  }

  if (ser.isOpen())
  {
    if (VERBOSE)ROS_INFO("Serial port initialized");
  }
  else
  {
    if (VERBOSE)ROS_ERROR("Unable to open port");
    return -1;
  }

  diagnostic_msgs::KeyValue gimbal_current_status;
  gimbal_current_status.key = "OFF"; gimbal_current_status.value = "0";
  while (ros::ok())
  {
    ser.flush();
    switch (gimbal_input_phase)
    {
      case 211:
        /* Initialize step 1 */
        if (VERBOSE)ROS_INFO("Initializing...");
        ser.write(gimbal_initialize_step_1, 64);
        break;
      case 12:
        /* Initialize step 2 */
        if (VERBOSE)ROS_INFO("Initializing...");
        ser.write(gimbal_initialize_step_2, 64);
        break;
      case 1:
        /* Reset init angle */
        if (VERBOSE)ROS_INFO("Reset all roll tilt and pan to 0 deg...");
        ser.write(gimbal_reset_init_angle, 64);
        break;
      case 121:
        /* Start motor */
        if (VERBOSE)ROS_INFO("Start gimbal motor");
        ser.write(gimbal_start_motor, 64);
        break;
      case 221:
        /* Stop motor */
        if (VERBOSE)ROS_INFO("Stop gimbal motor");
        ser.write(gimbal_stop_motor, 64);
        break;
      case 111:
        /* Control angle */
        if (VERBOSE)ROS_INFO("Control all angles");
        ser.write(gimbal_control_angle, 64);
        break;
      default:
        /* Get data, case 21 */
        if (VERBOSE)ROS_INFO("Request a data");
        ser.write(gimbal_get_data, 64);
        break;
    }
    
    while (not ser.waitReadable());
    if (VERBOSE)ROS_INFO("Reading from serial port");
    uint8_t index = 0;
    std::string output(128, ' ');
    while (ser.available() && index < 64)
    {
      ser.read(byte_read, 1);
      buffer[index] = byte_read[0];
      output[2*index] = hexmap[(byte_read[0] & 0xF0) >> 4];
      output[2*index + 1] = hexmap[byte_read[0] & 0x0F];
      index++;
    }
    if (VERBOSE)ROS_INFO("%s", output.c_str());
    // TODO: checksum
    if (incomingMessageDecode(buffer))
    {
      gimbal_current_attitude_pub.publish(gimbal_to_camera_current_attitude);
      gimbal_current_absolute_attitude_pub.publish(world_to_camera_current_angle);
    }

    switch (gimbal_input_phase)
    {
      case 211:
        /* Initialize step 1 */
        if (buffer[2] == 0x02 && buffer[12] == 0x11)
        {
          if (VERBOSE)ROS_INFO("Move to initialization step 2");
          gimbal_current_status.key = "OFF"; gimbal_current_status.value = "0";
          gimbal_input_phase = 12;
        }
        else
        {
          if (VERBOSE)ROS_ERROR("Initialization error");
        }
        break;
      case 12:
        /* Initialize step 2 */
        if (buffer[2] == 0x00 && buffer[12] == 0x12)
        {
          if (VERBOSE)ROS_INFO("Initialization done");
          gimbal_input_phase = 1;
        }
        else
        {
          if (VERBOSE)ROS_ERROR("Initialization error");
        }
        break;
      case 1:
        /* Reset init angle */
        if (buffer[2] == 0x00 && buffer[12] == 0x01)
        {
          if (VERBOSE)ROS_INFO("Reset init angle to 0 done");
          gimbal_input_phase = 121;
        }
        else
        {
          if (VERBOSE)ROS_ERROR("Cannot reset init angle");
        }
        break;
      case 121:
        /* Start motor */
        if (buffer[2] == 0x01 && buffer[12] == 0x21)
        {
          if (VERBOSE)ROS_INFO("Motor started");
          gimbal_current_status.key = "ON"; gimbal_current_status.value = "1";
          gimbal_input_phase = 21;
        }
        else
        {
          if (VERBOSE)ROS_ERROR("Cannot start the motor");
        }
        break;
      case 221:
        /* Stop motor */
        if (buffer[2] == 0x02 && buffer[12] == 0x21)
        {
          if (VERBOSE)ROS_INFO("Motor stopped");
          gimbal_current_status.key = "OFF"; gimbal_current_status.value = "0";
          gimbal_input_phase = 21;
        }
        else
        {
          if (VERBOSE)ROS_ERROR("Cannot stop the motor");
        }
        break;
      case 111:
        /* Control angle */
        if (buffer[2] == 0x00 && buffer[12] == 0x01)
        {
          if (VERBOSE)ROS_INFO("Changing the angle");
          gimbal_input_phase = 21;
        }
        else
        {
          if (VERBOSE)ROS_ERROR("Cannot control the angle");
        }
        break;
      default:
        /* Get data, case 21 */
        break;
    }

    gimbal_current_status_pub.publish(gimbal_current_status);
    rate.sleep();
    ros::spinOnce();
  }
}