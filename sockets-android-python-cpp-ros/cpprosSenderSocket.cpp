/** 
 *
 *  Modified from dji osdk for telemetry 
 *
 *
 *
 */

//INCLUDE
#include <ros/ros.h>
#include <dji_osdk_ros/common_type.h>
#include <dji_osdk_ros/dji_vehicle_node.h>

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <string>
#include <netdb.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>


//CODE
using namespace dji_osdk_ros;
using namespace std;
using namespace rapidjson;
// please download the rapidjson github repo, and put the headers in !

int connectSocket(string IPaddress, int Port);
void sendData(int socket, string DataOutput);

sensor_msgs::BatteryState battery_state_;
geometry_msgs::QuaternionStamped attitude_data_;
sensor_msgs::Imu imu_data_;
std_msgs::UInt8 flight_status_;
std_msgs::UInt8 gps_health_;
sensor_msgs::NavSatFix gps_position_;
dji_osdk_ros::VOPosition vo_position_;
std_msgs::Float32 height_above_takeoff_;
geometry_msgs::Vector3Stamped velocity_;
dji_osdk_ros::MobileData from_mobile_data_;
dji_osdk_ros::PayloadData from_payload_data_;
geometry_msgs::Vector3Stamped gimbal_angle_data_;
sensor_msgs::Joy rc_data_;
geometry_msgs::PointStamped local_position_;
sensor_msgs::NavSatFix local_Frame_ref_;
nmea_msgs::Sentence time_sync_nmea_msg_;
dji_osdk_ros::GPSUTC time_sync_gps_utc_;
dji_osdk_ros::FCTimeInUTC time_sync_fc_utc_;
std_msgs::String time_sync_pps_source_;
geometry_msgs::Vector3Stamped angular_rate_;
geometry_msgs::Vector3Stamped acceleration_;
std_msgs::UInt8 display_mode_;
sensor_msgs::TimeReference trigger_;
std_msgs::UInt8 rc_connection_status_;
sensor_msgs::NavSatFix rtk_position_;
geometry_msgs::Vector3Stamped rtk_velocity_;
std_msgs::Int16 rtk_yaw_;
std_msgs::UInt8 rtk_position_info_;
std_msgs::UInt8 rtk_yaw_info_;
std_msgs::UInt8 rtk_connection_status_;
dji_osdk_ros::FlightAnomaly flight_anomaly_;

void attitudeSubCallback(const geometry_msgs::QuaternionStampedConstPtr& attitudeData)
{
  attitude_data_ = *attitudeData;
}

void batteryStateSubCallback(const sensor_msgs::BatteryState::ConstPtr& batteryState)
{
  battery_state_ = *batteryState;
}

void imuSubCallback(const sensor_msgs::Imu::ConstPtr& imuData)
{
  imu_data_ = *imuData;
}

void flightStatusSubCallback(const std_msgs::UInt8::ConstPtr& flightData)
{
  flight_status_ = *flightData;
}

void gpsHealthSubCallback(const std_msgs::UInt8::ConstPtr& gpsHealth)
{

  gps_health_ = *gpsHealth;
}

void gpsPositionSubCallback(const sensor_msgs::NavSatFix::ConstPtr& gpsPosition)
{
  gps_position_ = *gpsPosition;
}

void voPositionSubCallback(const dji_osdk_ros::VOPosition::ConstPtr& voPosition)
{
  vo_position_ = *voPosition;
}

void heightSubCallback(const std_msgs::Float32::ConstPtr& heightAboveTakeoff)
{
  height_above_takeoff_ = *heightAboveTakeoff;
}

void velocitySubCallback(const geometry_msgs::Vector3Stamped::ConstPtr& velocity)
{
  velocity_ = *velocity;
}

void fromMobileDataSubCallback(const dji_osdk_ros::MobileData::ConstPtr& fromMobileData)
{
  from_mobile_data_ = *fromMobileData;
}

void fromPayloadDataSubCallback(const dji_osdk_ros::PayloadData::ConstPtr& fromPayloadData)
{
  from_payload_data_ = *fromPayloadData;
}

void gimbalAngleSubCallback(const geometry_msgs::Vector3Stamped::ConstPtr& gimbalAngleData)
{
  gimbal_angle_data_ = *gimbalAngleData;
}

void rcDataCallback(const sensor_msgs::Joy::ConstPtr& rcData)
{
  rc_data_ = *rcData;
}

void localPositionSubCallback(const geometry_msgs::PointStamped::ConstPtr& localPosition)
{
  local_position_ = *localPosition;
}

void localFrameRefSubCallback(const sensor_msgs::NavSatFix::ConstPtr& localFrameRef)
{
  local_Frame_ref_ = *localFrameRef;
}

void timeSyncNmeaSubSCallback(const nmea_msgs::Sentence::ConstPtr& timeSyncNmeaMsg)
{
  time_sync_nmea_msg_ = *timeSyncNmeaMsg;
}

void timeSyncGpsUtcSubCallback(const dji_osdk_ros::GPSUTC::ConstPtr& timeSyncGpsUtc)
{
  time_sync_gps_utc_ = *timeSyncGpsUtc;
}

void timeSyncFcUtcSubCallback(const dji_osdk_ros::FCTimeInUTC::ConstPtr& timeSyncFcUtc)
{
  time_sync_fc_utc_ = *timeSyncFcUtc;
}

void timeSyncPpsSourceSubCallback(const std_msgs::String::ConstPtr& timeSyncPpsSource)
{
  time_sync_pps_source_ = *timeSyncPpsSource;
}

void angularRateSubSCallback(const geometry_msgs::Vector3Stamped::ConstPtr& angularRate)
{
  angular_rate_ = *angularRate;
}

void accelerationSubCallback(const geometry_msgs::Vector3Stamped::ConstPtr& acceleration)
{
  acceleration_ = *acceleration;
}

void displayModeSubCallback(const std_msgs::UInt8::ConstPtr& displayMode)
{
  display_mode_ = *displayMode;
}

void triggerSubCallback(const sensor_msgs::TimeReference::ConstPtr& trigger)
{
  trigger_ = *trigger;
}

void rcConnectionStatusSubCallback(const std_msgs::UInt8::ConstPtr& rcConnectionStatus)
{
  rc_connection_status_ = *rcConnectionStatus;
}

void rtkPositionSubCallback(const sensor_msgs::NavSatFix::ConstPtr& rtkPosition)
{
  rtk_position_ = *rtkPosition;
}

void rtkVelocitySubCallback(const geometry_msgs::Vector3Stamped::ConstPtr& rtkVelocity)
{
  rtk_velocity_ = *rtkVelocity;
}

void rtkYawSubCallback(const std_msgs::Int16::ConstPtr& rtkYaw)
{
  rtk_yaw_ = *rtkYaw;
}

void rtkPositionInfoSubCallback(const std_msgs::UInt8::ConstPtr& rtkPositionInfo)
{
  rtk_position_info_ = *rtkPositionInfo;
}

void rtkYawInfoSubCallback(const std_msgs::UInt8::ConstPtr& rtkYawInfo)
{
  rtk_yaw_info_ = *rtkYawInfo;
}

void rtkConnectionStatusSubCallback(const std_msgs::UInt8::ConstPtr& rtkConnectionStatus)
{
  rtk_connection_status_ = *rtkConnectionStatus;
}

void flightAnomalySubCallback(const dji_osdk_ros::FlightAnomaly::ConstPtr& flightAnomaly)
{
  flight_anomaly_ = *flightAnomaly;
}

ros::Subscriber batteryStateSub;
ros::Subscriber imuSub;
ros::Subscriber flightStatusSub;
ros::Subscriber gpsHealthSub;
ros::Subscriber gpsPositionSub;
ros::Subscriber heightSub;
ros::Subscriber localPositionSub;
ros::Subscriber velocitySub;
ros::Subscriber gimbalAngleSub;
ros::Subscriber rcDataSub;
ros::Subscriber attitudeSub;

ros::Subscriber fromMobileDataSub;
ros::Subscriber fromPayloadDataSub;

ros::Subscriber localFrameRefSub;
ros::Subscriber timeSyncNmeaSub;
ros::Subscriber timeSyncGpsUtcSub;
ros::Subscriber timeSyncFcUtcSub;
ros::Subscriber timeSyncPpsSourceSub;

ros::Subscriber voPositionSub;
ros::Subscriber angularRateSub;
ros::Subscriber accelerationSub;
ros::Subscriber displayModeSub;
ros::Subscriber triggerSub;

ros::Subscriber rcConnectionStatusSub;
ros::Subscriber rtkPositionSub;
ros::Subscriber rtkVelocitySub;
ros::Subscriber rtkYawSub;
ros::Subscriber rtkPositionInfoSub;
ros::Subscriber rtkYawInfoSub;
ros::Subscriber rtkConnectionStatusSub;
ros::Subscriber flightAnomalySub;

int main(int argc ,char** argv) {

  string IPaddress = ""; // your ip address here
  int port = 8888;

  ros::init(argc, argv, "telemetry_node");
  ros::NodeHandle nh;
  bool userSelectBroadcast = false;
  nh.getParam("/vehicle_node/use_broadcast", userSelectBroadcast);

  batteryStateSub  = nh.subscribe("dji_osdk_ros/battery_state", 10, &batteryStateSubCallback);
  imuSub           = nh.subscribe("dji_osdk_ros/imu", 10, &imuSubCallback);
  flightStatusSub  = nh.subscribe("dji_osdk_ros/flight_status", 10, &flightStatusSubCallback);
  gpsHealthSub     = nh.subscribe("dji_osdk_ros/gps_health", 10, &gpsHealthSubCallback);
  gpsPositionSub   = nh.subscribe("dji_osdk_ros/gps_position", 10, &gpsPositionSubCallback);
  heightSub        = nh.subscribe("dji_osdk_ros/height_above_takeoff", 10, &heightSubCallback);
  localPositionSub = nh.subscribe("dji_osdk_ros/local_position", 10, &localPositionSubCallback);
  velocitySub      = nh.subscribe("dji_osdk_ros/velocity", 10, &velocitySubCallback);
  gimbalAngleSub   = nh.subscribe("dji_osdk_ros/gimbal_angle", 10, &gimbalAngleSubCallback);
  rcDataSub        = nh.subscribe("dji_osdk_ros/rc", 10, &rcDataCallback);

  fromMobileDataSub  = nh.subscribe("dji_osdk_ros/from_mobile_data", 10, &fromMobileDataSubCallback);
  fromPayloadDataSub = nh.subscribe("dji_osdk_ros/from_payload_data", 10, &fromPayloadDataSubCallback);

  /* only if you call the service of set_local_pos_reference ,the topic can be valid" */
  localFrameRefSub     = nh.subscribe("dji_osdk_ros/local_frame_ref", 10, &localFrameRefSubCallback);
  timeSyncNmeaSub      = nh.subscribe("dji_osdk_ros/time_sync_nmea_msg", 10, &timeSyncNmeaSubSCallback);
  timeSyncGpsUtcSub    = nh.subscribe("dji_osdk_ros/time_sync_gps_utc", 10, &timeSyncGpsUtcSubCallback);
  timeSyncFcUtcSub     = nh.subscribe("dji_osdk_ros/time_sync_fc_time_utc", 10, &timeSyncFcUtcSubCallback);
  timeSyncPpsSourceSub = nh.subscribe("dji_osdk_ros/time_sync_pps_source", 10, &timeSyncPpsSourceSubCallback);
  
  if (!userSelectBroadcast)
  {
    voPositionSub   = nh.subscribe("dji_osdk_ros/vo_position", 10, &voPositionSubCallback);
    angularRateSub  = nh.subscribe("dji_osdk_ros/angular_velocity_fused", 10, &angularRateSubSCallback);
    accelerationSub = nh.subscribe("dji_osdk_ros/acceleration_ground_fused", 10, &accelerationSubCallback);
    displayModeSub  = nh.subscribe("dji_osdk_ros/display_mode", 10, &displayModeSubCallback);
    triggerSub      = nh.subscribe("dji_osdk_ros/trigger_time", 10, &triggerSubCallback);

    rcConnectionStatusSub  = nh.subscribe("dji_osdk_ros/rc_connection_status", 10, &rcConnectionStatusSubCallback);
    rtkPositionSub         = nh.subscribe("dji_osdk_ros/rtk_position", 10, &rtkPositionSubCallback);
    rtkVelocitySub         = nh.subscribe("dji_osdk_ros/rtk_velocity", 10, &rtkVelocitySubCallback);
    rtkYawSub              = nh.subscribe("dji_osdk_ros/rtk_yaw", 10, &rtkYawSubCallback);
    rtkPositionInfoSub     = nh.subscribe("dji_osdk_ros/rtk_info_position", 10, &rtkPositionInfoSubCallback);
    rtkYawInfoSub          = nh.subscribe("dji_osdk_ros/rtk_info_yaw", 10, &rtkYawInfoSubCallback);
    rtkConnectionStatusSub = nh.subscribe("dji_osdk_ros/rtk_connection_status", 10, &rtkConnectionStatusSubCallback);
    flightAnomalySub       = nh.subscribe("dji_osdk_ros/flight_anomaly", 10, &flightAnomalySubCallback);
    attitudeSub      = nh.subscribe("dji_osdk_ros/attitude", 10, &attitudeSubCallback);
  }

  ros::Duration(1).sleep();
  ros::AsyncSpinner spinner(4);
  spinner.start();

  int GCSsocket = connectSocket(IPaddress, port);
  int count = 0;
  while (count < 15 )
  {
  
    ros::Duration(1).sleep();

   
    float latitude = gps_position_.latitude;
    float longitude = gps_position_.longitude;
    float altitude = gps_position_.altitude;
    float batteryPercent = battery_state_.percentage;
    
    float veloX = velocity_.vector.x;
    float veloY = velocity_.vector.y;
    float veloZ = velocity_.vector.z;

    StringBuffer telemetryData;
    Writer<StringBuffer> writer(telemetryData);

    writer.StartObject();
    writer.Key("latitude");
    writer.Double(latitude);
    writer.Key("longitude");
    writer.Double(longitude);
    writer.Key("altitude");
    writer.Double(altitude);
    writer.Key("Batt%");
    writer.Double(batteryPercent);
    writer.EndObject();
    
    const char* toSend = telemetryData.GetString();

    sendData(GCSsocket, toSend);
    count += 1;
  }

  std::cout << "Done printing!\n";
  
  ROS_INFO_STREAM("Finished. Press CTRL-C to terminate the node");
  ros::waitForShutdown();

  return 0;
}


int connectSocket(string IPaddress, int Port) {
  const char *IP = IPaddress.c_str();
  struct sockaddr_in serv_addr;

  int sock = socket(AF_INET, SOCK_STREAM, 0);
  ROS_INFO(" socket creation ok  \n");

  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = inet_addr(IP);
  serv_addr.sin_port = htons(Port);

       // Convert IPv4 and IPv6 addresses from text to binary form
    if(inet_pton(AF_INET, IP, &serv_addr.sin_addr)<=0) 
    {
        ROS_INFO("\n Invalid address/ Address not supported  \n");
        return -1;
    }
  ROS_INFO(" valid address \n");
  
  bind(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)); 
  ROS_INFO("socket binded");
  connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr));
  ROS_INFO("Socket connected");

return sock;

}

void sendData(int socket, string DataOutput) {
  const char* telemetryData = DataOutput.c_str();
  send(socket, telemetryData, strlen(telemetryData), 0);
  ROS_INFO(" Data has been sent \n");
}

