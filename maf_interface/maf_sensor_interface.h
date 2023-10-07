#ifndef MAF_INTERFACE_MAF_SENSOR_INTERFACE_H
#define MAF_INTERFACE_MAF_SENSOR_INTERFACE_H

#include "maf_interface/maf_std.h"
#include <stdint.h>
#include <string>
#include <vector>

/*!
 * \brief CompressedVideo include encoded video information.
 */
namespace maf_sensor_interface {

struct CompressedVideo {
  maf_std::Header header; //!< Sequence, frame_id, stamp are included in header.
  std::string format;     //!< Sequence, frame_id, stamp are included in header.
  std::vector<uint8_t> data; //!< Encoded data.
};

/*!
 * \brief ultrasonic sonar parkslot  bordertype
 * \note
 * * the bordertype  of  ultrasonic sonar parkslot  for hikvision
 * * the starting point is at the lower left corner of the parking space
 * opening, and it increases in a clockwise direction
 */
struct BordertType {
  uint8_t value;

  enum : uint8_t {
    HAT_USLOT_NON_CURB = 0,   //!< curb not exist
    HAT_USLOT_EXIST_CURB = 1, //!< curb not exist
    UNKNOWN = 2,              //!< unknown curb
    RESERVED_INFO = 3,        //!< reserved info
  };
};

/*!
 * \brief ultrasonic sonar result metatime
 * \note
 * * the metatime  of  ultrasonic sonar result
 */
struct UltrasonicResultMeta {
  /*!
   * \brief timestamp
   * \unit{us}
   */
  uint64_t timestamp_us;
  uint64_t seq; //!< record the number of times sent
};

/*!
 * \brief ultrasonic sonar obstacle height type
 * \note
 * * the type  of  ultrasonic sonar obstacle height
 */
struct UltrasonicObjectHeiType {
  uint8_t value;

  enum : uint8_t {
    LOW = 0,         //!< low type
    HIGH = 1,        //!< high type
    TRAVERSIBLE = 2, //!< traversible type
    UNKONOWN = 3,    //!< unknown type
  };
};

/*!
 * \brief ultrasonic sonar obstacle type
 * \note
 * * the type  of  ultrasonic sonar obstacle
 */
struct UltrasonicObjectType {
  uint8_t value;

  enum : uint8_t {
    NONE = 0,            //!< none
    POINT = 1,           //!< point type
    STRAIGHT0CORNER = 2, //!< infinite extension at both ends type
    STRAIGHT0C1RNER = 3, //!< rays type
    STRAIGHT0C2RNER = 5, //!< line segment type
    SNA = 7,             //!< sna type
    RESERVED_INFO = 8,   //!< reserved type
  };
};

/*!
 * \brief Velodyne each packet of point cloud data
 */
struct VelodynePacket {
  uint64_t stamp; //!< sensor meta timesamp \unit{ns}
  int16_t id;     // sensor packet id
  /*!
   * \brief Velodyne lidar pointcloud packet data
   * \vec_max_size{1206}
   */
  std::vector<uint8_t> data;
};

/*!
 * \brief lidar PointField
 * \note
 * * This message holds the description of one point entry in the
 * * PointCloud2 message format.
 */
struct PointField {
  std::string name; //!< Name of field
  uint32_t offset;  //!< Offset from start of point struct
  uint8_t datatype; //!< Datatype enumeration, see below
  uint32_t count;   //!< How many elements in the field

  enum : uint8_t {
    INT8 = 1,
    UINT8 = 2,
    INT16 = 3,
    UINT16 = 4,
    INT32 = 5,
    UINT32 = 6,
    FLOAT32 = 7,
    FLOAT64 = 8,
  };
};

/*!
 * \brief Robosense each packet of point cloud data
 */
struct RobosensePacket {
  uint64_t stamp; //!< sensor meta timesamp(ns)
  int16_t id;     // sensor packet id
  /*!
   * \brief robosense lidar pointcloud packet header data
   * \vec_max_size{80}
   */
  std::vector<uint8_t> header;
  /*!
   * \brief robosense lidar pointcloud packet data
   * \vec_max_size{1168}
   */
  std::vector<uint8_t> data;
};

/*!
 * \brief ultrasonic sonar parkslot result of hikvision
 * \note
 * * the ultrasonic sonar parkslot result of hikvision
 */
struct HikvisionUssParkSlotResult {
  uint8_t available;                         //!< data available flag
  UltrasonicResultMeta meta;                 //!< data metatime
  uint8_t parkslot_id;                       //!< parkslot id
  std::vector<BordertType> border_type_data; //!< parkslot border type
  /*!
   * \brief The four corners of the parking space
   * \note
   * *  The starting point is at the lower left corner of the parking space
   * opening, and it increases in a clockwise direction
   */
  float veh_obj1_x; //!< The x coordinate of the first point of the four corner
                    //!< points of the parking space
  float veh_obj1_y; //!< The y coordinate of the first point of the four corner
                    //!< points of the parking space
  float veh_obj2_x; //!< The x coordinate of the second point of the four corner
                    //!< points of the parking space
  float veh_obj2_y; //!< The y coordinate of the second point of the four corner
                    //!< points of the parking space
  float veh_obj3_x; //!< The x coordinate of the third point of the four corner
                    //!< points of the parking space
  float veh_obj3_y; //!< The y coordinate of the third point of the four corner
                    //!< points of the parking space
  float veh_obj4_x; //!< The x coordinate of the fourth point of the four corner
                    //!< points of the parking space
  float veh_obj4_y; //!< The y coordinate of the fourth point of the four corner
                    //!< points of the parking space
  float parkslot_prob;       //!< parkslot probability
  std::string reserved_info; //!< reserved type
};

/*!
 * \brief ultrasonic sonar parkslot result of bosch
 * \note
 * * the ultrasonic sonar parkslot result of bosch
 */
struct BoschUssParkSlotResult {
  uint8_t available;         //!< data available flag
  UltrasonicResultMeta meta; //!< data metatime
  float slot_size_width;     //!< Width of parking space
  float slot_size_length;    //!< length of parking space
  float veh_alpha; //!< The angle between the vehicle and the vehicle_obstacle 2
  float vehicle_x; //!< The abscissa of the vehicle relative to obstacle 2
  float vehicle_y; //!< The ordinate of the vehicle relative to obstacle 2
  /*!
   * \brief The first obstacle that the car passes through the opening of the
   * parking space is obstacle 1
   */
  float veh_obj1_x;     //!< The abscissa of vehicle obstacle 1
  float veh_obj1_y;     //!< The ordinate of vehicle obstacle 1
  float veh_obj1_alpha; //!< The angle of vehicle obstacle 1
  /*!
   * \brief The second  obstacle that the car passes through the opening of the
   * parking space is obstacle 1
   */
  float veh_obj2_x;          //!< The abscissa of vehicle obstacle 2
  float veh_obj2_y;          //!< The ordinate of vehicle obstacle 2
  float veh_obj2_alpha;      //!< The angle of vehicle obstacle 2
  float parkslot_prob;       //!<
  std::string reserved_info; //!< reserved type
};

/*!
 * \brief ultrasonic sonar obstacle data
 * \note
 * * the data of ultrasonic sonar obstacle
 */
struct UltrasonicObstacleData {
  uint8_t available;                  //!< data available flag
  UltrasonicResultMeta meta;          //!< data metatime
  UltrasonicObjectType obj_type;      //!< obstacle type
  UltrasonicObjectHeiType obj_height; //!<  obstacle height type
  float obj_height_probability;       //!< obstacle height probability
  float obj_probability;              //!< obstacle  probability
  /*!
   * \brief Use two points to represent obstacle information
   */
  float obj_start_X; //!< The abscissa of the starting point of the obstacle
  float obj_start_Y; //!< The ordinate of the end point of the obstacle
  float obj_end_X;   //!< The abscissa of the starting point of the obstacle
  float obj_end_Y;   //!< The ordinate of the end of the obstacle
  float obj_height_value;    //!< obstacle height value  \unit{m}
  std::string reserved_info; //!< reserved type
};

/*!
 * \brief ultrasonic sonar upa data
 * \note
 * * Universal Parking Assistance(upa) data
 */
struct UltrasonicUpaData {
  uint8_t available;         //!< data available flag
  UltrasonicResultMeta meta; //!< data metatime
  float upadistance;         //!< upa data
  std::string reserved_info; //!< reserved type
};

/*!
 * \brief Lidar Pointcloud2 data
 * \note
 * Stores the point cloud data scanned in a circle
 */
struct Pointcloud2 {
  maf_std::Header header;         //!< header
  uint32_t height;                //!< lidar pointcloud height
  uint32_t width;                 //!<  lidar pointcloud width
  std::vector<PointField> fields; //!< lidar pointcloud fields
  bool is_bigendian;              //!< Is bigendian
  uint32_t point_step;            //!< lidar pointcloud point step
  uint32_t row_step;              //!< lidar pointcloud row step
  /*!
   * \brief velodyne lidar pointcloud data
   * \vec_max_size{240000}
   */
  std::vector<uint8_t> data;
  bool is_dense; //!< Is dense
};

/*!
 * \brief Store the packaged data of each circle
 */
struct LidarScan {
  maf_std::Header header; //!< header
  uint8_t available;      //!< data available flag
  /*!
   * \brief velodyne lidar point packet include pointcloud data
   * \vec_max_size{625}
   */
  std::vector<VelodynePacket> velodyne_packets;
  /*!
   * \brief robosense lidar point packet include pointcloud data
   * \vec_max_size{625}
   */
  std::vector<RobosensePacket> robosense_packets;

  enum : uint8_t {
    LIDAR_TYPE_VELODYNE = 1,  //!< velodyne lidar
    LIDAR_TYPE_ROBOSENSE = 2, //!< robosense lidar
  };
};

/*!
 * \brief ultrasonic sonar parkslot data for hikvision
 * \note
 * Includes 9 types of parking space results
 */
struct HikvisionUssParkSlotData {
  /*!
   * \brief parkslot data
   * \vec_max_size{9}
   */
  std::vector<HikvisionUssParkSlotResult> park_slot_result;

  enum : uint8_t {
    HK_PSC_SLOT_PARALLE_LE =
        0, //!< Horizontal parking space on the left when park in
    HK_PSC_SLOT_PARALLE_RI =
        1, //!< Horizontal parking space on the right when park in
    HK_PSC_SLOT_CROSS_LE =
        2, //!< Vertical parking space on the left when park in
    HK_PSC_SLOT_CROSS_RI =
        3, //!< Vertical parking space on the right when park in
    HK_PSC_SLOT_OBLIQUE_LE =
        4, //!< Oblique parking space on the left when park in
    HK_PSC_SLOT_OBLIQUE_RI =
        5, //!< Oblique parking space on the right when park in
    HK_POC_SLOT_PARALLE_LE =
        6, //!< Horizontal parking space on the left when park out
    HK_POC_SLOT_PARALLE_RI =
        7,             //!< Horizontal parking space on the right when park out
    RESERVED_INFO = 8, //!< reserved info
  };
};

/*!
 * \brief ultrasonic sonar upa data  value
 * \note
 * * Universal Parking Assistance(upa) data value
 */
struct UltrasonicUpaDistance {
  /*!
   * \brief upa data
   * \vec_max_size{12}
   */
  std::vector<UltrasonicUpaData> sonar;

  enum : uint8_t {
    FRONT_LEFT_SIDE = 0,    //!< Front left side of the vehicle
    FRONT_LEFT_CORNER = 1,  //!< Front left corner of the vehicle
    FRONT_LEFT_CENTER = 2,  //!< Front left center of the vehicle
    FRONT_RIGHT_CENTER = 3, //!< Front right center of the vehicle
    FRONT_RIGHT_CORNER = 4, //!< Front right corner of the vehicle
    FRONT_RIGHT_SIDE = 5,   //!< Front right side of the vehicle
    REAR_LEFT_SIDE = 6,     //!< Rear left side of the vehicle
    REAR_LEFT_CORNER = 7,   //!< Rear left corner of the vehicle
    REAR_LEFT_CENTER = 8,   //!< Rear left center of the vehicle
    REAR_RIGHT_CENTER = 9,  //!< Rear right center of the vehicle
    REAR_RIGHT_CORNER = 10, //!< Rear right corner of the vehicle
    REAR_RIGHT_SIDE = 11,   //!< Rear right side of the vehicle
  };
};

/*!
 * \brief ultrasonic sonar park slot  data  for bosch
 * \note
 * * the parkslot result  of bach
 */
struct BoschUssParkSlotData {
  /*!
   * \brief parkslot result
   * \vec_max_size{9}
   */
  std::vector<BoschUssParkSlotResult> park_slot_result;

  enum : uint8_t {
    BOSCH_PSC_SLOT_PARALLE_LE =
        0, //!< Horizontal parking space on the left when park in
    BOSCH_PSC_SLOT_PARALLE_RI =
        1, //!< Horizontal parking space on the right when park in
    BOSCH_PSC_SLOT_CROSS_LE =
        2, //!< Vertical parking space on the left when park in
    BOSCH_PSC_SLOT_CROSS_RI =
        3, //!< Vertical parking space on the right when park in
    BOSCH_PSC_SLOT_OBLIQUE_LE =
        4, //!< Oblique parking space on the left when park in
    BOSCH_PSC_SLOT_OBLIQUE_RI =
        5, //!< Oblique parking space on the right when park in
    BOSCH_POC_SLOT_PARALLE_LE =
        6, //!< Horizontal parking space on the left when park out
    BOSCH_POC_SLOT_PARALLE_RI =
        7,             //!< Horizontal parking space on the right when park out
    RESERVED_INFO = 8, //!< reserved info
  };
};

/*!
 * \brief ultrasonic sonar obstacle data
 */
struct UltrasonicObstacleResult {
  /*!
   * \brief obstacle data
   * \vec_max_size{20}
   */
  std::vector<UltrasonicObstacleData> uss_obstacle_data;
};

/*!
 * \brief ultrasonic sonar park slot  result
 * \note
 * * Contains 2 types of parking spaces:bosch,hikvision
 */
struct UltrasonicParkSlotResult {
  uint8_t available;
  BoschUssParkSlotData bosch_uss_parkslot_data; //!< bosch parkslot data
  HikvisionUssParkSlotData
      hikvision_uss_parkslot_data; //!< ikvision parkslot data

  enum : uint8_t {
    BOSCH_USS_PARKSLOT_DATA = 1,
    HIKVISION_USS_PARKSLOT_DATA = 2,
  };
};

/*!
 * \brief ultrasonic sonar report result
 * \note
 * * Contains  upa,ostacle result and parkslot result
 */
struct UltrasonicReport {
  maf_std::Header header;
  uint8_t status;
  UltrasonicUpaDistance uss_upa_distance;
  UltrasonicObstacleResult uss_obstacle_result;
  UltrasonicParkSlotResult uss_parkslot_result;
};

} // namespace maf_sensor_interface

#endif
