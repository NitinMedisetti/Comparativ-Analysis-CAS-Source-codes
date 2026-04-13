#ifndef LD20_LIDAR_H
#define LD20_LIDAR_H

// --- Protocol & Configuration Constants ---
#define LIDAR_HEADER 0x54
#define LIDAR_VERLEN 0x2C
#define POINT_PER_PACK 12
#define PACKET_SIZE 47

#ifdef robot
  #define lidar_offset 180 
#else 
  #define lidar_offset 90
#endif

// --- Data Structures ---

/**
 * @brief Represents a single point measurement from the LiDAR.
 */
struct LidarPoint {
  uint16_t distance; 
  uint8_t intensity; 
  float angle;       
};

/**
 * @brief LD20 LiDAR driver class.
 */
class LD20_Lidar {
public:
  // --- Public Members (Exposed for direct data access) ---
  static const int POINTS_PER_FRAME = POINT_PER_PACK;
  
  // The calculated frame data is stored here after a successful read.
  uint16_t speed;
  LidarPoint points[POINTS_PER_FRAME];


  LD20_Lidar(HardwareSerial* serialPort) : _lidarSerial(serialPort), _bufferIndex(0) {}

 
  void begin(uint32_t baud, int rx_pin, int tx_pin) {
    _lidarSerial->begin(baud, SERIAL_8N1, rx_pin, tx_pin);
  }


  inline bool readPacket() {
    bool newFrameProcessed = false;
    int maxBytes = 256;
    while (_lidarSerial->available() && maxBytes--) {
      uint8_t inByte = _lidarSerial->read();
      
      if (_bufferIndex == 0 && inByte != LIDAR_HEADER) {
        continue;
      }
      
      _rxBuffer[_bufferIndex++] = inByte;
      
      if (_bufferIndex >= PACKET_SIZE) {
        if (_rxBuffer[0] == LIDAR_HEADER && _rxBuffer[1] == LIDAR_VERLEN) {
          uint8_t calculatedCRC = calculateCRC8(_rxBuffer, PACKET_SIZE - 1);
          
          if (calculatedCRC == _rxBuffer[PACKET_SIZE - 1]) {
            RawLidarPacket raw_packet;
            if (parsePacket(_rxBuffer, &raw_packet)) {
              // Calculate angles and update public members
              calculateAngles(raw_packet);
              newFrameProcessed = true;
            }
          }
        }
        _bufferIndex = 0; 
      }
    }
    
    return newFrameProcessed;
  }

private:
  HardwareSerial* _lidarSerial;
  uint8_t _rxBuffer[PACKET_SIZE];
  int _bufferIndex;

  // Internal structure for raw packet data
  struct RawLidarPacket {
    uint8_t header;
    uint8_t ver_len;
    uint16_t speed;
    uint16_t start_angle;
    struct {
      uint16_t distance;
      uint8_t intensity;
    } raw_points[POINT_PER_PACK];
    uint16_t end_angle;
    uint16_t timestamp;
    uint8_t crc8;
  };

  // CRC8 lookup table (Defined inline for header compilation)
  static const uint8_t CrcTable[256];

  // --- Private Inline Function Implementations ---
  
  inline uint8_t calculateCRC8(uint8_t *data, uint8_t len) const {
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
      crc = CrcTable[(crc ^ data[i]) & 0xff];
    }
    return crc;
  }

  inline bool parsePacket(uint8_t *packet, RawLidarPacket* raw_packet) {
    int idx = 0;
    raw_packet->header = packet[idx++];
    raw_packet->ver_len = packet[idx++];
    
    raw_packet->speed = packet[idx] | (packet[idx + 1] << 8); idx += 2;
    raw_packet->start_angle = packet[idx] | (packet[idx + 1] << 8); idx += 2;
    
    for (int i = 0; i < POINT_PER_PACK; i++) {
      raw_packet->raw_points[i].distance = packet[idx] | (packet[idx + 1] << 8); idx += 2;
      raw_packet->raw_points[i].intensity = packet[idx]; idx += 1;
    }
    
    raw_packet->end_angle = packet[idx] | (packet[idx + 1] << 8); idx += 2;
    raw_packet->timestamp = packet[idx] | (packet[idx + 1] << 8); idx += 2;
    raw_packet->crc8 = packet[idx];
    
    return true;
  }

  // NOTE: This now writes directly to the public members of the class instance.
  inline void calculateAngles(const RawLidarPacket& raw_packet) {
    this->speed = raw_packet.speed;
    
    float start_angle_deg = raw_packet.start_angle / 100.0;
    float end_angle_deg = raw_packet.end_angle / 100.0;
    
    if (end_angle_deg < start_angle_deg) {
      end_angle_deg += 360.0;
    }
    
    float angle_step = (end_angle_deg - start_angle_deg) / (POINT_PER_PACK - 1);
    
    for (int i = 0; i < POINT_PER_PACK; i++) {
      this->points[i].distance = raw_packet.raw_points[i].distance;
      this->points[i].intensity = raw_packet.raw_points[i].intensity;
      
      this->points[i].angle = start_angle_deg + (angle_step * i) + lidar_offset;
      
      if (this->points[i].angle >= 360.0) {
        this->points[i].angle -= 360.0;
      }
    }
  }
};

// --- Static CRC Table Definition ---
const uint8_t LD20_Lidar::CrcTable[256] = {
  0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
  0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
  0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
  0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
  0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
  0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
  0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
  0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
  0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
  0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
  0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
  0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
  0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
  0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
  0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
  0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
};

#endif // LD20_LIDAR_H