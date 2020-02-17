
#include "serial.h"
#include "ntpublish.h"

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <asm-generic/termbits.h>
#include <sys/ioctl.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <poll.h> 
#include <time.h>
#include <iostream>
#include <ntcore.h>
#include <networktables/NetworkTable.h>

using std::cout;
using std::cerr;
using std::endl;

void fail(const char *msg) {
  fprintf(stderr, "Operation failed: %s (%m)", msg);
  fflush(stderr);
  _exit(0);
}

struct Timer {
  time_t m_start;
  Timer() { m_start = time(0); }
  int elapsed() { return time(0) - m_start; }
};

void setSpeed(int fd) {
  struct termios2 tio;

  if (ioctl(fd, TCGETS2, &tio)) {
    fail("ioctl TCGETS2");
  }
  tio.c_cflag &= ~CBAUD;
  tio.c_cflag |= BOTHER;
  tio.c_ispeed = 128000;
  tio.c_ospeed = 128000;
  if (ioctl(fd, TCSETS2, &tio)) {
    fail("ioctl TCSETS2");
  }
  fprintf(stderr, "speed set\n");
} 


int timeoutRead(int fd, char *buf, int len) {
  Timer t;
  int bytes = 0;
  while (t.elapsed() < 5) {
    bytes = read(fd, buf, len);
    if ((bytes < 0) && (errno == EWOULDBLOCK)) {
      // no data was ready
      bytes = 0;
    }
    if (bytes) { break; }
  }
  return bytes;
}

int readExactly(int fd, char *buf, int len) {
  Timer t;
  char *p = buf;
  int left = len;
  while ((left > 0) && (t.elapsed() < 5))  {
    int bytes = timeoutRead(fd, p, left);
    if (bytes < 0) { break; }
    p += bytes;
    left -= bytes;
  }
  if (left != 0) {
      fprintf(stderr, "failed to get exactly %d bytes (got %d)\n", len, len - left);
      _exit(0);
  }
  return len;
}

void hexDump(const char *buf, int len) {
    for (int i = 0; i < len; ++i) {
      fprintf(stderr, "%02x.", buf[i] & 0xff);
    }
    fprintf(stderr, "\n");
}

LidarTable *g_table;

// Raw scan header -- corrections must still be applied
struct ScanHeader {
  uint8_t m_commandStart0;
  uint8_t m_commandStart1;
  uint8_t m_packageType;
  uint8_t m_sampleQuantity;
  uint16_t m_startAngle;
  uint16_t m_endAngle;
  uint16_t m_checkCode;

  float m_distances[512];
  float m_angles[512];

  int parse(const char *buf, int len) {
    if ( len < 10 ){ return -1; }
    m_commandStart0 = buf[0];
    m_commandStart1 = buf[1];

    if ((m_commandStart0 != 0xaa) ||
       (m_commandStart1 != 0x55)) {
         fprintf(stderr, "Scan Header magic mismatch: got %x %x, expected 5a a5\n", m_commandStart0 & 0xff, m_commandStart1 & 0xff);
        _exit(0);
    }

    m_packageType = buf[2];
    m_sampleQuantity = buf[3];
    m_startAngle = ((buf[5]>>1) << 8) + buf[4];
    m_endAngle = ((buf[7] >> 1) << 8) + buf[6];
    m_checkCode = buf[9] << 8 + buf[8];
    return 10;
  }

  float angleCorrect(int i) {
    float d = m_distances[i];
    if (d < 0.001f) { return 0; }
    // Correct the angle using the formula from the X4 documentation  
    float x = 21.8f * (155.3f - d)/(155.3f * d);
    float atanx = atan(x);
    float atanx_deg = atanx*(180 / M_PI);
    
    /*
    cerr << "correcting " << d << endl;
    cerr << "x = " << x << endl;
    cerr << "atanx = " << atanx << endl;
    cerr << "atanx_deg" << atanx_deg << endl;
    */
    return atanx_deg;
  }

  
  // parse and correct distances, storing into m_distances and m_angles
  void parseDistances(const char *buf, unsigned int len) {
    if (m_sampleQuantity < 0) { return; }
    // make sure we don't overrun 'buf' by reading too many samples
    if (m_sampleQuantity * 2 > len) { cerr << "too many samples / buffer too small\n"; _exit(0); }
    // make sure we don't overrun m_distances by reading too many samples
    if (m_sampleQuantity > 512) { cerr << "too many samples\n"; _exit(0); }
 
    // p will point to the current read location in the buffer
    const uint8_t *p = reinterpret_cast<const uint8_t *>(buf);
    // one past the end o fthe buffer
    const uint8_t *end = p + len;

    int i = 0;
    while (p < end) {
      // translate from little endian
      //hexDump(reinterpret_cast<const char *>(p), 2);
      unsigned int d = p[1] << 8;
      d += p[0];

      if (d != 0) {
        //fprintf(stderr, "p[0]=%x p[1]=%x d=%d\n", p[0] & 0xFF, p[1] & 0xFF, d);
      }

      // move pointer to the next distance
      p += 2;

      // correct raw distance using formula in datasheet
      float distance = (float)(d) / 4;
      if (d != 0) {
       // fprintf(stderr, "    d=%f\n", distance);
      }

      // Save the result for later user
      m_distances[i] = distance;
      ++i;
    }
    // We should have read all the data
    assert(i == m_sampleQuantity);
    
    // use distances to compute angle corrections
    float angleCorrection1 = angleCorrect(0);
    float angleCorrectionN = angleCorrect(m_sampleQuantity - 1);

    float fsa = ((float)(m_startAngle) / 64) + angleCorrection1;
    float lsa = ((float)(m_endAngle) / 64) + angleCorrectionN;
    if(fsa < 0){
	fsa += 360;
    }
    if(lsa < 0){
        lsa += 360;
    }
    cout << "FSA= " << fsa << endl;
    cout << "LSA= " << lsa << endl;
    float diffAngle;
    if(lsa >= fsa){
        diffAngle = (lsa - fsa) / (m_sampleQuantity - 1);
    } else{
        diffAngle = (fsa - lsa) / (m_sampleQuantity - 1);
    }
    // store corrected angles into m_angles
    for (i = 0; i < m_sampleQuantity; ++i) {
      float angle = fsa + (diffAngle * (i) ) + angleCorrect(i);
      // convert to range of 0-360
      angle = angle - floor(angle / 360) * 360;
      if (angle < 0) {
        angle += 360;
      }
      m_angles[i] = angle;
      g_table->setDistance(angle, m_distances[i]);
    }
    g_table->updateTable();
  }

  void printDistances() {
    int count = 0;

    float min = -1;
    float max = 0;
    
    for (int i = 0; i < m_sampleQuantity; ++i) {
      float d = m_distances[i];
      if ((min < 0) || (d < min)) {
        min = d;
      }
      if (d > max) {
        max = d;
      }
      if (d < 1) { continue; }
      fprintf(stdout, "%05.2f : %04.2f\n", m_angles[i], m_distances[i]);
    } 
    fprintf(stdout, "min=%f max=%f\n", min, max);
  }
  const char *lookupPackageType() {
    if (m_packageType == 0) { return "point cloud packet";}
    if (m_packageType == 1) { return "zero packet";}
    return "unknown package type";
  }

  void print() {
    cout << "----scan header ---" << endl;
    fprintf(stderr, "start0: %d (%x)\n", m_commandStart0, m_commandStart0 & 0xff);
    fprintf(stderr, "start1: %d (%x)\n", m_commandStart1, m_commandStart1 & 0xff);
    fprintf(stderr, "packageType: %d (%x = %s)\n", m_packageType, m_packageType &0xff, lookupPackageType());
    fprintf(stderr, "sampleQuantity: %d (%x)\n", m_sampleQuantity, m_sampleQuantity & 0xff);
    fprintf(stderr, "checkCode: %d (%x)\n", m_checkCode, m_checkCode & 0xff);

    float fsa = (m_startAngle) / 64;
    float lsa = (m_endAngle) / 64;
    fprintf(stderr, "fsa: %d => %.2f\n", m_startAngle, fsa);
    fprintf(stderr, "lsa: %d => %.2f\n", m_endAngle, lsa);
  }

};

struct DeviceInfo {
  uint8_t m_model;
  uint8_t m_firmwareMajor;
  uint8_t m_firmwareMinor;
  uint8_t m_hardwareVersion;
  uint8_t m_serialNumber[16];

  int parse(const char *buf, unsigned int len) {
    assert(len >= 20);
    m_model = buf[0];
    m_firmwareMajor = buf[2];
    m_firmwareMinor = buf[1];
    m_hardwareVersion = buf[3];
    ::memcpy(m_serialNumber, buf + 4, 16);
    return 20;
  }

  void print() {
    fprintf(stderr, "model: 0x%0x\n", m_model & 0xFF);
    fprintf(stderr, "firmware: %d.%d\n", m_firmwareMajor, m_firmwareMinor);
    fprintf(stderr, "hardware: v%d\n", m_hardwareVersion);
    fprintf(stderr, "Serial: ");
    for (int i = 0; i < 16; ++i) {
      fprintf(stderr, "%02x", m_serialNumber[i]);
    }
    fprintf(stderr, "\n");
  }

};


struct Header {
  uint8_t m_start0;
  uint8_t m_start1;
  uint32_t m_len;
  uint8_t m_mode;
  uint8_t m_typeCode;

  // takes a buffer and a length.
  // on failure, returns < 0
  // on success, returns number of bytes consumed from buf
  int parse(const char *buf, int len) {
    if (len < 7) { return -1; }
    m_start0 = buf[0];
    m_start1 = buf[1];

    if ((m_start0 != 0xa5) ||
       (m_start1 != 0x5a)) {
         fprintf(stderr, "Command Header magic mismatch: got %x %x, expected 5a a5\n", m_start0 & 0xff, m_start1 & 0xff);
         _exit(0);
       }
    // mode is the first 2 bits of byte 5
    uint8_t byte5 = buf[5];
    m_mode = byte5 >> 6;

    // len is 30 bits in bytes 2-5
    byte5 = (byte5 << 2) >> 2;
    m_len = byte5;
    m_len = (m_len << 8) + buf[4];
    m_len = (m_len << 8) + buf[3];
    m_len = (m_len << 8) + buf[2];
    m_typeCode = buf[6];
    return 7;
  } 

  void print() {
    fprintf(stderr, "Start0:   %x\n", m_start0 & 0xff);
    fprintf(stderr, "Start1:   %x\n", m_start1 & 0xff);
    fprintf(stderr, "Len:   %d (0x%x)\n", m_len, m_len);
    fprintf(stderr, "Mode:   %d ",m_mode);
    if (m_mode == 0) { fprintf(stderr, "(single response)\n");}
    else if (m_mode == 1) {fprintf(stderr, "(continuous response)\n");}
    else { fprintf(stderr, "(undefined)\n");}
    fprintf(stderr, "TypeCode:   0x%x\n", m_typeCode);
  }
};

void sendCmd(int fd, const uint8_t *cmd) {
  int bytes = write(fd, cmd, 2);
  if (bytes < 2) { fail("write"); }
}

int main(int, char **) {

  
  g_table = new LidarTable("10.0.20.2");
  sleep(3);
  int fd = open("/dev/ttyUSB0", O_RDWR | O_NONBLOCK, O_NOCTTY);

  if (fd < 0) { fail("open serial port"); }
  cout << "Serial port is open\n";

  setSpeed(fd);
  setupSerial(fd);
   std::cerr << "After the setup" << std::endl;

  const unsigned int cmd_response_headerLen = 7;  
  const uint8_t cmd_start[] = {0xa5, 0x60};
  const uint8_t cmd_end[] = {0xa5, 0x65};
  const uint8_t cmd_status[] = {0xA5, 0x90};
  const unsigned int cmd_status_responseContentLen = 20;
  const uint8_t cmd_health[] = {0xA5, 0x92};

  char buf[1024];
  int bytes;
  
  // reset the lidar by sending an END command and clearing
  // any buffered data
  write(fd, cmd_end, 2);
  sleep(1);
  while (read(fd, buf, bytes) > 0) {
  };  


  fprintf(stderr, "Checking status\n");
  sendCmd(fd, cmd_status);
  bytes = readExactly(fd, buf, cmd_response_headerLen + cmd_status_responseContentLen);
  fprintf(stderr, "read %d bytes of status\n", bytes);
  {
    const char *p = buf;
    unsigned int left = bytes;
    Header h;
    unsigned int used = h.parse(buf, left);
    p += used;
    left -= used;
    h.print();  

    DeviceInfo info;
    info.parse(p, left);
    info.print();
  }


  fprintf(stderr, "Starting scan\n");

  sendCmd(fd, cmd_start);
  cout << "command sent\n";
cerr << "About to read" << std::endl;
  // read a 7 byte packet header
  bytes = readExactly(fd, buf, 7);
cerr << "After read" << std::endl;
  cout << "Read " << bytes << " bytes of packet header\n";
  {
    cout << "--- header ---\n";
    hexDump(buf, 7);
    Header h;
    h.parse(buf, bytes);
    h.print(); 
    if (h.m_mode != 1) {
      cerr << "Expected continous mode response\n";
      _exit(0);
    }
    if (h.m_typeCode != 0x81) {
      cerr << "unexpected type code in response\n";
      _exit(0);
    }
  };
  cout << "---\n";

  int packets = 0;
  while (true) {
    ++packets;

   cerr << "In while" << std::endl;
    // read a 10 byte ScanHeader
    bytes = readExactly(fd, buf, 10);
    ScanHeader sh;
    {
        //hexDump(buf, 10);
        sh.parse(buf, 10);
        //sh.print();
    }
    int count = sh.m_sampleQuantity;
    if (sh.m_packageType == 1) {
      // it is a zero packet - not sure if it matters
    }

    if (count * 2 < 1024) {
      bytes = readExactly(fd, buf, count * 2);
      sh.parseDistances(buf, bytes);
      //sh.printDistances();
    } else {
      fprintf(stderr, "too big\n");
      _exit(0);
    }
  }

  
  sleep(5);
  fprintf(stderr, "Stopping scan\n");
  write(fd, cmd_end, 2);
  fprintf(stderr, "Done\n");
  close(fd);
  

   delete g_table;
  
  fprintf(stderr, "Goodbye.\n");
  
}

