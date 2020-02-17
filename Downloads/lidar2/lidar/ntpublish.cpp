
#include "ntpublish.h"

#include <ntcore.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <assert.h>
#include <math.h>
#include <iostream>

using nt::NetworkTable;
using nt::NetworkTableInstance;
using std::cerr;

// Rounds the angle to an index between 0 and 720
int roundAngle(float angle) {
    angle = angle - floor(angle / 360.0) * 360;
    if (angle < 0) {
        angle += 360;
    }
   return static_cast<int>(angle * 2);
}

static void logMsg(const nt::LogMessage &m) {
   cerr << "WPILIB: " << m.message << "\n";
};

LidarTable::LidarTable(const char *ipStr) :
    m_instance(NetworkTableInstance::GetDefault()), 
    m_table(m_instance.GetTable("LIDAR")),
    m_entry(m_table->GetEntry("PointCloud"))
 {
    // m_instance.StartLocal(); // remove on actual robot

    m_instance.StartClient();
    m_instance.SetServer(ipStr);
        fprintf(stderr, "========= %x table opned at address 0x%x\n", this, m_table.get());
   m_entry = m_table->GetEntry("PointCloud");

   m_instance.AddLogger(logMsg, nt::NetworkTableInstance::kLogDebug4, nt::NetworkTableInstance::kLogCritical);
}

LidarTable::~LidarTable() {
   cerr << "Shutting down table\n";
   m_instance.StopClient();
}

void LidarTable::setDistance(float angle, double distance) {
  //  std::cout << roundAngle(angle) << std::endl;
   m_distances[roundAngle(angle)] = distance;
}

void LidarTable::updateTable() {
    float leastDistance = 10000;
    for(int i = 706; i<727; ++i){
        if(m_distances[i%720] < leastDistance && m_distances[i%720] != 0){
	    leastDistance = m_distances[i%720];
	}
    }
    std::cout << "least distance: " << leastDistance <<std::endl;
    //auto p = m_table.get();
    //fprintf(stderr, "%x table=%x\n", this, p);
   //wpi::ArrayRef<double> arr(m_distances, LidarTable::SAMPLES);
   // m_entry.SetDoubleArray(arr);
  m_entry.SetDouble(static_cast<double>(leastDistance));
//    m_entry.SetDoubleArray(m_distances);
};
