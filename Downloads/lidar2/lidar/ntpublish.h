#include <ntcore.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
class LidarTable {
    public:
    LidarTable(const char *ipAddress);
    ~LidarTable();

    static const unsigned int SAMPLES = 720;

    void setDistance(float angle, double distance);

    void updateTable();

    private:
    double m_distances[SAMPLES];
    nt::NetworkTableInstance m_instance;
    std::shared_ptr<NetworkTable> m_table;
    nt::NetworkTableEntry m_entry;

};
