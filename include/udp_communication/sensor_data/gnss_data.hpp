
#ifndef UDP_COMMUNICATION_SENSOR_DATA_GNSS_DATA_HPP_
#define UDP_COMMUNICATION_SENSOR_DATA_GNSS_DATA_HPP_

#include <deque>

#include <Geocentric/LocalCartesian.hpp>
#include <yaml-cpp/yaml.h>

namespace udp_communication {
class GNSSData {
  public:
    double time = 0.0;
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    double local_E = 0.0;
    double local_N = 0.0;
    double local_U = 0.0;
    int status = 0;
    int service = 0;

    static double origin_longitude;
    static double origin_latitude;
    static double origin_altitude;

  private:
    static GeographicLib::LocalCartesian geo_converter;
    static bool origin_position_inited;

  public: 
    void InitOriginPosition();
    void UpdateXYZ();

    static void Reverse(
      const double &local_E, const double &local_N, const double &local_U,
      double &lat, double &lon, double &alt
    );

    static bool SyncData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData, double sync_time);
};
}
#endif