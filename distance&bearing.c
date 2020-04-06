#include <math.h>

double to_radians(double degrees) {
    // Convert degrees to radians
    return degrees/180.0*3.14159265359;
}

double calculateDistance(double lat1, double lat2, double deltaLat, double deltaLon){
    // Calculate distance between current location and destination using Haversine formula
//    double lat1 = to_radians(currLat);            // move these 4 lines to caller function
//    double lat2 = to_radians(destLat);
//    double deltaLat = to_radians(destLat-currLat);
//    double deltaLon = to_radians(destLon-currLon);
    double a = pow(sin(deltaLat/2), 2.0) + cos(lat1)*cos(lat2)*pow(sin(deltaLon/2), 2.0);
    return 2*atan2(sqrt(a), sqrt(1-a))*6371000.0;
}

double calculateBearing(double lat1, double lat2, double deltaLon) {
    // Calculate bearing, result is radians from north to east
    // Remember to convert returned value to degrees in caller function
    return atan2(sin(deltaLon)*cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(deltaLon));
}
