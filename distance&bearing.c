#include <math.h>

float to_radians(float degrees) {
    // Convert degrees to radians
    return degrees/180.0*Pi;
}

float calculateDistance(float lat1, float lat2, float deltaLat, float deltaLon){
    // Calculate distance between current location and destination using Haversine formula
//    float lat1 = to_radians(currLat);            // move these 4 lines to caller function
//    float lat2 = to_radians(destLat);
//    float deltaLat = to_radians(destLat-currLat);
//    float deltaLon = to_radians(destLon-currLon);
    float a = pow(sin(deltaLat/2), 2.0) + cos(lat1)*cos(lat2)*pow(sin(deltaLon/2), 2.0);
    return 2*atan2(sqrt(a), sqrt(1-a))*6371000.0;
}

float calculateBearing(float lat1, float lat2, float deltaLon) {
    // Calculate bearing, result is radians from north to east
    // Remember to convert returned value to degrees in caller function
    return atan2(sin(deltaLon)*cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(deltaLon));
}
