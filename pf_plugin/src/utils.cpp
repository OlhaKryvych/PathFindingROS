#include "utils.hpp"

#include <OgreVector3.h>

#include "config.hpp"

const Ogre::Vector3 convertFromImageToRealPosition(const Ogre::Vector3& mapPos) {
    Ogre::Vector3 res;
    
    using namespace CONFIG;

    res.x = normilizeValue(mapPos.x, WORLD_X_MIN, WORLD_X_MAX, 0, MAP_WIDTH);
    res.y = normilizeValue(mapPos.y, -WORLD_Y_MIN, -WORLD_Y_MAX, 0, MAP_HEIGHT); // need to switch as y axis is inverted (y_min is y_max)
    res.z = 0.0;

    return res;
}

double normilizeValue(double value, double tMin, double tMax, double vMin, double vMax) {
    return (tMax - tMin) * ( (value - vMin) / (vMax - vMin) ) + tMin;
}

Ogre::Vector3 orthogonalVecotor(Ogre::Vector3 v) {
    float x = std::abs(v.x);
    float y = std::abs(v.y);
    float z = std::abs(v.z);

    Ogre::Vector3 other;

    Ogre::Vector3 X_AXIS = {1.0, 0.0, 0.0};
    Ogre::Vector3 Y_AXIS = {0.0, 1.0, 0.0};
    Ogre::Vector3 Z_AXIS = {0.0, 0.0, 1.0};

    other = x < y ? ( x < z ? X_AXIS : Z_AXIS) : (y < z ? Y_AXIS : Z_AXIS);

    return vectorCross(v, other);
}

float vectorDot(Ogre::Vector3 u, Ogre::Vector3 v) {
    float res = 0.0;

    res += u.x * v.x;
    res += u.y * v.y;
    res += u.z * v.z;

    return res;
}

Ogre::Vector3 vectorCross(Ogre::Vector3 u, Ogre::Vector3 v) {
    Ogre::Vector3 r;

    r.x = u.y * v.z - u.z * v.y;
    r.y = u.z * v.x - u.x * v.z;
    r.z = u.x * v.y - u.y * v.x; 

    return r;
}

geometry_msgs::Quaternion getQuaternionOfVectors(Ogre::Vector3 u, Ogre::Vector3 v) {
   
    geometry_msgs::Quaternion res;

    u = u.normalisedCopy();
    v = v.normalisedCopy();

    if( u == -v ) {
        res.w = 0.0;
        auto _p = orthogonalVecotor(u).normalisedCopy();

        res.x = _p.x;
        res.y = _p.y;
        res.z = _p.z;
    } else {
        auto half = (u + v).normalisedCopy();

        res.w = vectorDot(u, half);
        
        auto _p = vectorCross(u, half);
        res.x = _p.x;
        res.y = _p.y;
        res.z = _p.z;
    }

    return res;
}