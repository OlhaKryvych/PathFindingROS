#ifndef UTILS_HPP
#define UTILS_HPP

#include <geometry_msgs/Quaternion.h>

namespace Ogre
{
class Vector3;
}

const Ogre::Vector3 convertFromImageToRealPosition(const Ogre::Vector3& mapPos);

double normilizeValue(double value, double tMin, double tMax, double vMin, double vMax);

geometry_msgs::Quaternion getQuaternionOfVectors(Ogre::Vector3 u, Ogre::Vector3 v);

Ogre::Vector3 orthogonalVecotor(Ogre::Vector3 v);

float vectorDot(Ogre::Vector3 u, Ogre::Vector3 v);

Ogre::Vector3 vectorCross(Ogre::Vector3 u, Ogre::Vector3 v);

#endif