#ifndef UTILS_HPP
#define UTILS_HPP

#include <geometry_msgs/Quaternion.h>

namespace Ogre
{
class Vector3;
}

/**
 * @brief convertFromImageToRealPosition
 * @param mapPos
 * @return Orge Vector3
 *
 * @details converst 2D map position to 3D real world (Gazebo) position
 */
const Ogre::Vector3 convertFromImageToRealPosition(const Ogre::Vector3& mapPos);

/**
 * @brief normilizeValue
 * @param value
 * @param tMin
 * @param tMax
 * @param vMin
 * @param vMax
 * @return a normilized value
 *
 * @details Normilize value to a new range
 */
double normilizeValue(double value, double tMin, double tMax, double vMin, double vMax);

/**
 * @brief getQuaternionOfVectors
 * @param u
 * @param v
 * @return Quaternion
 *
 * @details Calculate a Quaternion of two vectors
 */
geometry_msgs::Quaternion getQuaternionOfVectors(Ogre::Vector3 u, Ogre::Vector3 v);

/**
 * @brief orthogonalVecotor
 * @param v
 * @return an orthogonal vector to a provided vector
 */
Ogre::Vector3 orthogonalVecotor(Ogre::Vector3 v);

/**
 * @brief vectorDot
 * @param u
 * @param v
 * @return
 */
float vectorDot(Ogre::Vector3 u, Ogre::Vector3 v);

/**
 * @brief vectorCross
 * @param u
 * @param v
 * @return
 */
Ogre::Vector3 vectorCross(Ogre::Vector3 u, Ogre::Vector3 v);

#endif
