#ifndef XY2000_LB2000_H
#define XY2000_LB2000_H

#include <math.h>

namespace XY2000 {
/**
 * @brief XY2000_LB2000
 * @param y0
 * @param x0
 * @param L
 * @param B
 * @param z_zone
 */
void XY2000_LB2000(double y0, double x0, double& L, double& B, int z_zone);

/**
 * @brief LB2000_XY2000
 * @param L
 * @param B
 * @param globalX
 * @param globalY
 */
void LB2000_XY2000(double L, double B, int &globalX, int &globalY);
};


/**
 * @brief Mercator Projection Transform
 */
namespace Mercator {

/**
 * @brief XY2LB, Mercator Projection, from global x-y coordinate to lon-lat coordinate
 * @param x, global x, m
 * @param y, global y, m
 * @param L, longitude,degree
 * @param B, latitude, degree
 */
void XY2LB(double x, double y, double& L, double& B);

/**
 * @brief LB2XY, Mercator Projection, from lon-lat coordinate to global x-y coordinate
 * @param L, longitude, rad
 * @param B, latitude, rad
 * @param x, global x, 0.1m
 * @param y, global y, 0.1m
 */
void LB2XY(double L, double B, int& x, int& y);
}

#endif // XY2000_LB2000_H
