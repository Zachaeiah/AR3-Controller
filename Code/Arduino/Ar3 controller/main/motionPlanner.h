#ifndef MOTIONPLANNER_H
#define MOTIONPLANNER_H
#include "VelocityPlanner.h"
#include "global.h"

/**
 * @brief Returns a TCP_point at a specified displacement along the linear path from start to end.
 *        Both position and orientation (rx, ry, rz) are linearly interpolated.
 *
 * @param start Starting TCP_point
 * @param end Ending TCP_point
 * @param displacement Distance in mm from start along the path to end
 * @return TCP_point Resulting point along the path
 */
TCP_point point_along_tcp_line(const TCP_point *start, const TCP_point *end, float32_t displacement);

//Velocity_along_tcp_line(&point1, &point2, disp);
#endif