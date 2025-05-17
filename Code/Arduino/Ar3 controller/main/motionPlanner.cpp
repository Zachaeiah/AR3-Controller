#include "motionPlanner.h"

/**
 * @brief Returns a TCP_point at a specified displacement along the linear path from start to end.
 *        Both position and orientation (rx, ry, rz) are linearly interpolated.
 *
 * @param start Starting TCP_point
 * @param end Ending TCP_point
 * @param displacement Distance in mm from start along the path to end
 * @return TCP_point Resulting point along the path
 */
TCP_point point_along_tcp_line(const TCP_point *start, const TCP_point *end, float32_t displacement) {
  TCP_point result;
  float32_t t = 0;

  if (displacement < 0.0f) {
    // Return the start if the distance is zero
    return *start;
  }

  // Normalized interpolation factor
  t = displacement / VI_RT.Displacements.totel_dis;

  // Linearly interpolate position
  result.x = start->x + VI_RT.Displacements.x_dis * t;
  result.y = start->y + VI_RT.Displacements.y_dis * t;
  result.z = start->z + VI_RT.Displacements.z_dis * t;

  // Linearly interpolate rotation
  result.rx = start->rx + (VI_RT.Displacements.rx_dis) * t;
  result.ry = start->ry + (VI_RT.Displacements.ry_dis) * t;
  result.rz = start->rz + (VI_RT.Displacements.rz_dis) * t;

  return result;
}