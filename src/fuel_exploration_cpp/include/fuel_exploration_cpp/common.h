#pragma once
#include <vector>
#include <cmath>

namespace fuel {

struct Frontier {
  double x, y;
  std::vector<std::pair<double,double>> points;
  int gain;
  int size;
  double cost;
};

inline double dist2d(double x1, double y1, double x2, double y2) {
  double dx = x1-x2, dy = y1-y2;
  return std::sqrt(dx*dx + dy*dy);
}

} // namespace fuel
