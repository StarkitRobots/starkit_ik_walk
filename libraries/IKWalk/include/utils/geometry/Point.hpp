#pragma once

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <vector>

#include <Eigen/Core>

#include "Angle.hpp"

namespace rhoban
{
  namespace utils
  {
    class Point
    {
    public:

      double x;
      double y;

      Point() : x(0.0), y(0.0){};

      explicit Point(double x, double y) : x(x), y(y){};
      explicit Point(const Eigen::Vector2d& p) : x(p.x()), y(p.y()) {}

      Point(const Point& other) = default;
  
      inline const auto toVector() const
      {
        return Eigen::Vector2d(x, y);
      }

    public:

      inline constexpr auto getX() const
      {
        return x;
      };

      inline constexpr auto getY() const
      {
        return y;
      };
      
      inline const auto getTheta() const
      {
        return Angle(std::atan2(getY(), getX()), Angle::Type::RAD);
      }

      /* Return the distance to point in {0,0} */
      double getDist(const Point& other) const;

      inline constexpr auto getLength() const
      {
        return getDist(Point(0.0, 0.0));
      }

      Point& normalize(double newLength = 1);

      // TODO handle case with point at 0.0, 0.0 in getTheta and isCollinear
      bool isCollinear(const Point& other) const;

      /* Return a Point with a rotation of 90° */
      inline const auto perpendicular() const
      {
        return Point(-y, x);
      }

      Point rotation(const Angle& a) const;
      Point rotation(const Angle& a, const Point& rotCenter) const;

      static double dotProduct(const Point& p1, const Point& p2);
      static double perpDotProduct(const Point& p1, const Point& p2);

      Point operator-() const;

      Point operator+(const Point& other) const;
      Point operator-(const Point& other) const;
      Point operator*(double ratio) const;
      Point operator/(double ratio) const;

      Point& operator+=(const Point& other);
      Point& operator-=(const Point& other);
      Point& operator*=(double ratio);
      Point& operator/=(double ratio);

      static Point mkPointFromPolar(double rho, Angle theta);

      // Random points creators
      // static Point mkRandomPolar(double rhoMax, std::default_random_engine* engine = nullptr);
      // static Point mkRandomPolar(double rhoMin, double rhoMax, std::default_random_engine* engine = nullptr);
      // static Point mkRandomPolar(double rhoMin, double rhoMax, double thetaMin, double thetaMax,
      //                            std::default_random_engine* engine = nullptr);

      bool operator==(const Point& other) const;

    }; // class Point


  Point operator*(double ratio, const Point& p);

  std::ostream& operator<<(std::ostream& out, const Point& p);
  
  }; // namespace utils

  utils::Point average(const std::vector < utils::Point >& points);
  /* Return sum(points[i]* weights[i]) / sum(weights[i]) */
  utils::Point average(const std::vector< utils::Point >& points, const std::vector<double>& weights);
  /* Return the standard deviation to the average of the points */
  double stdDev(const std::vector< utils::Point >& points);

}  // namespace rhoban

/* Return the point which is the average of the points in the vector */
