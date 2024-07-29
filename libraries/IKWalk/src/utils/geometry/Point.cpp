#include "geometry/Point.hpp"

#define EPSILON 0.000001

namespace rhoban
{
  namespace utils
  {
    double Point::getDist(const Point& other) const
    {
      auto dx = getX() - other.getX();
      auto dy = getY() - other.getY();

      return std::sqrt(dx * dx + dy * dy);
    }

    Point& Point::normalize(double newLength)
    {
      auto length = getLength();
      if (std::abs(newLength) < std::numeric_limits < double > ::epsilon() 
        || std::abs(length) < std::numeric_limits < double > ::epsilon())
      {
        x = 0;
        y = 0;
      }
      else
      {
        auto denominator = length / newLength;
        x /= denominator;
        y /= denominator;
      }

      return *this;
    }

    bool Point::isCollinear(const Point& other) const
    {
      auto deltaTheta = getTheta() - other.getTheta();
      auto delta = abs(deltaTheta.getSignedValue());

      return (delta < EPSILON || (180.0 - delta) < EPSILON);
    }

    Point Point::rotation(const Angle& a) const
    {
      auto newX = x * cos(a) - y * sin(a);
      auto newY = x * sin(a) + y * cos(a);

      return Point(newX, newY);
    }

    Point Point::rotation(const Angle& a, const Point& rotCenter) const
    {
      auto diff = *this - rotCenter;
      return rotCenter + diff.rotation(a);
    }

    Point Point::operator-() const
    {
      return Point(-getX(), -getY());
    }

    double Point::dotProduct(const Point& p1, const Point& p2)
    {
      return p1.getX() * p2.getX() + p1.getY() * p2.getY();
    }

    double Point::perpDotProduct(const Point& p1, const Point& p2)
    {
      return p1.getX() * p2.getY() - p1.getY() * p2.getX();
    }

    Point Point::operator+(const Point& other) const
    {
      return Point(getX() + other.getX(), getY() + other.getY());
    }

    Point Point::operator-(const Point& other) const
    {
      return Point(getX() - other.getX(), getY() - other.getY());
    }

    Point Point::operator*(double ratio) const
    {
      return Point(getX() * ratio, getY() * ratio);
    }

    Point Point::operator/(double ratio) const
    {
      return Point(getX() / ratio, getY() / ratio);
    }

    Point& Point::operator+=(const Point& other)
    {
      *this = *this + other;
      return *this;
    }

    Point& Point::operator-=(const Point& other)
    {
      *this = *this - other;
      return *this;
    }

    Point& Point::operator*=(double ratio)
    {
      *this = *this * ratio;
      return *this;
    }
    Point& Point::operator/=(double ratio)
    {
      *this = *this / ratio;
      return *this;
    }

    bool Point::operator==(const Point& other) const
    {
      bool sameX = getX() == other.getX();
      bool sameY = getY() == other.getY();
      return sameX && sameY;
    }

    Point Point::mkPointFromPolar(double rho, Angle a)
    {
      double theta = a.getValue() * M_PI / 180.0;
      return Point(cos(theta) * rho, sin(theta) * rho);
    }

    // TODO: refactoring
    // Point Point::mkRandomPolar(double rhoMax, std::default_random_engine* engine)
    // {
    //   return mkRandomPolar(0.0, rhoMax, engine);
    // }

    // Point Point::mkRandomPolar(double rhoMin, double rhoMax, std::default_random_engine* engine)
    // {
    //   return mkRandomPolar(rhoMin, rhoMax, 0.0, 360.0, engine);
    // }

    // Point Point::mkRandomPolar(double rhoMin, double rhoMax, double thetaMin, double thetaMax,
    //                            std::default_random_engine* engine)
    // {
    //   bool generate_engine = engine == nullptr;
    //   if (generate_engine)
    //     engine = rhoban_random::newRandomEngine();
    //   std::uniform_real_distribution<double> rhoDist(rhoMin, rhoMax);
    //   std::uniform_real_distribution<double> thetaDist(thetaMin, thetaMax);
    //   double rho = rhoDist(*engine);
    //   double theta = thetaDist(*engine);
    //   if (generate_engine)
    //     delete (engine);
    //   return mkPointFromPolar(rho, theta);
    // }

      Point operator*(double ratio, const Point& p)
      {
        return p * ratio;
      }

      std::ostream& operator<<(std::ostream& out, const Point& p)
      {
        return out << '{' << p.getX() << ',' << p.getY() << '}';
      }

  } // namespace utils


  utils::Point average(const std::vector< utils::Point >& points)
  {
    double xSum(0), ySum(0);
    int n = points.size();
    for (int i = 0; i < n; i++)
    {
      xSum += points[i].getX();
      ySum += points[i].getY();
    }
    return utils::Point(xSum / n, ySum / n);
  }

  utils::Point average(const std::vector< utils::Point >& points, const std::vector< double >& weights)
  {
    double xSum(0), ySum(0);
    double totalWeight = 0;
    for (unsigned int i = 0; i < points.size(); i++)
    {
      xSum += points[i].getX() * weights[i];
      ySum += points[i].getY() * weights[i];

      totalWeight += weights[i];
    }
    return utils::Point(xSum, ySum) / totalWeight;
  }

  double stdDev(const std::vector< utils::Point >& points)
  {
    utils::Point middle = average(points);
    double squareDist(0);
    int n = points.size();
    for (int i = 0; i < n; i++)
    {
      double dist = middle.getDist(points[i]);
      squareDist += dist * dist;
    }
    return std::sqrt(squareDist / n);
  }
}  // namespace rhoban

