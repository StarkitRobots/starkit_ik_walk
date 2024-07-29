#pragma once

#include <type_traits>

namespace rhoban
{
    namespace utils
    {
        template < typename Scalar > 
        std::enable_if_t < std::is_integral_v < Scalar >, Scalar > coerce(Scalar x, Scalar min, Scalar max)
        {
            if (min > max)
            {
                return (min + max) >> 1;
            }
            else if (x >= max)
            {
                return max;
            }
            else if (x <= min)
            {
                return min;
            }

            return x;
        }

        template < typename Scalar >
        std::enable_if_t < std::is_floating_point_v < Scalar >, Scalar > coerce(Scalar x, Scalar min, Scalar max)
        {
            if (min > max)
            {
                return (min + max) * 0.5;
            }
            else if (x >= max)
            {
                return max;
            }
            else if (x <= min)
            {
                return min;
            }

            return x;
        }

        template < typename Scalar >
        std::enable_if_t < std::is_arithmetic_v < Scalar >, Scalar > coerceAbs(Scalar x, Scalar maxAbs)
        {
            if(maxAbs < 0)
            {
                return 0;
            }
            else if (x >= maxAbs)
            {
                return maxAbs;
            }    
            else if (x <= -maxAbs) 
            {
                return -maxAbs;
            }
            
            return x;
        }

        template < typename Scalar >
        std::enable_if_t < std::is_arithmetic_v < Scalar >, Scalar > coerceMax(Scalar x, Scalar max)
        {
            return (x >= max ? max : x);
        }

        template < typename Scalar >
        std::enable_if_t < std::is_arithmetic_v < Scalar >, Scalar > coerceMin(Scalar x, Scalar min)
        {
            return (x <= min ? min : x);
        }

        template < typename Scalar >
        std::enable_if_t < std::is_integral_v < Scalar >, int > sign0(Scalar x)
        {
            if (x > 0)
            {
                return 1;
            }
            else if (x == 0)
            {
                return 0;
            } 
            
            return -1;
        }

        template < typename Scalar >
        std::enable_if_t < std::is_floating_point_v < Scalar >, int > sign0(Scalar x)
        {
            if (std::abs(x) < std::numeric_limits < Scalar > ::epsilon())
            {
                return 0;
            }
            else if (x > 0.0)
            {
                return 1;
            }

            return -1;
        }
    }

} // namespace rhoban_utils
