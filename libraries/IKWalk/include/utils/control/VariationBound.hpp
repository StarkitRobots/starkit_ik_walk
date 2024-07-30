#pragma once

#include <type_traits>

#include "utils/math_functions.hpp"

namespace rhoban
{
    namespace utils
    {
        template < typename Scalar, typename Enable = std::enable_if_t < std::is_floating_point_v < Scalar > > >
        class VariationBound
        {
        public:
            static inline constexpr auto update(Scalar& output, const Scalar target, const Scalar limit, const Scalar dt)
            {
                output += coerce(target - output, -limit * dt, limit * dt);
            }

        }; // class VariationBound

    } // namespace utils

} // namespace rhoban
