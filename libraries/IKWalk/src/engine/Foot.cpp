#include "Foot.hpp"

namespace rhoban
{
    void Foot::clearSplines()
    {
        xSpline.clear();
        ySpline.clear();
        zSpline.clear();

        yawSpline.clear();
    }
    
} // namespace rhoban