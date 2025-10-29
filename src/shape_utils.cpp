#include "shape_utils.hpp"

namespace RayPh
{
Vec2  computeAabbHalf(const Shape& shape)
{
    switch (shape.type)
    {
        case circle:
            return { shape.circle.radius, shape.circle.radius };
        case box:
            return shape.box.halfExtents;
        default:
            return {0, 0}; // Fallback für spätere Formen
    }
}

}//namespace RayPh
