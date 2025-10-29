#pragma once
#include "shape.hpp"

namespace RayPh
{
// Gibt die AABB-Halbgröße zurück, passend zur Form
Vec2 computeAabbHalf(const Shape& shape);

} //namespace RayPh
