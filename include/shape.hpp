#pragma once
#include "vec2.h"


enum ShapeType {circle, box};

struct Shape
{

    ShapeType type;
    struct {float radius;} circle;
    struct {Vec2 halfExtents;} box;

};



