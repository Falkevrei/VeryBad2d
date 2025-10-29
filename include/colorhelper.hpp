#pragma once
#include "raylib.h"

struct ColorHelper
{
    int r = 0;
    int g = 0;
    int b = 0;
    int a = 255;

    // → Konvertierungsoperator für raylib Color
    operator Color() const {
        return { (unsigned char)r, (unsigned char)g, (unsigned char)b, (unsigned char)a };
    }
};
