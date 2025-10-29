#include <iostream>

#include "raylib.h"
#include "ray_physics.hpp"



int main()
{
    const float WIDTH = 1000;
    const float HEIGHT = 1000;

    InitWindow(WIDTH, HEIGHT, "PhysicsEnginePro");

    RayPh::World world;
    RayPh::Body body;

    world.initScreenSize(WIDTH, HEIGHT);

    for(int i = 0; i < 50; i++)
    {
        world.addCircle ("TestBall1", {(100+i*15),(100+i*15)}, 10, 0, {255,0,0,255}, 400);
        world.addCircle ("TestBall2", {(60+i*15),(100+i*15)}, 10, 0, {0,255,0,255}, 400);
        world.addCircle ("TestBall3", {(20+i*15),(950+i*-15)}, 10, 0, {0,0,255,255}, 400);
        world.addBox    ("TestBox1", {(200+i*15),(300)}, 10, 10, 0);
        world.addBox    ("TestBox2", {(200),(300+i*15)}, 10, 10, 0);
        world.addBox    ("TestBox3", {(200+i*15),(300+i*15)}, 10, 10, 0);
    }
    while(!WindowShouldClose())
    {
        world.setDeltaTime(GetFrameTime());
        RayPh::step(world);

        BeginDrawing();
        ClearBackground(SKYBLUE);
        for(auto& bodys : world.bodies)
        {
            if(bodys.shape.type == circle)
            {

                DrawCircleV(bodys.pos, bodys.shape.circle.radius, bodys.color);
            }

            if(bodys.shape.type == box)
            {
                DrawRectangleV({bodys.pos.x - bodys.shape.box.halfExtents.x, bodys.pos.y - bodys.shape.box.halfExtents.y},
                               {bodys.shape.box.halfExtents.x * 2.0f,bodys.shape.box.halfExtents.y * 2.0f}, RED);
            }
        }
        DrawFPS(10,10);
        EndDrawing();
    }
    CloseWindow();

    return 0;
}
