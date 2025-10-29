#pragma once

#include "shape.hpp"
#include "vec2.h"
#include "colorhelper.hpp"

#include <vector>
#include <string>
#include <stdint.h>

namespace RayPh
{


    class Body
    {
        public:
        Shape shape{};
        ColorHelper color{0,0,0,0};

        size_t id           = 0;
        std::string name    = "";
        Vec2 pos            = {0,0};
        Vec2 vel            = {0,0};
        float   mass        = 0.0f;
        Vec2 aabbHalf       = {0,0};
        uint8_t material    = 0;
        size_t soaIndex     = 0;

        Body()
        {
            // Default-Form: ein (noch leerer) Kreis
            shape.type = ShapeType::circle;
            shape.circle.radius = 0.0f;
            shape.box.halfExtents = {0.0f, 0.0f};
        }
    };

    class World
    {
        public:
        // World Framework
        int   WindowWIDTH       = 0;
        int   WindowHEIGHT      = 0;
        int   ground_y          = WindowHEIGHT;
        int   top_y             = 0;
        int   rightWall         = WindowWIDTH;
        int   leftWall          = 0;

        // Framework World timing for physics
        const float   physics_dt    = 1.0f / 120.0f;
        const float   clamp_dt      = 1.0f / 15.0f;
        float         accumulator   = 0.0f;
        size_t        subSteps      = 0;
        const float   maxSubSteps   = 8.0f;
        float         frame_dt      = 0.0f;


        //World Physical rules , Setter TODO extra functions
        float gravity         = 98.1f;
        float friction        = 0.0f;
        float bouncePwrLoss   = 0.8f;
        float surfaceFriction = 0.95f;
        float sleepMinSpeed_X = 0.05f;   // Schwellwert Horizontal
        float sleepMinSpeed_Y = 0.05f;   // Schwellwert Vertikal

        // creation Vector
        std::vector<Body> bodies;

        //useage bool, to ex and include parts.
        bool use_gravity            = true;
        bool use_bounce_walls       = true;
        bool use_ground_friction    = true;
        bool use_sleep              = true;
        bool use_air_drag           = true;

        //SoA(V) bools
        bool    soaReady = false;

        // SoA(Vector) - Vectoren (ein Eintrag pro Body)
        std::vector<float>      positionsX;
        std::vector<float>      positionsY;
        std::vector<float>      velocitiesX;
        std::vector<float>      velocitiesY;
        std::vector<float>      radii;
        std::vector<float>      halfExtentX;
        std::vector<float>      halfExtentY;
        std::vector<float>      masses;
        std::vector<uint8_t>    materials;


        //Uniform Grid Data structure

        float cellSize      = 0.0f;
        float invCellSize   = 0.0f;
        size_t gridCols     = 0;
        size_t gridRows     = 0;
        size_t numCells     = 0;
        bool gridParamsReady = false;

        //Collision data
        std::vector<int> pairA; // Index i
        std::vector<int> pairB; // Index j (immer i < j)

        //Grid Debug
        bool   debugPairs = false;
        bool   debugNeighbor = false;
        size_t debugPairsInCellTotal = 0;
        size_t debugPairsMaxPerCell = 0;

        // Uniform Grid for collisions and performant callculation
        std::vector<int> gridHead;
        std::vector<int> gridNext;   // pro Body (Verkettung), bleibt leer

        // Optional (für Schlafzustand)
        std::vector<float> sleepTimer;

        // initializez physics engine with minimum requirements and setters
        void initScreenSize(float winWidth, float winHeight);
        void setDeltaTime(float dt);
        void setWallBoundaries(int ground, int top, int left, int right);
        // add forms/bodies
        void addCircle(std::string bodyName, Vec2 bodyPos, float bodyRadius, float bodyMass, std::vector<int> color, float testVel_x);
        void addBox(std::string bodyName, Vec2 bodyPos, float width, float height, float bodyMass);
        // or create setter functions for the rest and work with defaults in the ones above

        //SOA buffer function
        void buildSoVBuffer();

        //Grid Functions

        void computeGridParams();
        void buildGrid();

        // Collision functions
        void buildBroadphasePairs();

        //Grid Debug
        void    testGridNeighborhood();
        size_t  countPairsInCell(int cellId);
        void    testPairsInCell();

        //more to come

    };
    //Single funktions for better controll in combination with world bools.
    void apply_gravity              (World& world, Body& body, float dt);
    void apply_air_drag             (World& world, Body& body, float dt);
    void integrate_motion           (Body& body, World& world, float dt);
    void solve_world_bounds         (World& world, Body& body);
    void apply_ground_friction      (World& world, Body& body);
    void sleep_if_slow              (World& world, Body& body);
    void solve_body_body_contacts   (World& world);
    //Kombines all
    void step(World& world);
    void test();
}// namespace RayPh



