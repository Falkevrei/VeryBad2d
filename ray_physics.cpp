#include "ray_physics.hpp"
#include "shape_utils.hpp"

#include <iostream>
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace RayPh
{
    // initializer World
    void World::initScreenSize(float winWidth, float winHeight)
    {
        WindowWIDTH     = winWidth;
        WindowHEIGHT    = winHeight;
        ground_y        = winHeight;
        top_y           = 0;
        leftWall        = 0;
        rightWall       = winWidth;

    }

    void World::setDeltaTime(float dt)
    {
        frame_dt = dt;
        if (frame_dt > clamp_dt) frame_dt = clamp_dt;
    }

    void World::setWallBoundaries(int ground, int top, int left, int right)
    {
        ground_y    = ground;
        top_y       = top;
        leftWall    = left;
        rightWall   = right;
    }
    void World::buildSoVBuffer()
    {
        if(soaReady) return;
        size_t N = bodies.size();

        positionsX.clear();     positionsX.resize(N);
        positionsY.clear();     positionsY.resize(N);
        velocitiesX.clear();    velocitiesX.resize(N);
        velocitiesY.clear();    velocitiesY.resize(N);
        radii.clear();          radii.resize(N);
        halfExtentX.clear();    halfExtentX.resize(N);
        halfExtentY.clear();    halfExtentY.resize(N);
        masses.clear();         masses.resize(N);
        materials.clear();      materials.resize(N);
        gridHead.clear();
        gridNext.clear();       gridNext.resize(N);
        sleepTimer.clear();     sleepTimer.resize(N);

        for (size_t i = 0; i < N; ++i)
        {
            Body& body = bodies[i];
            body.soaIndex = i;

            positionsX[i]  = body.pos.x;
            positionsY[i]  = body.pos.y;
            velocitiesX[i] = body.vel.x;
            velocitiesY[i] = body.vel.y;
            masses[i]      = body.mass;

            if (body.shape.type == ShapeType::circle)
            {
                radii[i]       = body.shape.circle.radius;
                halfExtentX[i] = radii[i];
                halfExtentY[i] = radii[i];
            }
            else if (body.shape.type == ShapeType::box)
            {
                halfExtentX[i] = body.shape.box.halfExtents.x;
                halfExtentY[i] = body.shape.box.halfExtents.y;
                radii[i]       = std::max(halfExtentX[i], halfExtentY[i]);
            }

            materials[i]  = 0;
            gridNext[i]   = -1;
            sleepTimer[i] = 0.0f;
        }
        soaReady = true;
    }

    void World::computeGridParams()
    {
        // Voraussetzung: SoV gebaut, Walls gesetzt
        const size_t N = bodies.size();

        // Max-Extent aus SoV-Vektoren bestimmen (falls leer → 0)
        auto max_of_vec = [](const std::vector<float>& v) -> float
        {
            return v.empty() ? 0.0f : *std::max_element(v.begin(), v.end());
        };
        const float maxR        = max_of_vec(radii);
        const float maxHX       = max_of_vec(halfExtentX);
        const float maxHY       = max_of_vec(halfExtentY);
        const float maxExtent   = std::max({maxR, maxHX, maxHY});

        // Zellgröße (mindestens minCellSize, sinnvoller Fallback bei 0-Extent)
        const float minCellSize = 1.0f;
        cellSize = (maxExtent > 0.0f)
                 ? std::max(2.0f * maxExtent, minCellSize)
                 : 32.0f; // Fallback, wenn noch keine Größen gesetzt

        invCellSize = (cellSize > 0.0f) ? 1.0f / cellSize : 0.0f;

        // Gitterdimensionen (aufrunden) – mindestens 1
        const float worldW = static_cast<float>(rightWall - leftWall);
        const float worldH = static_cast<float>(ground_y  - top_y);

        gridCols = std::max<size_t>(1, static_cast<size_t>(std::ceil(worldW / cellSize)));
        gridRows = std::max<size_t>(1, static_cast<size_t>(std::ceil(worldH / cellSize)));
        numCells = gridCols * gridRows;

        // Speicher vorbereiten
        gridHead.assign(numCells, -1);
        gridNext.assign(N, -1);
        gridParamsReady = true;
    }

    void World::buildGrid()
    {
        // Voraussetzungen
        if (!soaReady || numCells == 0) return;

        const size_t N = positionsX.size(); // = Anzahl Bodies

        // 1) Reset
        std::fill(gridHead.begin(), gridHead.end(), -1);
        if (gridNext.size() != N) gridNext.assign(N, -1);
        else                      std::fill(gridNext.begin(), gridNext.end(), -1);

        // 2) Einsortieren
        for (size_t i = 0; i < N; ++i)
        {
            // Zellkoordinaten
            int cx = static_cast<int>(std::floor((positionsX[i] - leftWall) * invCellSize));
            int cy = static_cast<int>(std::floor((positionsY[i] - top_y)    * invCellSize));

            // Clamp in gültigen Bereich
            if (cx < 0) cx = 0; else if (cx >= static_cast<int>(gridCols)) cx = static_cast<int>(gridCols) - 1;
            if (cy < 0) cy = 0; else if (cy >= static_cast<int>(gridRows)) cy = static_cast<int>(gridRows) - 1;

            // Zellindex
            const int cellId = cy * static_cast<int>(gridCols) + cx;

            // Verkettete Liste: Body i oben in den „Eimer“ legen
            gridNext[i]           = gridHead[cellId]; // wer bisher oben lag, rutscht „unter“ i
            gridHead[cellId]      = static_cast<int>(i);
        }
    }

    void World::buildBroadphasePairs()
    {
        if (!soaReady || numCells == 0) { pairA.clear(); pairB.clear(); return; }

        const size_t N = positionsX.size();
        pairA.clear(); pairB.clear();
        pairA.reserve(N); pairB.reserve(N);

        auto cellIdOf = [this](int cx, int cy) -> int {
            return cy * static_cast<int>(gridCols) + cx;
        };

        // Nur „halbe“ Nachbarschaft → keine Duplikate
        const int OFF[5][2] = { {1,-1}, {1,0}, {1,1}, {0,1}, {-1,1} };

        for (int cy = 0; cy < static_cast<int>(gridRows); ++cy)
        {
            for (int cx = 0; cx < static_cast<int>(gridCols); ++cx)
            {
                const int cid = cellIdOf(cx, cy);

                // --- Intra-Cell: i gegen spätere j (keine Duplikate) ---
                for (int i = gridHead[cid]; i != -1; i = gridNext[i])
                {
                    for (int j = gridNext[i]; j != -1; j = gridNext[j])
                    {
                        const float dx = std::abs(positionsX[i] - positionsX[j]);
                        const float dy = std::abs(positionsY[i] - positionsY[j]);
                        if (dx <= (halfExtentX[i] + halfExtentX[j]) &&
                            dy <= (halfExtentY[i] + halfExtentY[j]))
                        {
                            pairA.push_back(i);
                            pairB.push_back(j);
                        }
                    }
                }

                // --- Nachbar-Cells: i (cid) gegen alle j (nid), mit AABB-Filter ---
                for (auto &d : OFF)
                {
                    const int nx = cx + d[0];
                    const int ny = cy + d[1];
                    if (nx < 0 || ny < 0 ||
                        nx >= static_cast<int>(gridCols) ||
                        ny >= static_cast<int>(gridRows)) continue;

                    const int nid = cellIdOf(nx, ny);

                    for (int i = gridHead[cid]; i != -1; i = gridNext[i])
                    {
                        for (int j = gridHead[nid]; j != -1; j = gridNext[j])
                        {
                            const float dx = std::abs(positionsX[i] - positionsX[j]);
                            const float dy = std::abs(positionsY[i] - positionsY[j]);
                            if (dx <= (halfExtentX[i] + halfExtentX[j]) &&
                                dy <= (halfExtentY[i] + halfExtentY[j]))
                            {
                                // i<j sicherstellen (optional, meist schon gegeben)
                                if (i < j) { pairA.push_back(i); pairB.push_back(j); }
                                else       { pairA.push_back(j); pairB.push_back(i); }
                            }
                        }
                    }
                }
            }
        }
    }



    // Debug Test function will be deleted
    void World::testGridNeighborhood()
    {
        if (!soaReady || numCells == 0) return;

        const size_t N = positionsX.size();
        if (gridHead.size() != numCells || gridNext.size() != N) return;

        std::vector<int> counts(numCells, 0);

        auto cellIdOf = [this](int cx, int cy) -> int {
            return cy * static_cast<int>(gridCols) + cx;
        };

        for (int y = 0; y < static_cast<int>(gridRows); ++y)
        {
            for (int x = 0; x < static_cast<int>(gridCols); ++x)
            {
                const int cid = cellIdOf(x, y);

                const int minY = std::max(0, y - 1);
                const int maxY = std::min(static_cast<int>(gridRows) - 1, y + 1);
                const int minX = std::max(0, x - 1);
                const int maxX = std::min(static_cast<int>(gridCols) - 1, x + 1);

                for (int ny = minY; ny <= maxY; ++ny)
                {
                    for (int nx = minX; nx <= maxX; ++nx)
                    {
                        const int nid = cellIdOf(nx, ny);
                        for (int i = gridHead[nid]; i != -1; i = gridNext[i])
                            counts[cid]++; // nur zählen
                    }
                }
            }
        }

        int sum = 0, maxc = 0;
        for (int c : counts) { sum += c; if (c > maxc) maxc = c; }

        std::printf("[Grid] neighborhood counts: sum=%d, max=%d, avg=%.2f (N=%zu, cells=%zu)\n",
                    sum, maxc,
                    counts.empty() ? 0.0 : static_cast<double>(sum) / counts.size(),
                    N, numCells);
    }

    size_t World::countPairsInCell(int cellId)
    {
        size_t pairs = 0;
        for (int i = gridHead[cellId]; i != -1; i = gridNext[i])
        {
            for (int j = gridNext[i]; j != -1; j = gridNext[j])
            {
                if (std::abs(positionsX[i]-positionsX[j]) <= (halfExtentX[i]+halfExtentX[j]) &&
                    std::abs(positionsY[i]-positionsY[j]) <= (halfExtentY[i]+halfExtentY[j])) { ++pairs; }

            }
        }
        return pairs;
    }

    void World::testPairsInCell()
    {
        if (!soaReady || numCells == 0) return;

        debugPairsInCellTotal = 0;
        debugPairsMaxPerCell = 0;

        for (size_t cid = 0; cid < numCells; ++cid)
        {
            size_t pairs = countPairsInCell(static_cast<int>(cid));
            debugPairsInCellTotal += pairs;
            if (pairs > debugPairsMaxPerCell)
                debugPairsMaxPerCell = pairs;
        }

        std::printf("[Grid] pairsInCell total=%zu, max=%zu\n",
                    debugPairsInCellTotal, debugPairsMaxPerCell);
    }

    // Create Bodys
    void World::addCircle(std::string bodyName, Vec2 bodyPos, float bodyRadius, float bodyMass, std::vector<int> color, float testVel_x)// testVel_x will ve removed or extendet later on.
    {
            Body body;

            body.shape.type = circle;
            body.name = bodyName;
            body.pos  = bodyPos;
            body.shape.circle.radius = bodyRadius;
            body.mass = bodyMass;
            body.color.r = color[0];
            body.color.g = color[1];
            body.color.b = color[2];
            body.color.a = color[3];
            body.vel.x = testVel_x; // test Only
            //body.id++;
            soaReady = false;
            gridParamsReady = false;
            bodies.push_back(body);

    }

    void World::addBox(std::string bodyName, Vec2 bodyPos, float width, float height, float bodyMass)
    {
            Body body;

            body.shape.type = box;
            body.name = bodyName;
            body.pos  = bodyPos;
            body.shape.box.halfExtents.x = width/2;
            body.shape.box.halfExtents.y = height/2;
            body.mass = bodyMass;
            //body.id++;
            soaReady = false;
            gridParamsReady = false;
            bodies.push_back(body);
    }

    // physics Functions
    void apply_gravity(World& world, Body& body, float dt)
    {
        size_t i = body.soaIndex;
        if(!world.use_gravity) return;
        world.velocitiesY[i] += world.gravity * dt;
    }

    void apply_air_drag(World& world, Body& body, float dt)
    {
        size_t i = body.soaIndex;
        if(!world.use_air_drag) return;
        // TODO

    }

    void integrate_motion(Body& body, World& world, float dt)
    {
        size_t i = body.soaIndex;
        world.positionsX[i] += world.velocitiesX[i] * dt;
        world.positionsY[i] += world.velocitiesY[i] * dt;
    }


    void solve_world_bounds(World& world, Body& body)
    {
        size_t i = body.soaIndex;
        if(!world.use_bounce_walls) return;

        if (world.positionsY[i] > world.ground_y - world.halfExtentY[i] && world.velocitiesY[i] > 0)
            {
                world.positionsY[i]  = world.ground_y - world.halfExtentY[i];          // exakt an die Kante
                world.velocitiesY[i] = -world.velocitiesY[i]* world.bouncePwrLoss;          // Richtung umkehren + Dämpfung
            }
            else if (world.positionsY[i] < world.top_y + world.halfExtentY[i] && world.velocitiesY[i] < 0)
            {
                world.positionsY[i]  = world.top_y + world.halfExtentY[i];
                world.velocitiesY[i] = -world.velocitiesY[i] * world.bouncePwrLoss;
            }
            if (world.positionsX[i] > world.rightWall - world.halfExtentX[i] && world.velocitiesX[i]> 0)
            {
                world.positionsX[i]  = world.rightWall - world.halfExtentX[i];
                world.velocitiesX[i] = -world.velocitiesX[i] * world.bouncePwrLoss;
            }
            else if (world.positionsX[i] < world.leftWall + world.halfExtentX[i] && world.velocitiesX[i]< 0)
            {
                world.positionsX[i]  = world.leftWall + world.halfExtentX[i];
                world.velocitiesX[i] = -world.velocitiesX[i] * world.bouncePwrLoss;
            }
        if (world.positionsY[i] < world.top_y) world.positionsY[i] = world.top_y;
    }
    //************************************************************************************************** belongs together till next star line*******************
// ===== helpers =====
    static inline bool nearZero(float v, float t)
    {
        // robust: nie mit t==0 „false“ erzwingen
        const float thr = std::max(t, 1e-3f);
        return std::abs(v) <= thr;
    }

    static inline bool isOnGround(const World& world, const Body& body)
    {
        // rein positionsbasiert → Reibung greift zuverlässig nach dem Boden-Resolve
        size_t i = body.soaIndex;
        const float eps = 1e-3f;
        return (world.positionsY[i] >= world.ground_y - world.halfExtentY[i] - eps);
    }


    // ===== friction & sleep =====
    void apply_ground_friction(World& world, Body& body)
    {
        if (!world.use_ground_friction) return;
        size_t i = body.soaIndex;
        if (!isOnGround(world, body)) return;

        // dt-skalierte Dämpfung (per-Sekunde Rate in [0..1]):
        // vX <- vX * (1 - k * dt)
        float k  = std::clamp(world.surfaceFriction, 0.0f, 1.0f); // interpretiert als „pro Sekunde“-Stärke
        float dt = world.physics_dt;
        world.velocitiesX[i] -= world.velocitiesX[i] * k * dt;

        // statische Reibung: kleine Geschwindigkeiten auf 0 snappen
        const float stopThreshold = 0.05f; // gern in World verschieben
        if (std::abs(world.velocitiesX[i]) < stopThreshold)
            world.velocitiesX[i] = 0.0f;
    }

    void sleep_if_slow(World& world, Body& body)
    {
        if (!world.use_sleep) return;
        size_t i = body.soaIndex;

        // „berührt Boden“: enge Pos-Toleranz
        const float eps = 1e-3f;
        const bool touchingGround =
            world.positionsY[i] >= world.ground_y - world.halfExtentY[i] - eps &&
            world.positionsY[i] <= world.ground_y - world.halfExtentY[i] + eps;

        if (!touchingGround) return;

        // vertikal zur Ruhe kommen lassen
        if (nearZero(world.velocitiesY[i], world.sleepMinSpeed_Y)) {
            world.velocitiesY[i] = 0.0f;
            world.positionsY[i]  = world.ground_y - world.halfExtentY[i];
        }

        // horizontal ggf. ausdämpfen (ergänzt „apply_ground_friction“)
        if (nearZero(world.velocitiesX[i], world.sleepMinSpeed_X)) {
            world.velocitiesX[i] = 0.0f;
        }
    }

    //******************************************************************************************************************************************************

    void solve_body_body_contacts(World& world)
    {
        if()
    }

    // --------------------------------------------------Combination of all in one "step()"--------------------------------------------------------
    void step(World& world)
    {
        if (!world.soaReady) world.buildSoVBuffer();

        world.subSteps = 0;
        world.accumulator += world.frame_dt;

        while(world.accumulator >= world.physics_dt && world.subSteps < world.maxSubSteps)
        {
            for (auto& body : world.bodies)
            {

                size_t i = body.soaIndex;
                body.aabbHalf = computeAabbHalf(body.shape);

                apply_gravity           (world, body, world.physics_dt);
                apply_air_drag          (world, body, world.physics_dt);
                integrate_motion        (body, world, world.physics_dt);

                solve_world_bounds      (world, body);
                apply_ground_friction   (world, body);
                sleep_if_slow           (world, body);


                body.pos.x = world.positionsX[i];
                body.pos.y = world.positionsY[i];
                body.vel.x = world.velocitiesX[i];
                body.vel.y = world.velocitiesY[i];
            }
            world.accumulator -= world.physics_dt;
            world.subSteps += 1;
            if (!world.gridParamsReady)
            {
                world.computeGridParams();
            }
            world.buildGrid();
            world.buildBroadphasePairs();

            // bodies → Zellen einsortieren
            if (world.debugNeighbor) world.testGridNeighborhood(); // nur Debug-Ausgabe (B2.1-Test)
            if (world.debugPairs) world.testPairsInCell();


        }

        solve_body_body_contacts(world);
    }

}
