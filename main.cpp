#include <Eigen/Dense>

#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"

#include <vector>
#include <array>
#include <algorithm>
#include <numbers>
#include <string>

using vec = Eigen::Vector2d;
using mat = Eigen::Matrix2d;

namespace cfg {
    constexpr double CLOSE  = 20.0;
    constexpr double FAR    = 60.0;
    constexpr double CLOSE2 = CLOSE * CLOSE;
    constexpr double FAR2   = FAR * FAR;

    constexpr double AVOID_factor     = 0.05;
    constexpr double MATCHING_factor  = 0.05;
    constexpr double CENTERING_factor = 0.0005;
    constexpr double TURN_factor      = 0.2;

    constexpr double MAX_SPEED  = 100.0;
    constexpr double MIN_SPEED  = 1.0;
    constexpr double MAX_SPEED2 = MAX_SPEED * MAX_SPEED;
    constexpr double MIN_SPEED2 = MIN_SPEED * MIN_SPEED;

    constexpr double BOUND_X = 450.0;
    constexpr double BOUND_Y = 500.0;

    constexpr double DT = 1.0 / 60.0;
    constexpr int MAX_STEPS_PER_FRAME = 5;

    constexpr int NUM_BIRDS = 500;
}

struct Frame {
    vec origin{0.0, 0.0};
    double scale = 1.0;
    mat R = (mat() << 0, -1,
                     -1,  0).finished();

    olc::vi2d toScreen(const vec& w) const {
        vec s = origin + scale * (R * w);
        return { int(std::lround(s.x())), int(std::lround(s.y())) };
    }

    vec toWorld(const olc::vi2d& s) const {
        vec p{double(s.x), double(s.y)};
        return R * ((p - origin) / scale);
    }
};

struct BirdData {
    vec pos{0.0, 0.0};
    vec velocity{1.0, 0.0};

    inline static const std::array<vec, 3> shape{
        vec{ 2.0 / 3.0,  0.0},
        vec{-1.0 / 3.0,  1.0 / 3.0},
        vec{-1.0 / 3.0, -1.0 / 3.0}
    };
    inline static constexpr double size = 10.0;
};

class SpatialGrid {
public:
    explicit SpatialGrid(double cellSize) : cell(cellSize), invCell(1.0 / cellSize) {}

    void build(const std::vector<BirdData>& birds) {
        if (birds.empty()) { cols = rows = 0; cellStart.assign(1, 0); return; }

        vec lo = birds.front().pos, hi = lo;
        for (const auto& b : birds) {
            lo = lo.cwiseMin(b.pos);
            hi = hi.cwiseMax(b.pos);
        }
        origin = lo;
        cols = std::min(MAX_DIM, int((hi.x() - lo.x()) * invCell) + 1);
        rows = std::min(MAX_DIM, int((hi.y() - lo.y()) * invCell) + 1);

        const size_t nCells = size_t(cols) * size_t(rows);
        cellStart.assign(nCells + 1, 0);
        birdCell.resize(birds.size());
        sorted.resize(birds.size());

        for (size_t i = 0; i < birds.size(); ++i) {
            int c = cellIndex(cellX(birds[i].pos.x()), cellY(birds[i].pos.y()));
            birdCell[i] = c;
            ++cellStart[c + 1];
        }

		for (size_t c = 0; c < nCells; ++c)
            cellStart[c + 1] += cellStart[c];

		cursor.assign(cellStart.begin(), cellStart.end() - 1);

		for (size_t i = 0; i < birds.size(); ++i)
            sorted[cursor[birdCell[i]]++] = int(i);
    }

    template <class F>
    void forEachCandidate(const vec& p, F&& f) const {
        if (cols == 0) return;
        const int cx = cellX(p.x());
        const int cy = cellY(p.y());
        for (int y = std::max(0, cy - 1); y <= std::min(rows - 1, cy + 1); ++y)
            for (int x = std::max(0, cx - 1); x <= std::min(cols - 1, cx + 1); ++x) {
                const int c = cellIndex(x, y);
                for (int k = cellStart[c]; k < cellStart[c + 1]; ++k)
                    f(sorted[k]);
            }
    }

private:
    static constexpr int MAX_DIM = 512;

    double cell, invCell;
    vec origin{0.0, 0.0};
    int cols = 0, rows = 0;

    std::vector<int> cellStart, cursor, birdCell, sorted;

    int cellX(double x) const { return std::clamp(int((x - origin.x()) * invCell), 0, cols - 1); }
    int cellY(double y) const { return std::clamp(int((y - origin.y()) * invCell), 0, rows - 1); }
    int cellIndex(int x, int y) const { return y * cols + x; }
};

class Boids : public olc::PixelGameEngine
{
private:
    std::vector<BirdData> birds;
    std::vector<vec> newVel;          // podwójny bufor prędkości
    SpatialGrid grid{cfg::FAR};
    Frame world;

    double accumulator = 0.0;
    bool debugRadii = false;
    bool paused = false;

    std::mt19937 gen{std::random_device{}()};

    void DrawLineW(const vec& a, const vec& b, olc::Pixel c = olc::WHITE) {
        DrawLine(world.toScreen(a), world.toScreen(b), c);
    }
    void FillTriangleW(const vec& a, const vec& b, const vec& c, olc::Pixel col = olc::WHITE) {
        FillTriangle(world.toScreen(a), world.toScreen(b), world.toScreen(c), col);
    }
    void DrawCircleW(const vec& c, double r, olc::Pixel col = olc::WHITE) {
        DrawCircle(world.toScreen(c), int(r * world.scale), col);
    }

    void drawAxes(double len = 20.0) {
        DrawLineW({0.0, 0.0}, {len, 0.0}, olc::RED);    // X (góra)
        DrawLineW({0.0, 0.0}, {0.0, len}, olc::GREEN);  // Y (lewo)
    }

    void drawBird(const BirdData& b) {
        const double s2 = b.velocity.squaredNorm();
        const vec heading = s2 > 1e-12 ? vec(b.velocity / std::sqrt(s2)) : vec{1.0, 0.0};

        mat rot;
        rot << heading, vec{-heading.y(), heading.x()};

        std::array<vec, 3> p;
        for (int i = 0; i < 3; ++i)
            p[i] = b.pos + BirdData::size * (rot * BirdData::shape[i]);

        FillTriangleW(p[0], p[1], p[2]);
    }

    vec computeVelocity(int i) const {
        const BirdData& b = birds[i];

        vec closePos = vec::Zero();
        vec farPos   = vec::Zero();
        vec farVel   = vec::Zero();
        int nClose = 0, nFar = 0;

        grid.forEachCandidate(b.pos, [&](int j) {
            if (j == i) return;                       // ptak nie jest swoim sąsiadem
            const BirdData& o = birds[j];
            const double d2 = (o.pos - b.pos).squaredNorm();
            if (d2 < cfg::CLOSE2) {
                closePos += o.pos;
                ++nClose;
            } else if (d2 < cfg::FAR2) {
                farPos += o.pos;
                farVel += o.velocity;
                ++nFar;
            }
        });

        vec v = b.velocity;

        if (nFar > 0) {
            const double inv = 1.0 / nFar;
            v += (farPos * inv - b.pos) * cfg::CENTERING_factor;
            v += (farVel * inv - b.velocity) * cfg::MATCHING_factor;
        }
        if (nClose > 0) {
            v += (b.pos - closePos / nClose) * cfg::AVOID_factor;
        }

        if (b.pos.x() >  cfg::BOUND_X) v.x() -= cfg::TURN_factor;
        if (b.pos.x() < -cfg::BOUND_X) v.x() += cfg::TURN_factor;
        if (b.pos.y() >  cfg::BOUND_Y) v.y() -= cfg::TURN_factor;
        if (b.pos.y() < -cfg::BOUND_Y) v.y() += cfg::TURN_factor;

        const double s2 = v.squaredNorm();
        if (s2 < 1e-12) {
            const double p2 = b.velocity.squaredNorm();
            v = p2 > 1e-12 ? vec(b.velocity / std::sqrt(p2) * cfg::MIN_SPEED)
                           : vec{cfg::MIN_SPEED, 0.0};
        } else if (s2 < cfg::MIN_SPEED2) {
            v *= cfg::MIN_SPEED / std::sqrt(s2);
        } else if (s2 > cfg::MAX_SPEED2) {
            v *= cfg::MAX_SPEED / std::sqrt(s2);
        }
        return v;
    }

    void step() {
        grid.build(birds);

        const int n = int(birds.size());
        newVel.resize(n);

        #pragma omp parallel for schedule(static)
        for (int i = 0; i < n; ++i)
            newVel[i] = computeVelocity(i);

        for (int i = 0; i < n; ++i) {
            birds[i].velocity = newVel[i];
            birds[i].pos += newVel[i] * cfg::DT;
        }
    }

    void spawnBirds(int count) {
        std::uniform_real_distribution<double> pos_dist(-400.0, 400.0);
        std::uniform_real_distribution<double> speed_dist(20.0, cfg::MAX_SPEED);
        std::uniform_real_distribution<double> angle_dist(0.0, 2.0 * std::numbers::pi);

        birds.reserve(birds.size() + count);
        for (int i = 0; i < count; ++i) {
            const double angle = angle_dist(gen);
            const double speed = speed_dist(gen);
            birds.push_back({
                vec{pos_dist(gen), pos_dist(gen)},
                vec{speed * std::cos(angle), speed * std::sin(angle)}
            });
        }
    }

public:
    Boids() { sAppName = "Boids"; }

    bool OnUserCreate() override
    {
        world.origin = { ScreenWidth() / 2.0, ScreenHeight() / 2.0 };
        world.scale  = 1.0;
        spawnBirds(cfg::NUM_BIRDS);
        return true;
    }

    bool OnUserUpdate(float fElapsedTime) override
    {
        if (GetKey(olc::Key::D).bPressed)     debugRadii = !debugRadii;
        if (GetKey(olc::Key::SPACE).bPressed) paused = !paused;
        if (GetKey(olc::Key::A).bPressed)     spawnBirds(100);
        if (GetKey(olc::Key::R).bPressed)     { birds.clear(); spawnBirds(cfg::NUM_BIRDS); }

        if (!paused) {
            accumulator += fElapsedTime;
            int steps = 0;
            while (accumulator >= cfg::DT && steps < cfg::MAX_STEPS_PER_FRAME) {
                step();
                accumulator -= cfg::DT;
                ++steps;
            }
            if (steps == cfg::MAX_STEPS_PER_FRAME)
                accumulator = 0.0;
        }

        Clear(olc::BLACK);
        drawAxes();

        if (debugRadii) {
            for (const auto& b : birds) {
                DrawCircleW(b.pos, cfg::FAR,   olc::Pixel(0, 120, 120));
                DrawCircleW(b.pos, cfg::CLOSE, olc::Pixel(140, 0, 0));
            }
        }

        for (const auto& b : birds)
            drawBird(b);

        DrawString(4, 4, "Ptaki: " + std::to_string(birds.size()) +
                         "   [D] promienie  [A] +100  [R] reset  [Spacja] pauza",
                   olc::GREY);
        return true;
    }
};

int main()
{
    Boids demo;
    if (demo.Construct(256 * 4, 240 * 4, 1, 1, false, true))
        demo.Start();
    return 0;
}