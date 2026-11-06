#include <Eigen/Dense>

#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"

#include <vector>
#include <array>
#include <cmath>

using vec = Eigen::Vector2d;
using mat = Eigen::Matrix2d;

struct Frame {
    vec origin{0.0, 0.0};   // screen position of the world origin
    double scale = 1.0;     // screen pixels per world unit
    mat R = (mat() << 0, -1,
                     -1,  0).finished();

    olc::vi2d toScreen(const vec& w) const {
        vec s = origin + scale * (R * w);
        return { int(std::lround(s.x())), int(std::lround(s.y())) };
    }

    vec toWorld(const olc::vi2d& s) const {   // e.g. for the mouse
        vec p{double(s.x), double(s.y)};
        return R * ((p - origin) / scale);    // R is its own inverse
    }
};

struct Bird {
    vec pos{0.0, 0.0};
    vec heading{1.0, 0.0};

    inline static const std::array<vec, 3> shape{
        vec{ 2.0 / 3.0,  0.0},
        vec{-1.0 / 3.0,  1.0 / 3.0},
        vec{-1.0 / 3.0, -1.0 / 3.0}
    };
    inline static double size = 10.0;
};

class Boids : public olc::PixelGameEngine
{
private:
    std::vector<Bird> birds;
    Frame world;

    void DrawLineW(const vec& a, const vec& b, olc::Pixel c = olc::WHITE) {
        DrawLine(world.toScreen(a), world.toScreen(b), c);
    }
    void FillTriangleW(const vec& a, const vec& b, const vec& c, olc::Pixel col = olc::WHITE) {
        FillTriangle(world.toScreen(a), world.toScreen(b), world.toScreen(c), col);
    }
    void FillCircleW(const vec& c, double r, olc::Pixel col = olc::WHITE) {
        FillCircle(world.toScreen(c), int(r * world.scale), col);
    }

    void drawAxes(double len = 20.0) {
        DrawLineW({0.0, 0.0}, {len, 0.0}, olc::RED);    // X (up)
        DrawLineW({0.0, 0.0}, {0.0, len}, olc::GREEN);  // Y (left)
    }

    void drawBird(const Bird& b) {
        mat rot;
        rot << b.heading, vec{-b.heading.y(), b.heading.x()};

        std::array<vec, 3> p;
        for (int i = 0; i < 3; ++i)
            p[i] = b.pos + Bird::size * (rot * Bird::shape[i]);

        FillTriangleW(p[0], p[1], p[2]);
    }

public:
    Boids() { sAppName = "Boids"; }

    bool OnUserCreate() override
    {
        world.origin = { ScreenWidth() / 2.0, ScreenHeight() / 2.0 };
        world.scale  = 1.0;

        birds.push_back({ {0.0, 0.0},   {1.0, 0.0} });
        birds.push_back({ {30.0, 20.0}, vec{1.0, 1.0}.normalized() });
        birds.push_back({ {-40.0, -30.0}, {0.0, -1.0} });
        return true;
    }

    bool OnUserUpdate(float fElapsedTime) override
    {
        Clear(olc::BLACK);
        drawAxes();

        for (const auto& bird : birds)
            drawBird(bird);

        // Example: mouse position in world coordinates
        vec m = world.toWorld(GetMousePos());
        FillCircleW(m, 2.0, olc::YELLOW);

        return true;
    }
};

int main()
{
    Boids demo;
    if (demo.Construct(256*4, 240*4, 1, 1, false, true))
        demo.Start();
    return 0;
}