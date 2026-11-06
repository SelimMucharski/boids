#include <Eigen/Dense>

#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"

#include <vector>
#include <array>
#include <ranges>
#include <algorithm>
#include <numeric>

using vec = Eigen::Vector2d;
using mat = Eigen::Matrix2d;

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

	BirdData(vec p, vec s) : pos(p), velocity(s){}

    inline static const std::array<vec, 3> shape{
        vec{ 2.0 / 3.0,  0.0},
        vec{-1.0 / 3.0,  1.0 / 3.0},
        vec{-1.0 / 3.0, -1.0 / 3.0}
    };
    inline static double size = 10.0;

	void update(float dt){
		pos += velocity * dt;
	}
};

using Bird = std::shared_ptr<BirdData>;
using Birds = std::vector<Bird>;


constexpr double CLOSE = 40.0;
constexpr double FAR = 80.0;

constexpr double AVOID_factor = 0.05;
constexpr double MATCHING_factor = 0.005;
constexpr double CENTERING_factor = 0.0005;
constexpr double TURN_factor = 0.2;

// constexpr double AVOID_factor = 0.0;
// constexpr double MATCHING_factor = 0.0;
// constexpr double CENTERING_factor = 0.000;
// constexpr double TURN_factor = 0.0;

Birds findCloseNeighbours(const Bird& bird, const Birds& allBirds) {
    auto filtered = allBirds | std::views::filter([&](const auto& other) {
        return (bird->pos - other->pos).norm() < CLOSE;
    });

    Birds out;
    std::ranges::copy(filtered, std::back_inserter(out));
    return out;
}

Birds findFarNeighbours(const Bird& bird, const Birds& allBirds) {
    auto filtered = allBirds | std::views::filter([&](const auto& other) {
        double dist = (bird->pos - other->pos).norm();
        return dist >= CLOSE && dist < FAR;
    });

    Birds out;
    std::ranges::copy(filtered, std::back_inserter(out));
    return out;
}

class Boids : public olc::PixelGameEngine
{
private:
    Birds birds;
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

		auto heading = b->velocity.normalized();

        rot << heading, vec{-heading.y(), heading.x()};

        std::array<vec, 3> p;
        for (int i = 0; i < 3; ++i)
            p[i] = b->pos + BirdData::size * (rot * BirdData::shape[i]);

		// FillCircleW(b->pos, FAR, olc::CYAN);
		// FillCircleW(b->pos, CLOSE, olc::RED);

        FillTriangleW(p[0], p[1], p[2]);
    }

public:
    Boids() { sAppName = "Boids"; }

    bool OnUserCreate() override
    {
        world.origin = { ScreenWidth() / 2.0, ScreenHeight() / 2.0 };
        world.scale  = 1.0;

		birds.push_back(std::make_shared<BirdData>(vec{0.0, 0.0}, vec{1.0, 0.0}));
		birds.push_back(std::make_shared<BirdData>(vec{30.0, 20.0}, vec{1.0, 1.0}));
		birds.push_back(std::make_shared<BirdData>(vec{-40.0, -30.0}, vec{0.0, -1.0}));

		birds.push_back(std::make_shared<BirdData>(vec{100.0, 100.0}, vec{1.0, 1.0}));
		birds.push_back(std::make_shared<BirdData>(vec{105.0, 95.0},  vec{1.0, 1.0}));
		birds.push_back(std::make_shared<BirdData>(vec{95.0, 105.0},  vec{1.0, 1.0}));
		birds.push_back(std::make_shared<BirdData>(vec{110.0, 110.0}, vec{1.0, 1.0}));

		birds.push_back(std::make_shared<BirdData>(vec{-200.0, 0.0}, vec{1.0, 0.0}));
		birds.push_back(std::make_shared<BirdData>(vec{200.0, 0.0}, vec{-1.0, 0.0}));

		birds.push_back(std::make_shared<BirdData>(vec{300.0, -200.0}, vec{0.0, 1.0}));
		birds.push_back(std::make_shared<BirdData>(vec{-300.0, 200.0}, vec{-1.0, -1.0}));
		birds.push_back(std::make_shared<BirdData>(vec{400.0, 400.0},  vec{-1.0, 0.0}));

		birds.push_back(std::make_shared<BirdData>(vec{0.0, 35.0},   vec{-1.0, 0.0}));
		birds.push_back(std::make_shared<BirdData>(vec{0.0, -35.0},  vec{1.0, 0.0}));
		birds.push_back(std::make_shared<BirdData>(vec{35.0, 0.0},   vec{0.0, 1.0}));
		birds.push_back(std::make_shared<BirdData>(vec{-35.0, 0.0},  vec{0.0, -1.0}));


        return true;
    }

    bool OnUserUpdate(float fElapsedTime) override
    {
        Clear(olc::BLACK);
        drawAxes();

		for (auto& bird : birds){
			auto cls = findCloseNeighbours(bird, birds);
			auto far = findFarNeighbours(bird, birds);

			auto pos_accumulate = [](vec a, Bird b)
			{
				return std::move(a) + b->pos;
			};

			auto vel_accumulate = [](vec a, Bird b)
			{
				return std::move(a) + b->velocity;
			};

			if(!far.empty()){
				vec far_avg_pos = std::accumulate(far.begin(), far.end(), vec{0.0,0.0}, pos_accumulate) / far.size();
	
				bird->velocity += (far_avg_pos - bird->pos)*CENTERING_factor;
	
				vec far_avg_vel = std::accumulate(far.begin(), far.end(), vec{0.0,0.0}, vel_accumulate) / far.size();
	
				bird->velocity += (far_avg_vel - bird->velocity)*MATCHING_factor;
			}

			if(!cls.empty()){
				vec cls_avg_pos = std::accumulate(cls.begin(), cls.end(), vec{0.0,0.0}, pos_accumulate) / cls.size();
				
				bird->velocity += (bird->pos - cls_avg_pos)*AVOID_factor;
			}


			if(bird->pos.x() > 450) bird->velocity += vec{-1.0, 0.0} * TURN_factor;
			if(bird->pos.x() < -450) bird->velocity += vec{1.0, 0.0} * TURN_factor;
			if(bird->pos.y() > 500) bird->velocity += vec{0.0, -1.0} * TURN_factor;
			if(bird->pos.y() < -500) bird->velocity += vec{0.0, 1.0} * TURN_factor;
			
		}

        for (auto& bird : birds)
            bird->update(fElapsedTime);

        for (const auto& bird : birds)
            drawBird(bird);

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