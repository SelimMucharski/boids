#pragma once

#include <filesystem>
#include <string>

struct Config {
    double CLOSE = 20.0;
    double FAR   = 60.0;

    double AVOID_factor     = 0.05;
    double MATCHING_factor  = 0.05;
    double CENTERING_factor = 0.0005;
    double TURN_factor      = 0.2;

    double MAX_SPEED = 100.0;
    double MIN_SPEED = 1.0;

    double BOUND_X = 450.0;
    double BOUND_Y = 500.0;

    double DT = 1.0 / 60.0;
    int MAX_STEPS_PER_FRAME = 5;

    int NUM_BIRDS = 0;

    double MOUSE_RADIUS   = 150.0;
    double ATTRACT_factor = 2.0;
    double REPEL_factor   = 4.0;
    int    SPAWN_COUNT    = 20;
    double ZOOM_STEP      = 1.1;
    double MIN_ZOOM       = 0.2;
    double MAX_ZOOM       = 5.0;

    double CLOSE2 = 0, FAR2 = 0, MAX_SPEED2 = 0, MIN_SPEED2 = 0, MOUSE_RADIUS2 = 0;

    void finalize();
};

inline Config cfg;

class ConfigFile {
public:
    explicit ConfigFile(std::filesystem::path path) : path_(std::move(path)) {}

    bool load();

    bool reloadIfChanged();

    const std::filesystem::path& path() const { return path_; }

private:
    std::filesystem::path path_;
    std::filesystem::file_time_type lastWrite_{};
};
