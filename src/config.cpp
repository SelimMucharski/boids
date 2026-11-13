#include "config.hpp"

#include <charconv>
#include <fstream>
#include <iostream>
#include <string_view>
#include <unordered_map>
#include <variant>

namespace {

std::string_view trim(std::string_view s) {
    const auto b = s.find_first_not_of(" \t\r");
    if (b == std::string_view::npos) return {};
    const auto e = s.find_last_not_of(" \t\r");
    return s.substr(b, e - b + 1);
}

template <class T>
bool parseNumber(std::string_view s, T& out) {
    const char* end = s.data() + s.size();
    auto [ptr, ec] = std::from_chars(s.data(), end, out);
    return ec == std::errc{} && ptr == end;
}

using Field = std::variant<double Config::*, int Config::*>;

const std::unordered_map<std::string_view, Field>& fields() {
    static const std::unordered_map<std::string_view, Field> map{
        {"CLOSE", &Config::CLOSE},
        {"FAR", &Config::FAR},
        {"AVOID_factor", &Config::AVOID_factor},
        {"MATCHING_factor", &Config::MATCHING_factor},
        {"CENTERING_factor", &Config::CENTERING_factor},
        {"TURN_factor", &Config::TURN_factor},
        {"MAX_SPEED", &Config::MAX_SPEED},
        {"MIN_SPEED", &Config::MIN_SPEED},
        {"BOUND_X", &Config::BOUND_X},
        {"BOUND_Y", &Config::BOUND_Y},
        {"DT", &Config::DT},
        {"MAX_STEPS_PER_FRAME", &Config::MAX_STEPS_PER_FRAME},
        {"NUM_BIRDS", &Config::NUM_BIRDS},
        {"MOUSE_RADIUS", &Config::MOUSE_RADIUS},
        {"ATTRACT_factor", &Config::ATTRACT_factor},
        {"REPEL_factor", &Config::REPEL_factor},
        {"SPAWN_COUNT", &Config::SPAWN_COUNT},
        {"ZOOM_STEP", &Config::ZOOM_STEP},
        {"MIN_ZOOM", &Config::MIN_ZOOM},
        {"MAX_ZOOM", &Config::MAX_ZOOM},
    };
    return map;
}

} // namespace

void Config::finalize() {
    CLOSE2        = CLOSE * CLOSE;
    FAR2          = FAR * FAR;
    MAX_SPEED2    = MAX_SPEED * MAX_SPEED;
    MIN_SPEED2    = MIN_SPEED * MIN_SPEED;
    MOUSE_RADIUS2 = MOUSE_RADIUS * MOUSE_RADIUS;
}

bool ConfigFile::load() {
    std::error_code ec;
    lastWrite_ = std::filesystem::last_write_time(path_, ec);

    std::ifstream in(path_);
    if (!in) {
        std::cerr << "[config] cannot open " << path_ << ", using defaults\n";
        cfg.finalize();
        return false;
    }

    Config next;  // start from defaults so removed keys fall back cleanly
    bool ok = true;
    std::string line;
    for (int lineNo = 1; std::getline(in, line); ++lineNo) {
        std::string_view s = line;
        if (auto hash = s.find('#'); hash != std::string_view::npos) s = s.substr(0, hash);
        s = trim(s);
        if (s.empty()) continue;

        const auto eq = s.find('=');
        if (eq == std::string_view::npos) {
            std::cerr << "[config] " << path_ << ":" << lineNo << ": expected key = value\n";
            ok = false;
            continue;
        }
        const auto key = trim(s.substr(0, eq));
        const auto val = trim(s.substr(eq + 1));

        const auto it = fields().find(key);
        if (it == fields().end()) {
            std::cerr << "[config] " << path_ << ":" << lineNo << ": unknown key '" << key << "'\n";
            continue;
        }
        const bool parsed = std::visit([&](auto member) {
            return parseNumber(val, next.*member);
        }, it->second);
        if (!parsed) {
            std::cerr << "[config] " << path_ << ":" << lineNo << ": bad value for '" << key << "'\n";
            ok = false;
        }
    }

    if (next.FAR <= 0.0 || next.DT <= 0.0 || next.MIN_ZOOM <= 0.0 || next.MIN_ZOOM > next.MAX_ZOOM) {
        std::cerr << "[config] invalid values (FAR, DT, MIN_ZOOM must be > 0, MIN_ZOOM <= MAX_ZOOM)\n";
        ok = false;
    }

    if (!ok) {
        std::cerr << "[config] keeping previous settings\n";
        cfg.finalize();
        return false;
    }

    next.finalize();
    cfg = next;
    std::cerr << "[config] loaded " << path_ << "\n";
    return true;
}

bool ConfigFile::reloadIfChanged() {
    std::error_code ec;
    const auto t = std::filesystem::last_write_time(path_, ec);
    if (ec || t == lastWrite_) return false;
    return load();
}
