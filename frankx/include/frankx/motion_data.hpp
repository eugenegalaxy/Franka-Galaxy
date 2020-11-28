#pragma once

#include <frankx/reaction.hpp>


namespace frankx {

struct MotionData {
    double velocity_rel {1.0}, acceleration_rel {1.0}, jerk_rel {1.0};
    bool max_dynamics {false};

    std::vector<Reaction> reactions {};

    explicit MotionData(double dynamic_rel = 1.0): velocity_rel(dynamic_rel), acceleration_rel(dynamic_rel), jerk_rel(dynamic_rel) { }

    MotionData& withDynamicRel(double dynamic_rel) {
        velocity_rel = dynamic_rel;
        acceleration_rel = dynamic_rel;
        jerk_rel = dynamic_rel;
        return *this;
    }

    MotionData& withMaxDynamics() {
        max_dynamics = true;
        return *this;
    }

    MotionData& withReaction(const Reaction& reaction) {
        reactions.push_back(reaction);
        return *this;
    }

    //! Whether any of the reactions did occur (fire) during the motion
    bool didBreak() {
        return std::any_of(reactions.begin(), reactions.end(), [](auto r) { return r.has_fired; });
    }
};

} // namespace frankx
