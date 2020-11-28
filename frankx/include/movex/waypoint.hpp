#pragma once

#include <optional>

#include <movex/affine.hpp>


namespace movex {

struct Waypoint {
    enum class ReferenceType {
        Absolute,
        Relative
    };

    Affine affine;
    std::optional<double> elbow;
    ReferenceType reference_type;


    //! Dynamic Waypoint: Relative velocity factor
    double velocity_rel {1.0};

    //! Dynamic Waypoint: Use maximal dynamics of the robot independent on other parameters
    bool max_dynamics {false};

    //! Dynamic Waypoint: Minimum time to get to next waypoint
    std::optional<double> minimum_time;

    //! Path Waypoint: Maximum distance for blending.
    double blend_max_distance {0.0};


    explicit Waypoint(): affine(Affine()), reference_type(ReferenceType::Absolute) {}
    explicit Waypoint(const Affine& affine, ReferenceType reference_type = ReferenceType::Absolute): affine(affine), reference_type(reference_type) {}
    explicit Waypoint(const Affine& affine, double elbow, ReferenceType reference_type = ReferenceType::Absolute): affine(affine), reference_type(reference_type), elbow(elbow) {}

    explicit Waypoint(double minimum_time): affine(Affine()), reference_type(ReferenceType::Relative), minimum_time(minimum_time) {}
    explicit Waypoint(const Affine& affine, ReferenceType reference_type, double velocity_rel): affine(affine), reference_type(reference_type), velocity_rel(velocity_rel) {}
    explicit Waypoint(const Affine& affine, double elbow, ReferenceType reference_type, double velocity_rel): affine(affine), elbow(elbow), reference_type(reference_type), velocity_rel(velocity_rel) {}
    explicit Waypoint(const Affine& affine, double elbow, ReferenceType reference_type, bool max_dynamics): affine(affine), elbow(elbow), reference_type(reference_type), max_dynamics(max_dynamics) {}

    // explicit Waypoint(const Affine& affine, double blend_max_distance): affine(affine), blend_max_distance(blend_max_distance) {}
    explicit Waypoint(const Affine& affine, double elbow, double blend_max_distance): affine(affine), elbow(elbow), blend_max_distance(blend_max_distance) {}

    Affine getTargetAffine(const Affine& frame, const Affine& old_affine) const {
        switch (reference_type) {
            case ReferenceType::Absolute:
                return affine * frame.inverse();
            case ReferenceType::Relative:
                return old_affine * affine * frame.inverse();
        }
    }

    Vector7d getTargetVector(const Affine& frame, const Affine& old_affine, double old_elbow) const {
        double new_elbow;
        if (reference_type == ReferenceType::Relative) {
            new_elbow = elbow.has_value() ? elbow.value() + old_elbow : old_elbow;
        } else {
            new_elbow = elbow.has_value() ? elbow.value() : old_elbow;
        }
        return getTargetAffine(frame, old_affine).vector_with_elbow(new_elbow);
    }

    Vector7d getVector(double alt_elbow) const {
        return affine.vector_with_elbow(elbow.value_or(alt_elbow));
    }
};

} // namespace frankx