#define FMT_HEADER_ONLY
#include "fmt/core.h"

#include "genesis/pose.hpp"

genesis::Pose::Pose(float x, float y, float theta) {
    this->x = x;
    this->y = y;
    this->theta = theta;
}

genesis::Pose genesis::Pose::operator+(const genesis::Pose& other) const {
    return genesis::Pose(this->x + other.x, this->y + other.y, this->theta);
}

genesis::Pose genesis::Pose::operator-(const genesis::Pose& other) const {
    return genesis::Pose(this->x - other.x, this->y - other.y, this->theta);
}

float genesis::Pose::operator*(const genesis::Pose& other) const { return this->x * other.x + this->y * other.y; }

genesis::Pose genesis::Pose::operator*(const float& other) const {
    return genesis::Pose(this->x * other, this->y * other, this->theta);
}

genesis::Pose genesis::Pose::operator/(const float& other) const {
    return genesis::Pose(this->x / other, this->y / other, this->theta);
}

genesis::Pose genesis::Pose::lerp(genesis::Pose other, float t) const {
    return genesis::Pose(this->x + (other.x - this->x) * t, this->y + (other.y - this->y) * t, this->theta);
}

float genesis::Pose::distance(genesis::Pose other) const { return std::hypot(this->x - other.x, this->y - other.y); }

float genesis::Pose::angle(genesis::Pose other) const { return std::atan2(other.y - this->y, other.x - this->x); }

genesis::Pose genesis::Pose::rotate(float angle) const {
    return genesis::Pose(this->x * std::cos(angle) - this->y * std::sin(angle),
                        this->x * std::sin(angle) + this->y * std::cos(angle), this->theta);
}

std::string genesis::format_as(const genesis::Pose& pose) {
    // the double brackets become single brackets
    return fmt::format("genesis::Pose {{ x: {}, y: {}, theta: {} }}", pose.x, pose.y, pose.theta);
}
