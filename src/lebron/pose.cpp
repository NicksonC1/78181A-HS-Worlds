#define FMT_HEADER_ONLY
#include "fmt/core.h"

#include "lebron/pose.hpp"

lebron::Pose::Pose(float x, float y, float theta) {
    this->x = x;
    this->y = y;
    this->theta = theta;
}

lebron::Pose lebron::Pose::operator+(const lebron::Pose& other) const {
    return lebron::Pose(this->x + other.x, this->y + other.y, this->theta);
}

lebron::Pose lebron::Pose::operator-(const lebron::Pose& other) const {
    return lebron::Pose(this->x - other.x, this->y - other.y, this->theta);
}

float lebron::Pose::operator*(const lebron::Pose& other) const { return this->x * other.x + this->y * other.y; }

lebron::Pose lebron::Pose::operator*(const float& other) const {
    return lebron::Pose(this->x * other, this->y * other, this->theta);
}

lebron::Pose lebron::Pose::operator/(const float& other) const {
    return lebron::Pose(this->x / other, this->y / other, this->theta);
}

lebron::Pose lebron::Pose::lerp(lebron::Pose other, float t) const {
    return lebron::Pose(this->x + (other.x - this->x) * t, this->y + (other.y - this->y) * t, this->theta);
}

float lebron::Pose::distance(lebron::Pose other) const { return std::hypot(this->x - other.x, this->y - other.y); }

float lebron::Pose::angle(lebron::Pose other) const { return std::atan2(other.y - this->y, other.x - this->x); }

lebron::Pose lebron::Pose::rotate(float angle) const {
    return lebron::Pose(this->x * std::cos(angle) - this->y * std::sin(angle),
                        this->x * std::sin(angle) + this->y * std::cos(angle), this->theta);
}

std::string lebron::format_as(const lebron::Pose& pose) {
    // the double brackets become single brackets
    return fmt::format("lebron::Pose {{ x: {}, y: {}, theta: {} }}", pose.x, pose.y, pose.theta);
}
