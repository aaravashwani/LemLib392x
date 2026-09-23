#include "pid.hpp"
#include "util.hpp"
#include <vector>

namespace lemlib {
PID::PID(float kP, float kI, float kD, float windupRange, bool signFlipReset)
    : kP(kP),
      kI(kI),
      kD(kD),
      windupRange(windupRange),
      signFlipReset(signFlipReset) {}

float PID::update(const float error) {
    // calculate integral
    integral += error;
    if (sgn(error) != sgn((prevError)) && signFlipReset) integral = 0;
    if (fabs(error) > windupRange && windupRange != 0) integral = 0;

    // calculate derivative
    const float derivative = error - prevError;
    prevError = error;

    // calculate output
    return error * kP + integral * kI + derivative * kD;
}

std::pmr::vector<float> PID::decideConstants(float error, std::pmr::unordered_map<float, std::pmr::vector<float>>& customConstants) {
    if (customConstants.empty()) return std::pmr::vector<float>{kP, kI, kD, windupRange};
    float bottomBound = 0;
    float topBound = 360;
    for (const auto& pair : customConstants) {
        float threshold = pair.first;

        if (threshold <= error && threshold > bottomBound) {
            bottomBound = threshold;
        }
        if(threshold >= error && threshold < topBound) {
            topBound = threshold;
        }
    }
    if (bottomBound == 0) return customConstants[topBound];
    if (topBound == 360) return customConstants[bottomBound];

    return std::pmr::vector<float>{customConstants[bottomBound][0] + (customConstants[topBound][0] - customConstants[bottomBound][0]) * ((error - bottomBound) / (topBound - bottomBound)),
                                  customConstants[bottomBound][1] + (customConstants[topBound][1] - customConstants[bottomBound][1]) * ((error - bottomBound) / (topBound - bottomBound)),
                                  customConstants[bottomBound][2] + (customConstants[topBound][2] - customConstants[bottomBound][2]) * ((error - bottomBound) / (topBound - bottomBound)),
                                  customConstants[bottomBound][3]};
}

void PID::reset() {
    integral = 0;
    prevError = 0;
}
} // namespace lemlib