#include <vector>
#include <unordered_map>
#pragma once

namespace lemlib {
class PID {
    public:
        /**
         * @brief Construct a new PID
         *
         * @param kP proportional gain
         * @param kI integral gain
         * @param kD derivative gain
         * @param windupRange integral anti windup range
         * @param signFlipReset whether to reset integral when sign of error flips
         *
         * @b Example
         * @code {.cpp}
         * // create a PID
         * PID pid(5, // kP
         *         0.01, // kI
         *         20, // kD
         *         5, // integral anti windup range
         *         false); // don't reset integral when sign of error flips
         * @endcode
         */
        PID(float kP, float kI, float kD, float windupRange = 0, bool signFlipReset = false);

        /**
         * @brief Update the PID
         *
         * @param error target minus position - AKA error
         * @return float output
         *
         * @b Example
         * @code {.cpp}
         * void opcontrol() {
         *     // create a PID
         *     PID pid(5, 0, 20);
         *     // give the pid a test input
         *     // the pid will then return an output
         *     float output = pid.update(10);
         * }
         * @endcode
         */
        float update(float error);


        /**
         * @brief Decide which constants to use
         *
         * @param error target minus position - AKA error
         * @param customConstants a map of error thresholds to constants. If error is above a threshold, the corresponding constants will be used. If error is between two thresholds, the constants will be linearly interpolated. If this map is empty, the default constants will be used.
         * @return vector of pid constants
         *
         * 
         */
        std::pmr::vector<float> decideConstants(float error, std::pmr::unordered_map<float, std::pmr::vector<float>>& customConstants);

        /**
         * @brief reset integral, derivative, and prevTime
         *
         * @b Example
         * @code {.cpp}
         * void opcontrol() {
         *     // create a PID
         *     PID pid(5, 0, 20);
         *     // give the pid a test input
         *     // the pid will then return an output
         *     float output = pid.update(10);
         *     // reset the pid
         *     pid.reset();
         * }
         * @endcode
         */
        void reset();
    protected:
        // gains
        const float kP;
        const float kI;
        const float kD;

        // optimizations
        const float windupRange;
        const bool signFlipReset;

        float integral = 0;
        float prevError = 0;
};
} // namespace lemlib