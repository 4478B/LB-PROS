#ifndef AVERAGED_IMU_H
#define AVERAGED_IMU_H

#include "pros/imu.hpp"
#include <vector>

using namespace pros;

/**
 * @brief Wrapper class that averages readings from multiple IMUs
 * 
 * This class inherits from pros::Imu and overrides key methods to return
 * averaged values from multiple IMU sensors. This allows it to be used
 * as a drop-in replacement for a single IMU in LemLib's OdomSensors.
 */
class AveragedIMU : public pros::Imu {
private:
    std::vector<pros::Imu*> imus;
    
    /**
     * @brief Check if an IMU is connected and working
     * @param imu Pointer to the IMU to check
     * @return true if the IMU is connected and responding, false otherwise
     */
    bool is_imu_connected(pros::Imu* imu) const {
        // Check status - if it's in error state, it's disconnected
        pros::ImuStatus status = imu->get_status();
        if (status == pros::ImuStatus::error) {
            return false;
        }
        
        // If status is ready or calibrating, the IMU is connected
        // The actual read error checking (PROS_ERR_F) will be done in the getter methods
        return true;
    }
    
public:
    /**
     * @brief Construct an AveragedIMU from two IMU pointers
     * 
     * @param imu1 Pointer to the first IMU
     * @param imu2 Pointer to the second IMU
     */
    AveragedIMU(pros::Imu* imu1, pros::Imu* imu2) 
        : pros::Imu(imu1->get_port()) {  // Use first IMU's port as base
        imus.push_back(imu1);
        imus.push_back(imu2);
    }
    
    /**
     * @brief Calibrate all connected IMUs
     * Failsafe: only calibrates connected IMUs
     */
    virtual std::int32_t reset(bool blocking = false) const override {
        std::int32_t result = 1;
        bool any_success = false;
        for (auto imu : imus) {
            // Try to reset even if not currently connected (might reconnect)
            if (imu->reset(blocking) != PROS_ERR) {
                any_success = true;
            }
        }
        return any_success ? 1 : PROS_ERR;
    }
    
    /**
     * @brief Get averaged rotation value
     * Failsafe: automatically switches to connected IMU if one disconnects
     */
    virtual double get_rotation() const override {
        double sum = 0;
        int count = 0;
        for (auto imu : imus) {
            // Only use IMUs that are connected
            if (is_imu_connected(imu)) {
                double val = imu->get_rotation();
                if (val != PROS_ERR_F) {
                    sum += val;
                    count++;
                }
            }
        }
        return count > 0 ? sum / count : PROS_ERR_F;
    }
    
    /**
     * @brief Get averaged heading value
     * Failsafe: automatically switches to connected IMU if one disconnects
     */
    virtual double get_heading() const override {
        double sum = 0;
        int count = 0;
        for (auto imu : imus) {
            // Only use IMUs that are connected
            if (is_imu_connected(imu)) {
                double val = imu->get_heading();
                if (val != PROS_ERR_F) {
                    sum += val;
                    count++;
                }
            }
        }
        return count > 0 ? sum / count : PROS_ERR_F;
    }
    
    /**
     * @brief Get averaged quaternion
     * Failsafe: automatically switches to connected IMU if one disconnects
     */
    virtual pros::quaternion_s_t get_quaternion() const override {
        // For quaternions, we'll use the first connected IMU's value
        // Proper quaternion averaging is complex
        for (auto imu : imus) {
            if (is_imu_connected(imu)) {
                return imu->get_quaternion();
            }
        }
        // If no IMU is connected, return error quaternion
        pros::quaternion_s_t error_quat = {PROS_ERR_F, PROS_ERR_F, PROS_ERR_F, PROS_ERR_F};
        return error_quat;
    }
    
    /**
     * @brief Get averaged euler angles
     * Failsafe: automatically switches to connected IMU if one disconnects
     */
    virtual pros::euler_s_t get_euler() const override {
        pros::euler_s_t result = {0, 0, 0};
        int count = 0;
        
        for (auto imu : imus) {
            // Only use IMUs that are connected
            if (is_imu_connected(imu)) {
                pros::euler_s_t val = imu->get_euler();
                if (val.pitch != PROS_ERR_F) {
                    result.pitch += val.pitch;
                    result.roll += val.roll;
                    result.yaw += val.yaw;
                    count++;
                }
            }
        }
        
        if (count > 0) {
            result.pitch /= count;
            result.roll /= count;
            result.yaw /= count;
        } else {
            // If no IMU is connected, return error values
            result.pitch = PROS_ERR_F;
            result.roll = PROS_ERR_F;
            result.yaw = PROS_ERR_F;
        }
        
        return result;
    }
    
    /**
     * @brief Get averaged pitch
     * Failsafe: automatically switches to connected IMU if one disconnects
     */
    virtual double get_pitch() const override {
        double sum = 0;
        int count = 0;
        for (auto imu : imus) {
            // Only use IMUs that are connected
            if (is_imu_connected(imu)) {
                double val = imu->get_pitch();
                if (val != PROS_ERR_F) {
                    sum += val;
                    count++;
                }
            }
        }
        return count > 0 ? sum / count : PROS_ERR_F;
    }
    
    /**
     * @brief Get averaged roll
     * Failsafe: automatically switches to connected IMU if one disconnects
     */
    virtual double get_roll() const override {
        double sum = 0;
        int count = 0;
        for (auto imu : imus) {
            // Only use IMUs that are connected
            if (is_imu_connected(imu)) {
                double val = imu->get_roll();
                if (val != PROS_ERR_F) {
                    sum += val;
                    count++;
                }
            }
        }
        return count > 0 ? sum / count : PROS_ERR_F;
    }
    
    /**
     * @brief Get averaged yaw
     * Failsafe: automatically switches to connected IMU if one disconnects
     */
    virtual double get_yaw() const override {
        double sum = 0;
        int count = 0;
        for (auto imu : imus) {
            // Only use IMUs that are connected
            if (is_imu_connected(imu)) {
                double val = imu->get_yaw();
                if (val != PROS_ERR_F) {
                    sum += val;
                    count++;
                }
            }
        }
        return count > 0 ? sum / count : PROS_ERR_F;
    }
    
    /**
     * @brief Get averaged gyro rate
     * Failsafe: automatically switches to connected IMU if one disconnects
     */
    virtual pros::imu_gyro_s_t get_gyro_rate() const override {
        pros::imu_gyro_s_t result = {0, 0, 0};
        int count = 0;
        
        for (auto imu : imus) {
            // Only use IMUs that are connected
            if (is_imu_connected(imu)) {
                pros::imu_gyro_s_t val = imu->get_gyro_rate();
                if (val.x != PROS_ERR_F) {
                    result.x += val.x;
                    result.y += val.y;
                    result.z += val.z;
                    count++;
                }
            }
        }
        
        if (count > 0) {
            result.x /= count;
            result.y /= count;
            result.z /= count;
        } else {
            // If no IMU is connected, return error values
            result.x = PROS_ERR_F;
            result.y = PROS_ERR_F;
            result.z = PROS_ERR_F;
        }
        
        return result;
    }
    
    /**
     * @brief Get averaged acceleration
     * Failsafe: automatically switches to connected IMU if one disconnects
     */
    virtual pros::imu_accel_s_t get_accel() const override {
        pros::imu_accel_s_t result = {0, 0, 0};
        int count = 0;
        
        for (auto imu : imus) {
            // Only use IMUs that are connected
            if (is_imu_connected(imu)) {
                pros::imu_accel_s_t val = imu->get_accel();
                if (val.x != PROS_ERR_F) {
                    result.x += val.x;
                    result.y += val.y;
                    result.z += val.z;
                    count++;
                }
            }
        }
        
        if (count > 0) {
            result.x /= count;
            result.y /= count;
            result.z /= count;
        } else {
            // If no IMU is connected, return error values
            result.x = PROS_ERR_F;
            result.y = PROS_ERR_F;
            result.z = PROS_ERR_F;
        }
        
        return result;
    }
    
    /**
     * @brief Get status - returns ready if at least one IMU is ready
     * Failsafe: returns ready if any connected IMU is ready
     */
    virtual pros::ImuStatus get_status() const override {
        bool has_ready = false;
        bool has_calibrating = false;
        
        for (auto imu : imus) {
            pros::ImuStatus status = imu->get_status();
            if (status == pros::ImuStatus::ready) {
                // Only count as ready if the IMU is actually connected
                if (is_imu_connected(imu)) {
                    has_ready = true;
                }
            } else if (status == pros::ImuStatus::calibrating) {
                has_calibrating = true;
            }
        }
        
        if (has_ready) {
            return pros::ImuStatus::ready;
        } else if (has_calibrating) {
            return pros::ImuStatus::calibrating;
        } else {
            return pros::ImuStatus::error;
        }
    }
    
    /**
     * @brief Check if any IMU is calibrating
     */
    virtual bool is_calibrating() const override {
        for (auto imu : imus) {
            if (imu->is_calibrating()) {
                return true;
            }
        }
        return false;
    }
    
    /**
     * @brief Tare all connected IMUs rotation
     * Failsafe: only tares connected IMUs
     */
    virtual std::int32_t tare_rotation() const override {
        std::int32_t result = 1;
        bool any_success = false;
        for (auto imu : imus) {
            if (is_imu_connected(imu)) {
                if (imu->tare_rotation() != PROS_ERR) {
                    any_success = true;
                }
            }
        }
        return any_success ? 1 : PROS_ERR;
    }
    
    /**
     * @brief Tare all connected IMUs heading
     * Failsafe: only tares connected IMUs
     */
    virtual std::int32_t tare_heading() const override {
        std::int32_t result = 1;
        bool any_success = false;
        for (auto imu : imus) {
            if (is_imu_connected(imu)) {
                if (imu->tare_heading() != PROS_ERR) {
                    any_success = true;
                }
            }
        }
        return any_success ? 1 : PROS_ERR;
    }
    
    /**
     * @brief Tare all connected IMUs
     * Failsafe: only tares connected IMUs
     */
    virtual std::int32_t tare() const override {
        std::int32_t result = 1;
        bool any_success = false;
        for (auto imu : imus) {
            if (is_imu_connected(imu)) {
                if (imu->tare() != PROS_ERR) {
                    any_success = true;
                }
            }
        }
        return any_success ? 1 : PROS_ERR;
    }
    
    /**
     * @brief Set heading on all connected IMUs
     * Failsafe: only sets heading on connected IMUs
     */
    virtual std::int32_t set_heading(const double target) const override {
        std::int32_t result = 1;
        bool any_success = false;
        for (auto imu : imus) {
            if (is_imu_connected(imu)) {
                if (imu->set_heading(target) != PROS_ERR) {
                    any_success = true;
                }
            }
        }
        return any_success ? 1 : PROS_ERR;
    }
    
    /**
     * @brief Set rotation on all connected IMUs
     * Failsafe: only sets rotation on connected IMUs
     */
    virtual std::int32_t set_rotation(const double target) const override {
        std::int32_t result = 1;
        bool any_success = false;
        for (auto imu : imus) {
            if (is_imu_connected(imu)) {
                if (imu->set_rotation(target) != PROS_ERR) {
                    any_success = true;
                }
            }
        }
        return any_success ? 1 : PROS_ERR;
    }
};

#endif // AVERAGED_IMU_H

