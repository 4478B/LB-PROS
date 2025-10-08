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
     * @brief Calibrate all IMUs
     */
    virtual std::int32_t reset(bool blocking = false) const override {
        std::int32_t result = 1;
        for (auto imu : imus) {
            if (imu->reset(blocking) == PROS_ERR) {
                result = PROS_ERR;
            }
        }
        return result;
    }
    
    /**
     * @brief Get averaged rotation value
     */
    virtual double get_rotation() const override {
        double sum = 0;
        int count = 0;
        for (auto imu : imus) {
            double val = imu->get_rotation();
            if (val != PROS_ERR_F) {
                sum += val;
                count++;
            }
        }
        return count > 0 ? sum / count : PROS_ERR_F;
    }
    
    /**
     * @brief Get averaged heading value
     */
    virtual double get_heading() const override {
        double sum = 0;
        int count = 0;
        for (auto imu : imus) {
            double val = imu->get_heading();
            if (val != PROS_ERR_F) {
                sum += val;
                count++;
            }
        }
        return count > 0 ? sum / count : PROS_ERR_F;
    }
    
    /**
     * @brief Get averaged quaternion
     */
    virtual pros::quaternion_s_t get_quaternion() const override {
        // For quaternions, we'll use the first IMU's value
        // Proper quaternion averaging is complex
        return imus[0]->get_quaternion();
    }
    
    /**
     * @brief Get averaged euler angles
     */
    virtual pros::euler_s_t get_euler() const override {
        pros::euler_s_t result = {0, 0, 0};
        int count = 0;
        
        for (auto imu : imus) {
            pros::euler_s_t val = imu->get_euler();
            if (val.pitch != PROS_ERR_F) {
                result.pitch += val.pitch;
                result.roll += val.roll;
                result.yaw += val.yaw;
                count++;
            }
        }
        
        if (count > 0) {
            result.pitch /= count;
            result.roll /= count;
            result.yaw /= count;
        }
        
        return result;
    }
    
    /**
     * @brief Get averaged pitch
     */
    virtual double get_pitch() const override {
        double sum = 0;
        int count = 0;
        for (auto imu : imus) {
            double val = imu->get_pitch();
            if (val != PROS_ERR_F) {
                sum += val;
                count++;
            }
        }
        return count > 0 ? sum / count : PROS_ERR_F;
    }
    
    /**
     * @brief Get averaged roll
     */
    virtual double get_roll() const override {
        double sum = 0;
        int count = 0;
        for (auto imu : imus) {
            double val = imu->get_roll();
            if (val != PROS_ERR_F) {
                sum += val;
                count++;
            }
        }
        return count > 0 ? sum / count : PROS_ERR_F;
    }
    
    /**
     * @brief Get averaged yaw
     */
    virtual double get_yaw() const override {
        double sum = 0;
        int count = 0;
        for (auto imu : imus) {
            double val = imu->get_yaw();
            if (val != PROS_ERR_F) {
                sum += val;
                count++;
            }
        }
        return count > 0 ? sum / count : PROS_ERR_F;
    }
    
    /**
     * @brief Get averaged gyro rate
     */
    virtual pros::imu_gyro_s_t get_gyro_rate() const override {
        pros::imu_gyro_s_t result = {0, 0, 0};
        int count = 0;
        
        for (auto imu : imus) {
            pros::imu_gyro_s_t val = imu->get_gyro_rate();
            if (val.x != PROS_ERR_F) {
                result.x += val.x;
                result.y += val.y;
                result.z += val.z;
                count++;
            }
        }
        
        if (count > 0) {
            result.x /= count;
            result.y /= count;
            result.z /= count;
        }
        
        return result;
    }
    
    /**
     * @brief Get averaged acceleration
     */
    virtual pros::imu_accel_s_t get_accel() const override {
        pros::imu_accel_s_t result = {0, 0, 0};
        int count = 0;
        
        for (auto imu : imus) {
            pros::imu_accel_s_t val = imu->get_accel();
            if (val.x != PROS_ERR_F) {
                result.x += val.x;
                result.y += val.y;
                result.z += val.z;
                count++;
            }
        }
        
        if (count > 0) {
            result.x /= count;
            result.y /= count;
            result.z /= count;
        }
        
        return result;
    }
    
    /**
     * @brief Get status - returns error if any IMU has an error
     */
    virtual pros::ImuStatus get_status() const override {
        for (auto imu : imus) {
            pros::ImuStatus status = imu->get_status();
            if (status == pros::ImuStatus::error) {
                return pros::ImuStatus::error;
            }
            if (status == pros::ImuStatus::calibrating) {
                return pros::ImuStatus::calibrating;
            }
        }
        return pros::ImuStatus::ready;
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
     * @brief Tare all IMUs rotation
     */
    virtual std::int32_t tare_rotation() const override {
        std::int32_t result = 1;
        for (auto imu : imus) {
            if (imu->tare_rotation() == PROS_ERR) {
                result = PROS_ERR;
            }
        }
        return result;
    }
    
    /**
     * @brief Tare all IMUs heading
     */
    virtual std::int32_t tare_heading() const override {
        std::int32_t result = 1;
        for (auto imu : imus) {
            if (imu->tare_heading() == PROS_ERR) {
                result = PROS_ERR;
            }
        }
        return result;
    }
    
    /**
     * @brief Tare all IMUs
     */
    virtual std::int32_t tare() const override {
        std::int32_t result = 1;
        for (auto imu : imus) {
            if (imu->tare() == PROS_ERR) {
                result = PROS_ERR;
            }
        }
        return result;
    }
    
    /**
     * @brief Set heading on all IMUs
     */
    virtual std::int32_t set_heading(const double target) const override {
        std::int32_t result = 1;
        for (auto imu : imus) {
            if (imu->set_heading(target) == PROS_ERR) {
                result = PROS_ERR;
            }
        }
        return result;
    }
    
    /**
     * @brief Set rotation on all IMUs
     */
    virtual std::int32_t set_rotation(const double target) const override {
        std::int32_t result = 1;
        for (auto imu : imus) {
            if (imu->set_rotation(target) == PROS_ERR) {
                result = PROS_ERR;
            }
        }
        return result;
    }
};

#endif // AVERAGED_IMU_H

