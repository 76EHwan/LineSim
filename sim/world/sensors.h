// Sensor models: the 18-channel reflective IR bar and the LSM6DS3TR-C gyro (register level).
#pragma once
#include <array>
#include <cstdint>
#include <random>
#include <vector>

#include "world/robot.h"
#include "world/track.h"

class IrBar {
public:
	static constexpr int kChannels = 18;   // 16 line + left/right marker (sensor.h NUM_SENSORS)

	void init(const RobotParams& p, uint32_t seed);
	// Call after every physics step: keeps the poses the bar sees with `sensor_delay`.
	void record(const Robot& robot);
	// Change sensor_delay between drives (auto-tune stress test of the model's steering lag).
	void set_delay(double seconds);
	double delay() const { return p_.sensor_delay; }
	// 12-bit ADC reading of `channel` for the robot's pose `sensor_delay` ago.
	uint16_t sample(int channel, const Robot& robot, const Track& track);
	// What the firmware's own white/black sweep calibration would record.
	void calibration(uint16_t white[kChannels], uint16_t black[kChannels]) const;
	double sensor_y(int channel) const { return y_[channel]; }
	// Robot picked up by the operator: every channel sees only the dark floor far below.
	void set_lifted(bool lifted) { lifted_ = lifted; }

private:
	RobotParams p_;
	double x_[kChannels]{}, y_[kChannels]{};
	double white_[kChannels]{}, black_[kChannels]{};
	bool lifted_ = false;
	std::vector<std::array<double, 8>> hist_;   // ring of ground frames, one per physics step
	size_t head_ = 0, filled_ = 0;
	std::mt19937 rng_;
	std::normal_distribution<double> noise_{0.0, 1.0};
};

class Imu {
public:
	void init(const RobotParams& p, uint32_t seed);
	// Integrate the true yaw rate over a physics step (the chip's ODR filter).
	void accumulate(double yaw_rate_rad_s, double dt);
	// Called at the output data rate (104 Hz): latch the averaged rate into OUTZ_G.
	void latch();
	double odr_period() const { return 1.0 / 104.0; }

	bool read(uint8_t reg, uint8_t* buf, uint16_t len);
	bool write(uint8_t reg, const uint8_t* buf, uint16_t len);

private:
	uint8_t regs_[128]{};
	double sum_ = 0, sum_t_ = 0;
	double bias_dps_ = 0, noise_dps_ = 0;
	std::mt19937 rng_;
	std::normal_distribution<double> noise_{0.0, 1.0};
};
