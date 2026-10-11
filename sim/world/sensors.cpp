#include "world/sensors.h"

#include <algorithm>
#include <cmath>

void IrBar::init(const RobotParams& p, uint32_t seed) {
	p_ = p;
	rng_.seed(seed);
	// sensor.c line_sensor_pos[]: sensor 0 = -1.0 ... sensor 15 = +1.0, and a positive
	// position steers right (motor.c), so sensor 0 sits on the robot's left (+y).
	for (int i = 0; i < 16; i++) {
		if (p.ir_sensors.size() >= 16) {   // measured positions from the CAD model (curved bar)
			x_[i] = p.ir_sensors[i][0];
			y_[i] = p.ir_sensors[i][1];
		} else {
			x_[i] = p.sensor_x;
			y_[i] = -(-1.0 + 2.0 * i / 15.0) * p.sensor_half_width;
		}
	}
	x_[16] = x_[17] = p.ir_sensors.size() >= 16 ? p.ir_sensors[0][0] : p.sensor_x;
	y_[16] = p.marker_sensor_y;    // LEFT_MARK_SENSOR_INDEX (not on the CAD board)
	y_[17] = -p.marker_sensor_y;   // RIGHT_MARK_SENSOR_INDEX
	std::normal_distribution<double> spread(1.0, p.ir_spread);
	for (int i = 0; i < kChannels; i++) {
		double g = std::max(0.5, spread(rng_));
		white_[i] = p.ir_white * g;
		black_[i] = p.ir_black * std::max(0.5, spread(rng_));
	}
	set_delay(p.sensor_delay);
}

void IrBar::set_delay(double seconds) {
	p_.sensor_delay = std::max(0.0, seconds);
	size_t n = static_cast<size_t>(std::lround(p_.sensor_delay / p_.timestep));
	hist_.assign(n > 0 ? n + 1 : 0, {});
	head_ = filled_ = 0;   // refilled with the current pose at the next physics step
}

void IrBar::record(const Robot& robot) {
	if (hist_.empty())
		return;
	std::array<double, 8> f;
	robot.ground_frame(f.data());
	const auto& last = hist_[head_];
	// first step, or the robot was placed somewhere else: the bar has seen nothing else yet
	if (filled_ == 0 || std::hypot(f[0] - last[0], f[1] - last[1]) > 0.02) {
		std::fill(hist_.begin(), hist_.end(), f);
		filled_ = hist_.size();
	}
	head_ = (head_ + 1) % hist_.size();
	hist_[head_] = f;
}

uint16_t IrBar::sample(int channel, const Robot& robot, const Track& track) {
	if (channel < 0 || channel >= kChannels)
		return 0;
	Vec2 g;
	if (filled_ > 0) {   // oldest frame in the ring: sensor_delay ago
		const auto& f = hist_[(head_ + 1) % hist_.size()];
		const double bx = x_[channel], by = y_[channel], bz = -p_.wheel_radius;
		g = {f[0] + f[2] * bx + f[3] * by + f[4] * bz, f[1] + f[5] * bx + f[6] * by + f[7] * bz};
	} else {
		g = robot.body_to_ground(x_[channel], y_[channel]);
	}
	double c = lifted_ ? 0.0 : track.coverage(g, p_.sensor_sigma);
	double v = black_[channel] + (white_[channel] - black_[channel]) * c + p_.ir_noise * noise_(rng_);
	return static_cast<uint16_t>(std::clamp(v, 0.0, 4095.0));
}

void IrBar::calibration(uint16_t white[kChannels], uint16_t black[kChannels]) const {
	// The firmware keeps the maximum seen over each sweep, i.e. roughly level + 3 sigma noise.
	for (int i = 0; i < kChannels; i++) {
		white[i] = static_cast<uint16_t>(std::min(4095.0, white_[i] + 3 * p_.ir_noise));
		black[i] = static_cast<uint16_t>(std::min(4095.0, black_[i] + 3 * p_.ir_noise));
	}
}

// ---------------------------------------------------------------------------
// LSM6DS3TR-C
// ---------------------------------------------------------------------------
namespace {
constexpr uint8_t kWhoAmI = 0x0F;
constexpr uint8_t kOutxLG = 0x22;   // gyro X,Y,Z then accel X,Y,Z (little endian)
constexpr double kGyroLsbDps = 0.070;   // FS = 2000 dps
constexpr double kAccelLsbG = 0.061e-3; // FS = 2 g
constexpr double kRadToDeg = 57.29577951308232;

void put16(uint8_t* r, int16_t v) {
	r[0] = static_cast<uint8_t>(v & 0xFF);
	r[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}
}  // namespace

void Imu::init(const RobotParams& p, uint32_t seed) {
	rng_.seed(seed);
	std::uniform_real_distribution<double> b(-1.0, 1.0);
	bias_dps_ = p.gyro_bias_dps * b(rng_);
	noise_dps_ = p.gyro_noise_dps;
	regs_[kWhoAmI] = 0x6A;
	put16(&regs_[kOutxLG + 10], static_cast<int16_t>(1.0 / kAccelLsbG));   // 1 g on Z
}

void Imu::accumulate(double yaw_rate_rad_s, double dt) {
	sum_ += yaw_rate_rad_s * dt;
	sum_t_ += dt;
}

void Imu::latch() {
	double dps = (sum_t_ > 0 ? sum_ / sum_t_ : 0.0) * kRadToDeg + bias_dps_ + noise_dps_ * noise_(rng_);
	double lsb = std::clamp(std::round(dps / kGyroLsbDps), -32768.0, 32767.0);
	put16(&regs_[kOutxLG + 4], static_cast<int16_t>(lsb));
	sum_ = sum_t_ = 0;
}

bool Imu::read(uint8_t reg, uint8_t* buf, uint16_t len) {
	for (uint16_t i = 0; i < len; i++)
		buf[i] = (reg + i < 128) ? regs_[reg + i] : 0;
	return true;
}

bool Imu::write(uint8_t reg, const uint8_t* buf, uint16_t len) {
	for (uint16_t i = 0; i < len; i++)
		if (reg + i < 128 && reg + i != kWhoAmI)
			regs_[reg + i] = buf[i];
	return true;
}
