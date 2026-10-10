// Virtual-time co-simulation of the firmware (on a fiber) and the robot plant.
//
// Time advances in integer nanoseconds. Peripherals the firmware started produce
// interrupts at their configured rates; each interrupt runs to completion on the
// simulator fiber, exactly between two "instructions" of the firmware main loop.
// The firmware main loop runs on its own fiber and hands control back whenever
// it checks a loop condition (sim_fw_prelude.h) or sleeps in HAL_Delay().
#pragma once
#include <cstdint>
#include <functional>
#include <string>

#include "world/robot.h"
#include "world/sensors.h"
#include "world/track.h"

class Simulation {
public:
	enum class Stop { Predicate, FirmwareDone, Halted, TimeLimit };

	Simulation(Robot& robot, IrBar& ir, Imu& imu, const Track& track, const std::string& sd_root);
	~Simulation();

	// Starts the firmware fiber; `fw_main` runs on it (it calls fwb_boot(), fwb_drive() ...).
	void start(std::function<void()> fw_main);
	// Runs until `stop` returns true, the firmware fiber returns or halts, or t >= t_end.
	Stop run(double t_end_s, const std::function<bool()>& stop = nullptr);

	double t() const { return now_ * 1e-9; }
	uint64_t now_ns() const { return now_; }
	const std::string& halt_reason() const { return halt_reason_; }

	// Called after every SysTick (1 kHz), on the simulator fiber.
	std::function<void()> on_millisecond;
	// Called on the simulator fiber whenever the ramp timer (TIM14) starts or stops.
	std::function<void(bool running)> on_ramp_timer;

	Robot& robot() { return robot_; }
	const Track& track() const { return track_; }

	// --- hooks for the C side (sim_mcu.h) ---
	bool in_fw_main() const;
	void fw_yield();
	void fw_sleep_until(uint64_t t_ns);
	void fw_halt(const char* why);
	bool i2c_read(uint16_t dev, uint16_t reg, uint8_t* buf, uint16_t len);
	bool i2c_write(uint16_t dev, uint16_t reg, const uint8_t* buf, uint16_t len);
	const char* sd_root() const { return sd_root_.c_str(); }
	int buzzer_count() const { return buzzer_count_; }
	// profiling counters: firmware main-loop resumes and fired events per source
	uint64_t resumes = 0, sleeps = 0;
	uint64_t fired[16] = {};
	void buzzer(bool on) { buzzer_count_ += on ? 1 : 0; }

private:
	enum Source { kPhysics, kSysTick, kTim7, kAdc3, kTim13, kTim14, kLptim3, kI2c, kImuOdr, kSourceCount };
	enum Req { kNone, kYield, kSleep, kDone, kHalt };

	void fire(int src);
	void sync_to_fw();
	void after_fw();
	void periodic(int src, bool enabled, uint64_t period);

	Robot& robot_;
	IrBar& ir_;
	Imu& imu_;
	const Track& track_;
	std::string sd_root_;

	uint64_t now_ = 0;
	uint64_t next_[kSourceCount];
	uint64_t period_[kSourceCount];
	uint64_t phys_t_ = 0;          // time of the last physics step
	uint64_t quantum_ = 10000;     // virtual time of one firmware main-loop pass [ns]

	void* sim_fiber_ = nullptr;
	void* fw_fiber_ = nullptr;
	bool fw_alive_ = false;
	bool on_fw_ = false;
	uint64_t t_resume_ = 0;
	Req req_ = kNone;
	uint64_t req_t_ = 0;
	bool halted_ = false;
	std::string halt_reason_;
	std::function<void()> fw_main_;
	bool ramp_running_ = false;
	int buzzer_count_ = 0;

	static void __stdcall fiber_proc(void* arg);
};
