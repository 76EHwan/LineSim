#include "core/simulation.h"

#include <windows.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

#include "fw_bridge.h"
#include "sim_mcu.h"

namespace {
constexpr uint64_t kNever = std::numeric_limits<uint64_t>::max();
constexpr uint64_t kUs = 1000;
constexpr uint64_t kMs = 1000000;
constexpr double kTwoPi = 6.283185307179586;

// Firmware timer configuration (Core/Src/tim.c, 240 MHz timer clock):
constexpr uint64_t kTim7Period = 25 * kUs;     // PSC 240, ARR 25  -> IR scan slot
constexpr uint64_t kTim2Delay = 5 * kUs;       // PSC 240, ARR 5, one pulse -> ADC3 trigger
constexpr uint64_t kAdc3Conv = 500;            // 12-bit conversion + BDMA
constexpr uint64_t kTim13Period = 500 * kUs;   // PSC 240, ARR 500 -> speed loop
constexpr uint64_t kTim14Period = 500 * kUs;   // PSC 240, ARR 500 -> ramp / steering
constexpr uint64_t kLptim3Period = 1 * kMs;    // buzzer countdown (LPTIM_TICK_DT)
constexpr uint64_t kI2cRead2 = 115 * kUs;      // 400 kHz: addr+reg+addr+2 bytes
constexpr double kEncoderCounts = 2048.0;      // foc.h ENCODER_RESOLUTION
constexpr uint16_t kImuAddr = 0xD4;            // lsm6ds3tr-c.h LSM6DS3_ADDR

Simulation* g_sim = nullptr;
}  // namespace

// ---------------------------------------------------------------------------
// C hooks (sim_mcu.h)
// ---------------------------------------------------------------------------
extern "C" {
uint64_t sim_now_ns(void) { return g_sim ? g_sim->now_ns() : 0; }
int sim_in_fw_main(void) { return g_sim && g_sim->in_fw_main(); }
void sim_fw_yield(void) {
	if (g_sim)
		g_sim->fw_yield();
}
void sim_fw_sleep_until_ns(uint64_t t_ns) {
	if (g_sim)
		g_sim->fw_sleep_until(t_ns);
}
void sim_fw_halt(const char* why) {
	if (g_sim)
		g_sim->fw_halt(why);
}
int sim_i2c_mem_read(uint16_t dev, uint16_t reg, uint8_t* buf, uint16_t len) {
	return g_sim && g_sim->i2c_read(dev, reg, buf, len);
}
int sim_i2c_mem_write(uint16_t dev, uint16_t reg, const uint8_t* buf, uint16_t len) {
	return g_sim && g_sim->i2c_write(dev, reg, buf, len);
}
const char* sim_sd_root(void) { return g_sim ? g_sim->sd_root() : "."; }
void sim_buzzer(int on) {
	if (g_sim)
		g_sim->buzzer(on != 0);
}
}

// ---------------------------------------------------------------------------

Simulation::Simulation(Robot& robot, IrBar& ir, Imu& imu, const Track& track, const std::string& sd_root)
	: robot_(robot), ir_(ir), imu_(imu), track_(track), sd_root_(sd_root) {
	g_sim = this;
	std::fill(std::begin(next_), std::end(next_), kNever);
	std::fill(std::begin(period_), std::end(period_), kNever);
	period_[kPhysics] = static_cast<uint64_t>(std::llround(robot.params().timestep * 1e9));
	period_[kSysTick] = kMs;
	period_[kTim7] = kTim7Period;
	period_[kTim13] = kTim13Period;
	period_[kTim14] = kTim14Period;
	period_[kLptim3] = kLptim3Period;
	period_[kImuOdr] = static_cast<uint64_t>(std::llround(imu.odr_period() * 1e9));
	next_[kPhysics] = period_[kPhysics];
	next_[kSysTick] = kMs;
	next_[kImuOdr] = period_[kImuOdr];
}

Simulation::~Simulation() {
	if (fw_fiber_)
		DeleteFiber(fw_fiber_);
	if (g_sim == this)
		g_sim = nullptr;
}

void __stdcall Simulation::fiber_proc(void* arg) {
	auto* self = static_cast<Simulation*>(arg);
	self->fw_main_();
	self->req_ = kDone;
	for (;;)
		SwitchToFiber(self->sim_fiber_);   // never resumed after kDone
}

void Simulation::start(std::function<void()> fw_main) {
	if (!sim_fiber_) {
		sim_fiber_ = ConvertThreadToFiber(nullptr);
		if (!sim_fiber_)
			sim_fiber_ = GetCurrentFiber();
	}
	fw_main_ = std::move(fw_main);
	fw_fiber_ = CreateFiberEx(256 * 1024, 8 * 1024 * 1024, 0, &Simulation::fiber_proc, this);
	fw_alive_ = fw_fiber_ != nullptr;
	t_resume_ = now_;
	fwb_set_vbus(static_cast<float>(robot_.params().vbat));
	fwb_set_phase_adc(0, 32768, 32768);   // zero-current CSA output (mid-scale)
	fwb_set_phase_adc(1, 32768, 32768);
}

bool Simulation::in_fw_main() const { return on_fw_ && GetCurrentFiber() == fw_fiber_; }

void Simulation::fw_yield() {
	if (!in_fw_main())
		return;
	req_ = kYield;
	SwitchToFiber(sim_fiber_);
}

void Simulation::fw_sleep_until(uint64_t t_ns) {
	if (!in_fw_main())
		return;
	req_ = kSleep;
	req_t_ = t_ns;
	SwitchToFiber(sim_fiber_);
}

void Simulation::fw_halt(const char* why) {
	halted_ = true;
	halt_reason_ = why ? why : "halt";
	if (in_fw_main()) {
		req_ = kHalt;
		for (;;)
			SwitchToFiber(sim_fiber_);
	}
}

bool Simulation::i2c_read(uint16_t dev, uint16_t reg, uint8_t* buf, uint16_t len) {
	if (dev != kImuAddr)
		return false;
	return imu_.read(static_cast<uint8_t>(reg), buf, len);
}

bool Simulation::i2c_write(uint16_t dev, uint16_t reg, const uint8_t* buf, uint16_t len) {
	if (dev != kImuAddr)
		return false;
	return imu_.write(static_cast<uint8_t>(reg), buf, len);
}

void Simulation::sync_to_fw() {
	sim_mcu_set_time(now_);
	// Encoders: interpolate the motor angle since the last physics step so the 2 kHz
	// speed loop sees counts at its own sampling instant.
	const double dt = (static_cast<double>(now_) - static_cast<double>(phys_t_)) * 1e-9;
	for (int s = 0; s < 2; s++) {
		double a = robot_.motor_angle_fw(s) + robot_.motor_speed_fw(s) * dt;
		// The firmware runs both motors with enc_dir = -1 (MTR_Setup_And_Start), i.e. the
		// raw LPTIM count runs opposite to its own speed sign.
		double counts = -a / kTwoPi * kEncoderCounts;
		int64_t c = static_cast<int64_t>(std::floor(counts));
		fwb_set_encoder(s, static_cast<uint16_t>(c & 0xFFFF));
	}
}

void Simulation::periodic(int src, bool enabled, uint64_t period) {
	if (enabled && next_[src] == kNever)
		next_[src] = now_ + period;
	else if (!enabled)
		next_[src] = kNever;
}

void Simulation::after_fw() {
	sim_gpio_commit();
	periodic(kTim7, sim_mcu.tim_it[7] != 0, kTim7Period);
	periodic(kTim13, sim_mcu.tim_it[13] != 0, kTim13Period);
	periodic(kTim14, sim_mcu.tim_it[14] != 0, kTim14Period);
	periodic(kLptim3, sim_mcu.lptim_it[3] != 0, kLptim3Period);
	if (sim_mcu.i2c_dma_busy && next_[kI2c] == kNever)
		next_[kI2c] = now_ + kI2cRead2;
	bool ramp = sim_mcu.tim_it[14] != 0;
	if (ramp != ramp_running_) {
		ramp_running_ = ramp;
		if (on_ramp_timer)
			on_ramp_timer(ramp);
	}
}

void Simulation::fire(int src) {
	switch (src) {
	case kPhysics:
		for (int s = 0; s < 2; s++)
			robot_.set_drive(s, fwb_motor_powered(s) != 0, fwb_motor_iq_cmd(s));
		robot_.set_fan_duty(fwb_fan_duty());
		robot_.step();
		ir_.record(robot_);
		imu_.accumulate(robot_.yaw_rate(), robot_.params().timestep);
		phys_t_ = now_;
		next_[src] += period_[src];
		break;
	case kSysTick:
		sync_to_fw();
		fwb_systick();
		after_fw();
		next_[src] += period_[src];
		if (on_millisecond)
			on_millisecond();
		break;
	case kTim7:
		sync_to_fw();
		fwb_tim_update_irq(7);
		after_fw();
		// sensor.c restarts TIM2 (one pulse); its TRGO starts the ADC3 conversion.
		if ((sim_tim_cr1(2) & 1u) && sim_mcu.adc_dma[3] && next_[kAdc3] == kNever)
			next_[kAdc3] = now_ + kTim2Delay + kAdc3Conv;
		next_[src] += period_[src];
		break;
	case kAdc3: {
		next_[src] = kNever;
		sim_tim_clear_cen(2);
		sync_to_fw();
		uint16_t raw = ir_.sample(fwb_sensor_channel(), robot_, track_);
		fwb_adc3_conversion(raw);
		after_fw();
		break;
	}
	case kTim13:
	case kTim14:
		sync_to_fw();
		fwb_tim_update_irq(src == kTim13 ? 13 : 14);
		next_[src] += period_[src];
		after_fw();
		break;
	case kLptim3:
		sync_to_fw();
		fwb_lptim_irq(3);
		next_[src] += period_[src];
		after_fw();
		break;
	case kI2c:
		next_[src] = kNever;
		sync_to_fw();
		fwb_i2c_dma_complete();
		after_fw();
		break;
	case kImuOdr:
		imu_.latch();
		next_[src] += period_[src];
		break;
	default:
		break;
	}
}

Simulation::Stop Simulation::run(double t_end_s, const std::function<bool()>& stop) {
	const uint64_t t_end = static_cast<uint64_t>(t_end_s * 1e9);
	for (;;) {
		if (halted_)
			return Stop::Halted;
		if (stop && stop())
			return Stop::Predicate;

		if (fw_alive_ && t_resume_ <= now_) {
			sync_to_fw();
			req_ = kNone;
			resumes++;
			on_fw_ = true;
			SwitchToFiber(fw_fiber_);
			on_fw_ = false;
			after_fw();
			switch (req_) {
			case kYield: t_resume_ = now_ + quantum_; break;
			case kSleep: t_resume_ = std::max(req_t_, now_ + 1); sleeps++; break;
			case kDone:
				fw_alive_ = false;
				t_resume_ = kNever;
				return Stop::FirmwareDone;
			case kHalt:
				fw_alive_ = false;
				t_resume_ = kNever;
				return Stop::Halted;
			default: t_resume_ = now_ + quantum_; break;
			}
			continue;
		}

		const uint64_t target = std::min(fw_alive_ ? t_resume_ : kNever, t_end);
		for (;;) {
			int src = 0;
			for (int i = 1; i < kSourceCount; i++)
				if (next_[i] < next_[src])
					src = i;
			if (next_[src] > target)
				break;
			now_ = next_[src];
			fired[src]++;
			fire(src);
			if (halted_)
				return Stop::Halted;
		}
		now_ = target;
		if (now_ >= t_end)
			return Stop::TimeLimit;
	}
}
