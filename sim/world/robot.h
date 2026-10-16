// Robot plant: MuJoCo rigid-body model (chassis, two driven wheels, front skids) and the
// two BLDC motors with their gearbox.
#pragma once
#include <array>
#include <string>
#include <vector>

#include "world/track.h"

struct mjModel_;
struct mjData_;
namespace json { struct Value; }

struct MotorParams {
	// maxon ECX SPEED 16 M (values from the firmware's foc.h)
	double kt = 6.46e-3;           // torque constant [Nm/A]
	double r_phase = 0.96;         // phase resistance [ohm]
	double j_rotor = 0.812e-7;     // rotor inertia [kg m^2]
	double gear = 39.0 / 11.0;     // motor turns per wheel turn
	double efficiency = 0.90;      // gearbox
	double friction = 0.25e-3;     // Coulomb friction at the motor shaft [Nm]
	double viscous = 2.0e-8;       // viscous friction at the motor shaft [Nm/(rad/s)]
	double current_tau = 1.3e-4;   // closed current-loop time constant [s] (25 kHz / 20 bandwidth)
	double voltage_ratio = 0.40;   // max |Vq| / Vbus (duty clamp 0..90 %, sinusoidal PWM)
	// First-order lag on the wheel torque for everything the model does not resolve
	// (real FOC current loop, gear backlash, tread wind-up ...). Calibrated against the robot:
	// 2.0 m/s, steer kp 33 / kd 0 oscillates on straights, kp ~16 does not -> about 5 ms.
	double drive_lag = 0.005;      // [s]
};

// Tire-road force model.
//   "mujoco": MuJoCo's own contact friction (rigid tread, Coulomb).
//   "brush":  MuJoCo gives the normal force only; the tread is a bristle layer whose
//             deflection u (longitudinal, lateral) grows with the sliding velocity of the
//             contact and relaxes while rolling over `relaxation_length`:
//                 du/dt = v_slide - |v_roll| / sigma * u,   F = -k u - c du/dt,   |F| <= mu Fz
//             with k = slip_stiffness * Fz / sigma, i.e. in steady rolling F = slip_stiffness * Fz * slip.
//             Soft treads (low Shore hardness) have a lower slip stiffness and a longer
//             relaxation length: the yaw response lags the wheel-speed difference.
struct TireParams {
	std::string model = "brush";
	double mu = 1.6;                  // friction coefficient (sliding limit)
	double slip_stiffness = 8.0;      // [N per N of load per unit slip]
	double relaxation_length = 0.004; // [m]
	double damping_ratio = 0.7;       // of the tread deflection mode
};

struct RobotParams {
	// mass properties of everything except the two wheels (from CAD when available)
	double mass = 0.20;
	std::array<double, 3> com{0.035, 0.0, 0.004};      // in the axle frame (x fwd, y left, z up from axle)
	std::array<double, 3> inertia{1.6e-4, 2.4e-4, 3.6e-4};

	double wheel_radius = 0.0115;  // TIRE_DIAMETER / 2
	double wheel_width = 0.008;
	double track = 0.186;          // THREAD
	double wheel_mass = 0.006;
	double tire_mu = 1.3;

	// sliding pads under the sensor bar (x, y in the axle frame)
	std::vector<std::array<double, 2>> skids{{0.125, 0.05}, {0.125, -0.05}};
	double skid_radius = 0.0025;
	double skid_mu = 0.15;

	// collision box for the body (center x, y, z and half sizes, axle frame)
	std::array<double, 6> body_box{0.05, 0.0, -0.002, 0.08, 0.075, 0.004};

	// From the CAD import (tools/step_import.py); empty = the simple two-wheel model above.
	std::vector<std::array<double, 2>> wheels;            // wheel centers (x, y); same-side wheels are geared together
	std::vector<std::array<double, 6>> collision_boxes;   // replaces body_box
	struct VisualMesh {
		std::string file;     // absolute path
		std::array<float, 4> rgba{0.5f, 0.5f, 0.5f, 1.0f};
	};
	std::vector<VisualMesh> visual_meshes;
	std::vector<std::array<double, 2>> ir_sensors;        // line sensors 0..15 (x, y), left to right

	// IR sensor bar
	double sensor_x = 0.13;            // SENSOR_DIST_L: axle to sensor bar
	double sensor_half_width = 0.08;   // SENSOR_HALF_WIDTH: center to sensor 15
	double marker_sensor_y = 0.09;     // sensors 16/17
	double sensor_sigma = 0.0025;      // optical spot std-dev on the floor [m]
	double ir_white = 3300, ir_black = 350, ir_noise = 6, ir_spread = 0.06;  // 12-bit ADC counts
	double sensor_delay = 0.0;         // the bar sees the floor this long ago [s] (lumped steering-path lag)

	double fan_force = 0.0;        // downforce at 100 % fan duty [N]
	double vbat = 16.8;
	double gyro_noise_dps = 0.08, gyro_bias_dps = 0.4;

	double timestep = 1e-4;        // physics step [s]
	MotorParams motor;
	TireParams tire;

	// `base_dir`: folder of the robot file, for relative mesh paths.
	void load(const json::Value& j, const std::string& base_dir = "");
};

class Robot {
public:
	~Robot();
	bool init(const RobotParams& p, const std::string& mjcf_out_path, std::string* err);
	void place(const Pose2& pose);   // put the robot at rest on the floor

	// Motor inputs (per side, 0 = left, 1 = right), applied from the next step on.
	void set_drive(int side, bool powered, double iq_cmd) {
		powered_[side] = powered;
		iq_cmd_[side] = iq_cmd;
	}
	void set_fan_duty(double d) { fan_duty_ = d; }
	void step();

	double time() const;
	Pose2 pose() const;                    // planar pose of the axle center
	double forward_speed() const;          // along heading [m/s]
	double yaw_rate() const;               // [rad/s], world z
	double wheel_angle(int side) const;    // [rad], + = rolling forward
	double wheel_speed(int side) const;    // [rad/s]
	double iq(int side) const { return iq_[side]; }
	double motor_speed_fw(int side) const; // motor shaft speed in the firmware's sign convention
	double motor_angle_fw(int side) const;
	double pitch() const;
	Vec2 body_to_ground(double bx, double by) const;   // point under (bx, by) of the axle frame
	// body_to_ground as numbers: ground = (f0 + f2*bx + f3*by + f4*bz, f1 + f5*bx + f6*by + f7*bz)
	void ground_frame(double f[8]) const;
	bool chassis_contact() const;          // body box touching the floor
	const RobotParams& params() const { return p_; }
	// for the 3D viewer
	mjModel_* model() const { return m_; }
	mjData_* data() const { return d_; }
	int body_id() const { return body_; }

private:
	std::string build_mjcf() const;

	RobotParams p_;
	mjModel_* m_ = nullptr;
	mjData_* d_ = nullptr;
	int body_ = -1, floor_geom_ = -1;
	std::vector<int> box_geoms_;
	struct Wheel {
		int body = -1, geom = -1;
		double ux = 0, uy = 0;   // tread deflection [m] (brush model)
		double fx = 0, fy = 0, fz = 0;
	};
	std::vector<Wheel> wheels_;
	void tire_forces();
	int wheel_qpos_[2]{}, wheel_dof_[2]{};
	int act_[2]{};
	bool powered_[2]{};
	double iq_cmd_[2]{}, iq_[2]{};
	double torque_[2]{};   // wheel torque after drive_lag
	double fan_duty_ = 0;
	double current_alpha_ = 1.0;
	double lag_alpha_ = 1.0;
};
