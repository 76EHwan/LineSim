#include "world/robot.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>

#include <mujoco/mujoco.h>

#include "core/json.h"

namespace {
void read3(const json::Value& j, const char* key, std::array<double, 3>& out) {
	if (const json::Value* v = j.get(key); v && v->is_arr() && v->size() >= 3)
		for (int i = 0; i < 3; i++)
			out[i] = (*v)[i].n;
}
}  // namespace

void RobotParams::load(const json::Value& j, const std::string& base_dir) {
	mass = j.num("mass", mass);
	read3(j, "com", com);
	read3(j, "inertia", inertia);
	wheel_radius = j.num("wheel_radius", wheel_radius);
	wheel_width = j.num("wheel_width", wheel_width);
	track = j.num("track", track);
	wheel_mass = j.num("wheel_mass", wheel_mass);
	tire_mu = j.num("tire_mu", tire_mu);
	if (const json::Value* t = j.get("tire")) {
		tire.model = t->str("model", tire.model);
		tire.mu = t->num("mu", tire.mu);
		tire.slip_stiffness = t->num("slip_stiffness", tire.slip_stiffness);
		tire.relaxation_length = t->num("relaxation_length", tire.relaxation_length);
		tire.damping_ratio = t->num("damping_ratio", tire.damping_ratio);
	} else {
		tire.model = "mujoco";   // older robot files: MuJoCo friction with tire_mu
	}
	if (const json::Value* s = j.get("skids"); s && s->is_arr()) {
		skids.clear();
		for (const json::Value& e : s->a)
			if (e.is_arr() && e.size() >= 2)
				skids.push_back({e[0].n, e[1].n});
	}
	skid_radius = j.num("skid_radius", skid_radius);
	skid_mu = j.num("skid_mu", skid_mu);
	if (const json::Value* b = j.get("body_box"); b && b->is_arr() && b->size() >= 6)
		for (int i = 0; i < 6; i++)
			body_box[i] = (*b)[i].n;
	auto read_pairs = [&](const char* key, std::vector<std::array<double, 2>>& out) {
		if (const json::Value* v = j.get(key); v && v->is_arr()) {
			out.clear();
			for (const json::Value& e : v->a)
				if (e.is_arr() && e.size() >= 2)
					out.push_back({e[0].n, e[1].n});
		}
	};
	read_pairs("wheels", wheels);
	read_pairs("ir_sensors", ir_sensors);
	if (const json::Value* v = j.get("collision_boxes"); v && v->is_arr()) {
		collision_boxes.clear();
		for (const json::Value& e : v->a)
			if (e.is_arr() && e.size() >= 6)
				collision_boxes.push_back({e[0].n, e[1].n, e[2].n, e[3].n, e[4].n, e[5].n});
	}
	if (const json::Value* v = j.get("visual_meshes"); v && v->is_arr()) {
		visual_meshes.clear();
		for (const json::Value& e : v->a) {
			VisualMesh vm;
			std::string f = e.str("file", "");
			if (f.empty())
				continue;
			bool abs = f.size() > 1 && (f[1] == ':' || f[0] == '/' || f[0] == '\\');
			vm.file = abs || base_dir.empty() ? f : base_dir + "/" + f;
			if (const json::Value* c = e.get("rgba"); c && c->is_arr() && c->size() >= 4)
				for (int i = 0; i < 4; i++)
					vm.rgba[i] = static_cast<float>((*c)[i].n);
			visual_meshes.push_back(vm);
		}
	}
	sensor_x = j.num("sensor_x", sensor_x);
	sensor_half_width = j.num("sensor_half_width", sensor_half_width);
	marker_sensor_y = j.num("marker_sensor_y", marker_sensor_y);
	sensor_sigma = j.num("sensor_sigma", sensor_sigma);
	sensor_delay = j.num("sensor_delay", sensor_delay);
	ir_white = j.num("ir_white", ir_white);
	ir_black = j.num("ir_black", ir_black);
	ir_noise = j.num("ir_noise", ir_noise);
	ir_spread = j.num("ir_spread", ir_spread);
	fan_force = j.num("fan_force", fan_force);
	vbat = j.num("vbat", vbat);
	gyro_noise_dps = j.num("gyro_noise_dps", gyro_noise_dps);
	gyro_bias_dps = j.num("gyro_bias_dps", gyro_bias_dps);
	timestep = j.num("timestep", timestep);
	if (const json::Value* m = j.get("motor")) {
		motor.kt = m->num("kt", motor.kt);
		motor.r_phase = m->num("r_phase", motor.r_phase);
		motor.j_rotor = m->num("j_rotor", motor.j_rotor);
		motor.gear = m->num("gear", motor.gear);
		motor.efficiency = m->num("efficiency", motor.efficiency);
		motor.friction = m->num("friction", motor.friction);
		motor.viscous = m->num("viscous", motor.viscous);
		motor.current_tau = m->num("current_tau", motor.current_tau);
		motor.voltage_ratio = m->num("voltage_ratio", motor.voltage_ratio);
		motor.drive_lag = m->num("drive_lag", motor.drive_lag);
	}
}

Robot::~Robot() {
	if (d_)
		mj_deleteData(d_);
	if (m_)
		mj_deleteModel(m_);
}

std::string Robot::build_mjcf() const {
	const RobotParams& p = p_;
	const MotorParams& mo = p.motor;
	const double R = p.wheel_radius;
	const double G = mo.gear;
	const double iw_spin = 0.5 * p.wheel_mass * R * R;
	const double iw_side = p.wheel_mass * (3 * R * R + p.wheel_width * p.wheel_width) / 12.0;
	const bool cad = !p.visual_meshes.empty();

	// wheels: from the CAD model, or the simple left/right pair on the axle
	std::vector<std::array<double, 2>> wheels = p.wheels;
	if (wheels.empty())
		wheels = {{0.0, 0.5 * p.track}, {0.0, -0.5 * p.track}};
	std::vector<std::array<double, 6>> boxes = p.collision_boxes;
	if (boxes.empty())
		boxes = {p.body_box};

	std::ostringstream x;
	x.precision(9);
	x << "<mujoco model=\"linesim_robot\">\n"
	  << "  <compiler angle=\"radian\" autolimits=\"true\"/>\n"
	  << "  <option timestep=\"" << p.timestep << "\" integrator=\"implicitfast\" cone=\"elliptic\" impratio=\"10\"/>\n"
	  << "  <visual><quality shadowsize=\"4096\"/><map znear=\"0.002\"/><headlight ambient=\"0.35 0.35 0.35\"/></visual>\n"
	  << "  <default><geom solref=\"0.003 1\" solimp=\"0.95 0.99 0.0005\" condim=\"3\"/></default>\n"
	  << "  <asset>\n"
	  << "    <texture name=\"grid\" type=\"2d\" builtin=\"checker\" rgb1=\"0.06 0.06 0.07\" rgb2=\"0.09 0.09 0.1\" width=\"512\" height=\"512\"/>\n"
	  << "    <material name=\"floor\" texture=\"grid\" texrepeat=\"4 4\" texuniform=\"true\"/>\n";
	for (size_t i = 0; i < p.visual_meshes.size(); i++)
		x << "    <mesh name=\"cad" << i << "\" file=\"" << p.visual_meshes[i].file << "\" inertia=\"shell\"/>\n";
	x << "  </asset>\n"
	  << "  <worldbody>\n"
	  << "    <light pos=\"0 0 3\" dir=\"0 0 -1\" diffuse=\"0.6 0.6 0.6\"/>\n"
	  // floor friction is tiny so each robot geom's own coefficient wins (MuJoCo takes the max)
	  << "    <geom name=\"floor\" type=\"plane\" size=\"50 50 0.1\" material=\"floor\" friction=\"0.01 0.0001 0.00001\"/>\n"
	  << "    <body name=\"chassis\" pos=\"0 0 " << R << "\">\n"
	  << "      <freejoint name=\"root\"/>\n"
	  << "      <inertial pos=\"" << p.com[0] << " " << p.com[1] << " " << p.com[2] << "\" mass=\"" << p.mass
	  << "\" diaginertia=\"" << p.inertia[0] << " " << p.inertia[1] << " " << p.inertia[2] << "\"/>\n";
	// parts that can touch the floor (plate, sensor board ...); invisible when the CAD meshes are shown
	for (size_t i = 0; i < boxes.size(); i++) {
		const auto& b = boxes[i];
		x << "      <geom name=\"body" << i << "\" type=\"box\" mass=\"0\" friction=\"0.3 0.005 0.0001\" rgba=\""
		  << (cad ? "0 0 0 0" : "0.15 0.45 0.2 1") << "\" pos=\"" << b[0] << " " << b[1] << " " << b[2] << "\" size=\""
		  << b[3] << " " << b[4] << " " << b[5] << "\"/>\n";
	}
	if (cad) {
		for (size_t i = 0; i < p.visual_meshes.size(); i++) {
			const auto& c = p.visual_meshes[i].rgba;
			x << "      <geom type=\"mesh\" mesh=\"cad" << i << "\" contype=\"0\" conaffinity=\"0\" mass=\"0\" rgba=\"" << c[0]
			  << " " << c[1] << " " << c[2] << " " << c[3] << "\"/>\n";
		}
	} else {
		// simple look: sensor bar with its IR spots and a top deck (visual only)
		x << "      <geom type=\"box\" contype=\"0\" conaffinity=\"0\" mass=\"0\" rgba=\"0.05 0.35 0.15 1\" pos=\""
		  << p.sensor_x << " 0 " << (-R + 0.006) << "\" size=\"0.008 " << p.marker_sensor_y + 0.008 << " 0.0008\"/>\n";
		x << "      <geom type=\"box\" contype=\"0\" conaffinity=\"0\" mass=\"0\" rgba=\"0.1 0.45 0.2 1\" pos=\""
		  << boxes[0][0] << " " << boxes[0][1] << " " << boxes[0][2] + boxes[0][5] + 0.006 << "\" size=\""
		  << boxes[0][3] * 0.8 << " " << p.track / 2 - p.wheel_width << " 0.0008\"/>\n";
	}
	for (size_t i = 0; i < p.skids.size(); i++)
		x << "      <geom name=\"skid" << i << "\" type=\"sphere\" mass=\"0\" size=\"" << p.skid_radius << "\" pos=\""
		  << p.skids[i][0] << " " << p.skids[i][1] << " " << (-R + p.skid_radius) << "\" friction=\"" << p.skid_mu
		  << " 0.005 0.0001\" rgba=\"0.8 0.8 0.8 1\"/>\n";

	// One driven joint per side carries the motor (rotor inertia, friction); the other wheels
	// on that side are geared to it 1:1 (both 39T gears mesh the same 11T pinion).
	std::vector<std::string> coupled[2];
	int count[2] = {0, 0};
	for (const auto& w : wheels) {
		int s = w[1] >= 0 ? 0 : 1;
		std::string name = std::string("wheel_") + (s == 0 ? "l" : "r") + std::to_string(count[s]++);
		bool driven = coupled[s].empty();
		x << "      <body name=\"" << name << "\" pos=\"" << w[0] << " " << w[1] << " 0\">\n"
		  << "        <joint name=\"" << name << "\" type=\"hinge\" axis=\"0 1 0\"";
		if (driven)
			x << " armature=\"" << mo.j_rotor * G * G << "\" damping=\"" << mo.viscous * G * G << "\" frictionloss=\""
			  << mo.friction * G << "\"";
		x << "/>\n"
		  << "        <inertial pos=\"0 0 0\" mass=\"" << p.wheel_mass << "\" diaginertia=\"" << iw_side << " " << iw_spin
		  << " " << iw_side << "\"/>\n"
		  << "        <geom name=\"tire_" << name << "\" type=\"cylinder\" mass=\"0\" size=\"" << R << " "
		  << p.wheel_width / 2 << "\" zaxis=\"0 1 0\" friction=\"" << p.tire_mu << " 0.005 0.0001\""
		  // brush model: frictionless contact here (normal force only), tread forces added in tire_forces()
		  << (p.tire.model == "brush" ? " condim=\"1\" priority=\"1\"" : "") << " rgba=\""
		  << (cad ? "0 0 0 0" : "0.1 0.1 0.1 1") << "\"/>\n"
		  << "      </body>\n";
		coupled[s].push_back(name);
	}
	x << "    </body>\n"
	  << "  </worldbody>\n";
	if (coupled[0].size() > 1 || coupled[1].size() > 1) {
		x << "  <equality>\n";
		for (int s = 0; s < 2; s++)
			for (size_t k = 1; k < coupled[s].size(); k++)
				x << "    <joint joint1=\"" << coupled[s][k] << "\" joint2=\"" << coupled[s][0]
				  << "\" polycoef=\"0 1 0 0 0\" solref=\"0.002 1\"/>\n";
		x << "  </equality>\n";
	}
	x << "  <actuator>\n"
	  << "    <motor name=\"m_l\" joint=\"" << coupled[0].front() << "\"/>\n"
	  << "    <motor name=\"m_r\" joint=\"" << coupled[1].front() << "\"/>\n"
	  << "  </actuator>\n"
	  << "</mujoco>\n";
	return x.str();
}

bool Robot::init(const RobotParams& p, const std::string& mjcf_out_path, std::string* err) {
	p_ = p;
	{
		std::ofstream f(mjcf_out_path, std::ios::binary);
		if (!f) {
			*err = "cannot write " + mjcf_out_path;
			return false;
		}
		f << build_mjcf();
	}
	char e[1024] = "";
	m_ = mj_loadXML(mjcf_out_path.c_str(), nullptr, e, sizeof e);
	if (!m_) {
		*err = std::string("MuJoCo: ") + e;
		return false;
	}
	d_ = mj_makeData(m_);
	body_ = mj_name2id(m_, mjOBJ_BODY, "chassis");
	floor_geom_ = mj_name2id(m_, mjOBJ_GEOM, "floor");
	box_geoms_.clear();
	for (int g = 0; g < m_->ngeom; g++) {
		const char* n = mj_id2name(m_, mjOBJ_GEOM, g);
		if (n && std::strncmp(n, "body", 4) == 0)
			box_geoms_.push_back(g);
	}
	wheels_.clear();
	for (int b = 0; b < m_->nbody; b++) {
		const char* n = mj_id2name(m_, mjOBJ_BODY, b);
		if (!n || std::strncmp(n, "wheel_", 6) != 0)
			continue;
		Wheel w;
		w.body = b;
		w.geom = mj_name2id(m_, mjOBJ_GEOM, (std::string("tire_") + n).c_str());
		wheels_.push_back(w);
	}
	const char* jn[2] = {"wheel_l0", "wheel_r0"};
	const char* an[2] = {"m_l", "m_r"};
	for (int s = 0; s < 2; s++) {
		int jid = mj_name2id(m_, mjOBJ_JOINT, jn[s]);
		wheel_qpos_[s] = m_->jnt_qposadr[jid];
		wheel_dof_[s] = m_->jnt_dofadr[jid];
		act_[s] = mj_name2id(m_, mjOBJ_ACTUATOR, an[s]);
	}
	return true;
}

void Robot::place(const Pose2& pose) {
	mj_resetData(m_, d_);
	d_->qpos[0] = pose.x;
	d_->qpos[1] = pose.y;
	d_->qpos[2] = p_.wheel_radius + 0.0002;
	d_->qpos[3] = std::cos(pose.th / 2);
	d_->qpos[4] = 0;
	d_->qpos[5] = 0;
	d_->qpos[6] = std::sin(pose.th / 2);
	iq_[0] = iq_[1] = 0;
	torque_[0] = torque_[1] = 0;
	for (Wheel& w : wheels_)
		w.ux = w.uy = w.fx = w.fy = w.fz = 0;
	mj_forward(m_, d_);
}

double Robot::motor_speed_fw(int side) const {
	// Firmware convention (motor.c Steer_Motor): left forward = +omega, right forward = -omega.
	double w = d_->qvel[wheel_dof_[side]] * p_.motor.gear;
	return side == 0 ? w : -w;
}

double Robot::motor_angle_fw(int side) const {
	double a = d_->qpos[wheel_qpos_[side]] * p_.motor.gear;
	return side == 0 ? a : -a;
}

void Robot::step() {
	const MotorParams& mo = p_.motor;
	const double lambda = mo.kt / 1.5;   // flux linkage, pole pairs = 1 (foc.h MOTOR_FLUX_LINKAGE)
	const double vmax = mo.voltage_ratio * p_.vbat;
	const double alpha = 1.0 - std::exp(-p_.timestep / mo.current_tau);
	for (int s = 0; s < 2; s++) {
		if (powered_[s]) {
			double e = motor_speed_fw(s) * lambda;
			double hi = (vmax - e) / mo.r_phase;
			double lo = (-vmax - e) / mo.r_phase;
			double target = std::clamp(iq_cmd_[s], std::min(lo, hi), std::max(lo, hi));
			iq_[s] += (target - iq_[s]) * alpha;
		} else {
			iq_[s] = 0;
		}
		double t_fw = mo.kt * iq_[s];
		double t_wheel = t_fw * mo.gear * mo.efficiency;
		if (mo.drive_lag > 0)
			torque_[s] += (t_wheel - torque_[s]) * (1.0 - std::exp(-p_.timestep / mo.drive_lag));
		else
			torque_[s] = t_wheel;
		d_->ctrl[act_[s]] = (s == 0) ? torque_[s] : -torque_[s];
	}
	double* f = d_->xfrc_applied + 6 * body_;
	f[0] = f[1] = 0;
	f[2] = -p_.fan_force * fan_duty_;
	mju_zero(d_->qfrc_applied, static_cast<int>(m_->nv));
	if (p_.tire.model == "brush")
		tire_forces();
	mj_step(m_, d_);
}

void Robot::tire_forces() {
	const TireParams& tp = p_.tire;
	const double dt = p_.timestep;
	const double sigma = std::max(1e-4, tp.relaxation_length);
	// effective mass of the tread mode: wheel + reflected rotor inertia at the contact radius
	const double r = p_.wheel_radius;
	const double m_eff = p_.wheel_mass + p_.motor.j_rotor * p_.motor.gear * p_.motor.gear / (r * r);
	for (Wheel& w : wheels_) {
		// normal force and contact point from the contacts of the last step
		double fz = 0, px = 0, py = 0, pz = 0, nx = 0, ny = 0, nz = 0;
		for (int i = 0; i < d_->ncon; i++) {
			const mjContact& c = d_->contact[i];
			if (c.geom[0] != w.geom && c.geom[1] != w.geom)
				continue;
			mjtNum cf[6];
			mj_contactForce(m_, d_, i, cf);
			if (cf[0] <= 0)
				continue;
			double sgn = (c.geom[0] == w.geom) ? -1.0 : 1.0;   // normal pointing from the floor to the tire
			fz += cf[0];
			px += cf[0] * c.pos[0];
			py += cf[0] * c.pos[1];
			pz += cf[0] * c.pos[2];
			nx += cf[0] * sgn * c.frame[0];
			ny += cf[0] * sgn * c.frame[1];
			nz += cf[0] * sgn * c.frame[2];
		}
		w.fz = fz;
		if (fz <= 1e-6) {   // wheel in the air: tread relaxes
			w.ux = w.uy = w.fx = w.fy = 0;
			continue;
		}
		mjtNum p[3] = {px / fz, py / fz, pz / fz};
		double nn = std::sqrt(nx * nx + ny * ny + nz * nz);
		mjtNum n[3] = {nx / nn, ny / nn, nz / nn};
		// wheel axle (body y axis), forward tangent t = axle x normal, lateral a = n x t
		const mjtNum* xm = d_->xmat + 9 * w.body;
		mjtNum axle[3] = {xm[1], xm[4], xm[7]};
		mjtNum t[3], a[3];
		mju_cross(t, axle, n);
		mju_normalize3(t);
		mju_cross(a, n, t);
		// velocity of the tread material at the contact point (ground is at rest)
		mjtNum v6[6];
		mj_objectVelocity(m_, d_, mjOBJ_XBODY, w.body, v6, 0);
		const mjtNum* c0 = d_->xpos + 3 * w.body;
		mjtNum rr[3] = {p[0] - c0[0], p[1] - c0[1], p[2] - c0[2]};
		mjtNum wxr[3];
		mju_cross(wxr, v6, rr);
		mjtNum vp[3] = {v6[3] + wxr[0], v6[4] + wxr[1], v6[5] + wxr[2]};
		mjtNum vroll = mju_dot3(v6 + 3, t);   // travel speed of the wheel center
		double vsx = mju_dot3(vp, t), vsy = mju_dot3(vp, a);   // sliding velocity of the tread

		// bristle deflection: grows with sliding, relaxes while rolling
		const double k = tp.slip_stiffness * fz / sigma;
		const double c = 2.0 * tp.damping_ratio * std::sqrt(k * m_eff);
		const double relax = std::fabs(vroll) / sigma;
		double dux = vsx - relax * w.ux, duy = vsy - relax * w.uy;
		w.ux += dux * dt;
		w.uy += duy * dt;
		double fx = -k * w.ux - c * dux, fy = -k * w.uy - c * duy;
		const double fmax = tp.mu * fz;
		double fm = std::sqrt(fx * fx + fy * fy);
		if (fm > fmax) {   // sliding: the tread holds the friction limit
			fx *= fmax / fm;
			fy *= fmax / fm;
			w.ux = -fx / k;
			w.uy = -fy / k;
		}
		w.fx = fx;
		w.fy = fy;
		mjtNum force[3] = {fx * t[0] + fy * a[0], fx * t[1] + fy * a[1], fx * t[2] + fy * a[2]};
		mjtNum torque[3] = {0, 0, 0};
		mj_applyFT(m_, d_, force, torque, p, w.body, d_->qfrc_applied);
	}
}

double Robot::time() const { return d_->time; }

Pose2 Robot::pose() const {
	const double* xp = d_->xpos + 3 * body_;
	const double* xm = d_->xmat + 9 * body_;
	return {xp[0], xp[1], std::atan2(xm[3], xm[0])};
}

double Robot::forward_speed() const {
	mjtNum v[6];
	mj_objectVelocity(m_, d_, mjOBJ_XBODY, body_, v, 0);
	const double* xm = d_->xmat + 9 * body_;
	return v[3] * xm[0] + v[4] * xm[3] + v[5] * xm[6];
}

double Robot::yaw_rate() const {
	mjtNum v[6];
	mj_objectVelocity(m_, d_, mjOBJ_XBODY, body_, v, 0);
	return v[2];
}

double Robot::wheel_angle(int side) const { return d_->qpos[wheel_qpos_[side]]; }
double Robot::wheel_speed(int side) const { return d_->qvel[wheel_dof_[side]]; }

double Robot::pitch() const {
	const double* xm = d_->xmat + 9 * body_;
	return std::asin(std::clamp(xm[6], -1.0, 1.0));
}

Vec2 Robot::body_to_ground(double bx, double by) const {
	const double* xp = d_->xpos + 3 * body_;
	const double* xm = d_->xmat + 9 * body_;
	double bz = -p_.wheel_radius;
	return {xp[0] + xm[0] * bx + xm[1] * by + xm[2] * bz, xp[1] + xm[3] * bx + xm[4] * by + xm[5] * bz};
}

void Robot::ground_frame(double f[8]) const {
	const double* xp = d_->xpos + 3 * body_;
	const double* xm = d_->xmat + 9 * body_;
	f[0] = xp[0];
	f[1] = xp[1];
	for (int i = 0; i < 3; i++) {
		f[2 + i] = xm[i];
		f[5 + i] = xm[3 + i];
	}
}

bool Robot::chassis_contact() const {
	for (int i = 0; i < d_->ncon; i++)
		for (int g : box_geoms_)
			if (d_->contact[i].geom[0] == g || d_->contact[i].geom[1] == g)
				return true;
	return false;
}
