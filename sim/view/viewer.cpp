#include "view/viewer.h"

#include <windows.h>
#include <windowsx.h>
#include <GL/gl.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <mujoco/mujoco.h>

namespace {
constexpr int kMaxGeom = 12000;
constexpr size_t kMaxTrail = 6000;
const wchar_t* kClass = L"LineSimViewer";

void box_geom(mjvGeom* g, double cx, double cy, double cz, double hx, double hy, double hz, double yaw,
	const float rgba[4]) {
	mjtNum size[3] = {hx, hy, hz};
	mjtNum pos[3] = {cx, cy, cz};
	double c = std::cos(yaw), s = std::sin(yaw);
	mjtNum mat[9] = {c, -s, 0, s, c, 0, 0, 0, 1};
	mjv_initGeom(g, mjGEOM_BOX, size, pos, mat, rgba);
	g->category = mjCAT_DECOR;
	g->emission = 0.35f;
}

void speed_rgb(double t, float* c) {
	t = std::clamp(t, 0.0, 1.0);
	const float st[4][3] = {{0.16f, 0.24f, 0.8f}, {0.0f, 0.75f, 0.86f}, {0.98f, 0.86f, 0.16f}, {0.9f, 0.16f, 0.12f}};
	double x = t * 3;
	int i = std::min(2, static_cast<int>(x));
	float f = static_cast<float>(x - i);
	for (int k = 0; k < 3; k++)
		c[k] = st[i][k] + (st[i + 1][k] - st[i][k]) * f;
	c[3] = 1;
}
}  // namespace

struct Viewer::Impl {
	mjvScene scn;
	mjrContext con;
	mjvCamera cam;
	mjvCamera inset_cam;
	mjvOption opt;
	std::vector<mjvGeom> track;
};

Viewer::~Viewer() {
	if (im_) {
		mjr_freeContext(&im_->con);
		mjv_freeScene(&im_->scn);
		delete im_;
	}
	if (glrc_) {
		wglMakeCurrent(nullptr, nullptr);
		wglDeleteContext(static_cast<HGLRC>(glrc_));
	}
	if (hwnd_ && !closed_)
		DestroyWindow(static_cast<HWND>(hwnd_));
}

long long __stdcall Viewer::wndproc(void* hwnd, unsigned msg, unsigned long long wp, long long lp) {
	auto* self = reinterpret_cast<Viewer*>(GetWindowLongPtrW(static_cast<HWND>(hwnd), GWLP_USERDATA));
	if (self)
		return self->handle(msg, wp, lp);
	return DefWindowProcW(static_cast<HWND>(hwnd), msg, wp, lp);
}

bool Viewer::open(int width, int height, const char* title, std::string* err) {
	SetProcessDPIAware();
	HINSTANCE inst = GetModuleHandleW(nullptr);
	WNDCLASSW wc{};
	wc.style = CS_OWNDC;
	wc.lpfnWndProc = reinterpret_cast<WNDPROC>(&Viewer::wndproc);
	wc.hInstance = inst;
	wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
	wc.lpszClassName = kClass;
	RegisterClassW(&wc);

	wchar_t wtitle[256];
	MultiByteToWideChar(CP_UTF8, 0, title, -1, wtitle, 256);
	RECT r{0, 0, width, height};
	AdjustWindowRect(&r, WS_OVERLAPPEDWINDOW, FALSE);
	// LINESIM_NOACTIVATE: automated test windows must not take the keyboard from the user
	const bool no_activate = std::getenv("LINESIM_NOACTIVATE") != nullptr;
	HWND hwnd = CreateWindowExW(no_activate ? WS_EX_NOACTIVATE : 0, kClass, wtitle,
		WS_OVERLAPPEDWINDOW | (no_activate ? 0 : WS_VISIBLE), CW_USEDEFAULT, CW_USEDEFAULT, r.right - r.left,
		r.bottom - r.top, nullptr, nullptr, inst, nullptr);
	if (!hwnd) {
		*err = "CreateWindow failed";
		return false;
	}
	if (no_activate)
		ShowWindow(hwnd, SW_SHOWNOACTIVATE);
	hwnd_ = hwnd;
	SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(this));
	HDC dc = GetDC(hwnd);
	hdc_ = dc;

	PIXELFORMATDESCRIPTOR pfd{};
	pfd.nSize = sizeof pfd;
	pfd.nVersion = 1;
	pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
	pfd.iPixelType = PFD_TYPE_RGBA;
	pfd.cColorBits = 32;
	pfd.cDepthBits = 24;
	pfd.cStencilBits = 8;
	int pf = ChoosePixelFormat(dc, &pfd);
	if (!pf || !SetPixelFormat(dc, pf, &pfd)) {
		*err = "no OpenGL pixel format";
		return false;
	}
	HGLRC rc = wglCreateContext(dc);
	if (!rc || !wglMakeCurrent(dc, rc)) {
		*err = "cannot create an OpenGL context";
		return false;
	}
	glrc_ = rc;
	typedef BOOL(WINAPI* PFNWGLSWAPINTERVALEXTPROC)(int interval);
	auto wglSwapIntervalEXT = reinterpret_cast<PFNWGLSWAPINTERVALEXTPROC>(wglGetProcAddress("wglSwapIntervalEXT"));
	if (wglSwapIntervalEXT) {
		wglSwapIntervalEXT(0); // 0 = Disable VSync to decouple GPU buffer swaps from monitor refresh rate
	}
	const char* vendor = reinterpret_cast<const char*>(glGetString(GL_VENDOR));
	const char* renderer = reinterpret_cast<const char*>(glGetString(GL_RENDERER));
	std::fprintf(stderr, "viewer: OpenGL %s / %s (VSync OFF for max tuning throughput)\n", vendor ? vendor : "?", renderer ? renderer : "?");
	RECT cr;
	GetClientRect(hwnd, &cr);
	width_ = cr.right;
	height_ = cr.bottom;
	return true;
}

void Viewer::attach(mjModel_* m, mjData_* d, int follow_body, const Track& track) {
	m_ = m;
	d_ = d;
	follow_body_ = follow_body;
	im_ = new Impl;
	mjv_defaultCamera(&im_->cam);
	mjv_defaultCamera(&im_->inset_cam);
	mjv_defaultOption(&im_->opt);
	mjv_defaultScene(&im_->scn);
	mjr_defaultContext(&im_->con);
	mjv_makeScene(m, &im_->scn, kMaxGeom);
	mjr_makeContext(m, &im_->con, mjFONTSCALE_150);
	im_->scn.flags[mjRND_SHADOW] = 0;   // S toggles; shadows double the cost of a frame
	set_track(track);
	set_cam_mode(kFollow);
}

void Viewer::set_track(const Track& track) {
	// the track: white bands / arc pieces / mark bars a hair above the floor
	im_->track.clear();
	trail_.clear();
	const float white[4] = {0.95f, 0.95f, 0.95f, 1.0f};
	const double z = 0.0003, hz = 0.0003;
	for (const Shape& s : track.shapes()) {
		mjvGeom g;
		if (s.kind == Shape::Band) {
			double yaw = std::atan2(s.u.y, s.u.x);
			box_geom(&g, s.a.x + s.u.x * s.len / 2, s.a.y + s.u.y * s.len / 2, z, s.len / 2, s.hw, hz, yaw, white);
			im_->track.push_back(g);
		} else if (s.kind == Shape::Rect) {
			box_geom(&g, s.a.x, s.a.y, z, s.hl, s.hw, hz, std::atan2(s.u.y, s.u.x), white);
			im_->track.push_back(g);
		} else {
			int n = std::max(4, static_cast<int>(std::fabs(s.sweep) * s.R / 0.01));
			double da = s.sweep / n;
			double half = std::fabs(da) * s.R / 2 * 1.02;   // slight overlap so no gaps show
			for (int k = 0; k < n; k++) {
				double a = s.a0 + da * (k + 0.5);
				double x = s.a.x + s.R * std::cos(a), y = s.a.y + s.R * std::sin(a);
				box_geom(&g, x, y, z, half, s.hw, hz, a + 1.5707963267948966, white);
				im_->track.push_back(g);
			}
		}
	}
	double x0, y0, x1, y1;
	track.bounds(&x0, &y0, &x1, &y1);
	cx_ = 0.5 * (x0 + x1);
	cy_ = 0.5 * (y0 + y1);
	extent_ = std::max(x1 - x0, y1 - y0);
	if (cam_mode_ == kTop)
		set_cam_mode(kTop);
}

// Straight down onto the sensor bar, driving direction pointing up in the image.
void Viewer::sensor_cam(void* camp) const {
	mjvCamera* c = static_cast<mjvCamera*>(camp);
	const mjtNum* xp = d_->xpos + 3 * follow_body_;
	const mjtNum* xm = d_->xmat + 9 * follow_body_;
	c->type = mjCAMERA_FREE;
	c->lookat[0] = xp[0] + xm[0] * sensor_x_;
	c->lookat[1] = xp[1] + xm[3] * sensor_x_;
	c->lookat[2] = 0;
	c->distance = 0.3;
	c->elevation = -89.9;
	c->azimuth = std::atan2(xm[3], xm[0]) * 57.29577951308232;
}

void Viewer::map_cam(void* camp) const {
	mjvCamera* c = static_cast<mjvCamera*>(camp);
	c->type = mjCAMERA_FREE;
	c->lookat[0] = cx_;
	c->lookat[1] = cy_;
	c->lookat[2] = 0;
	c->distance = extent_ * 1.30;
	c->elevation = -89.9;
	c->azimuth = 90;
}

void Viewer::toggle_shadows() {
	if (im_)
		im_->scn.flags[mjRND_SHADOW] = !im_->scn.flags[mjRND_SHADOW];
}

void Viewer::set_cam_mode(int mode) {
	cam_mode_ = mode;
	mjvCamera& c = im_->cam;
	switch (mode) {
	case kFollow:
		c.type = mjCAMERA_TRACKING;
		c.trackbodyid = follow_body_;
		c.distance = 0.6;
		c.elevation = -25;
		follow_az_ = 0;   // from behind, looking along the heading
		break;
	case kTop:
		c.type = mjCAMERA_FREE;
		c.lookat[0] = cx_;
		c.lookat[1] = cy_;
		c.lookat[2] = 0;
		c.distance = extent_ * 1.25;
		c.elevation = -89.9;
		c.azimuth = 90;
		break;
	default:
		c.type = mjCAMERA_FREE;
		c.elevation = -35;
		break;
	}
}

void Viewer::jump_to_next_marker_miss() {
	if (marker_diags_.empty() || !im_)
		return;
	int n = static_cast<int>(marker_diags_.size());
	int start = (focused_miss_idx_ >= 0) ? (focused_miss_idx_ + 1) % n : 0;
	int next_idx = -1;
	// 1. First priority: find next misread marker (status >= 2: missed, wrong type, or ghost)
	for (int step = 0; step < n; ++step) {
		int idx = (start + step) % n;
		if (marker_diags_[idx].status >= 2) {
			next_idx = idx;
			break;
		}
	}
	// 2. If no misreads exist, cycle through any marker
	if (next_idx < 0) {
		next_idx = start;
	}
	focused_miss_idx_ = next_idx;
	if (focused_miss_idx_ >= 0 && focused_miss_idx_ < n) {
		const auto& d = marker_diags_[focused_miss_idx_];
		double tx = d.has_reconstruction ? d.robot_x : d.tape_center.x;
		double ty = d.has_reconstruction ? d.robot_y : d.tape_center.y;
		im_->cam.lookat[0] = tx;
		im_->cam.lookat[1] = ty;
		im_->cam.lookat[2] = 0.02;
		im_->cam.distance = 0.45;
		im_->cam.elevation = -45.0;
		double th_deg = (d.has_reconstruction ? d.robot_th : d.tape_th) * 57.29577951308232;
		im_->cam.azimuth = th_deg - 40.0;
		im_->cam.type = mjCAMERA_FREE;
		cam_mode_ = kFree;
	}
}

bool Viewer::pump() {
	MSG msg;
	while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)) {
		TranslateMessage(&msg);
		DispatchMessageW(&msg);
	}
	return !closed_;
}

bool Viewer::key_held(int vk) const { return vk >= 0 && vk < 256 && held_[vk]; }

int Viewer::pop_key() {
	if (keys_.empty())
		return 0;
	int k = keys_.front();
	keys_.erase(keys_.begin());
	return k;
}

void Viewer::mouse_move(int x, int y) {
	int dx = x - mx_, dy = y - my_;
	mx_ = x;
	my_ = y;
	if (!(btn_l_ || btn_r_ || btn_m_) || !im_ || height_ <= 0)
		return;
	bool shift = (GetKeyState(VK_SHIFT) & 0x8000) != 0;
	int action;
	if (btn_r_)
		action = shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
	else if (btn_l_)
		action = shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
	else
		action = mjMOUSE_ZOOM;
	if (cam_mode_ == kTop && action != mjMOUSE_ZOOM)
		cam_mode_ = kFree;
	if (cam_mode_ == kFollow && (action == mjMOUSE_MOVE_H || action == mjMOUSE_MOVE_V)) {
		// panning leaves follow mode: keep looking at where the robot is now
		for (int i = 0; i < 3; i++)
			im_->cam.lookat[i] = d_->xpos[3 * follow_body_ + i];
		im_->cam.type = mjCAMERA_FREE;
		cam_mode_ = kFree;
	}
	mjv_moveCamera(m_, action, static_cast<mjtNum>(dx) / height_, static_cast<mjtNum>(dy) / height_, &im_->cam);
	if (cam_mode_ == kFollow) {
		const mjtNum* xm = d_->xmat + 9 * follow_body_;
		follow_az_ = im_->cam.azimuth - std::atan2(xm[3], xm[0]) * 57.29577951308232;
	}
}

long long Viewer::handle(unsigned msg, unsigned long long wp, long long lp) {
	HWND hwnd = static_cast<HWND>(hwnd_);
	switch (msg) {
	case WM_CLOSE:
		closed_ = true;
		DestroyWindow(hwnd);
		return 0;
	case WM_SIZE:
		width_ = LOWORD(lp);
		height_ = HIWORD(lp);
		return 0;
	case WM_KEYDOWN:
	case WM_SYSKEYDOWN:
		if (wp < 256) {
			if (!held_[wp] || !(lp & (1LL << 30)))
				keys_.push_back(static_cast<int>(wp));
			held_[wp] = true;
		}
		return 0;
	case WM_KEYUP:
	case WM_SYSKEYUP:
		if (wp < 256)
			held_[wp] = false;
		return 0;
	case WM_KILLFOCUS:
		std::memset(held_, 0, sizeof held_);
		btn_l_ = btn_r_ = btn_m_ = false;
		return 0;
	case WM_LBUTTONDOWN: btn_l_ = true; SetCapture(hwnd); mx_ = GET_X_LPARAM(lp); my_ = GET_Y_LPARAM(lp); return 0;
	case WM_RBUTTONDOWN: btn_r_ = true; SetCapture(hwnd); mx_ = GET_X_LPARAM(lp); my_ = GET_Y_LPARAM(lp); return 0;
	case WM_MBUTTONDOWN: btn_m_ = true; SetCapture(hwnd); mx_ = GET_X_LPARAM(lp); my_ = GET_Y_LPARAM(lp); return 0;
	case WM_LBUTTONUP: btn_l_ = false; ReleaseCapture(); return 0;
	case WM_RBUTTONUP: btn_r_ = false; ReleaseCapture(); return 0;
	case WM_MBUTTONUP: btn_m_ = false; ReleaseCapture(); return 0;
	case WM_MOUSEMOVE:
		mouse_move(GET_X_LPARAM(lp), GET_Y_LPARAM(lp));
		return 0;
	case WM_MOUSEWHEEL:
		if (im_)
			mjv_moveCamera(m_, mjMOUSE_ZOOM, 0, -0.05 * GET_WHEEL_DELTA_WPARAM(wp) / WHEEL_DELTA, &im_->cam);
		return 0;
	default:
		return DefWindowProcW(hwnd, msg, wp, lp);
	}
}

void Viewer::add_trail(double x, double y, double speed, double beta, double lat_mm) {
	if (trail_.size() >= kMaxTrail)
		trail_.erase(trail_.begin(), trail_.begin() + kMaxTrail / 10);
	trail_.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(speed), static_cast<float>(beta), static_cast<float>(lat_mm)});
	trail_vmax_ = std::max(trail_vmax_, speed);
}

void Viewer::overlay(int grid, int left, int bottom, int width, int height, const std::string& text) {
	// whole lines, each piece under the mjr_overlay limit
	std::vector<std::string> pieces(1);
	std::vector<int> rows(1, 0);
	size_t p = 0;
	while (p <= text.size()) {
		size_t q = text.find('\n', p);
		if (q == std::string::npos)
			q = text.size();
		std::string line = text.substr(p, q - p);
		if (line.size() > mjMAXOVERLAY - 2)
			line.resize(mjMAXOVERLAY - 2);
		if (!pieces.back().empty() && pieces.back().size() + 1 + line.size() > mjMAXOVERLAY - 2) {
			pieces.emplace_back();
			rows.push_back(0);
		}
		if (rows.back() > 0)
			pieces.back() += '\n';
		pieces.back() += line;
		rows.back()++;
		p = q + 1;
	}
	// line pitch of mjr_overlay: about 1.17 character heights (measured on screen: 33 px at 28 px)
	const int lh = static_cast<int>(im_->con.charHeight * 1.17 + 0.5);
	const bool from_bottom = grid == mjGRID_BOTTOMLEFT || grid == mjGRID_BOTTOMRIGHT;
	const bool is_right = grid == mjGRID_TOPRIGHT || grid == mjGRID_BOTTOMRIGHT;

	auto measure_line_width = [&](const std::string& s) {
		int max_w = 0, cur_w = 0;
		for (char c : s) {
			if (c == '\n') {
				max_w = std::max(max_w, cur_w);
				cur_w = 0;
			} else if (c == '\r') {
				continue;
			} else if (static_cast<unsigned char>(c) < 127) {
				cur_w += im_->con.charWidth[static_cast<unsigned char>(c)];
			}
		}
		return std::max(max_w, cur_w);
	};

	if (is_right) {
		const int margin = im_->con.charHeight;

		// 1. Measure the global maximum line width across ALL pieces
		int max_w = 0;
		for (const auto& piece : pieces) {
			max_w = std::max(max_w, measure_line_width(piece));
		}

		// 2. Single unified left position and unified width across all pieces docked to right edge
		const int unified_left = std::max(left, left + width - max_w - 2 * margin);
		const int unified_width = max_w + 2 * margin;
		const int piece_grid = from_bottom ? mjGRID_BOTTOMLEFT : mjGRID_TOPLEFT;

		int off = 0;
		for (size_t k = 0; k < pieces.size(); k++) {
			const size_t i = from_bottom ? pieces.size() - 1 - k : k;
			mjrRect r{unified_left, from_bottom ? bottom + off : bottom, unified_width, height - off};
			if (r.height > lh)
				mjr_overlay(mjFONT_NORMAL, piece_grid, r, pieces[i].c_str(), nullptr, &im_->con);
			off += rows[i] * lh;
		}
		return;
	}


	// Normal (left-aligned) overlay
	int off = 0;
	for (size_t k = 0; k < pieces.size(); k++) {
		const size_t i = from_bottom ? pieces.size() - 1 - k : k;   // stack away from the anchor corner
		mjrRect r{left, from_bottom ? bottom + off : bottom, width, height - off};
		if (r.height > lh)
			mjr_overlay(mjFONT_NORMAL, grid, r, pieces[i].c_str(), nullptr, &im_->con);
		off += rows[i] * lh;
	}
}

void Viewer::render(const ViewerHud& hud) {
	if (closed_ || !im_ || width_ <= 0 || height_ <= 0)
		return;
	mjvScene& scn = im_->scn;
	if (cam_mode_ == kSensor)
		sensor_cam(&im_->cam);
	if (cam_mode_ == kFollow) {   // chase camera: stay behind the robot as it turns
		const mjtNum* xm = d_->xmat + 9 * follow_body_;
		im_->cam.azimuth = std::atan2(xm[3], xm[0]) * 57.29577951308232 + follow_az_;
	}
	mjv_updateScene(m_, d_, &im_->opt, nullptr, &im_->cam, mjCAT_ALL, &scn);

	for (const mjvGeom& g : im_->track) {
		if (scn.ngeom >= scn.maxgeom)
			break;
		scn.geoms[scn.ngeom++] = g;
	}
	for (size_t i = 1; i < trail_.size() && scn.ngeom < scn.maxgeom; i++) {
		const TrailPt& a = trail_[i - 1];
		const TrailPt& b = trail_[i];
		if (std::hypot(b.x - a.x, b.y - a.y) > 0.3)
			continue;   // robot was put back on the start line
		float c[4];
		const bool is_off_limit = (std::fabs(b.lat_mm) > 70.0f);
		const bool is_oversteer = (b.beta >= 25.0f);

		int line_width = 3;
		float emission = 0.8f;
		if (is_off_limit && is_oversteer) {
			// Red: Oversteer causing track limit violation!
			c[0] = 1.0f; c[1] = 0.05f; c[2] = 0.15f; c[3] = 1.0f;
			line_width = 6;
			emission = 1.0f;
		} else if (is_off_limit) {
			// Magenta: Line departure / Track limit violation (> 70mm)
			c[0] = 1.0f; c[1] = 0.10f; c[2] = 0.85f; c[3] = 1.0f;
			line_width = 5;
			emission = 1.0f;
		} else if (is_oversteer) {
			// Orange/Gold: Controlled drift / oversteer within limits
			c[0] = 1.0f; c[1] = (b.beta >= 32.0f) ? 0.35f : 0.65f; c[2] = 0.05f; c[3] = 1.0f;
			line_width = 5;
			emission = 1.0f;
		} else {
			speed_rgb(b.v / trail_vmax_, c);
		}
		mjvGeom* g = &scn.geoms[scn.ngeom++];
		mjv_initGeom(g, mjGEOM_LINE, nullptr, nullptr, nullptr, c);
		mjtNum p0[3] = {a.x, a.y, 0.0012}, p1[3] = {b.x, b.y, 0.0012};
		mjv_connector(g, mjGEOM_LINE, line_width, p0, p1);
		g->category = mjCAT_DECOR;
		g->emission = emission;
	}

	// Floor indicators for drift and off-track events
	for (size_t i = 1; i < trail_.size() && scn.ngeom < scn.maxgeom; i += 4) {
		const TrailPt& b = trail_[i];
		const bool is_off_limit = (std::fabs(b.lat_mm) > 70.0f);
		const bool is_oversteer = (b.beta >= 25.0f);
		if (is_off_limit || is_oversteer) {
			mjtNum pos[3] = {b.x, b.y, 0.0014};
			double r = (is_off_limit && is_oversteer) ? 0.016 : 0.011;
			mjtNum size[3] = {r, r, 0.0004};
			float rgba[4];
			if (is_off_limit && is_oversteer) {
				rgba[0] = 1.0f; rgba[1] = 0.05f; rgba[2] = 0.15f; rgba[3] = 0.90f; // Red
			} else if (is_off_limit) {
				rgba[0] = 1.0f; rgba[1] = 0.10f; rgba[2] = 0.85f; rgba[3] = 0.90f; // Magenta
			} else {
				rgba[0] = 1.0f; rgba[1] = (b.beta >= 32.0f) ? 0.35f : 0.65f; rgba[2] = 0.05f; rgba[3] = 0.85f; // Orange
			}
			mjvGeom* g = &scn.geoms[scn.ngeom++];
			mjv_initGeom(g, mjGEOM_CYLINDER, size, pos, nullptr, rgba);
			g->category = mjCAT_DECOR;
			g->emission = 1.0f;
		}
	}

	// IR sensor spots on the floor under the bar (seen in the sensor view)
	{
		const mjtNum* xp = d_->xpos + 3 * follow_body_;
		const mjtNum* xm = d_->xmat + 9 * follow_body_;
		for (size_t i = 0; i < sensor_pts_.size() && scn.ngeom < scn.maxgeom; i++) {
			double bx = sensor_pts_[i][0], by = sensor_pts_[i][1];
			mjtNum pos[3] = {xp[0] + xm[0] * bx + xm[1] * by, xp[1] + xm[3] * bx + xm[4] * by, 0.0012};
			mjtNum size[3] = {0.0025, 0.0025, 0.0004};
			const bool on = i < 18 && hud.sensor_on[i];
			const float rgba[4] = {on ? 1.0f : 0.85f, on ? 0.85f : 0.1f, on ? 0.1f : 0.1f, 1.0f};
			mjvGeom* g = &scn.geoms[scn.ngeom++];
			mjv_initGeom(g, mjGEOM_CYLINDER, size, pos, nullptr, rgba);
			g->category = mjCAT_DECOR;
			g->emission = 1;
		}
	}

	// Robot position marker on floor (hidden by default)
	static const bool show_floor_disc = (std::getenv("LINESIM_SHOW_DISC") != nullptr);
	if (show_floor_disc && scn.ngeom < scn.maxgeom) {
		const mjtNum* xp = d_->xpos + 3 * follow_body_;
		mjtNum pos[3] = {xp[0], xp[1], 0.0020};
		mjtNum size[3] = {0.035, 0.035, 0.0006};
		float rgba[4] = {0.2f, 0.95f, 0.35f, 0.95f}; // Green: Normal
		const bool is_off_limit = (std::fabs(hud.live_lat_mm) > 70.0);
		const bool is_oversteer = (hud.live_beta >= 25.0);
		if (is_off_limit && is_oversteer) {
			rgba[0] = 1.0f; rgba[1] = 0.05f; rgba[2] = 0.15f; rgba[3] = 1.0f; // Red: Oversteer Off
		} else if (is_off_limit) {
			rgba[0] = 1.0f; rgba[1] = 0.10f; rgba[2] = 0.85f; rgba[3] = 1.0f; // Magenta: Line Out
		} else if (is_oversteer) {
			rgba[0] = 1.0f; rgba[1] = 0.65f; rgba[2] = 0.10f; rgba[3] = 1.0f; // Orange: Drift
		}
		mjvGeom* g = &scn.geoms[scn.ngeom++];
		mjv_initGeom(g, mjGEOM_CYLINDER, size, pos, nullptr, rgba);
		g->category = mjCAT_DECOR;
		g->emission = 1.0f;
	}

	// Marker visual diagnostics: tape boxes on floor
	for (size_t i = 0; i < marker_diags_.size() && scn.ngeom < scn.maxgeom; ++i) {
		const auto& m = marker_diags_[i];
		float c[4];
		float emission = 0.4f;
		if (m.status == 1) { // OK
			c[0] = 0.15f; c[1] = 0.85f; c[2] = 0.25f; c[3] = 0.75f; // Green
		} else if (m.status == 2) { // MISSED (unread)
			c[0] = 1.0f; c[1] = 0.15f; c[2] = 0.15f; c[3] = 0.95f; // Red
			emission = 0.9f;
		} else if (m.status == 3) { // WRONG_TYPE
			c[0] = 1.0f; c[1] = 0.65f; c[2] = 0.05f; c[3] = 0.95f; // Amber
			emission = 0.9f;
		} else if (m.status == 4) { // GHOST
			c[0] = 0.95f; c[1] = 0.15f; c[2] = 0.95f; c[3] = 0.95f; // Magenta
			emission = 0.9f;
		} else {
			c[0] = 0.5f; c[1] = 0.5f; c[2] = 0.5f; c[3] = 0.5f;
		}

		mjvGeom* g = &scn.geoms[scn.ngeom++];
		box_geom(g, m.tape_center.x, m.tape_center.y, 0.0007, m.tape_hl, m.tape_hw, 0.0006, m.tape_th, c);
		g->emission = emission;

		// Highlight currently focused marker with an outer halo
		if ((int)i == focused_miss_idx_ && scn.ngeom < scn.maxgeom) {
			const float halo[4] = {1.0f, 1.0f, 0.2f, 0.9f};
			mjvGeom* h = &scn.geoms[scn.ngeom++];
			box_geom(h, m.tape_center.x, m.tape_center.y, 0.0009, m.tape_hl + 0.006, m.tape_hw + 0.006, 0.0003, m.tape_th, halo);
			h->emission = 1.0f;
		}
	}

	// Reconstructed Robot Crime Scene Visualization
	if (focused_miss_idx_ >= 0 && focused_miss_idx_ < (int)marker_diags_.size()) {
		const auto& d = marker_diags_[focused_miss_idx_];
		if (d.has_reconstruction) {
			const double c_th = std::cos(d.robot_th);
			const double s_th = std::sin(d.robot_th);

			// 1. Robot ghost chassis box
			if (scn.ngeom < scn.maxgeom) {
				float ghost_c[4];
				if (d.status == 2) { // Missed: translucent red
					ghost_c[0] = 1.0f; ghost_c[1] = 0.25f; ghost_c[2] = 0.25f; ghost_c[3] = 0.40f;
				} else if (d.status == 3) { // Wrong type: translucent amber
					ghost_c[0] = 1.0f; ghost_c[1] = 0.65f; ghost_c[2] = 0.15f; ghost_c[3] = 0.40f;
				} else if (d.status == 4) { // Ghost: translucent magenta
					ghost_c[0] = 0.95f; ghost_c[1] = 0.20f; ghost_c[2] = 0.95f; ghost_c[3] = 0.40f;
				} else {
					ghost_c[0] = 0.25f; ghost_c[1] = 0.75f; ghost_c[2] = 1.0f; ghost_c[3] = 0.40f;
				}
				mjvGeom* g = &scn.geoms[scn.ngeom++];
				box_geom(g, d.robot_x, d.robot_y, 0.014, 0.055, 0.038, 0.010, d.robot_th, ghost_c);
				g->emission = 0.7f;
			}

			// 2. Ghost wheels
			if (scn.ngeom + 1 < scn.maxgeom) {
				const float wheel_c[4] = {0.15f, 0.15f, 0.15f, 0.60f};
				// Left wheel
				mjvGeom* gw_l = &scn.geoms[scn.ngeom++];
				double wx_l = d.robot_x - 0.038 * s_th;
				double wy_l = d.robot_y + 0.038 * c_th;
				box_geom(gw_l, wx_l, wy_l, 0.013, 0.015, 0.005, 0.013, d.robot_th, wheel_c);
				gw_l->emission = 0.4f;

				// Right wheel
				mjvGeom* gw_r = &scn.geoms[scn.ngeom++];
				double wx_r = d.robot_x + 0.038 * s_th;
				double wy_r = d.robot_y - 0.038 * c_th;
				box_geom(gw_r, wx_r, wy_r, 0.013, 0.015, 0.005, 0.013, d.robot_th, wheel_c);
				gw_r->emission = 0.4f;
			}

			// 3. Ghost sensor bar
			if (scn.ngeom < scn.maxgeom) {
				const float bar_c[4] = {0.2f, 0.9f, 0.9f, 0.75f};
				double bar_x = d.robot_x + sensor_x_ * c_th;
				double bar_y = d.robot_y + sensor_x_ * s_th;
				mjvGeom* gb = &scn.geoms[scn.ngeom++];
				box_geom(gb, bar_x, bar_y, 0.003, 0.003, 0.045, 0.002, d.robot_th, bar_c);
				gb->emission = 0.8f;
			}

			// 3b. 18 ghost sensor spots on floor under the ghost bar
			for (size_t k = 0; k < sensor_pts_.size() && scn.ngeom < scn.maxgeom; ++k) {
				double bx = sensor_pts_[k][0], by = sensor_pts_[k][1];
				mjtNum gpos[3] = {d.robot_x + c_th * bx - s_th * by,
				                  d.robot_y + s_th * bx + c_th * by, 0.0020};
				mjtNum gsz[3] = {0.0025, 0.0025, 0.0005};
				const bool son = (k < 18) ? d.sensor_on[k] : false;
				float sv = (k < 18) ? d.sensors[k] : 0.0f;
				float srgba[4];
				if (k == 16 || k == 17) {
					if (son) {
						srgba[0] = 0.2f; srgba[1] = 1.0f; srgba[2] = 0.3f; srgba[3] = 1.0f; // Green: marker hit
					} else {
						srgba[0] = 0.85f; srgba[1] = 0.15f; srgba[2] = 0.15f; srgba[3] = 0.8f; // Red: marker missed
					}
				} else {
					if (son) {
						srgba[0] = 1.0f; srgba[1] = 0.85f; srgba[2] = 0.1f; srgba[3] = 1.0f; // Yellow: line detected
					} else {
						srgba[0] = 0.15f + 0.6f * sv; srgba[1] = 0.15f + 0.6f * sv; srgba[2] = 0.2f + 0.6f * sv; srgba[3] = 0.7f;
					}
				}
				mjvGeom* gs = &scn.geoms[scn.ngeom++];
				mjv_initGeom(gs, mjGEOM_CYLINDER, gsz, gpos, nullptr, srgba);
				gs->category = mjCAT_DECOR;
				gs->emission = son ? 1.0f : 0.2f;
			}

			// 4. Misread Sensor Spot (glowing cylinder on floor)
			if (scn.ngeom < scn.maxgeom) {
				mjtNum spos[3] = {d.sensor_pos.x, d.sensor_pos.y, 0.0022};
				mjtNum ssize[3] = {0.004, 0.004, 0.0012};
				float sc[4];
				if (d.miss_gap_mm > 0.0) {
					sc[0] = 1.0f; sc[1] = 0.1f; sc[2] = 0.1f; sc[3] = 1.0f; // Red: off tape
				} else {
					sc[0] = 0.2f; sc[1] = 1.0f; sc[2] = 0.3f; sc[3] = 1.0f; // Green: on tape
				}
				mjvGeom* gs = &scn.geoms[scn.ngeom++];
				mjv_initGeom(gs, mjGEOM_CYLINDER, ssize, spos, nullptr, sc);
				gs->category = mjCAT_DECOR;
				gs->emission = 1.0f;
			}

			// 5. Tape Edge Target Point (glowing cylinder on floor)
			if (scn.ngeom < scn.maxgeom) {
				mjtNum epos[3] = {d.tape_edge_pos.x, d.tape_edge_pos.y, 0.0022};
				mjtNum esize[3] = {0.004, 0.004, 0.0012};
				const float ec[4] = {1.0f, 0.9f, 0.1f, 1.0f}; // Yellow: tape edge target
				mjvGeom* ge = &scn.geoms[scn.ngeom++];
				mjv_initGeom(ge, mjGEOM_CYLINDER, esize, epos, nullptr, ec);
				ge->category = mjCAT_DECOR;
				ge->emission = 1.0f;
			}

			// 6. Miss Gap Vector Line (connector between sensor and tape edge)
			if (d.miss_gap_mm > 0.1 && scn.ngeom < scn.maxgeom) {
				const float lc[4] = {1.0f, 0.15f, 0.15f, 1.0f};
				mjvGeom* gl = &scn.geoms[scn.ngeom++];
				mjv_initGeom(gl, mjGEOM_LINE, nullptr, nullptr, nullptr, lc);
				mjtNum p0[3] = {d.sensor_pos.x, d.sensor_pos.y, 0.0025};
				mjtNum p1[3] = {d.tape_edge_pos.x, d.tape_edge_pos.y, 0.0025};
				mjv_connector(gl, mjGEOM_LINE, 5, p0, p1);
				gl->category = mjCAT_DECOR;
				gl->emission = 1.0f;
			}

			// 7. Velocity / Slip vector arrow (showing robot motion direction vs heading)
			if (scn.ngeom < scn.maxgeom) {
				double beta_rad = d.robot_beta * 0.017453292519943295;
				double v_ang = d.robot_th + beta_rad;
				double arrow_len = std::clamp(d.robot_v * 0.05, 0.06, 0.15);
				const float vc[4] = {1.0f, 0.55f, 0.05f, 0.95f}; // Orange
				mjvGeom* gv = &scn.geoms[scn.ngeom++];
				mjv_initGeom(gv, mjGEOM_LINE, nullptr, nullptr, nullptr, vc);
				mjtNum vp0[3] = {d.robot_x, d.robot_y, 0.014};
				mjtNum vp1[3] = {d.robot_x + arrow_len * std::cos(v_ang), d.robot_y + arrow_len * std::sin(v_ang), 0.014};
				mjv_connector(gv, mjGEOM_LINE, 4, vp0, vp1);
				gv->category = mjCAT_DECOR;
				gv->emission = 1.0f;
			}
		}
	}

	mjrRect view{0, 0, width_, height_};
	mjr_render(view, &scn, &im_->con);

	if (inset_mode_ != kInsetOff && cam_mode_ != kSensor && cam_mode_ != kTop) {
		const int iw = std::max(220, width_ * 28 / 100), ih = iw * 3 / 4;
		mjrRect r{width_ - iw - 12, 12, iw, ih};
		mjr_rectangle({r.left - 3, r.bottom - 3, r.width + 6, r.height + 6}, 0.25f, 0.25f, 0.3f, 1);
		if (inset_mode_ == kInsetMap) {
			map_cam(&im_->inset_cam);
			mjv_updateCamera(m_, d_, &im_->inset_cam, &scn);
			mjr_render(r, &scn, &im_->con);
			mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, r, "track map (M: switch view)", nullptr, &im_->con);
		} else {
			sensor_cam(&im_->inset_cam);
			mjv_updateCamera(m_, d_, &im_->inset_cam, &scn);
			// see through the robot: the sensor board would hide the line
			std::vector<float> alpha(scn.ngeom);
			for (int i = 0; i < scn.ngeom; i++) {
				mjvGeom& g = scn.geoms[i];
				alpha[i] = g.rgba[3];
				if (g.objtype == mjOBJ_GEOM && g.objid >= 0 && m_->geom_bodyid[g.objid] != 0)
					g.rgba[3] = std::min(g.rgba[3], 0.12f);
			}
			mjr_render(r, &scn, &im_->con);
			for (int i = 0; i < scn.ngeom; i++)
				scn.geoms[i].rgba[3] = alpha[i];
			mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, r, "sensor view (V)", nullptr, &im_->con);
		}
	}

	overlay(mjGRID_TOPLEFT, view.left, view.bottom, view.width, view.height, hud.status);
	std::string lcd;
	if (!hud.lcd.empty()) {
		lcd = hud.lcd_inverted ? "LCD (inverted)\n" : "LCD\n";
		for (const std::string& row : hud.lcd)
			lcd += row + "\n";
	}
	// with the session panel on the right, the LCD sits on the left just above the help lines
	// (the status block above it carries the lap list)
	const bool lcd_left = !lcd.empty() && !hud.panel.empty();
	std::string bottom = (lcd_left ? lcd + "\n" : "") + hud.help;
	if (!bottom.empty())
		overlay(mjGRID_BOTTOMLEFT, view.left, view.bottom, view.width, view.height, bottom);
	if (!hud.panel.empty())
		overlay(mjGRID_TOPRIGHT, view.left, view.bottom, view.width, view.height, hud.panel);
	if (!lcd.empty() && !lcd_left)
		overlay(mjGRID_TOPRIGHT, view.left, view.bottom, view.width, view.height, lcd);

	// HUD overlay card for misread marker crime scene reconstruction
	if (focused_miss_idx_ >= 0 && focused_miss_idx_ < (int)marker_diags_.size()) {
		const auto& d = marker_diags_[focused_miss_idx_];
		char line_bits[17];
		for (int k = 0; k < 16; ++k)
			line_bits[k] = d.sensor_on[k] ? '#' : '.';
		line_bits[16] = '\0';

		char card[640];
		std::snprintf(card, sizeof card,
			"[MARKER RECONSTRUCTION: #%d '%c' -> '%c' (%s)]\n"
			"Pose: pos=(%+.3f, %+.3f)m | yaw=%+.1f deg | v=%.2fm/s\n"
			"Drift: lat_err=%+.1f mm | sideslip beta=%+.1f deg\n"
			"Sensors: M_L=%d%% (%s) | Lines=[%s] | M_R=%d%% (%s)\n"
			"Gap to Tape: %.1f mm (%s) | Focus: %s\n"
			"Reason: %s\n"
			"Press [K] to jump / cycle misread markers",
			d.marker_idx, d.want, d.got, d.label.c_str(),
			d.robot_x, d.robot_y, d.robot_th * 57.29577951308232, d.robot_v,
			d.robot_lat_mm, d.robot_beta,
			(int)(d.sensors[16] * 100.0f + 0.5f), d.sensor_on[16] ? "ON" : "OFF",
			line_bits,
			(int)(d.sensors[17] * 100.0f + 0.5f), d.sensor_on[17] ? "ON" : "OFF",
			d.miss_gap_mm, (d.miss_gap_mm > 0.0 ? "MISSED" : "HIT"), d.sensor_name.c_str(),
			d.reason.c_str());

		overlay(mjGRID_BOTTOMRIGHT, view.left, view.bottom, view.width, view.height, card);
	}

	// IR bar: 16 line sensors left->right as mounted (sensor 0 = left), then the two mark sensors
	const bool show_miss_sensors = (focused_miss_idx_ >= 0 && focused_miss_idx_ < (int)marker_diags_.size());
	const MarkerVisualInfo* miss_m = show_miss_sensors ? &marker_diags_[focused_miss_idx_] : nullptr;

	const int cell = std::max(14, width_ / 60), gap = 3, h = cell * 2;
	const int n = 18, total = n * (cell + gap) + 2 * cell;
	int x = (width_ - total) / 2, y = height_ - h - 16;   // top center

	if (miss_m) {
		mjr_rectangle({x - 12, y - 24, total + 24, h + 34}, 0.08f, 0.04f, 0.05f, 0.88f);
		char top_title[160];
		std::snprintf(top_title, sizeof top_title,
			"[MISREAD MOMENT SENSORS #%d '%c'] M_L:%d%% (%s) | M_R:%d%% (%s)",
			miss_m->marker_idx, miss_m->want,
			(int)(miss_m->sensors[16] * 100.0f + 0.5f), miss_m->sensor_on[16] ? "ON" : "OFF",
			(int)(miss_m->sensors[17] * 100.0f + 0.5f), miss_m->sensor_on[17] ? "ON" : "OFF");
		mjrRect title_r{x - 10, y - 22, total + 20, 20};
		mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, title_r, top_title, nullptr, &im_->con);
	} else {
		mjr_rectangle({x - 8, y - 8, total + 16, h + 16}, 0.05f, 0.05f, 0.06f, 0.75f);
	}

	auto draw_cell = [&](int i, int px) {
		float v = miss_m ? miss_m->sensors[i] : hud.sensors[i];
		bool on = miss_m ? miss_m->sensor_on[i] : hud.sensor_on[i];
		if (on) {
			if (miss_m && (i == 16 || i == 17))
				mjr_rectangle({px - 2, y - 2, cell + 4, h + 4}, 0.2f, 1.0f, 0.3f, 1);
			else
				mjr_rectangle({px - 2, y - 2, cell + 4, h + 4}, 0.95f, 0.25f, 0.2f, 1);
		}
		mjr_rectangle({px, y, cell, h}, v, v, v, 1);
	};
	draw_cell(16, x);
	x += cell + gap + cell;
	for (int i = 0; i < 16; i++, x += cell + gap)
		draw_cell(i, x);
	x += cell;
	draw_cell(17, x);

	if (!shot_path_.empty()) {
		std::vector<unsigned char> rgb(static_cast<size_t>(width_) * height_ * 3);
		mjr_readPixels(rgb.data(), nullptr, view, &im_->con);
		FILE* f = std::fopen(shot_path_.c_str(), "wb");
		if (f) {
			const int stride = (width_ * 3 + 3) & ~3;
			const uint32_t data = static_cast<uint32_t>(stride) * height_;
			unsigned char hdr[54] = {'B', 'M'};
			auto put32 = [&](int at, uint32_t v) {
				for (int k = 0; k < 4; k++)
					hdr[at + k] = static_cast<unsigned char>(v >> (8 * k));
			};
			put32(2, 54 + data);
			put32(10, 54);
			put32(14, 40);
			put32(18, static_cast<uint32_t>(width_));
			put32(22, static_cast<uint32_t>(height_));   // bottom-up, same as OpenGL
			hdr[26] = 1;
			hdr[28] = 24;
			put32(34, data);
			std::fwrite(hdr, 1, 54, f);
			std::vector<unsigned char> row(stride, 0);
			for (int y = 0; y < height_; y++) {
				for (int x = 0; x < width_; x++) {
					const unsigned char* px = &rgb[(static_cast<size_t>(y) * width_ + x) * 3];
					row[x * 3 + 0] = px[2];
					row[x * 3 + 1] = px[1];
					row[x * 3 + 2] = px[0];
				}
				std::fwrite(row.data(), 1, stride, f);
			}
			std::fclose(f);
		}
		shot_path_.clear();
	}
	SwapBuffers(static_cast<HDC>(hdc_));
}
