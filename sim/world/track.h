// Track: centerline made of straights and arcs, plus markers, all white on a black floor.
#pragma once
#include <string>
#include <vector>

struct Vec2 {
	double x = 0, y = 0;
};

struct Pose2 {
	double x = 0, y = 0, th = 0;   // th: heading [rad], CCW from +x
};

struct PathPiece {
	bool arc = false;
	double s0 = 0, len = 0;
	Vec2 p0;
	double th0 = 0;
	double radius = 0;
	int turn = 0;          // +1 left (CCW), -1 right
	Vec2 center;           // arcs only
	double ux = 1.0, uy = 0.0; // precomputed cos(th0), sin(th0)
	double a0 = 0.0;           // precomputed arc start angle
	double sweep = 0.0;        // precomputed len / radius
	double minx = -1e9, miny = -1e9, maxx = 1e9, maxy = 1e9; // bounding box

	Pose2 at(double ds) const;
};

// A white region on the floor.
struct Shape {
	enum Kind { Band, ArcBand, Rect } kind = Band;
	Vec2 a;                 // Band: start point; ArcBand: center; Rect: center
	Vec2 u;                 // Band/Rect: unit direction along the long axis
	double len = 0;         // Band: length
	double hl = 0;          // Rect: half length along u
	double hw = 0;          // half width
	double R = 0, a0 = 0, sweep = 0;   // ArcBand: radius, start angle, signed sweep
	double minx = 0, miny = 0, maxx = 0, maxy = 0;
	double r_in_sq = 0, r_out_sq = 0;   // Precomputed squared radius limits for fast circle rejection

	bool contains(Vec2 p) const;
	void compute_bounds();
};

struct TrackCrossing {
	double s1 = 0, s2 = 0;   // arc-length of the two passes
	Vec2 p;
	double angle_deg = 0;    // angle between the two lines
};

struct TrackMarker {
	double s;
	std::string kind;      // "left", "right", "end", "both"
};

class Track {
public:
	bool load(const std::string& path, std::string* err);
	// Same, from JSON text (`label` names it in messages). Replaces any previous content.
	bool load_text(const std::string& json_text, const std::string& label, std::string* err);

	// Where the course crosses itself (found from the geometry).
	const std::vector<TrackCrossing>& crossings() const { return crossings_; }
	// Marker sequence the firmware should log for one run: 'U' (end mark), 'L', 'R', 'C'.
	std::string expected_markers() const;
	// ZETIN rule check of the geometry; empty when the course is fine.
	std::vector<std::string> rule_problems(double board_w, double board_h, double min_clearance = 0.2) const;
	// Straight line available before / after arc-length s (0 when s is on an arc).
	void straight_run(double s, double* before, double* after) const;

	// Fraction of a Gaussian spot (std-dev sigma) centered at p that is white.
	double coverage(Vec2 p, double sigma) const;
	bool white(Vec2 p) const;

	Pose2 pose_at(double s) const;
	// Signed lateral offset from the centerline (left of travel = +) and the arc-length there.
	bool nearest(Vec2 p, double* lateral, double* s) const;

	double length() const { return length_; }
	bool closed() const { return closed_; }
	// Start line position, and the robot's start position if the track file fixes it
	// (otherwise the scenario places the robot so the sensor bar is just behind the start line).
	double start_s() const { return start_s_; }
	double goal_s() const { return goal_s_; }
	const std::vector<std::string>& warnings() const { return warnings_; }
	bool has_robot_start() const { return has_robot_start_; }
	double robot_start_s() const { return robot_start_s_; }
	double line_width() const { return line_width_; }
	const std::string& name() const { return name_; }
	const std::vector<Shape>& shapes() const { return shapes_; }
	const std::vector<PathPiece>& pieces() const { return pieces_; }
	const std::vector<TrackMarker>& markers() const { return markers_; }
	void bounds(double* x0, double* y0, double* x1, double* y1) const;

private:
	void add_mark(double s, int side);
	void build_index();
	void find_crossings();
	double wrap_s(double s) const;

	std::string name_;
	bool closed_ = true;
	double line_width_ = 0.02;
	// ZETIN rules: marks are 2 cm (driving direction) x 5 cm bars, 4 cm from the edge of the line
	double mark_gap_ = 0.04, mark_along_ = 0.02, mark_across_ = 0.05;
	double length_ = 0;
	double start_s_ = 0, goal_s_ = 0;
	std::vector<std::string> warnings_;
	double robot_start_s_ = 0;
	bool has_robot_start_ = false;
	std::vector<PathPiece> pieces_;
	std::vector<Shape> shapes_;
	std::vector<TrackMarker> markers_;
	std::vector<TrackCrossing> crossings_;

	double gx0_ = 0, gy0_ = 0, cell_ = 0.05;
	int gnx_ = 0, gny_ = 0;
	std::vector<std::vector<int>> grid_;
	mutable size_t last_piece_idx_ = 0;
};
