// linesim: runs the BLDC line-tracer firmware against a MuJoCo robot on a virtual track.
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <string>
#include <vector>

#include "core/scenario.h"
#include "out/report.h"
#include "world/track.h"
#include "world/track_gen.h"

namespace fs = std::filesystem;

// linesim gen: random ZETIN-rule course
static int run_gen(int argc, char** argv) {
	TrackGenOptions g;
	g.seed = static_cast<unsigned>(std::time(nullptr));
	std::string out;
	for (int i = 2; i < argc; i++) {
		std::string a = argv[i];
		auto next = [&]() -> std::string { return i + 1 < argc ? argv[++i] : ""; };
		if (a == "--seed")
			g.seed = static_cast<unsigned>(std::strtoul(next().c_str(), nullptr, 10));
		else if (a == "--out")
			out = next();
		else if (a == "--board") {   // e.g. 6.1x4.88
			std::string v = next();
			std::sscanf(v.c_str(), "%lfx%lf", &g.board_w, &g.board_h);
		} else if (a == "--crossings")
			g.crossings = std::atoi(next().c_str());
		else if (a == "--pieces")
			std::sscanf(next().c_str(), "%d,%d", &g.min_pieces, &g.max_pieces);
		else if (a == "--length")
			std::sscanf(next().c_str(), "%lf,%lf", &g.min_length, &g.max_length);
		else {
			std::fprintf(stderr, "unknown argument: %s\n", a.c_str());
			return 2;
		}
	}
	if (out.empty())
		out = "tracks/random_" + std::to_string(g.seed) + ".json";
	std::string err;
	int attempts = 0;
	std::string text = generate_track(g, &err, &attempts);
	if (text.empty()) {
		std::fprintf(stderr, "%s\n", err.c_str());
		return 1;
	}
	fs::create_directories(fs::absolute(out).parent_path());
	FILE* f = std::fopen(out.c_str(), "wb");
	if (!f) {
		std::fprintf(stderr, "cannot write %s\n", out.c_str());
		return 1;
	}
	std::fwrite(text.data(), 1, text.size(), f);
	std::fclose(f);
	Track t;
	t.load(out, &err);
	double x0, y0, x1, y1;
	t.bounds(&x0, &y0, &x1, &y1);
	std::printf("%s: seed %u, %.2f m, %.2f x %.2f m, %zu crossing(s), %d attempt(s)\n", out.c_str(), g.seed,
		t.length(), x1 - x0, y1 - y0, t.crossings().size(), attempts);
	std::printf("expected markers: %s\n", t.expected_markers().c_str());
	return 0;
}

static void usage() {
	std::puts(
		"usage:\n"
		"  linesim run <scenario.json> [options]\n"
		"  linesim run --track <track.json> [--robot <robot.json>] [options]\n"
		"  linesim view <scenario.json> [options]             same run in a 3D window, real time\n"
		"  linesim view --interactive --track <track.json>    firmware menu driven from the keyboard\n"
		"  linesim view --session --track <track.json>        keep driving: pick drive 1-4, tune params, new maps\n"
		"  linesim gen [--seed n] [--out file] [--board 6.1x4.88] [--length 35,55] [--crossings n]\n"
		"                                                     random ZETIN course (short: --board 4.88x4.88 --length 8,30)\n"
		"\n"
		"options:\n"
		"  --out <dir>          output folder (default: runs/<date-time>)\n"
		"  --drives 1,2,3,4     drives to run in order (default from scenario, else 1)\n"
		"  --set key=value      override a firmware drive parameter (repeatable):\n"
		"                       base_mps max_mps accel decel steer_kp steer_kd pos_atten_gain\n"
		"                       pit_in_m target_shift fan_en, steer_kd_s (kd in seconds)\n"
		"  --robot-param k=v    override a robot model value: sensor_delay tire_mu drive_lag fan_force\n"
		"  --seed <n>           sensor noise / gain spread seed\n"
		"  --max-time <s>       per-drive time limit before the robot is lifted\n"
		"  --quiet              no progress lines\n"
		"  --speed <x>          (view) playback speed, e.g. 0.25 for slow motion\n"
		"  --interactive        (view) boot to the firmware menu: arrows = U/D/L/R, Enter = K,\n"
		"                       R = robot back on the start line");
}

int main(int argc_in, char** argv_in) {
	// Arguments may arrive as one string from an IDE prompt ("--drives 1 --set base_mps=2"):
	// split every argument on spaces and drop empty ones.
	std::vector<std::string> args_s;
	for (int i = 0; i < argc_in; i++) {
		std::string a = argv_in[i];
		if (i == 0) {
			args_s.push_back(a);
			continue;
		}
		if (a.find_first_not_of(' ') == std::string::npos)   // empty prompt input from an IDE task
			continue;
		if (a.rfind("--", 0) != 0 && a.find(" --") == std::string::npos) {   // a path: keep its spaces
			args_s.push_back(a);
			continue;
		}
		size_t p = 0;
		while (p < a.size()) {
			size_t q = a.find(' ', p);
			if (q == std::string::npos)
				q = a.size();
			if (q > p)
				args_s.push_back(a.substr(p, q - p));
			p = q + 1;
		}
	}
	std::vector<char*> argv_v;
	for (std::string& a : args_s)
		argv_v.push_back(a.data());
	int argc = static_cast<int>(argv_v.size());
	char** argv = argv_v.data();

	std::string cmd = argc >= 2 ? argv[1] : "";
	if (cmd == "gen")
		return run_gen(argc, argv);
	if (cmd != "run" && cmd != "view") {
		usage();
		return 2;
	}
	ScenarioOptions opt;
	opt.view = cmd == "view";
	bool drives_set = false;
	std::vector<int> drives_cli;
	std::vector<std::pair<std::string, double>> sets;
	std::string out_cli;
	for (int i = 2; i < argc; i++) {
		std::string a = argv[i];
		auto next = [&]() -> std::string {
			if (i + 1 >= argc) {
				std::fprintf(stderr, "missing value for %s\n", a.c_str());
				std::exit(2);
			}
			return argv[++i];
		};
		if (a == "--track") {
			opt.track_path = next();
		} else if (a == "--robot") {
			opt.robot_path = next();
		} else if (a == "--out") {
			out_cli = next();
		} else if (a == "--drives") {
			drives_set = true;
			std::string v = next();
			for (size_t p = 0; p < v.size();) {
				size_t q = v.find(',', p);
				drives_cli.push_back(std::atoi(v.substr(p, q - p).c_str()));
				p = (q == std::string::npos) ? v.size() : q + 1;
			}
		} else if (a == "--set") {
			std::string kv = next();
			size_t eq = kv.find('=');
			if (eq == std::string::npos) {
				std::fprintf(stderr, "--set expects key=value\n");
				return 2;
			}
			sets.emplace_back(kv.substr(0, eq), std::atof(kv.c_str() + eq + 1));
		} else if (a == "--robot-param") {
			std::string kv = next();
			size_t eq = kv.find('=');
			if (eq == std::string::npos) {
				std::fprintf(stderr, "--robot-param expects key=value\n");
				return 2;
			}
			opt.robot_params[kv.substr(0, eq)] = std::atof(kv.c_str() + eq + 1);
		} else if (a == "--seed") {
			opt.seed = static_cast<unsigned>(std::atoi(next().c_str()));
		} else if (a == "--max-time") {
			opt.max_drive_time = std::atof(next().c_str());
		} else if (a == "--quiet") {
			opt.quiet = true;
		} else if (a == "--session") {
			opt.session = true;
			if (cmd == "view")
				opt.view = true;
		} else if (a == "--tune") {
			std::string m = next();
			opt.session = true;
			opt.auto_tune = true;
			if (cmd == "view")
				opt.view = true;
			if (m == "A" || m == "a" || m == "1" || m == "bayes")
				opt.tune_mode = 1;
			else if (m == "E" || m == "e" || m == "2" || m == "endless")
				opt.tune_mode = 2;
			else
				opt.tune_mode = 1;
		} else if (a == "--interactive") {
			opt.interactive = true;
			opt.view = true;
		} else if (a == "--trials" || a == "--candidates" || a == "--tune-count") {
			opt.tune_trials = std::atoi(next().c_str());
		} else if (a == "--fail-rate" || a == "--max-fail-rate") {
			opt.endless_max_fail_pct = std::atof(next().c_str());
		} else if (a == "--pass-rate" || a == "--success-rate") {
			opt.endless_max_fail_pct = std::max(1.0, 100.0 - std::atof(next().c_str()));
		} else if (a == "--window") {
			opt.endless_window = std::atoi(next().c_str());
		} else if (a == "--min-speed" || a == "--speed-floor") {
			opt.endless_min_speed = std::atof(next().c_str());
		} else if (a == "--speed") {
			opt.view_speed = std::atof(next().c_str());
			if (opt.view_speed <= 0)
				opt.view_speed = 1;
		} else if (a.size() > 5 && a.substr(a.size() - 5) == ".json") {
			std::string err;
			if (!load_scenario_file(a, &opt, &err)) {
				std::fprintf(stderr, "%s\n", err.c_str());
				return 1;
			}
		} else {
			std::fprintf(stderr, "unknown argument: %s\n", a.c_str());
			usage();
			return 2;
		}
	}
	if (drives_set)
		opt.drives = drives_cli;
	else if (opt.session)
		opt.drives.clear();   // session: drives are started from the keyboard
	for (const auto& kv : sets) {
		FwDriveParams probe{};
		if (!apply_param(&probe, kv.first, kv.second)) {
			std::fprintf(stderr, "unknown parameter: %s\n", kv.first.c_str());
			return 2;
		}
		opt.params[kv.first] = kv.second;
	}
	if (!out_cli.empty())
		opt.out_dir = out_cli;
	if (opt.out_dir.empty()) {
		char stamp[32];
		std::time_t now = std::time(nullptr);
		std::tm tmv;
		localtime_s(&tmv, &now);
		std::strftime(stamp, sizeof stamp, "%Y%m%d_%H%M%S", &tmv);
		opt.out_dir = (fs::path("runs") / stamp).string();
	}
	if (opt.track_path.empty()) {
		if (opt.tune_mode == 2 || opt.session) {
			opt.track_path = "random";
		} else if (opt.tune_mode == 1 || opt.auto_tune) {
			if (fs::exists("tracks/practice_board.json"))
				opt.track_path = "tracks/practice_board.json";
			else
				opt.track_path = "random";
		} else {
			std::fprintf(stderr, "no track given\n");
			usage();
			return 2;
		}
	}

	ScenarioResult res = run_scenario(opt);

	Track track;
	std::string err;
	if (!res.track_path_used.empty() && res.track_path_used != "random" && track.load(res.track_path_used, &err))
		write_reports(opt, res, track);
	print_summary(res);
	std::printf(" output: %s\n", fs::absolute(opt.out_dir).string().c_str());
	if (!res.ok) {
		std::fprintf(stderr, "\nERROR: %s\n", res.error.c_str());
		return 1;
	}
	return 0;
}
