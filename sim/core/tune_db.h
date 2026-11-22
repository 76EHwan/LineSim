// Tuning record DB (runs/tune_db.json, kept across sessions).
//
// Random courses are not kept as files: a seed (+ generator mode) rebuilds the same course.
// Per course only the best lap of each drive and the parameter set that drove it are kept.
// The champion is the parameter set that has won the benchmark (fixed bench seeds + recent
// hard seeds the champion failed). A newly tuned set replaces it only if it wins that
// benchmark; otherwise the champion stays and is applied again.
#pragma once

#include <map>
#include <string>
#include <vector>

#include "fw_bridge.h"

namespace tunedb {

using Params = std::map<std::string, double>;   // apply_param() keys

Params from_fw(const FwDriveParams& p);
void to_fw(const Params& kv, FwDriveParams* p);
bool same_params(const Params& a, const Params& b);

struct BenchResult {
	bool pass = false;
	double pace = 0;   // s per 10 m of drive 4 (length-normalized: comparable across courses)
	double lap4 = 0;
};

struct DriveBest {
	double lap = 0;    // firmware lap time [s], 0 = none
	int set = -1;      // index into sets
};

struct MapRecord {
	std::string name;    // track name ("random_<seed>" for generated courses)
	unsigned seed = 0;   // 0: not a generated course
	std::string mode;    // generator mode: "complex" / "hyper" ("" for files)
	double length = 0;
	int runs = 0;
	DriveBest best[6];   // index = drive number 1..5
};

class TuneDb {
public:
	bool load(const std::string& path);   // a missing file is an empty DB (returns true)
	bool save() const;
	const std::string& path() const { return path_; }

	int intern(const Params& p);   // id of an identical stored set, else a new one
	const Params& set(int id) const { return sets_[static_cast<size_t>(id)]; }

	// one finished (or failed) drive on a course; returns true if it is a new best for that drive
	bool record_drive(const std::string& name, unsigned seed, const std::string& mode, double length, int drive,
		bool ok, double lap, const Params& params);

	bool has_champion() const { return champion_ >= 0; }
	const Params& champion() const { return sets_[static_cast<size_t>(champion_)]; }
	void set_champion(const Params& p, const std::map<unsigned, BenchResult>& results);
	std::map<unsigned, BenchResult>& champion_results() { return champion_results_; }

	const std::vector<unsigned>& bench_seeds() const { return bench_seeds_; }
	const std::vector<unsigned>& hard_seeds() const { return hard_seeds_; }
	void add_hard_seed(unsigned seed, const std::string& mode);
	std::string mode_of(unsigned seed) const;   // "complex" if unknown
	const MapRecord* find(const std::string& name) const;
	size_t map_count() const { return maps_.size(); }
	std::string champion_since() const { return champion_since_; }

	static constexpr size_t kMaxMaps = 300;
	static constexpr size_t kMaxHard = 20;

private:
	MapRecord* find_mut(const std::string& name);
	void prune_sets();

	std::string path_;
	std::vector<Params> sets_;
	std::vector<MapRecord> maps_;   // most recently driven last
	int champion_ = -1;
	std::string champion_since_;
	std::map<unsigned, BenchResult> champion_results_;
	std::vector<unsigned> bench_seeds_{7001, 7002, 7003, 7004, 7005};
	std::vector<unsigned> hard_seeds_;
	std::map<unsigned, std::string> seed_mode_;
};

}  // namespace tunedb
