#include "core/tune_db.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>

#include "core/json.h"
#include "core/scenario.h"

namespace fs = std::filesystem;

namespace tunedb {

Params from_fw(const FwDriveParams& p) {
	Params kv;
	kv["base_mps"] = p.base_mps;
	kv["max_mps"] = p.max_mps;
	kv["accel"] = p.accel;
	kv["decel"] = p.decel;
	kv["steer_kp"] = p.steer_kp;
	kv["steer_kd"] = p.steer_kd;
	kv["pos_atten_gain"] = p.pos_atten_gain;
	kv["pit_in_m"] = p.pit_in_m;
	kv["target_shift"] = p.target_shift;
	kv["fan_en"] = p.fan_en;
	kv["yaw_kp"] = p.yaw_kp;
	kv["yaw_ki"] = p.yaw_ki;
	kv["target_curve_gain"] = p.target_curve_gain;
	kv["slalom_ratio"] = p.slalom_ratio;
	kv["brake_margin_m"] = p.brake_margin_m;
	kv["bulge_lat_acc"] = p.bulge_lat_acc;
	kv["corner45_max_mps"] = p.corner45_max_mps;
	kv["omega_add_mps"] = p.omega_add_mps;
	kv["wave_add_mps"] = p.wave_add_mps;
	kv["ramp_jerk_s"] = p.ramp_jerk_s;
	kv["ramp_settle_ms"] = p.ramp_settle_ms;
	kv["fourth_in_mm"] = p.fourth_in_mm;
	kv["fourth_side_mm"] = p.fourth_side_mm;
	kv["fourth_alat_gain"] = p.fourth_alat_gain;
	kv["fourth_hgain"] = p.fourth_hgain;
	return kv;
}

void to_fw(const Params& kv, FwDriveParams* p) {
	for (const auto& [k, v] : kv)
		apply_param(p, k, v);
}

bool same_params(const Params& a, const Params& b) {
	if (a.size() != b.size())
		return false;
	for (const auto& [k, v] : a) {
		auto it = b.find(k);
		if (it == b.end() || std::fabs(it->second - v) > 1e-5 * std::max(1.0, std::fabs(v)))
			return false;
	}
	return true;
}

static std::string now_text() {
	std::time_t t = std::time(nullptr);
	char buf[32];
	std::strftime(buf, sizeof buf, "%Y-%m-%d %H:%M", std::localtime(&t));
	return buf;
}

bool TuneDb::load(const std::string& path) {
	path_ = path;
	std::error_code ec;
	if (!fs::exists(path, ec))
		return true;
	json::Value j;
	std::string err;
	if (!json::parse_file(path, &j, &err)) {
		std::fprintf(stderr, "tune db %s: %s (starting empty)\n", path.c_str(), err.c_str());
		return false;
	}
	if (const json::Value* ss = j.get("sets"); ss && ss->is_arr())
		for (const json::Value& s : ss->a) {
			Params p;
			for (const auto& [k, v] : s.o)
				if (v.is_num())
					p[k] = v.n;
			sets_.push_back(p);
		}
	auto valid_set = [&](double id) { return id >= 0 && id < static_cast<double>(sets_.size()) ? static_cast<int>(id) : -1; };
	if (const json::Value* c = j.get("champion"); c && c->is_obj()) {
		champion_ = valid_set(c->num("set", -1));
		champion_since_ = c->str("since", "");
		if (const json::Value* r = c->get("bench"); r && r->is_arr())
			for (const json::Value& e : r->a) {
				BenchResult b;
				b.pass = e.boolean("pass", false);
				b.pace = e.num("pace", 0);
				b.lap4 = e.num("lap4", 0);
				champion_results_[static_cast<unsigned>(e.num("seed", 0))] = b;
			}
	}
	if (const json::Value* b = j.get("bench_seeds"); b && b->is_arr() && b->size() > 0) {
		bench_seeds_.clear();
		for (const json::Value& e : b->a)
			bench_seeds_.push_back(static_cast<unsigned>(e.n));
	}
	if (const json::Value* h = j.get("hard_seeds"); h && h->is_arr())
		for (const json::Value& e : h->a)
			hard_seeds_.push_back(static_cast<unsigned>(e.n));
	if (const json::Value* ms = j.get("maps"); ms && ms->is_arr())
		for (const json::Value& m : ms->a) {
			MapRecord r;
			r.name = m.str("name", "");
			r.seed = static_cast<unsigned>(m.num("seed", 0));
			r.mode = m.str("mode", "");
			r.length = m.num("length", 0);
			r.runs = static_cast<int>(m.num("runs", 0));
			if (const json::Value* bs = m.get("best"); bs && bs->is_arr())
				for (const json::Value& b : bs->a) {
					const int d = static_cast<int>(b.num("drive", 0));
					if (d < 1 || d > 5)
						continue;
					r.best[d].lap = b.num("lap", 0);
					r.best[d].set = valid_set(b.num("set", -1));
				}
			if (r.seed && !r.mode.empty())
				seed_mode_[r.seed] = r.mode;
			if (!r.name.empty())
				maps_.push_back(r);
		}
	if (const json::Value* sm = j.get("seed_modes"); sm && sm->is_obj())
		for (const auto& [k, v] : sm->o)
			seed_mode_[static_cast<unsigned>(std::strtoul(k.c_str(), nullptr, 10))] = v.s;
	return true;
}

void TuneDb::prune_sets() {
	std::vector<int> remap(sets_.size(), -1);
	std::vector<Params> kept;
	auto keep = [&](int id) {
		if (id < 0 || id >= static_cast<int>(sets_.size()))
			return -1;
		if (remap[static_cast<size_t>(id)] < 0) {
			remap[static_cast<size_t>(id)] = static_cast<int>(kept.size());
			kept.push_back(sets_[static_cast<size_t>(id)]);
		}
		return remap[static_cast<size_t>(id)];
	};
	TuneDb* self = const_cast<TuneDb*>(this);
	self->champion_ = keep(champion_);
	for (MapRecord& m : self->maps_)
		for (DriveBest& b : m.best)
			b.set = keep(b.set);
	self->sets_.swap(kept);
}

bool TuneDb::save() const {
	if (path_.empty())
		return false;
	const_cast<TuneDb*>(this)->prune_sets();
	std::error_code ec;
	fs::create_directories(fs::absolute(path_).parent_path(), ec);
	const std::string tmp = path_ + ".tmp";
	FILE* f = std::fopen(tmp.c_str(), "wb");
	if (!f)
		return false;
	std::fprintf(f, "{\n  \"version\": 1,\n");
	std::fprintf(f, "  \"champion\": {\"set\": %d, \"since\": \"%s\", \"bench\": [", champion_, champion_since_.c_str());
	bool first = true;
	for (const auto& [seed, b] : champion_results_) {
		std::fprintf(f, "%s\n    {\"seed\": %u, \"pass\": %s, \"pace\": %.5f, \"lap4\": %.4f}", first ? "" : ",", seed,
			b.pass ? "true" : "false", b.pace, b.lap4);
		first = false;
	}
	std::fprintf(f, "%s]},\n", first ? "" : "\n  ");
	auto seeds = [&](const char* key, const std::vector<unsigned>& v) {
		std::fprintf(f, "  \"%s\": [", key);
		for (size_t i = 0; i < v.size(); ++i)
			std::fprintf(f, "%s%u", i ? ", " : "", v[i]);
		std::fprintf(f, "],\n");
	};
	seeds("bench_seeds", bench_seeds_);
	seeds("hard_seeds", hard_seeds_);
	std::fprintf(f, "  \"seed_modes\": {");
	first = true;
	for (unsigned s : hard_seeds_) {
		auto it = seed_mode_.find(s);
		if (it == seed_mode_.end())
			continue;
		std::fprintf(f, "%s\"%u\": \"%s\"", first ? "" : ", ", s, it->second.c_str());
		first = false;
	}
	std::fprintf(f, "},\n  \"sets\": [");
	for (size_t i = 0; i < sets_.size(); ++i) {
		std::fprintf(f, "%s\n    {", i ? "," : "");
		bool f2 = true;
		for (const auto& [k, v] : sets_[i]) {
			std::fprintf(f, "%s\"%s\": %.6g", f2 ? "" : ", ", k.c_str(), v);
			f2 = false;
		}
		std::fprintf(f, "}");
	}
	std::fprintf(f, "%s],\n  \"maps\": [", sets_.empty() ? "" : "\n  ");
	for (size_t i = 0; i < maps_.size(); ++i) {
		const MapRecord& m = maps_[i];
		std::fprintf(f, "%s\n    {\"name\": \"%s\", \"seed\": %u, \"mode\": \"%s\", \"length\": %.3f, \"runs\": %d, \"best\": [",
			i ? "," : "", m.name.c_str(), m.seed, m.mode.c_str(), m.length, m.runs);
		bool f2 = true;
		for (int d = 1; d <= 5; ++d) {
			if (m.best[d].lap <= 0)
				continue;
			std::fprintf(f, "%s{\"drive\": %d, \"lap\": %.3f, \"set\": %d}", f2 ? "" : ", ", d, m.best[d].lap, m.best[d].set);
			f2 = false;
		}
		std::fprintf(f, "]}");
	}
	std::fprintf(f, "%s]\n}\n", maps_.empty() ? "" : "\n  ");
	std::fclose(f);
	fs::rename(tmp, path_, ec);
	if (ec) {
		fs::remove(path_, ec);
		fs::rename(tmp, path_, ec);
	}
	return !ec;
}

int TuneDb::intern(const Params& p) {
	for (size_t i = 0; i < sets_.size(); ++i)
		if (same_params(sets_[i], p))
			return static_cast<int>(i);
	sets_.push_back(p);
	return static_cast<int>(sets_.size() - 1);
}

MapRecord* TuneDb::find_mut(const std::string& name) {
	for (MapRecord& m : maps_)
		if (m.name == name)
			return &m;
	return nullptr;
}

const MapRecord* TuneDb::find(const std::string& name) const {
	for (const MapRecord& m : maps_)
		if (m.name == name)
			return &m;
	return nullptr;
}

bool TuneDb::record_drive(const std::string& name, unsigned seed, const std::string& mode, double length, int drive,
	bool ok, double lap, const Params& params) {
	if (name.empty() || drive < 1 || drive > 5)
		return false;
	MapRecord rec;
	if (MapRecord* m = find_mut(name)) {
		rec = *m;
		maps_.erase(maps_.begin() + (m - maps_.data()));
	} else {
		rec.name = name;
		rec.seed = seed;
		rec.mode = mode;
		rec.length = length;
	}
	if (seed && !mode.empty())
		seed_mode_[seed] = mode;
	rec.runs++;
	bool better = false;
	if (ok && lap > 0 && (rec.best[drive].lap <= 0 || lap < rec.best[drive].lap - 1e-4)) {
		rec.best[drive].lap = lap;
		rec.best[drive].set = intern(params);
		better = true;
	}
	maps_.push_back(rec);   // most recent last
	while (maps_.size() > kMaxMaps)
		maps_.erase(maps_.begin());
	return better;
}

void TuneDb::set_champion(const Params& p, const std::map<unsigned, BenchResult>& results) {
	champion_ = intern(p);
	champion_results_ = results;
	champion_since_ = now_text();
}

void TuneDb::add_hard_seed(unsigned seed, const std::string& mode) {
	if (!seed)
		return;
	if (std::find(bench_seeds_.begin(), bench_seeds_.end(), seed) != bench_seeds_.end())
		return;
	auto it = std::find(hard_seeds_.begin(), hard_seeds_.end(), seed);
	if (it != hard_seeds_.end())
		hard_seeds_.erase(it);
	hard_seeds_.push_back(seed);   // most recent last
	if (!mode.empty())
		seed_mode_[seed] = mode;
	while (hard_seeds_.size() > kMaxHard)
		hard_seeds_.erase(hard_seeds_.begin());
}

std::string TuneDb::mode_of(unsigned seed) const {
	auto it = seed_mode_.find(seed);
	return it == seed_mode_.end() ? "complex" : it->second;
}

}  // namespace tunedb
