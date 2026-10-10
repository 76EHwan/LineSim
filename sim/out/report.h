// Run outputs: summary.json, per-drive telemetry CSV, marker logs, and SVG plots.
#pragma once
#include <string>

#include "core/scenario.h"
#include "world/track.h"

void write_reports(const ScenarioOptions& opt, const ScenarioResult& res, const Track& track);
void print_summary(const ScenarioResult& res);
