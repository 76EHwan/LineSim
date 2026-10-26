// Random ZETIN-rule courses: straights and R 0.25 m arcs (45..270 deg in 45 deg steps),
// closed, crossings only at right angles, end-mark straight, fits the board.
#pragma once
#include <string>

struct TrackGenOptions {
	unsigned seed = 1;
	// 1.5x scale of ZETIN championship finals board (6.10 x 4.88 m * 1.5 = 9.15 x 7.32 m).
	// In complex mode (default for 1.5x finals), generates high-density courses with 2+ crossings,
	// dense chicane cascades, multi-lobe hairpins, and unbroken ~8m launch straights.
	double board_w = 9.15, board_h = 7.32;
	int min_pieces = 14, max_pieces = 28;     // random walk only: pieces before the closing ones
	int crossings = -1;                       // exact number of crossings, -1 = at least min_crossings
	int min_crossings = 2;                    // courses always have at least 2 crossings in complex mode
	double min_length = 65.0, max_length = 85.0;
	bool complex_mode = true;                 // enhanced complexity: higher density, multi-crossings, chicane cascades
	bool hyper_mode = false;                  // hyper speedway: extreme drag strip
};

// Returns the track JSON (same format as tracks/*.json) or "" with *err set.
std::string generate_track(const TrackGenOptions& opt, std::string* err, int* attempts = nullptr);
