// Random ZETIN-rule courses: straights and R 0.25 m arcs (45..270 deg in 45 deg steps),
// closed, crossings only at right angles, end-mark straight, fits the board.
#pragma once
#include <string>

struct TrackGenOptions {
	unsigned seed = 1;
	double board_w = 4.88, board_h = 4.88;   // ZETIN qualifying board
	int min_pieces = 14, max_pieces = 28;     // random pieces before the closing ones
	int crossings = -1;                       // exact number of crossings, -1 = at least min_crossings
	int min_crossings = 1;                    // courses always have a crossing unless set to 0
	double min_length = 8.0, max_length = 30.0;
};

// Returns the track JSON (same format as tracks/*.json) or "" with *err set.
std::string generate_track(const TrackGenOptions& opt, std::string* err, int* attempts = nullptr);
