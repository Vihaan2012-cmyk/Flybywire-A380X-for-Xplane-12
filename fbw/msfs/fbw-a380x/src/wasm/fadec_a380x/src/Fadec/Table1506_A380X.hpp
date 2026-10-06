// Copyright (c) 2026 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

#ifndef FLYBYWIRE_AIRCRAFT_TABLE1506_A380X_HPP
#define FLYBYWIRE_AIRCRAFT_TABLE1506_A380X_HPP

#include <algorithm>

class Table1506_A380X {
  static constexpr int    ROWS              = 20;
  static constexpr int    COLS              = 10;
  static constexpr double MACH[COLS]        = {0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9};
  static constexpr double CN1[ROWS]         = {0, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110};
  static constexpr double TABLE[ROWS][COLS] = {
      {0.015, -0.15, 0, 0, 0, 0, 0, 0, 0, 0},
      {0.1286, -0.008, 0.015, 0.008, -0.023, -0.061, -0.081, -0.135, -0.21, -0.30},
      {0.155, 0.0236, 0.018, 0.021, -0.0145, -0.038, -0.08, -0.106, -0.150, -0.170},
      {0.16, 0.16, 0.11, 0.10, 0.08, 0.03, -0.01, -0.044, -0.064, -0.088},
      {0.315, 0.151, 0.123, 0.115, 0.100, 0.053, 0.028, 0.006, -0.008, -0.046},
      {0.396, 0.195, 0.149, 0.070, 0.065, 0.051, 0.042, 0.021, 0.015, -0.038},
      {0.472, 0.300, 0.188, 0.114, 0.080, 0.076, 0.070, 0.032, 0.021, -0.021},
      {0.543, 0.373, 0.254, 0.140, 0.109, 0.160, 0.110, 0.033, 0.026, -0.015},
      {0.611, 0.448, 0.324, 0.180, 0.145, 0.190, 0.175, 0.085, 0.055, 0.011},
      {0.714, 0.528, 0.402, 0.250, 0.182, 0.225, 0.220, 0.120, 0.124, 0.024},
      {0.792, 0.615, 0.489, 0.320, 0.218, 0.236, 0.275, 0.195, 0.130, 0.030},
      {0.880, 0.712, 0.586, 0.380, 0.280, 0.250, 0.320, 0.285, 0.170, 0.100},
      {0.982, 0.821, 0.695, 0.430, 0.390, 0.300, 0.330, 0.390, 0.320, 0.190},
      {1.092, 0.941, 0.811, 0.550, 0.450, 0.400, 0.405, 0.460, 0.380, 0.280},
      {1.230, 1.070, 0.932, 0.660, 0.570, 0.480, 0.500, 0.430, 0.440, 0.380},
      {1.388, 1.206, 1.049, 0.750, 0.680, 0.560, 0.540, 0.500, 0.480, 0.400},
      {1.538, 1.340, 1.153, 0.875, 0.750, 0.600, 0.570, 0.560, 0.660, 0.690},
      {1.603, 1.465, 1.229, 0.950, 0.800, 0.700, 0.740, 0.740, 0.680, 0.750},
      {1.655, 1.488, 1.258, 1.03, 0.859, 0.800, 0.820, 0.770, 0.690, 0.770},
      {1.671, 1.503, 1.283, 1.100, 0.935, 0.900, 0.920, 0.800, 0.710, 0.780},
  };

  static double lerp(double x, double x0, double x1, double y0, double y1) { return x1 == x0 ? y0 : y0 + (y1 - y0) * (x - x0) / (x1 - x0); }

 public:
  static double thrustFraction(double cn1, double mach) {
    cn1  = std::clamp(cn1, CN1[0], CN1[ROWS - 1]);
    mach = std::clamp(mach, MACH[0], MACH[COLS - 1]);
    int r = 1;
    while (r < ROWS - 1 && CN1[r] < cn1) {
      r++;
    }
    int c = 1;
    while (c < COLS - 1 && MACH[c] < mach) {
      c++;
    }
    const double atC0 = lerp(cn1, CN1[r - 1], CN1[r], TABLE[r - 1][c - 1], TABLE[r][c - 1]);
    const double atC1 = lerp(cn1, CN1[r - 1], CN1[r], TABLE[r - 1][c], TABLE[r][c]);
    return lerp(mach, MACH[c - 1], MACH[c], atC0, atC1);
  }

  static double cn1ForThrustFraction(double fraction, double cn1Max, double mach) {
    constexpr double STEP = 0.1;
    double           cn1  = std::clamp(cn1Max, CN1[0], CN1[ROWS - 1]);
    while (cn1 > CN1[0] && thrustFraction(cn1, mach) > fraction) {
      cn1 -= STEP;
    }
    return std::max(cn1, CN1[0]);
  }
};

#endif
