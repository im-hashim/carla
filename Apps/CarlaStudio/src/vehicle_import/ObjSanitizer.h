// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later —
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#pragma once

#include <QString>

namespace carla_studio::vehicle_import {

struct SanitizeReport {
  QString outputPath;
  int     skippedFaceLines = 0;
  bool    ok = false;
  float   appliedScale = 1.0f;
  bool    swapXY       = false;
  float   zLiftCm      = 0.0f;
};

// Auto-mode (scaleToCm <= 0): two-pass heuristic — scale to fit ~250cm largest
// extent if input is in meters or normalized units, rotate so longest horizontal
// axis becomes +X (UE forward), translate so min-Z = 0 (wheels on ground).
// Explicit-mode (scaleToCm > 0): scale only, no rotate or lift (legacy compat).
// Always co-locates any .mtl + texture files from the source directory next
// to the output OBJ so UE Interchange can resolve mtllib references.
SanitizeReport sanitizeOBJ(const QString &inputPath, float scaleToCm);

}  // namespace carla_studio::vehicle_import
