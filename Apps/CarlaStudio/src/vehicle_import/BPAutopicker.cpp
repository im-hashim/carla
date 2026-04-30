// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later —
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "vehicle_import/BPAutopicker.h"

#include <QString>
#include <cmath>

namespace carla_studio::vehicle_import {

QString pickClosestBaseVehicleBP(float /*lengthCm*/) {
  // Production vehicle BPs (Sprinter, Tesla, Charger…) are static-mesh rigs:
  // they have no USkeletalMesh + UPhysicsAsset that the importer can inherit.
  // BaseUSDImportVehicle is the only stock CARLA BP shaped right for the
  // importer to clone, so always recommend it regardless of vehicle length.
  return QStringLiteral("/Game/Carla/Blueprints/USDImportTemplates/BaseUSDImportVehicle");
}

}  // namespace carla_studio::vehicle_import
