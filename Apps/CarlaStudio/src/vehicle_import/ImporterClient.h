// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later —
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#pragma once

#include <QJsonObject>
#include <QString>

namespace carla_studio::vehicle_import {

// Default port; runtime-overridable via env CARLA_VEHICLE_IMPORTER_PORT so
// the offline test rig can run on a non-clashing port while Studio stays on
// the default. Use importerPort() at call time, not the constant — keep the
// constant for headers/messages only.
inline constexpr int kImporterPort = 18583;
int importerPort();

bool       probeImporterPort();
QString    sendJson(const QJsonObject &spec);
QJsonObject buildSpawnSpec(const QString &assetPath,
                          double x = 0.0, double y = 0.0, double z = 100.0,
                          double yaw = 0.0);

}  // namespace carla_studio::vehicle_import
