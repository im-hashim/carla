// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later —
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#pragma once

#include <QDialog>
#include <QString>
#include <QVector3D>

namespace carla_studio::vehicle_import {

class VehiclePreviewPage;

// Pop-out wrapper around VehiclePreviewPage. Singleton-ish: the same window
// is reused across multiple imports so the user can keep it on a second
// monitor and watch it auto-load each new canonical OBJ.
class VehiclePreviewWindow : public QDialog {
  Q_OBJECT
 public:
  static VehiclePreviewWindow *instance();

  // Load a mesh, place wheel markers at the four positions (cm, post-import
  // canonical convention: lateral=+X forward=+Y up=+Z), raise the window.
  void showFor(const QString &meshPath,
               const QVector3D &fl, const QVector3D &fr,
               const QVector3D &rl, const QVector3D &rr);

  // Bare load without wheel markers (for the manual Browse → Load path).
  void showForMesh(const QString &meshPath);

  VehiclePreviewPage *page() const { return mPage; }

 private:
  explicit VehiclePreviewWindow(QWidget *parent = nullptr);
  VehiclePreviewPage *mPage = nullptr;
};

}  // namespace carla_studio::vehicle_import
