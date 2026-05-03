// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later —
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later
//
// Standalone end-to-end harness for the vehicle-import wire format.
//
// Spawns its OWN headless UE Editor (separate port from Studio's, default
// 18584), constructs each fixture's VehicleSpec, sends it through the same
// `specToJson` + `sendJson` code path Studio uses (so wire-format bugs like
// missing null-terminators are exercised), reads the response, and prints a
// per-vehicle pass/fail table.
//
// Doesn't touch Studio. No Python. Pure C++.
//
// Usage:
//   carla-studio-vehicle-import-test [--port N] [--no-spawn-editor]

#include "vehicle_import/ImporterClient.h"
#include "vehicle_import/VehicleSpec.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcess>
#include <QRegularExpression>
#include <QString>
#include <QStringList>
#include <QStandardPaths>
#include <QTextStream>
#include <QThread>

#include <chrono>
#include <cstdio>
#include <cstdlib>

extern "C" int system(const char *);

namespace cs = carla_studio::vehicle_import;

namespace {

struct Fixture {
  QString name;
  QString meshPath;
};

QList<Fixture> loadFixtures()
{
  QList<Fixture> out;
  const QString root = qEnvironmentVariable("CARLA_VEHICLE_TEST_FIXTURES_DIR");
  if (root.isEmpty()) return out;
  QDir d(root);
  const QStringList exts = { "*.obj", "*.fbx", "*.glb", "*.gltf", "*.dae", "*.blend" };
  for (const QFileInfo &fi : d.entryInfoList(exts, QDir::Files, QDir::Name)) {
    out.push_back({ fi.completeBaseName().toLower().replace(QRegularExpression("[^a-z0-9_]+"), "_"),
                    fi.absoluteFilePath() });
  }
  return out;
}

QString tail(const QString &path, int lines)
{
  QProcess p;
  p.start("/bin/sh", QStringList() << "-c"
      << QString("tail -n %1 %2 2>/dev/null").arg(lines).arg(path));
  p.waitForFinished(2000);
  return QString::fromLocal8Bit(p.readAllStandardOutput());
}

bool portOpen(int port)
{
  QProcess p;
  p.start("/bin/sh", QStringList() << "-c"
      << QString("ss -tln 2>/dev/null | grep -q ':%1 '").arg(port));
  p.waitForFinished(1500);
  return p.exitCode() == 0;
}

qint64 launchEditor(int port, const QString &logPath)
{
  const QString engine  = qEnvironmentVariable("CARLA_UNREAL_ENGINE_PATH");
  const QString srcRoot = qEnvironmentVariable("CARLA_SRC_ROOT");
  if (engine.isEmpty() || srcRoot.isEmpty()) return -1;
  const QString editor = engine + "/Engine/Binaries/Linux/UnrealEditor";
  const QString uproject = srcRoot + "/Unreal/CarlaUnreal/CarlaUnreal.uproject";

  auto shEsc = [](const QString &s) {
    QString out = s; out.replace('\'', "'\\''"); return "'" + out + "'";
  };
  const QString cmd = QString("CARLA_VEHICLE_IMPORTER_PORT=%1 exec %2 %3 "
      "-unattended -RenderOffScreen -nosplash -nosound >>%4 2>&1 & echo $!")
      .arg(port).arg(shEsc(editor), shEsc(uproject), shEsc(logPath));
  QProcess p;
  p.start("/bin/sh", QStringList() << "-c" << cmd);
  p.waitForFinished(5000);
  bool ok = false;
  const qint64 pid = QString::fromLocal8Bit(p.readAllStandardOutput()).trimmed().toLongLong(&ok);
  return ok ? pid : -1;
}

bool waitPort(int port, int seconds)
{
  for (int i = 0; i < seconds; ++i) {
    if (portOpen(port)) return true;
    QThread::sleep(1);
  }
  return false;
}

void killEditor(qint64 pid)
{
  if (pid > 0) { ::system(QString("kill -9 %1 2>/dev/null").arg(pid).toLocal8Bit().constData()); }
}

cs::VehicleSpec makeSpec(const Fixture &f)
{
  cs::VehicleSpec s;
  s.name          = QString(f.name);
  s.meshPath      = QString(f.meshPath);
  s.contentPath   = "/Game/Carla/Static/Vehicles/4Wheeled";
  s.baseVehicleBP = "/Game/Carla/Blueprints/Vehicles/BaseVehiclePawnNW";
  s.mass          = 1500.0f;
  s.suspDamping   = 0.55f;
  // Reasonable wheel placeholders (in cm). Stage_Preflight + canonicalize
  // re-derive from the mesh; these are mostly placeholders.
  for (int i = 0; i < 4; ++i) {
    s.wheels[i].radius         = 35.f;
    s.wheels[i].width          = 22.f;
    s.wheels[i].maxSteerAngle  = (i < 2) ? 35.f : 0.f;
    s.wheels[i].maxBrakeTorque = 1500.f;
    s.wheels[i].suspMaxRaise   = 10.f;
    s.wheels[i].suspMaxDrop    = 10.f;
  }
  s.wheels[0].x =  140; s.wheels[0].y = -80; s.wheels[0].z = 35;
  s.wheels[1].x =  140; s.wheels[1].y =  80; s.wheels[1].z = 35;
  s.wheels[2].x = -140; s.wheels[2].y = -80; s.wheels[2].z = 35;
  s.wheels[3].x = -140; s.wheels[3].y =  80; s.wheels[3].z = 35;
  return s;
}

}  // namespace

int main(int argc, char **argv)
{
  QCoreApplication app(argc, argv);
  int port = 18584;
  bool spawnEditor = true;
  for (int i = 1; i < argc; ++i) {
    const QString a = QString::fromLocal8Bit(argv[i]);
    if (a == "--port" && i + 1 < argc) port = QString(argv[++i]).toInt();
    else if (a == "--no-spawn-editor")   spawnEditor = false;
  }

  QTextStream out(stdout);

  qint64 editorPid = -1;
  const QString editorLog = QString("/tmp/cs_test_editor_%1.log").arg(port);
  if (spawnEditor) {
    if (portOpen(port)) {
      out << "Port " << port << " already in use — pass --no-spawn-editor or pick another with --port.\n";
      return 2;
    }
    QFile::remove(editorLog);
    out << "Launching headless editor on port " << port << " (stdout→" << editorLog << ") …\n";
    editorPid = launchEditor(port, editorLog);
    if (editorPid <= 0) { out << "Editor launch failed.\n"; return 3; }
    out << "Editor pid: " << editorPid << "\n";
    out << "Waiting up to 90 s for port " << port << " …\n";
    if (!waitPort(port, 90)) {
      out << "Editor never opened port. Tail of log:\n" << tail(editorLog, 25) << "\n";
      killEditor(editorPid);
      return 4;
    }
    out << "Editor ready.\n";
  } else if (!portOpen(port)) {
    out << "No editor on port " << port << " and --no-spawn-editor passed. Aborting.\n";
    return 5;
  }

  const QList<Fixture> fixtures = loadFixtures();
  if (fixtures.isEmpty()) {
    out << "No fixtures found. Set CARLA_VEHICLE_TEST_FIXTURES_DIR to a "
           "directory containing .obj/.fbx/.glb/.gltf/.dae/.blend test meshes.\n";
    if (spawnEditor && editorPid > 0) killEditor(editorPid);
    return 6;
  }

  int passes = 0;
  out << "\n";
  out << QString("%1 %2 %3 %4\n")
        .arg("Vehicle", -18).arg("OK", 5).arg("BP", -54).arg("Detail").simplified() << "\n";
  for (const Fixture &f : fixtures) {
    if (!QFile::exists(f.meshPath)) {
      out << QString("[skip] %1 — mesh missing at %2\n").arg(f.name, f.meshPath);
      continue;
    }
    cs::VehicleSpec s = makeSpec(f);
    QJsonObject json = cs::specToJson(s);
    QString resp = cs::sendJson(json);   // <-- Studio's exact wire format
    if (resp.isEmpty()) {
      out << QString("[FAIL] %1 — no response from editor\n").arg(f.name);
      continue;
    }
    const QJsonObject obj = QJsonDocument::fromJson(resp.toUtf8()).object();
    const bool ok = obj.value("success").toBool();
    const QString path = obj.value("asset_path").toString();
    const QString err  = obj.value("error").toString();
    if (ok) {
      ++passes;
      out << QString("[ OK ] %1 → %2\n").arg(f.name, path);
    } else {
      out << QString("[FAIL] %1 — %2\n").arg(f.name, err);
    }
  }

  out << "\n" << passes << " / " << fixtures.size() << " imports succeeded.\n";

  if (spawnEditor && editorPid > 0) {
    out << "Killing editor pid " << editorPid << " …\n";
    killEditor(editorPid);
  }
  return passes == fixtures.size() ? 0 : 1;
}
