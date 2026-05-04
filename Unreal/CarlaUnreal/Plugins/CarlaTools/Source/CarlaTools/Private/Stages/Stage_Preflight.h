#pragma once

#include "Stages/Stage_Common.h"

namespace VehicleImport
{
  FStageError RunStage_Preflight(const FVehicleImportSpec& InSpec, FNormalizedSpec& OutNorm);
}
