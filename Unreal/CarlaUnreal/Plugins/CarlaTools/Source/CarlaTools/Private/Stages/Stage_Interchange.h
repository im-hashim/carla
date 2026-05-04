#pragma once

#include "Stages/Stage_Common.h"

class UStaticMesh;
struct FVehicleImportSpec;

namespace VehicleImport
{
  struct FInterchangeOutputs
  {
    UStaticMesh* BodyMesh = nullptr;
  };

  FStageError RunStage_Interchange(
      const FNormalizedSpec& Norm,
      const FVehicleImportSpec& Spec,
      FInterchangeOutputs& Out);
}
