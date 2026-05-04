#pragma once

#include "Stages/Stage_Common.h"

namespace VehicleImport
{
  struct FPersistInputs
  {
    FString VehicleContentPath;
    FString VehicleName;
    FString BPPath;
    FString PhysAssetPath;
    TArray<FString> WheelBPPaths;
    TArray<FString> ShrunkWheelShapePaths;
  };

  struct FPersistOutputs
  {
    int32 SavedCount = 0;
    int32 RequestedCount = 0;
  };

  FStageError RunStage_Persist(const FPersistInputs& In, FPersistOutputs& Out);
}
