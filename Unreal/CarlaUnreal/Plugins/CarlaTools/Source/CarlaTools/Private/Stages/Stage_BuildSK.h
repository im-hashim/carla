#pragma once

#include "Stages/Stage_Common.h"

class USkeletalMesh;
class UPhysicsAsset;
struct FVehicleImportSpec;

namespace VehicleImport
{
  struct FBuildSKOutputs
  {
    USkeletalMesh* SkelMesh = nullptr;
    UPhysicsAsset* PhysAsset = nullptr;
    FString SkPath;
    FString PaPath;
  };

  FStageError RunStage_BuildSK(
      const FNormalizedSpec& Norm,
      const FVehicleImportSpec& Spec,
      FBuildSKOutputs& Out);
}
