#pragma once

#include "Stages/Stage_Common.h"

#include <util/ue-header-guard-begin.h>
#include "Templates/SubclassOf.h"
#include <util/ue-header-guard-end.h>

class UStaticMesh;
class USkeletalMesh;
class UPhysicsAsset;
class UBlueprint;
class UClass;
class UChaosVehicleWheel;
struct FVehicleImportSpec;

namespace VehicleImport
{
  struct FParentBPSlots
  {
    UClass* BaseClass = nullptr;
    FName BodyStaticMeshSlotName = NAME_None;
    bool bHasSkelMeshSlot = false;
    int32 NumStaticMeshSlots = 0;
    int32 NumWheelSlots = 0;
    FString DiagnosticDump;
  };

  FParentBPSlots ProbeParentClassSlots(UClass* ParentClass);

  struct FBuildBPInputs
  {
    UStaticMesh* BodyMesh = nullptr;
    USkeletalMesh* SkelMesh = nullptr;
    UPhysicsAsset* PhysAsset = nullptr;
  };

  struct FBuildBPOutputs
  {
    FString BPPath;
    UBlueprint* BP = nullptr;
    TArray<FString> WheelBPPaths;
    TArray<FString> ShrunkWheelShapePaths;
    FString PhysAssetPath;
  };

  FStageError RunStage_BuildBP(
      const FNormalizedSpec& Norm,
      const FVehicleImportSpec& Spec,
      const FBuildBPInputs& In,
      FBuildBPOutputs& Out);
}
