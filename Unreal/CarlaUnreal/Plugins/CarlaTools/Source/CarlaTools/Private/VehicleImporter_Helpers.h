#pragma once

#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include "Templates/SubclassOf.h"
#include <util/ue-header-guard-end.h>

class UStaticMesh;
class USkeletalMesh;
class UPhysicsAsset;
class UChaosVehicleWheel;
struct FVehicleImportSpec;
struct FWheelImportSpec;

namespace VehicleImport
{
  FString SanitizeAssetName(const FString& In);

  UStaticMesh* ImportStaticMesh(const FString& FilePath,
                                const FString& ContentPath,
                                const FString& AssetName,
                                const FVehicleImportSpec& Spec);

  UStaticMesh* MakeShrunkWheelShape(const FString& VehicleContentPath,
                                    const FString& Suffix,
                                    float RadiusCm,
                                    float WidthCm);

  TSubclassOf<UChaosVehicleWheel> CreateWheelBlueprint(
      const FString& VehicleContentPath,
      const FString& AssetName,
      const FString& Suffix,
      const FWheelImportSpec& W,
      UStaticMesh* OptionalShrunkShape);

  void ApplyChassisAabbResize(UPhysicsAsset* PhysAsset,
                              const FVehicleImportSpec& Spec,
                              bool bHasAabb);
}
