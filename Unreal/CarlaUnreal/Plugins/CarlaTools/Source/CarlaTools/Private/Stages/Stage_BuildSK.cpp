#include "Stages/Stage_BuildSK.h"
#include "VehicleImporter.h"
#include "CarlaTools.h"

#include <util/ue-header-guard-begin.h>
#include "Engine/SkeletalMesh.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "EditorAssetLibrary.h"
#include <util/ue-header-guard-end.h>

namespace VehicleImport
{

static const TCHAR* const kSkSrcPath =
    TEXT("/Game/Carla/Blueprints/USDImportTemplates/SK_USDVehicleBase");
static const TCHAR* const kPaSrcPath =
    TEXT("/Game/Carla/Blueprints/USDImportTemplates/SK_USDVehicleBase_PhysicsAsset");

FStageError RunStage_BuildSK(
    const FNormalizedSpec& Norm,
    const FVehicleImportSpec& Spec,
    FBuildSKOutputs& Out)
{
  Out.SkPath = Norm.VehicleContentPath / (TEXT("SK_") + Norm.VehicleName);
  Out.PaPath = Norm.VehicleContentPath / (TEXT("PA_") + Norm.VehicleName);

  if (UEditorAssetLibrary::DoesAssetExist(Out.SkPath))
    UEditorAssetLibrary::DeleteAsset(Out.SkPath);
  if (UEditorAssetLibrary::DoesAssetExist(Out.PaPath))
    UEditorAssetLibrary::DeleteAsset(Out.PaPath);

  UObject* SkObj = UEditorAssetLibrary::DuplicateAsset(kSkSrcPath, Out.SkPath);
  UObject* PaObj = UEditorAssetLibrary::DuplicateAsset(kPaSrcPath, Out.PaPath);

  Out.SkelMesh  = Cast<USkeletalMesh>(SkObj);
  Out.PhysAsset = Cast<UPhysicsAsset>(PaObj);

  if (!Out.SkelMesh || !Out.PhysAsset)
  {
    return FStageError::Make(EStageCode::BuildSK, EStageErrorKind::DuplicateFailed,
        FString::Printf(TEXT("Could not duplicate SK_USDVehicleBase or its PhysicsAsset (SK=%s PA=%s)"),
            Out.SkelMesh  ? *Out.SkelMesh->GetName()  : TEXT("null"),
            Out.PhysAsset ? *Out.PhysAsset->GetName() : TEXT("null")));
  }

  UE_LOG(LogCarlaTools, Display,
         TEXT("VI.Stage.BuildSK: duplicated SK=%s PA=%s"),
         *Out.SkelMesh->GetName(), *Out.PhysAsset->GetName());

  return FStageError::Ok();
}

}
