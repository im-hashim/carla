#include "Stages/Stage_Interchange.h"
#include "VehicleImporter.h"
#include "VehicleImporter_Helpers.h"
#include "CarlaTools.h"

#include <util/ue-header-guard-begin.h>
#include "Engine/StaticMesh.h"
#include "EditorAssetLibrary.h"
#include <util/ue-header-guard-end.h>

namespace VehicleImport
{

FStageError RunStage_Interchange(
    const FNormalizedSpec& Norm,
    const FVehicleImportSpec& Spec,
    FInterchangeOutputs& Out)
{
  if (UEditorAssetLibrary::DoesDirectoryExist(Norm.VehicleContentPath))
  {
    UE_LOG(LogCarlaTools, Display,
           TEXT("VI.Stage.Interchange: %s already exists - deleting before re-import"),
           *Norm.VehicleContentPath);
    UEditorAssetLibrary::DeleteDirectory(Norm.VehicleContentPath);
  }

  UStaticMesh* BodyMesh = ImportStaticMesh(
      Norm.MeshFilePathToImport,
      Norm.VehicleContentPath,
      Norm.VehicleName + TEXT("_body"),
      Spec);

  if (!BodyMesh)
  {
    return FStageError::Make(EStageCode::Interchange, EStageErrorKind::InterchangeFailed,
        FString::Printf(TEXT("Interchange returned no UStaticMesh for %s"),
                        *Norm.MeshFilePathToImport));
  }

  Out.BodyMesh = BodyMesh;
  return FStageError::Ok();
}

}
