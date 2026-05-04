#include "Stages/Stage_Persist.h"
#include "CarlaTools.h"

#include <util/ue-header-guard-begin.h>
#include "EditorAssetLibrary.h"
#include <util/ue-header-guard-end.h>

namespace VehicleImport
{

FStageError RunStage_Persist(const FPersistInputs& In, FPersistOutputs& Out)
{
  TArray<FString> ToSave;
  ToSave.Add(In.VehicleContentPath / (In.VehicleName + TEXT("_body")));
  for (const FString& W : In.WheelBPPaths) ToSave.Add(W);
  ToSave.Add(In.BPPath);
  if (!In.PhysAssetPath.IsEmpty()) ToSave.Add(In.PhysAssetPath);
  for (const FString& P : In.ShrunkWheelShapePaths) ToSave.Add(P);

  Out.RequestedCount = ToSave.Num();
  Out.SavedCount = 0;
  for (const FString& Path : ToSave)
  {
    if (UEditorAssetLibrary::SaveAsset(Path, false))
      ++Out.SavedCount;
    else
      UE_LOG(LogCarlaTools, Warning,
             TEXT("VI.Stage.Persist: SaveAsset failed for %s"), *Path);
  }

  UE_LOG(LogCarlaTools, Display,
         TEXT("VI.Stage.Persist: %d/%d assets saved"),
         Out.SavedCount, Out.RequestedCount);

  if (Out.SavedCount == 0 && Out.RequestedCount > 0)
  {
    return FStageError::Make(EStageCode::Persist, EStageErrorKind::PersistFailed,
        TEXT("No assets persisted to disk"));
  }
  return FStageError::Ok();
}

}
