#include "Stages/Stage_BuildBP.h"
#include "VehicleImporter.h"
#include "VehicleImporter_Helpers.h"
#include "USDImporterWidget.h"
#include "CarlaTools.h"

#include <util/ue-header-guard-begin.h>
#include "Engine/Blueprint.h"
#include "Engine/BlueprintGeneratedClass.h"
#include "Engine/StaticMesh.h"
#include "Engine/SkeletalMesh.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "Components/StaticMeshComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "ChaosVehicleWheel.h"
#include "GameFramework/Actor.h"
#include "Editor.h"
#include "EditorAssetLibrary.h"
#include "Engine/EngineTypes.h"
#include <util/ue-header-guard-end.h>

namespace VehicleImport
{

FParentBPSlots ProbeParentClassSlots(UClass* ParentClass)
{
  FParentBPSlots Out;
  Out.BaseClass = ParentClass;
  if (!ParentClass) return Out;

  AActor* CDO = Cast<AActor>(ParentClass->GetDefaultObject());
  if (!CDO) return Out;

  TArray<UActorComponent*> Comps;
  CDO->GetComponents(Comps);

  TArray<FString> Lines;
  Lines.Add(FString::Printf(TEXT("ParentClass=%s"), *ParentClass->GetName()));

  float BestVol = 0.f;
  FName BestSlot = NAME_None;

  for (UActorComponent* C : Comps)
  {
    if (!C) continue;
    if (auto* SK = Cast<USkeletalMeshComponent>(C))
    {
      Out.bHasSkelMeshSlot = true;
      Lines.Add(FString::Printf(TEXT("  SK: %s"), *SK->GetName()));
      continue;
    }
    if (auto* SM = Cast<UStaticMeshComponent>(C))
    {
      ++Out.NumStaticMeshSlots;
      const FName Slot = SM->GetFName();
      if (UStaticMesh* Mesh = SM->GetStaticMesh())
      {
        const FBoxSphereBounds B = Mesh->GetBounds();
        const float Vol = B.BoxExtent.X * B.BoxExtent.Y * B.BoxExtent.Z;
        Lines.Add(FString::Printf(TEXT("  SM: %s vol=%.0f"), *Slot.ToString(), Vol));
        if (Vol > BestVol) { BestVol = Vol; BestSlot = Slot; }
      }
      else
      {
        Lines.Add(FString::Printf(TEXT("  SM: %s (empty)"), *Slot.ToString()));
        if (BestSlot == NAME_None) BestSlot = Slot;
      }
    }
  }

  Out.BodyStaticMeshSlotName = BestSlot;
  Out.DiagnosticDump = FString::Join(Lines, TEXT("\n"));
  return Out;
}

static UClass* LoadBaseClass(FStageError& OutErr)
{
  UObject* TplObj = UEditorAssetLibrary::LoadAsset(
      TEXT("/Game/Carla/Blueprints/Vehicles/BaseVehiclePawnNW"));
  UBlueprint* TplBP = Cast<UBlueprint>(TplObj);
  if (!TplBP || !TplBP->GeneratedClass)
  {
    OutErr = FStageError::Make(EStageCode::BuildBP, EStageErrorKind::ParentLoadFailed,
        TEXT("Could not load /Game/Carla/Blueprints/Vehicles/BaseVehiclePawnNW (Chaos parent)"));
    return nullptr;
  }
  return TplBP->GeneratedClass;
}

static bool BuildWheelBPs(
    const FNormalizedSpec& Norm,
    const FVehicleImportSpec& Spec,
    FBuildBPOutputs& Out,
    FWheelTemplates& OutTemplates,
    FStageError& OutErr)
{
  auto Make = [&](const FString& Suffix, const FWheelImportSpec& W)
      -> TSubclassOf<UChaosVehicleWheel>
  {
    UStaticMesh* Shrunk = MakeShrunkWheelShape(
        Norm.VehicleContentPath, Suffix, W.Radius, W.Width);
    if (Shrunk)
      Out.ShrunkWheelShapePaths.Add(Shrunk->GetPathName());
    auto R = CreateWheelBlueprint(
        Norm.VehicleContentPath,
        Norm.VehicleName + TEXT("_Wheel_") + Suffix,
        Suffix, W, Shrunk);
    UE_LOG(LogCarlaTools, Display,
           TEXT("VI.Stage.BuildBP: wheel %s %s (shape=%s)"),
           *Suffix, R ? TEXT("ok") : TEXT("FAILED"),
           Shrunk ? TEXT("shrunk") : TEXT("stock"));
    return R;
  };

  OutTemplates.WheelFL = Make(TEXT("FLW"), Spec.WheelFL);
  OutTemplates.WheelFR = Make(TEXT("FRW"), Spec.WheelFR);
  OutTemplates.WheelRL = Make(TEXT("RLW"), Spec.WheelRL);
  OutTemplates.WheelRR = Make(TEXT("RRW"), Spec.WheelRR);

  if (!OutTemplates.WheelFL || !OutTemplates.WheelFR ||
      !OutTemplates.WheelRL || !OutTemplates.WheelRR)
  {
    OutErr = FStageError::Make(EStageCode::BuildBP, EStageErrorKind::WheelBPFailed,
        TEXT("One or more wheel blueprints failed to create"));
    return false;
  }

  Out.WheelBPPaths.Add(Norm.VehicleContentPath / (Norm.VehicleName + TEXT("_Wheel_FLW")));
  Out.WheelBPPaths.Add(Norm.VehicleContentPath / (Norm.VehicleName + TEXT("_Wheel_FRW")));
  Out.WheelBPPaths.Add(Norm.VehicleContentPath / (Norm.VehicleName + TEXT("_Wheel_RLW")));
  Out.WheelBPPaths.Add(Norm.VehicleContentPath / (Norm.VehicleName + TEXT("_Wheel_RRW")));
  return true;
}

static void ApplyPostBPPhysicsAndSafety(
    const FString& BPPath,
    UPhysicsAsset* DupedPA,
    const FVehicleImportSpec& Spec,
    FBuildBPOutputs& Out)
{
  UObject* NewBPObj = UEditorAssetLibrary::LoadAsset(BPPath);
  UBlueprint* NewBP = Cast<UBlueprint>(NewBPObj);
  if (!NewBP) return;

  USkeletalMeshComponent* SkelComp2 = nullptr;
  if (NewBP->SimpleConstructionScript)
  {
    for (USCS_Node* Node : NewBP->SimpleConstructionScript->GetAllNodes())
    {
      if (auto* C = Cast<USkeletalMeshComponent>(
              Node->GetActualComponentTemplate(
                Cast<UBlueprintGeneratedClass>(NewBP->GeneratedClass))))
      {
        SkelComp2 = C; break;
      }
    }
  }
  if (!SkelComp2 && NewBP->GeneratedClass)
  {
    if (AActor* CDO = NewBP->GeneratedClass->GetDefaultObject<AActor>())
    {
      TArray<UActorComponent*> Comps;
      CDO->GetComponents(Comps);
      for (UActorComponent* C : Comps)
      {
        if (auto* SK = Cast<USkeletalMeshComponent>(C)) { SkelComp2 = SK; break; }
      }
    }
  }

  if (DupedPA)
  {
    ApplyChassisAabbResize(DupedPA, Spec, Spec.HasChassisAabb);
    if (SkelComp2)
    {
      SkelComp2->SetPhysicsAsset(DupedPA);
      SkelComp2->Modify();
    }
    NewBP->MarkPackageDirty();
    Out.PhysAssetPath = DupedPA->GetPathName();
    UE_LOG(LogCarlaTools, Display,
           TEXT("VI.Stage.BuildBP: applied PA pass on %s (resize=%s)"),
           *Out.PhysAssetPath, Spec.HasChassisAabb ? TEXT("yes") : TEXT("no"));
  }

  bool bChanged = false;
  if (auto* GenClass = Cast<UBlueprintGeneratedClass>(NewBP->GeneratedClass))
  {
    if (AActor* CDO = Cast<AActor>(GenClass->GetDefaultObject()))
    {
      if (CDO->SpawnCollisionHandlingMethod !=
          ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn)
      {
        CDO->SpawnCollisionHandlingMethod =
            ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;
        bChanged = true;
      }
    }
  }
  if (bChanged)
  {
    NewBP->Modify();
    if (UPackage* Pkg = NewBP->GetOutermost()) Pkg->MarkPackageDirty();
  }

#if ENGINE_MAJOR_VERSION >= 5
  auto FixWheelDefaults = [&](const FString& WheelBpPath, float Radius, float Width)
  {
    UObject* WObj = UEditorAssetLibrary::LoadAsset(WheelBpPath);
    UBlueprint* WBP = Cast<UBlueprint>(WObj);
    if (!WBP || !WBP->GeneratedClass) return;
    UChaosVehicleWheel* WCDO = WBP->GeneratedClass->GetDefaultObject<UChaosVehicleWheel>();
    if (!WCDO) return;
    bool bWheelChanged = false;
    const float SafeR = Radius > 1.f ? Radius : 33.f;
    const float SafeW = Width  > 1.f ? Width  : 22.f;
    if (WCDO->WheelRadius < 1.f) { WCDO->WheelRadius = SafeR; bWheelChanged = true; }
    if (WCDO->WheelWidth  < 1.f) { WCDO->WheelWidth  = SafeW; bWheelChanged = true; }
    if (bWheelChanged)
    {
      WBP->Modify();
      if (UPackage* Pkg = WBP->GetOutermost()) Pkg->MarkPackageDirty();
    }
  };
  if (Out.WheelBPPaths.Num() == 4)
  {
    FixWheelDefaults(Out.WheelBPPaths[0], Spec.WheelFL.Radius, Spec.WheelFL.Width);
    FixWheelDefaults(Out.WheelBPPaths[1], Spec.WheelFR.Radius, Spec.WheelFR.Width);
    FixWheelDefaults(Out.WheelBPPaths[2], Spec.WheelRL.Radius, Spec.WheelRL.Width);
    FixWheelDefaults(Out.WheelBPPaths[3], Spec.WheelRR.Radius, Spec.WheelRR.Width);
  }
#endif

  Out.BP = NewBP;
}

FStageError RunStage_BuildBP(
    const FNormalizedSpec& Norm,
    const FVehicleImportSpec& Spec,
    const FBuildBPInputs& In,
    FBuildBPOutputs& Out)
{
  FStageError Err = FStageError::Ok();

  UClass* BaseClass = LoadBaseClass(Err);
  if (!BaseClass) return Err;

  FParentBPSlots Slots = ProbeParentClassSlots(BaseClass);
  UE_LOG(LogCarlaTools, Display,
         TEXT("VI.Stage.BuildBP: parent slot probe:\n%s"),
         *Slots.DiagnosticDump);

  FWheelTemplates WheelTemplates;
  if (!BuildWheelBPs(Norm, Spec, Out, WheelTemplates, Err))
    return Err;

  UWorld* World = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
  if (!World)
  {
    return FStageError::Make(EStageCode::BuildBP, EStageErrorKind::CreateBPFailed,
        TEXT("No editor world available"));
  }

  Out.BPPath = Norm.VehicleContentPath / (TEXT("BP_") + Norm.VehicleName);
  UEditorAssetLibrary::DeleteAsset(Out.BPPath);

  FMergedVehicleMeshParts Parts;
  Parts.Body = In.BodyMesh;
  Parts.Anchors.WheelFL = FVector(Spec.WheelFL.X, Spec.WheelFL.Y, Spec.WheelFL.Z);
  Parts.Anchors.WheelFR = FVector(Spec.WheelFR.X, Spec.WheelFR.Y, Spec.WheelFR.Z);
  Parts.Anchors.WheelRL = FVector(Spec.WheelRL.X, Spec.WheelRL.Y, Spec.WheelRL.Z);
  Parts.Anchors.WheelRR = FVector(Spec.WheelRR.X, Spec.WheelRR.Y, Spec.WheelRR.Z);

  UUSDImporterWidget::GenerateNewVehicleBlueprint(
      World, BaseClass, In.SkelMesh, In.PhysAsset,
      Out.BPPath, Parts, WheelTemplates);

  ApplyPostBPPhysicsAndSafety(Out.BPPath, In.PhysAsset, Spec, Out);

  if (!Out.BP)
  {
    return FStageError::Make(EStageCode::BuildBP, EStageErrorKind::CreateBPFailed,
        FString::Printf(TEXT("Vehicle BP not created at %s"), *Out.BPPath));
  }
  return FStageError::Ok();
}

}
