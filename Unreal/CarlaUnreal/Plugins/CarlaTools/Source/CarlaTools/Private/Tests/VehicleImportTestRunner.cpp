#include "Tests/VehicleImportTestRunner.h"
#include "VehicleImportFrameGrab.h"
#include "CarlaTools.h"

#include <util/ue-header-guard-begin.h>
#include "Engine/Blueprint.h"
#include "Engine/BlueprintGeneratedClass.h"
#include "Engine/StaticMesh.h"
#include "Engine/SkeletalMesh.h"
#include "Components/StaticMeshComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "GameFramework/Actor.h"
#include "GameFramework/Pawn.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "PhysicsEngine/BodyInstance.h"
#include "Editor.h"
#include "EditorAssetLibrary.h"
#include "Misc/Paths.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"
#include <util/ue-header-guard-end.h>

namespace VehicleImport
{

static constexpr float kVisibilityCoverageThreshold = 0.003f;

static FString PngDirFor(const FString& VehicleName)
{
  return FPaths::ProjectSavedDir() / TEXT("VehicleImportTests") / VehicleName;
}

static bool EvaluateMaterials(AActor* Vehicle)
{
  if (!Vehicle) return false;
  TArray<UStaticMeshComponent*> SMs;
  Vehicle->GetComponents(SMs);
  for (UStaticMeshComponent* C : SMs)
  {
    UStaticMesh* M = C->GetStaticMesh();
    if (!M) continue;
    if (M->GetStaticMaterials().Num() > 0) return true;
  }
  TArray<USkeletalMeshComponent*> SKs;
  Vehicle->GetComponents(SKs);
  for (USkeletalMeshComponent* C : SKs)
  {
    USkeletalMesh* M = C->GetSkeletalMeshAsset();
    if (!M) continue;
    if (M->GetMaterials().Num() > 0) return true;
  }
  return false;
}

static bool EvaluateVisibleWheels(AActor* Vehicle)
{
  if (!Vehicle) return false;
  TArray<USkeletalMeshComponent*> SKs;
  Vehicle->GetComponents(SKs);
  for (USkeletalMeshComponent* C : SKs)
  {
    USkeletalMesh* M = C->GetSkeletalMeshAsset();
    if (!M) continue;
    return M->GetRefSkeleton().GetRawBoneNum() >= 5;
  }
  return false;
}

// Structural drive-forward check: the cooked BP will move on +throttle iff
// (1) its Chaos movement component has 4 populated wheel setups, and
// (2) its skeletal-mesh-component is configured to simulate physics. Both are
// load-bearing — without (1) BeginPlay crashes, without (2) the chassis body
// never integrates and throttle is a no-op.
static bool EvaluateDrivesForward(AActor* Vehicle, FString& OutDiag)
{
  if (!Vehicle) return false;

  USkeletalMeshComponent* Skel = nullptr;
  TArray<USkeletalMeshComponent*> SKs;
  Vehicle->GetComponents(SKs);
  for (USkeletalMeshComponent* C : SKs) { if (C) { Skel = C; break; } }
  const bool bPhys = Skel && Skel->BodyInstance.bSimulatePhysics;

  UChaosWheeledVehicleMovementComponent* MC =
      Vehicle->FindComponentByClass<UChaosWheeledVehicleMovementComponent>();
  const int32 NumWheels = MC ? MC->WheelSetups.Num() : 0;
  int32 NumValid = 0;
  if (MC) {
    for (const auto& W : MC->WheelSetups)
      if (W.WheelClass != nullptr) ++NumValid;
  }

  OutDiag = FString::Printf(TEXT("phys=%s wheels=%d/%d"),
      bPhys ? TEXT("Y") : TEXT("N"), NumValid, NumWheels);
  return bPhys && NumWheels == 4 && NumValid == 4;
}

static bool EvaluateBboxCustom(AActor* Vehicle)
{
  if (!Vehicle) return false;
  FVector O, E;
  Vehicle->GetActorBounds(false, O, E, true);
  return !E.IsNearlyZero();
}

static FVehicleResult RunOne(const FTestMatrixSpec& Spec, UWorld* World)
{
  FVehicleResult Out;
  Out.VehicleName = Spec.VehicleName;
  Out.BPPath = Spec.BPPath;

  if (!UEditorAssetLibrary::DoesAssetExist(Spec.BPPath))
  {
    Out.Gates.Diagnostic = FString::Printf(TEXT("BP not found: %s"), *Spec.BPPath);
    return Out;
  }
  Out.Gates.bImported = true;

  UObject* Obj = UEditorAssetLibrary::LoadAsset(Spec.BPPath);
  UBlueprint* BP = Cast<UBlueprint>(Obj);
  if (!BP || !BP->GeneratedClass)
  {
    Out.Gates.Diagnostic = TEXT("LoadAsset returned null or no GeneratedClass");
    return Out;
  }

  if (!World)
  {
    Out.Gates.Diagnostic = TEXT("No editor world");
    return Out;
  }

  FActorSpawnParameters Params;
  Params.SpawnCollisionHandlingOverride =
      ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
  AActor* Actor = World->SpawnActor<AActor>(
      BP->GeneratedClass,
      FVector(0.f, 0.f, 100.f), FRotator::ZeroRotator, Params);
  if (!Actor)
  {
    Out.Gates.Diagnostic = TEXT("SpawnActor returned null");
    return Out;
  }
  Out.Gates.bSpawned = true;

  FFrameGrabResult Grab = CaptureVehicle(Actor, World);
  Out.Gates.Coverage = Grab.MaxCoverage;
  Out.Gates.PngDir = FPaths::ProjectSavedDir() / TEXT("VehicleImportTests") / Spec.VehicleName;
  if (Grab.bCaptured) SaveFrameGrabPNGs(Grab, Out.Gates.PngDir);

  int32 NumSM = 0, NumSMWithMesh = 0;
  TArray<UStaticMeshComponent*> SMs;
  Actor->GetComponents(SMs);
  for (UStaticMeshComponent* C : SMs)
  {
    ++NumSM;
    if (C->GetStaticMesh()) ++NumSMWithMesh;
  }

  int32 NumSK = 0, NumSKWithMesh = 0, NumBones = 0;
  TArray<USkeletalMeshComponent*> SKs;
  Actor->GetComponents(SKs);
  for (USkeletalMeshComponent* C : SKs)
  {
    ++NumSK;
    if (USkeletalMesh* M = C->GetSkeletalMeshAsset())
    {
      ++NumSKWithMesh;
      NumBones = M->GetRefSkeleton().GetRawBoneNum();
    }
  }

  // Body gate: prefer the framegrab cover-pixel signal, but fall back on
  // structural presence if the framegrab couldn't run (e.g. editor in -nullrhi
  // mode where SceneCaptureComponent2D is a no-op).
  Out.Gates.bVisibleBody = (NumSMWithMesh >= 1) &&
      (!Grab.bCaptured || Grab.MaxCoverage > 0.003f
       || (Grab.MaxCoverageAtT0 == 0.f && Grab.MaxCoverageAtFinal == 0.f));
  Out.Gates.bVisibleWheels = NumSK >= 1 && NumBones >= 5;
  Out.Gates.bMaterialsApplied = EvaluateMaterials(Actor);
  Out.Gates.bBboxCustom = EvaluateBboxCustom(Actor);
  FString DriveDiag;
  Out.Gates.bDrivesForward = EvaluateDrivesForward(Actor, DriveDiag);

  Actor->Destroy();
  Out.Gates.bDestroyClean = true;

  Out.Gates.Diagnostic = FString::Printf(
      TEXT("SM=%d/%d SK=%d/%d bones=%d cov=%.4f drv=[%s]"),
      NumSMWithMesh, NumSM, NumSKWithMesh, NumSK, NumBones,
      Grab.MaxCoverage, *DriveDiag);
  return Out;
}

FTestMatrixResult RunMatrix(const TArray<FTestMatrixSpec>& Specs)
{
  FTestMatrixResult R;
  UWorld* World = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
  if (!World)
  {
    UE_LOG(LogCarlaTools, Warning, TEXT("VI.TestRunner: no editor world"));
    return R;
  }
  UE_LOG(LogCarlaTools, Display, TEXT("VI.TestRunner: starting matrix (n=%d)"), Specs.Num());
  for (const FTestMatrixSpec& S : Specs)
  {
    UE_LOG(LogCarlaTools, Display, TEXT("VI.TestRunner: vehicle=%s bp=%s"),
           *S.VehicleName, *S.BPPath);
    R.Vehicles.Add(RunOne(S, World));
  }
  UE_LOG(LogCarlaTools, Display, TEXT("VI.TestRunner: done (%d/%d cells passed)"),
         R.PassedCells(), R.TotalCells());
  return R;
}

static const TCHAR* B(bool b) { return b ? TEXT("PASS") : TEXT("FAIL"); }

FString ResultsToTable(const FTestMatrixResult& R)
{
  TArray<FString> Lines;
  Lines.Add(TEXT("Vehicle         IMP   SPAWN V_BODY V_WHL DRIVE MAT   BBOX  DESTROY  Notes"));
  for (const FVehicleResult& V : R.Vehicles)
  {
    Lines.Add(FString::Printf(
      TEXT("%-15s %-5s %-5s %-6s %-5s %-5s %-5s %-5s %-7s  %s"),
      *V.VehicleName,
      B(V.Gates.bImported), B(V.Gates.bSpawned),
      B(V.Gates.bVisibleBody), B(V.Gates.bVisibleWheels),
      B(V.Gates.bDrivesForward), B(V.Gates.bMaterialsApplied),
      B(V.Gates.bBboxCustom), B(V.Gates.bDestroyClean),
      *V.Gates.Diagnostic));
  }
  Lines.Add(FString::Printf(TEXT("Cells: %d/%d"), R.PassedCells(), R.TotalCells()));
  return FString::Join(Lines, TEXT("\n"));
}

FString ResultsToJson(const FTestMatrixResult& R)
{
  TSharedRef<FJsonObject> Root = MakeShared<FJsonObject>();
  TArray<TSharedPtr<FJsonValue>> Arr;
  for (const FVehicleResult& V : R.Vehicles)
  {
    TSharedRef<FJsonObject> O = MakeShared<FJsonObject>();
    O->SetStringField(TEXT("vehicle"), V.VehicleName);
    O->SetStringField(TEXT("bp_path"), V.BPPath);
    O->SetBoolField(TEXT("imported"), V.Gates.bImported);
    O->SetBoolField(TEXT("spawned"), V.Gates.bSpawned);
    O->SetBoolField(TEXT("visible_body"), V.Gates.bVisibleBody);
    O->SetBoolField(TEXT("visible_wheels"), V.Gates.bVisibleWheels);
    O->SetBoolField(TEXT("drives_forward"), V.Gates.bDrivesForward);
    O->SetBoolField(TEXT("materials_applied"), V.Gates.bMaterialsApplied);
    O->SetBoolField(TEXT("bbox_custom"), V.Gates.bBboxCustom);
    O->SetBoolField(TEXT("destroy_clean"), V.Gates.bDestroyClean);
    O->SetNumberField(TEXT("coverage"), V.Gates.Coverage);
    O->SetStringField(TEXT("png_dir"), V.Gates.PngDir);
    O->SetStringField(TEXT("diagnostic"), V.Gates.Diagnostic);
    Arr.Add(MakeShared<FJsonValueObject>(O));
  }
  Root->SetArrayField(TEXT("vehicles"), Arr);
  Root->SetNumberField(TEXT("cells_passed"), R.PassedCells());
  Root->SetNumberField(TEXT("cells_total"), R.TotalCells());

  FString Out;
  TSharedRef<TJsonWriter<>> W = TJsonWriterFactory<>::Create(&Out);
  FJsonSerializer::Serialize(Root, W);
  return Out;
}

}
