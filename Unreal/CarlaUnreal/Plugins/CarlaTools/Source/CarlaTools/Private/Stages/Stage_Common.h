#pragma once

#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include <util/ue-header-guard-end.h>

class UStaticMesh;
class USkeletalMesh;
class UPhysicsAsset;
class UBlueprint;
class UClass;
struct FVehicleImportSpec;

enum class EStageCode : uint8
{
  Preflight,
  Interchange,
  BuildSK,
  BuildBP,
  Persist
};

enum class EStageErrorKind : uint8
{
  None,
  BadFilename,
  MalformedMesh,
  NoVerts,
  AxisAmbiguous,
  InterchangeFailed,
  EmptyMesh,
  TemplateMissing,
  DuplicateFailed,
  BoneEditFailed,
  ParentLoadFailed,
  ParentSlotProbeMismatch,
  CreateBPFailed,
  WheelBPFailed,
  PersistFailed
};

struct FStageError
{
  EStageCode Stage = EStageCode::Preflight;
  EStageErrorKind Kind = EStageErrorKind::None;
  FString Diagnostic;

  bool IsOk() const { return Kind == EStageErrorKind::None; }
  static FStageError Ok() { return FStageError{}; }
  static FStageError Make(EStageCode S, EStageErrorKind K, const FString& Msg)
  {
    return FStageError{ S, K, Msg };
  }

  FString ToString() const;
};

struct FNormalizedSpec
{
  FString VehicleName;
  FString OriginalMeshFilePath;
  FString MeshFilePathToImport;
  FString VehicleContentPath;
  bool bWasSanitized = false;
  bool bHeadersStripped = false;
  bool bWasCanonicalized = false;
  int32 ForwardAxisSign = +1;
  // Canonicalize() result: source axis index that is mapped to +X / +Y / +Z
  // in the rewritten OBJ. -1 means "not canonicalized". Useful for downstream
  // diagnostics and for keeping wheel-position math in sync with the rewrite.
  int32 SrcLateralAxis = -1;
  int32 SrcForwardAxis = -1;
  int32 SrcUpAxis = -1;
  float ScaleAppliedToCm = 1.0f;
};
