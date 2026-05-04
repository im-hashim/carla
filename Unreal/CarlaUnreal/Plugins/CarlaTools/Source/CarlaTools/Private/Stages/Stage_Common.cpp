#include "Stages/Stage_Common.h"

static const TCHAR* StageName(EStageCode S)
{
  switch (S)
  {
    case EStageCode::Preflight:   return TEXT("Preflight");
    case EStageCode::Interchange: return TEXT("Interchange");
    case EStageCode::BuildSK:     return TEXT("BuildSK");
    case EStageCode::BuildBP:     return TEXT("BuildBP");
    case EStageCode::Persist:     return TEXT("Persist");
  }
  return TEXT("?");
}

static const TCHAR* KindName(EStageErrorKind K)
{
  switch (K)
  {
    case EStageErrorKind::None:                    return TEXT("None");
    case EStageErrorKind::BadFilename:             return TEXT("BadFilename");
    case EStageErrorKind::MalformedMesh:           return TEXT("MalformedMesh");
    case EStageErrorKind::NoVerts:                 return TEXT("NoVerts");
    case EStageErrorKind::AxisAmbiguous:           return TEXT("AxisAmbiguous");
    case EStageErrorKind::InterchangeFailed:       return TEXT("InterchangeFailed");
    case EStageErrorKind::EmptyMesh:               return TEXT("EmptyMesh");
    case EStageErrorKind::TemplateMissing:         return TEXT("TemplateMissing");
    case EStageErrorKind::DuplicateFailed:         return TEXT("DuplicateFailed");
    case EStageErrorKind::BoneEditFailed:          return TEXT("BoneEditFailed");
    case EStageErrorKind::ParentLoadFailed:        return TEXT("ParentLoadFailed");
    case EStageErrorKind::ParentSlotProbeMismatch: return TEXT("ParentSlotProbeMismatch");
    case EStageErrorKind::CreateBPFailed:          return TEXT("CreateBPFailed");
    case EStageErrorKind::WheelBPFailed:           return TEXT("WheelBPFailed");
    case EStageErrorKind::PersistFailed:           return TEXT("PersistFailed");
  }
  return TEXT("?");
}

FString FStageError::ToString() const
{
  if (IsOk()) return TEXT("Ok");
  return FString::Printf(TEXT("[%s/%s] %s"), StageName(Stage), KindName(Kind), *Diagnostic);
}
