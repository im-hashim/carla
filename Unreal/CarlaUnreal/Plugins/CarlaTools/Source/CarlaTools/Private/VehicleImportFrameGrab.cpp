#include "VehicleImportFrameGrab.h"
#include "CarlaTools.h"

#include <util/ue-header-guard-begin.h>
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/SceneCapture2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/DirectionalLight.h"
#include "Engine/SkyLight.h"
#include "Components/SkyLightComponent.h"
#include "Components/DirectionalLightComponent.h"
#include "GameFramework/Actor.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/World.h"
#include "Engine/Engine.h"
#include "RenderingThread.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/PlatformFileManager.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Modules/ModuleManager.h"
#include "Math/Box.h"
#include <util/ue-header-guard-end.h>

namespace VehicleImport
{

static constexpr int32 kFGW = 512;
static constexpr int32 kFGH = 512;
static constexpr int32 kDiffThreshold = 18;
static const float kSampleTimes[] = { 0.0f, 0.5f, 1.0f };
static const EFrameGrabAngle kAngles[] = {
    EFrameGrabAngle::Front, EFrameGrabAngle::ThreeQuarter, EFrameGrabAngle::Top };

static const TCHAR* AngleName(EFrameGrabAngle A)
{
  switch (A)
  {
    case EFrameGrabAngle::Front:        return TEXT("front");
    case EFrameGrabAngle::ThreeQuarter: return TEXT("3q");
    case EFrameGrabAngle::Top:          return TEXT("top");
  }
  return TEXT("?");
}

FString FFrameGrabSample::ToTraceString() const
{
  if (DiffPixelCount == 0)
  {
    return FString::Printf(TEXT("%s_t%.1f cov=0.0000 -"), AngleName(Angle), TimeSec);
  }
  return FString::Printf(
    TEXT("%s_t%.1f cov=%.4f ctr=(%d,%d) lum=%.0f bbox=%dx%d=%d"),
    AngleName(Angle), TimeSec, Coverage,
    (int32)CentroidX, (int32)CentroidY, MeanLuminance,
    (BBoxMaxX - BBoxMinX), (BBoxMaxY - BBoxMinY), BBoxArea);
}

FString FFrameGrabResult::ToTraceString() const
{
  TArray<FString> Parts;
  for (const FFrameGrabSample& S : Samples) Parts.Add(S.ToTraceString());
  return FString::Join(Parts, TEXT(" | "));
}

static void GetCameraTransform(EFrameGrabAngle A, const FVector& VehicleOrigin,
                               const FVector& VehicleExtent,
                               FVector& OutLoc, FRotator& OutRot)
{
  const float Reach = FMath::Max(200.f, 2.2f * VehicleExtent.Size());
  switch (A)
  {
    case EFrameGrabAngle::Front:
      OutLoc = VehicleOrigin + FVector(Reach, 0.f, VehicleExtent.Z * 0.6f);
      break;
    case EFrameGrabAngle::ThreeQuarter:
      OutLoc = VehicleOrigin + FVector(Reach * 0.7f, Reach * 0.6f, Reach * 0.4f);
      break;
    case EFrameGrabAngle::Top:
      OutLoc = VehicleOrigin + FVector(0.f, 0.f, Reach);
      break;
  }
  OutRot = (VehicleOrigin - OutLoc).Rotation();
}

static UTextureRenderTarget2D* MakeRT(UWorld* World)
{
  UTextureRenderTarget2D* RT = NewObject<UTextureRenderTarget2D>(World);
  RT->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8;
  RT->ClearColor = FLinearColor(0.f, 0.f, 0.f, 1.f);
  RT->bAutoGenerateMips = false;
  RT->InitAutoFormat(kFGW, kFGH);
  RT->UpdateResourceImmediate(true);
  return RT;
}

static void HideVehicle(AActor* V, bool bHide)
{
  if (!V) return;
  V->SetActorHiddenInGame(bHide);
  TArray<UPrimitiveComponent*> Prims;
  V->GetComponents(Prims);
  for (UPrimitiveComponent* P : Prims)
  {
    P->SetVisibility(!bHide, true);
    P->SetHiddenInGame(bHide, true);
  }
}

static bool CaptureOnce(USceneCaptureComponent2D* Cap, UTextureRenderTarget2D* RT,
                       TArray<FColor>& OutPixels)
{
  if (!Cap || !RT) return false;
  Cap->TextureTarget = RT;
  Cap->bCaptureEveryFrame = false;
  Cap->bCaptureOnMovement = false;
  Cap->bAlwaysPersistRenderingState = true;
  Cap->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
  Cap->CaptureScene();
  FlushRenderingCommands();

  FTextureRenderTargetResource* RTRes = RT->GameThread_GetRenderTargetResource();
  if (!RTRes) return false;
  OutPixels.Empty(kFGW * kFGH);
  const bool ok = RTRes->ReadPixels(OutPixels);

  if (ok && OutPixels.Num() > 0)
  {
    int32 Mn = 255, Mx = 0;
    for (const FColor& C : OutPixels)
    {
      const int32 V = FMath::Max3((int32)C.R, (int32)C.G, (int32)C.B);
      if (V < Mn) Mn = V;
      if (V > Mx) Mx = V;
    }
    UE_LOG(LogCarlaTools, Display,
           TEXT("VI.FrameGrab.Capture: pix=%d minMaxRGB=[%d,%d]"),
           OutPixels.Num(), Mn, Mx);
  }
  return ok;
}

static void AnalyseDiff(const TArray<FColor>& Bg, const TArray<FColor>& Fg,
                        FFrameGrabSample& S)
{
  if (Bg.Num() != Fg.Num() || Bg.Num() == 0) return;
  const int32 W = S.Width, H = S.Height;
  int32 N = 0;
  int64 SumX = 0, SumY = 0;
  int64 SumLum = 0;
  int32 MinX = W, MinY = H, MaxX = -1, MaxY = -1;

  for (int32 i = 0; i < Fg.Num(); ++i)
  {
    const int32 D = FMath::Abs((int32)Bg[i].R - (int32)Fg[i].R)
                  + FMath::Abs((int32)Bg[i].G - (int32)Fg[i].G)
                  + FMath::Abs((int32)Bg[i].B - (int32)Fg[i].B);
    if (D <= kDiffThreshold) continue;
    ++N;
    const int32 X = i % W;
    const int32 Y = i / W;
    SumX += X;
    SumY += Y;
    const int32 Lum = (int32)Fg[i].R * 299 / 1000
                   + (int32)Fg[i].G * 587 / 1000
                   + (int32)Fg[i].B * 114 / 1000;
    SumLum += Lum;
    if (X < MinX) MinX = X;
    if (Y < MinY) MinY = Y;
    if (X > MaxX) MaxX = X;
    if (Y > MaxY) MaxY = Y;
  }

  S.DiffPixelCount = N;
  S.Coverage = (W * H) > 0 ? float(N) / float(W * H) : 0.f;
  if (N > 0)
  {
    S.CentroidX = float(SumX) / float(N);
    S.CentroidY = float(SumY) / float(N);
    S.MeanLuminance = float(SumLum) / float(N);
    S.BBoxMinX = MinX; S.BBoxMinY = MinY;
    S.BBoxMaxX = MaxX; S.BBoxMaxY = MaxY;
    S.BBoxArea = (MaxX - MinX) * (MaxY - MinY);
  }
}

static FFrameGrabSample CaptureSample(EFrameGrabAngle Angle, float TimeSec,
                                      AActor* Vehicle, UWorld* World,
                                      USceneCaptureComponent2D* Cap,
                                      UTextureRenderTarget2D* RT,
                                      const FVector& Origin, const FVector& Extent)
{
  FFrameGrabSample S;
  S.Angle = Angle;
  S.TimeSec = TimeSec;
  S.Width = kFGW;
  S.Height = kFGH;

  FVector Loc; FRotator Rot;
  GetCameraTransform(Angle, Origin, Extent, Loc, Rot);
  Cap->SetWorldLocationAndRotation(Loc, Rot.Quaternion());

  TArray<FColor> Bg, Fg;
  HideVehicle(Vehicle, true);
  CaptureOnce(Cap, RT, Bg);
  HideVehicle(Vehicle, false);
  CaptureOnce(Cap, RT, Fg);

  S.Pixels = MoveTemp(Fg);
  AnalyseDiff(Bg, S.Pixels, S);
  return S;
}

static void TickWorldSeconds(UWorld* World, float DeltaSec)
{
  if (!World || DeltaSec <= 0.f) return;
  FPlatformProcess::Sleep(FMath::Min(DeltaSec, 0.05f));
}

FFrameGrabResult CaptureVehicle(AActor* Vehicle, UWorld* World)
{
  FFrameGrabResult Out;
  if (!Vehicle || !World)
  {
    Out.Diagnostic = TEXT("Null Vehicle or World");
    return Out;
  }

  FVector Origin = Vehicle->GetActorLocation();
  FVector Extent = FVector(200.f, 100.f, 80.f);
  {
    FVector O, E;
    Vehicle->GetActorBounds(false, O, E, true);
    if (!E.IsNearlyZero()) { Origin = O; Extent = E; }
  }

  ADirectionalLight* SunLight = World->SpawnActor<ADirectionalLight>(
      ADirectionalLight::StaticClass(),
      Origin + FVector(0.f, 0.f, 500.f),
      FRotator(-45.f, 30.f, 0.f));
  if (SunLight && SunLight->GetLightComponent())
  {
    SunLight->GetLightComponent()->SetMobility(EComponentMobility::Movable);
    SunLight->GetLightComponent()->SetIntensity(8.f);
  }
  ASkyLight* Sky = World->SpawnActor<ASkyLight>(
      ASkyLight::StaticClass(), Origin, FRotator::ZeroRotator);
  if (Sky && Sky->GetLightComponent())
  {
    Sky->GetLightComponent()->SetMobility(EComponentMobility::Movable);
    Sky->GetLightComponent()->SetIntensity(2.5f);
    Sky->GetLightComponent()->RecaptureSky();
  }

  ASceneCapture2D* CapActor = World->SpawnActor<ASceneCapture2D>(
      ASceneCapture2D::StaticClass(), Origin, FRotator::ZeroRotator);
  if (!CapActor)
  {
    if (SunLight) SunLight->Destroy();
    if (Sky) Sky->Destroy();
    Out.Diagnostic = TEXT("SpawnActor<ASceneCapture2D> failed");
    return Out;
  }
  USceneCaptureComponent2D* Cap = CapActor->GetCaptureComponent2D();
  UTextureRenderTarget2D* RT = MakeRT(World);

  Out.Samples.Reserve(9);
  for (int32 ti = 0; ti < UE_ARRAY_COUNT(kSampleTimes); ++ti)
  {
    const float T = kSampleTimes[ti];
    if (ti > 0)
    {
      TickWorldSeconds(World, T - kSampleTimes[ti - 1]);
      FVector NewOrigin = Vehicle->GetActorLocation();
      Origin = NewOrigin;
    }
    for (EFrameGrabAngle A : kAngles)
    {
      Out.Samples.Add(CaptureSample(A, T, Vehicle, World, Cap, RT, Origin, Extent));
    }
  }

  CapActor->Destroy();
  if (SunLight) SunLight->Destroy();
  if (Sky) Sky->Destroy();

  for (const FFrameGrabSample& S : Out.Samples)
  {
    Out.MaxCoverage = FMath::Max(Out.MaxCoverage, S.Coverage);
    if (S.TimeSec == 0.0f) Out.MaxCoverageAtT0 = FMath::Max(Out.MaxCoverageAtT0, S.Coverage);
    if (S.TimeSec >= 0.99f) Out.MaxCoverageAtFinal = FMath::Max(Out.MaxCoverageAtFinal, S.Coverage);
  }
  Out.bCaptured = true;
  Out.Diagnostic = Out.ToTraceString();

  UE_LOG(LogCarlaTools, Display, TEXT("VI.FrameGrab: %s"), *Out.Diagnostic);
  return Out;
}

static bool WritePNG(const FFrameGrabSample& S, const FString& Path)
{
  if (S.Pixels.Num() != S.Width * S.Height) return false;
  IImageWrapperModule& M = FModuleManager::LoadModuleChecked<IImageWrapperModule>(
      FName("ImageWrapper"));
  TSharedPtr<IImageWrapper> W = M.CreateImageWrapper(EImageFormat::PNG);
  if (!W.IsValid()) return false;
  if (!W->SetRaw(S.Pixels.GetData(), S.Pixels.Num() * sizeof(FColor),
                 S.Width, S.Height, ERGBFormat::BGRA, 8))
    return false;
  TArray64<uint8> Encoded = W->GetCompressed(100);
  return FFileHelper::SaveArrayToFile(Encoded, *Path);
}

bool SaveFrameGrabPNGs(const FFrameGrabResult& R, const FString& OutDir)
{
  IPlatformFile& PF = FPlatformFileManager::Get().GetPlatformFile();
  if (!PF.DirectoryExists(*OutDir)) PF.CreateDirectoryTree(*OutDir);
  bool ok = true;
  for (const FFrameGrabSample& S : R.Samples)
  {
    const FString FName = FString::Printf(TEXT("%s_t%03d.png"),
        AngleName(S.Angle), (int32)FMath::RoundToInt(S.TimeSec * 100.f));
    ok &= WritePNG(S, OutDir / FName);
  }
  UE_LOG(LogCarlaTools, Display,
         TEXT("VI.FrameGrab: %d PNGs %s -> %s (max=%.3f, t0=%.3f, final=%.3f)"),
         R.Samples.Num(), ok ? TEXT("written") : TEXT("FAILED"), *OutDir,
         R.MaxCoverage, R.MaxCoverageAtT0, R.MaxCoverageAtFinal);
  return ok;
}

}
