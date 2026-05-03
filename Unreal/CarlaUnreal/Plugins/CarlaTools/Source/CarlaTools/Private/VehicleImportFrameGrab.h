#pragma once

#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include <util/ue-header-guard-end.h>

class AActor;
class UWorld;

namespace VehicleImport
{
  enum class EFrameGrabAngle : uint8 { Front, ThreeQuarter, Top };

  struct FFrameGrabSample
  {
    EFrameGrabAngle Angle = EFrameGrabAngle::Front;
    float TimeSec = 0.f;
    int32 Width = 512;
    int32 Height = 512;
    TArray<FColor> Pixels;
    int32 DiffPixelCount = 0;
    float Coverage = 0.f;
    float CentroidX = 0.f;
    float CentroidY = 0.f;
    float MeanLuminance = 0.f;
    int32 BBoxMinX = 0, BBoxMinY = 0, BBoxMaxX = 0, BBoxMaxY = 0;
    int32 BBoxArea = 0;

    FString ToTraceString() const;
  };

  struct FFrameGrabResult
  {
    bool bCaptured = false;
    FString Diagnostic;
    TArray<FFrameGrabSample> Samples;
    float MaxCoverage = 0.f;
    float MaxCoverageAtT0 = 0.f;
    float MaxCoverageAtFinal = 0.f;

    FString ToTraceString() const;
  };

  FFrameGrabResult CaptureVehicle(AActor* Vehicle, UWorld* World);
  bool SaveFrameGrabPNGs(const FFrameGrabResult& R, const FString& OutDir);
}
