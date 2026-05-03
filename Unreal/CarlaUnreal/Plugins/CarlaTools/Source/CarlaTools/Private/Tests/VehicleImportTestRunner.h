#pragma once

#include "VehicleImportFrameGrab.h"

#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include <util/ue-header-guard-end.h>

namespace VehicleImport
{
  struct FTestMatrixSpec
  {
    FString VehicleName;
    FString BPPath;
  };

  struct FGateResults
  {
    bool bImported = false;
    bool bSpawned = false;
    bool bVisibleBody = false;
    bool bVisibleWheels = false;
    bool bDrivesForward = false;
    bool bMaterialsApplied = false;
    bool bBboxCustom = false;
    bool bDestroyClean = false;
    float Coverage = 0.f;
    FString PngDir;
    FString Diagnostic;

    int32 PassCount() const
    {
      int32 N = 0;
      if (bImported) ++N;
      if (bSpawned) ++N;
      if (bVisibleBody) ++N;
      if (bVisibleWheels) ++N;
      if (bDrivesForward) ++N;
      if (bMaterialsApplied) ++N;
      if (bBboxCustom) ++N;
      if (bDestroyClean) ++N;
      return N;
    }
  };

  struct FVehicleResult
  {
    FString VehicleName;
    FString BPPath;
    FGateResults Gates;
  };

  struct FTestMatrixResult
  {
    TArray<FVehicleResult> Vehicles;
    int32 TotalCells() const { return Vehicles.Num() * 8; }
    int32 PassedCells() const
    {
      int32 N = 0;
      for (const FVehicleResult& V : Vehicles) N += V.Gates.PassCount();
      return N;
    }
  };

  FTestMatrixResult RunMatrix(const TArray<FTestMatrixSpec>& Specs);
  FString ResultsToJson(const FTestMatrixResult& R);
  FString ResultsToTable(const FTestMatrixResult& R);
}
