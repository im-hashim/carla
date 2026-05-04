// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB). This work is licensed under the terms of the MIT license. For a copy, see <https://opensource.org/licenses/MIT>.

#include "USDImporterWidget.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"
#include "CarlaTools.h"

#include <util/ue-header-guard-begin.h>
#include "Runtime/Launch/Resources/Version.h"
#include "ChaosVehicleMovementComponent.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "ChaosVehicleWheel.h"
#include <util/ue-header-guard-end.h>

#ifdef WITH_OMNIVERSE
  #include "USDCARLAInterface.h"
#endif

#include <util/ue-header-guard-begin.h>
#include "Kismet/GameplayStatics.h"
#include "MeshMerge/MeshMergingSettings.h"
#include "Modules/ModuleManager.h"
#include "IMeshMergeUtilities.h"
#include "MeshMergeModule.h"
#include "Components/PrimitiveComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Kismet/KismetSystemLibrary.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "Engine/Blueprint.h"
#include "ReferenceSkeleton.h"
#include "Components/SkeletalMeshComponent.h"
#include "PackageHelperFunctions.h"
#include "EditorAssetLibrary.h"
#include "Components/LightComponent.h"
#include "Components/PointLightComponent.h"
#include "Components/SpotLightComponent.h"
#include "IAssetTools.h"
#include "AssetToolsModule.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "Factories/BlueprintFactory.h"
#include "Kismet2/BlueprintEditorUtils.h"
#include "BlueprintEditor.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "PhysicsEngine/SkeletalBodySetup.h"
#include "UObject/SavePackage.h"
#include "Misc/PackageName.h"
#include <util/ue-header-guard-end.h>

#include <unordered_map>
#include <string>

void UUSDImporterWidget::ImportUSDProp(
    const FString& USDPath, const FString& DestinationAssetPath, bool bAsBlueprint)
{
#ifdef WITH_OMNIVERSE
  UUSDCARLAInterface::ImportUSD(USDPath, DestinationAssetPath, false, bAsBlueprint);
#else
  UE_LOG(LogCarlaTools, Error, TEXT("Omniverse Plugin is not enabled"));
#endif

}

void UUSDImporterWidget::ImportUSDVehicle(
    const FString& USDPath,
    const FString& DestinationAssetPath,
    FWheelTemplates BaseWheelData,
    TArray<FVehicleLight>& LightList,
    FWheelTemplates& WheelObjects,
    bool bAsBlueprint)
{
#ifdef WITH_OMNIVERSE
  // Import meshes
  UUSDCARLAInterface::ImportUSD(USDPath, DestinationAssetPath, false, bAsBlueprint);
  // Import Lights
  TArray<FUSDCARLALight> USDLights = UUSDCARLAInterface::GetUSDLights(USDPath);
  LightList.Empty();
  for (const FUSDCARLALight& USDLight : USDLights)
  {
    FVehicleLight Light {USDLight.Name, USDLight.Location, USDLight.Color};
    LightList.Add(Light);
  }
  // Import Wheel and suspension data
  TArray<FUSDCARLAWheelData> WheelsData = UUSDCARLAInterface::GetUSDWheelData(USDPath);
  auto CreateVehicleWheel =
      [&](const FUSDCARLAWheelData& WheelData,
         TSubclassOf<UChaosVehicleWheel> TemplateClass,
         const FString &PackagePathName)
      -> TSubclassOf<UChaosVehicleWheel>
  {
    // Get a reference to the editor subsystem
    constexpr float MToCM = 100.f;
    constexpr float RadToDeg = 360.f/3.14159265359f;
    FString BlueprintName =  FPaths::GetBaseFilename(PackagePathName);
    FString BlueprintPath = FPaths::GetPath(PackagePathName);
    IAssetTools& AssetTools = FModuleManager::LoadModuleChecked<FAssetToolsModule>("AssetTools").Get();
    // Create a new Blueprint factory
    UBlueprintFactory* Factory = NewObject<UBlueprintFactory>();
    // Set the parent class for the new Blueprint
    Factory->ParentClass = TemplateClass;
    // Create a new Blueprint asset with the given name
    UObject* NewAsset = AssetTools.CreateAsset(BlueprintName, BlueprintPath, UBlueprint::StaticClass(), Factory);
    // Cast the new asset to a UBlueprint
    UBlueprint* NewBlueprint = Cast<UBlueprint>(NewAsset);
    // Modify the new Blueprint
    NewBlueprint->Modify();
    // Edit the default object for the new Blueprint
    UChaosVehicleWheel* Result = Cast<UChaosVehicleWheel>(NewBlueprint->GeneratedClass->ClassDefaultObject);
    Result->MaxBrakeTorque = MToCM*WheelData.MaxBrakeTorque;
    if (WheelData.MaxHandBrakeTorque != 0)
    {
      Result->MaxHandBrakeTorque = MToCM*WheelData.MaxHandBrakeTorque;
    }
    Result->SteerAngle = RadToDeg*WheelData.MaxSteerAngle;
    Result->SuspensionMaxDrop = MToCM*WheelData.MaxDroop;
    Result->LatStiffValue = WheelData.LateralStiffnessY;
    Result->LongStiffValue = WheelData.LongitudinalStiffness;
    return Result->GetClass();
  };
  // Save wheel objects
  FString AssetPath = DestinationAssetPath + FPaths::GetBaseFilename(USDPath);
  FString PathWheelFL = AssetPath + "_Wheel_FLW";
  FString PathWheelFR = AssetPath + "_Wheel_FRW";
  FString PathWheelRL = AssetPath + "_Wheel_RLW";
  FString PathWheelRR = AssetPath + "_Wheel_RRW";
  WheelObjects.WheelFL = CreateVehicleWheel(
      WheelsData[0], BaseWheelData.WheelFL, PathWheelFL);
  WheelObjects.WheelFR = CreateVehicleWheel(
      WheelsData[1], BaseWheelData.WheelFR, PathWheelFR);
  WheelObjects.WheelRL = CreateVehicleWheel(
      WheelsData[2], BaseWheelData.WheelRL, PathWheelRL);
  WheelObjects.WheelRR = CreateVehicleWheel(
      WheelsData[3], BaseWheelData.WheelRR, PathWheelRR);

#else
  UE_LOG(LogCarlaTools, Error, TEXT("Omniverse Plugin is not enabled"));
#endif
}

AActor* UUSDImporterWidget::GetGeneratedBlueprint(UWorld* World, const FString& USDPath)
{
  TArray<AActor*> Actors;
  UGameplayStatics::GetAllActorsOfClass(World, AActor::StaticClass(), Actors);
  FString USDFileName = FPaths::GetBaseFilename(USDPath, true);
  UE_LOG(LogCarlaTools, Log, TEXT("Searching for name %s"), *USDFileName);
  for (AActor* Actor : Actors)
  {
    if(Actor->GetName().Contains(USDFileName))
    {
      return Actor;
    }
  }
  return nullptr;
}

bool UUSDImporterWidget::MergeStaticMeshComponents(
    TArray<AActor*> Actors, const FString& DestMesh)
{
  if (Actors.Num() == 0)
  {
    UE_LOG(LogCarlaTools, Error, TEXT("No actors for merge"));
    return false;
  }
  UWorld* World = Actors[0]->GetWorld();
  const IMeshMergeUtilities& MeshUtilities = FModuleManager::Get().LoadModuleChecked<IMeshMergeModule>("MeshMergeUtilities").GetUtilities();
  TArray<UPrimitiveComponent*> ComponentsToMerge;
  for(AActor* Actor : Actors)
  {
    TArray<UPrimitiveComponent*> ActorComponents;
    Actor->GetComponents(ActorComponents, false);
    ComponentsToMerge.Append(ActorComponents);
  }
  FMeshMergingSettings MeshMergeSettings;
  TArray<UObject*> AssetsToSync;
  const float ScreenAreaSize = TNumericLimits<float>::Max();
  FVector NewLocation;
  MeshUtilities.MergeComponentsToStaticMesh(ComponentsToMerge, World, MeshMergeSettings, nullptr, nullptr, DestMesh, AssetsToSync, NewLocation, ScreenAreaSize, true);
  return true;
}

TArray<UObject*> UUSDImporterWidget::MergeMeshComponents(
    TArray<UPrimitiveComponent*> ComponentsToMerge,
    const FString& DestMesh)
{
  if(!ComponentsToMerge.Num())
  {
    return {};
  }
  UWorld* World = ComponentsToMerge[0]->GetWorld();
  const IMeshMergeUtilities& MeshUtilities = FModuleManager::Get().LoadModuleChecked<IMeshMergeModule>("MeshMergeUtilities").GetUtilities();
  FMeshMergingSettings MeshMergeSettings;
  TArray<UObject*> AssetsToSync;
  const float ScreenAreaSize = TNumericLimits<float>::Max();
  FVector NewLocation;
  MeshUtilities.MergeComponentsToStaticMesh(ComponentsToMerge, World, MeshMergeSettings, nullptr, nullptr, DestMesh, AssetsToSync, NewLocation, ScreenAreaSize, true);
  return AssetsToSync;
}

bool IsChildrenOf(USceneComponent* Component, FString StringInParent)
{
  USceneComponent* CurrentComponent = Component;
  while(CurrentComponent)
  {
    FString ComponentName = UKismetSystemLibrary::GetDisplayName(CurrentComponent);
    if(ComponentName.Contains(StringInParent))
    {
      return true;
    }
    CurrentComponent = CurrentComponent->GetAttachParent();
  }
  return false;
}

FVehicleMeshParts UUSDImporterWidget::SplitVehicleParts(
    AActor* BlueprintActor,
    const TArray<FVehicleLight>& LightList,
    UMaterialInterface* GlassMaterial)
{
  FVehicleMeshParts Result;
  Result.Lights = LightList;
  TArray<UStaticMeshComponent*> MeshComponents;
  BlueprintActor->GetComponents(MeshComponents, false);
  FVector BodyLocation = FVector(0,0,0);
  TArray<UStaticMeshComponent*> GlassComponents;
  for (UStaticMeshComponent* Component : MeshComponents)
  {
    if (!Component->GetStaticMesh())
    {
      continue;
    }
    FString ComponentName = UKismetSystemLibrary::GetDisplayName(Component);
    if (IsChildrenOf(Component, "door_0"))
    {
      Result.DoorFL.Add(Component);
      Result.Anchors.DoorFL = Component->GetComponentTransform().GetLocation();
    }
    else if (IsChildrenOf(Component, "door_1"))
    {
      Result.DoorFR.Add(Component);
      Result.Anchors.DoorFR = Component->GetComponentTransform().GetLocation();
    }
    else if (IsChildrenOf(Component, "door_2"))
    {
      Result.DoorRL.Add(Component);
      Result.Anchors.DoorRL = Component->GetComponentTransform().GetLocation();
    }
    else if (IsChildrenOf(Component, "door_3"))
    {
      Result.DoorRR.Add(Component);
      Result.Anchors.DoorRR = Component->GetComponentTransform().GetLocation();
    }
    else if (ComponentName.Contains("trunk"))
    {
      Result.Trunk.Add(Component);
      Result.Anchors.Trunk = Component->GetComponentTransform().GetLocation();
    }
    else if (ComponentName.Contains("hood"))
    {
      Result.Hood.Add(Component);
      Result.Anchors.Hood = Component->GetComponentTransform().GetLocation();
    }
    else if (IsChildrenOf(Component, "suspension_0"))
    {
      Result.WheelFL.Add(Component);
      if (ComponentName.Contains("wheel"))
      {
        Result.Anchors.WheelFL = Component->GetComponentTransform().GetLocation();
      }
    }
    else if (IsChildrenOf(Component, "suspension_1"))
    {
      Result.WheelFR.Add(Component);
      if (ComponentName.Contains("wheel"))
      {
        Result.Anchors.WheelFR = Component->GetComponentTransform().GetLocation();
      }
    }
    else if (IsChildrenOf(Component, "suspension_2"))
    {
      Result.WheelRL.Add(Component);
      if (ComponentName.Contains("wheel"))
      {
        Result.Anchors.WheelRL = Component->GetComponentTransform().GetLocation();
      }
    }
    else if (IsChildrenOf(Component, "suspension_3"))
    {
      Result.WheelRR.Add(Component);
      if (ComponentName.Contains("wheel"))
      {
        Result.Anchors.WheelRR = Component->GetComponentTransform().GetLocation();
      }
    }
    else if (ComponentName.Contains("Collision"))
    {

    }
    else
    {
      Result.Body.Add(Component);
      if (ComponentName.Contains("body"))
      {
        BodyLocation = Component->GetComponentTransform().GetLocation();
      }
    }

    if(ComponentName.Contains("glass") ||
       IsChildrenOf(Component, "glass"))
    {
      GlassComponents.Add(Component);
    }
  }
  Result.Anchors.DoorFR -= BodyLocation;
  Result.Anchors.DoorFL -= BodyLocation;
  Result.Anchors.DoorRR -= BodyLocation;
  Result.Anchors.DoorRL -= BodyLocation;
  Result.Anchors.WheelFR -= BodyLocation;
  Result.Anchors.WheelFL -= BodyLocation;
  Result.Anchors.WheelRR -= BodyLocation;
  Result.Anchors.WheelRL -= BodyLocation;
  Result.Anchors.Hood -= BodyLocation;
  Result.Anchors.Trunk -= BodyLocation;
  for (FVehicleLight& Light : Result.Lights)
  {
    Light.Location -= BodyLocation;
  }
  // fix glass materials not being transparent
  for (UStaticMeshComponent* Compopnent : GlassComponents)
  {
    const TArray<UMaterialInterface*>& Materials = Compopnent->GetMaterials();
    for (int32 i = 0; i < Materials.Num(); i++)
    {
      UMaterialInterface* Material = Materials[i];
      if (Material)
      {
        FString MaterialName = Material->GetName();
        if (MaterialName == "DefaultMaterial")
        {
          Compopnent->SetMaterial(i, GlassMaterial);
        }
      }
    }
  }
  return Result;
}

FMergedVehicleMeshParts UUSDImporterWidget::GenerateVehicleMeshes(
    const FVehicleMeshParts& VehicleMeshParts, const FString& DestPath)
{
  FMergedVehicleMeshParts Result;
  auto MergePart =
      [](TArray<UPrimitiveComponent*> Components, const FString& DestMeshPath)
      -> UStaticMesh*
      {
        if (!Components.Num())
        {
          return nullptr;
        }
        TArray<UObject*> Output = MergeMeshComponents(Components, DestMeshPath);
        if (Output.Num())
        {
          return Cast<UStaticMesh>(Output[0]);
        }
        else
        {
          return nullptr;
        }
      };
  Result.DoorFR = MergePart(VehicleMeshParts.DoorFR, DestPath + "_door_fr");
  Result.DoorFL = MergePart(VehicleMeshParts.DoorFL, DestPath + "_door_fl");
  Result.DoorRR = MergePart(VehicleMeshParts.DoorRR, DestPath + "_door_rr");
  Result.DoorRL = MergePart(VehicleMeshParts.DoorRL, DestPath + "_door_rl");
  Result.Trunk = MergePart(VehicleMeshParts.Trunk, DestPath + "_trunk");
  Result.Hood = MergePart(VehicleMeshParts.Hood, DestPath + "_hood");
  Result.WheelFR = MergePart(VehicleMeshParts.WheelFR, DestPath + "_wheel_fr");
  Result.WheelFL = MergePart(VehicleMeshParts.WheelFL, DestPath + "_wheel_fl");
  Result.WheelRR = MergePart(VehicleMeshParts.WheelRR, DestPath + "_wheel_rr");
  Result.WheelRL = MergePart(VehicleMeshParts.WheelRL, DestPath + "_wheel_rl");
  Result.Body = MergePart(VehicleMeshParts.Body, DestPath + "_body");
  Result.Anchors = VehicleMeshParts.Anchors;
  Result.Lights = VehicleMeshParts.Lights;
  return Result;
}

FString GetCarlaLightName(const FString &USDName)
{
  FString LowerCaseUSDName = USDName.ToLower();
  FString LightType = "";
  if (LowerCaseUSDName.Contains("headlight"))
  {
    LightType = "low_beam";
  }
  else if (LowerCaseUSDName.Contains("brakelight"))
  {
    LightType = "brake";
  }
  else if (LowerCaseUSDName.Contains("blinker"))
  {
    LightType = "blinker";
  }
  else if (LowerCaseUSDName.Contains("night"))
  {
    LightType = "high_beam";
  }
  else if (LowerCaseUSDName.Contains("reverse"))
  {
    LightType = "reverse";
  }
  else if (LowerCaseUSDName.Contains("highbeamlight"))
  {
    LightType = "high_beam";
  }
  else if (LowerCaseUSDName.Contains("foglight"))
  {
    LightType = "fog";
  }
  else if (LowerCaseUSDName.Contains("TailLight"))
  {
    LightType = "position";
  }
  else
  {
    LightType = USDName;
  }

  FString FinalName = "-" + LightType + "-";
  if (LowerCaseUSDName.EndsWith("_fr"))
  {
    FinalName = "front" + FinalName + "r-";
  }
  else if (LowerCaseUSDName.EndsWith("_fl"))
  {
    FinalName = "front" + FinalName + "l-";
  }
  else if (LowerCaseUSDName.EndsWith("_rr"))
  {
    FinalName = "back" + FinalName + "r-";
  }
  else if (LowerCaseUSDName.EndsWith("_rl"))
  {
    FinalName = "back" + FinalName + "l-";
  }

  return FinalName;
}

AActor* UUSDImporterWidget::GenerateNewVehicleBlueprint(
    UWorld* World,
    UClass* BaseClass,
    USkeletalMesh* NewSkeletalMesh,
    UPhysicsAsset* NewPhysicsAsset,
    const FString &DestPath, 
    const FMergedVehicleMeshParts& VehicleMeshes,
    const FWheelTemplates& WheelTemplates)
{
  std::unordered_map<std::string, std::pair<UStaticMesh*, FVector>> MeshMap = {
    {"SM_DoorFR", {VehicleMeshes.DoorFR, VehicleMeshes.Anchors.DoorFR}},
    {"SM_DoorFL", {VehicleMeshes.DoorFL, VehicleMeshes.Anchors.DoorFL}},
    {"SM_DoorRR", {VehicleMeshes.DoorRR, VehicleMeshes.Anchors.DoorRR}},
    {"SM_DoorRL", {VehicleMeshes.DoorRL, VehicleMeshes.Anchors.DoorRL}},
    {"Trunk", {VehicleMeshes.Trunk, VehicleMeshes.Anchors.Trunk}},
    {"Hood", {VehicleMeshes.Hood, VehicleMeshes.Anchors.Hood}},
    {"Wheel_FR", {VehicleMeshes.WheelFR, FVector(0,0,0)}},
    {"Wheel_FL", {VehicleMeshes.WheelFL, FVector(0,0,0)}},
    {"Wheel_RR", {VehicleMeshes.WheelRR, FVector(0,0,0)}},
    {"Wheel_RL", {VehicleMeshes.WheelRL, FVector(0,0,0)}},
    {"Body", {VehicleMeshes.Body, FVector(0,0,0)}}
  };

  AActor* TemplateActor = World->SpawnActor<AActor>(BaseClass);
  // Get an replace all static meshes with the appropiate mesh
  TArray<UStaticMeshComponent*> MeshComponents;
  TemplateActor->GetComponents(MeshComponents);
  bool bBodyComponentMatched = false;
  for (UStaticMeshComponent* Component : MeshComponents)
  {
    std::string ComponentName = TCHAR_TO_UTF8(*Component->GetName());
    auto &MapElement = MeshMap[ComponentName];
    UStaticMesh* ComponentMesh = MapElement.first;
    FVector MeshLocation = MapElement.second;
    if(ComponentMesh)
    {
      Component->SetStaticMesh(ComponentMesh);
      Component->SetRelativeLocation(MeshLocation);
      

      Component->SetVisibility(true);
      Component->SetHiddenInGame(false);
      if (ComponentName == "Body") bBodyComponentMatched = true;
    }
    UE_LOG(LogCarlaTools, Log, TEXT("Component name %s, name %s"),
    *UKismetSystemLibrary::GetDisplayName(Component), *Component->GetName());
  }

  

  

  // Body StaticMeshComponent will be added via SCS_Node AFTER CreateBlueprintFromActor.
  // AddInstanceComponent on the template actor does not reliably persist into the BP
  // when the parent class (e.g. BaseVehiclePawnNW) exposes no body slot in its CDO.

  // Get the skeletal mesh and modify it to match the vehicle parameters
  USkeletalMeshComponent* SkeletalMeshComponent = Cast<USkeletalMeshComponent>(
      TemplateActor->GetComponentByClass(USkeletalMeshComponent::StaticClass()));
  if(!SkeletalMeshComponent)
  {
    UE_LOG(LogCarlaTools, Log, TEXT("Skeletal mesh component not found"));
    return nullptr;
  }
  

  TMap<FString, FTransform> NewBoneTransform = {
    {"Wheel_Front_Left", FTransform(VehicleMeshes.Anchors.WheelFL)},
    {"Wheel_Front_Right", FTransform(VehicleMeshes.Anchors.WheelFR)},
    {"Wheel_Rear_Right", FTransform(VehicleMeshes.Anchors.WheelRR)},
    {"Wheel_Rear_Left", FTransform(VehicleMeshes.Anchors.WheelRL)}
  };
  if(!NewSkeletalMesh)
  {
    UE_LOG(LogCarlaTools, Log, TEXT("Mesh not generated, skeletal mesh missing"));
    return nullptr;
  }
  bool bSuccess = EditSkeletalMeshBones(NewSkeletalMesh, NewBoneTransform);
  if (!NewSkeletalMesh || !bSuccess)
  {
    UE_LOG(LogCarlaTools, Log, TEXT("Blueprint generation error"));
    return nullptr;
  }
  SkeletalMeshComponent->SetSkeletalMesh(NewSkeletalMesh);
  

  SkeletalMeshComponent->SetMobility(EComponentMobility::Movable);
  SkeletalMeshComponent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
  SkeletalMeshComponent->SetSimulatePhysics(true);

  

  

  
  SkeletalMeshComponent->SetAnimInstanceClass(nullptr);
  UE_LOG(LogCarlaTools, Log, TEXT("Num Lights %d"), VehicleMeshes.Lights.Num());
  for (const FVehicleLight& Light : VehicleMeshes.Lights)
  {
    FString FixedLightName = GetCarlaLightName(Light.Name);
    UClass * LightClass = UPointLightComponent::StaticClass();
    if (FixedLightName.Contains("beam"))
    {
      LightClass = USpotLightComponent::StaticClass();
    }
    ULocalLightComponent * LightComponent = NewObject<ULocalLightComponent>(TemplateActor, LightClass, FName(*FixedLightName));
    LightComponent->RegisterComponent();
    LightComponent->AttachToComponent(
        TemplateActor->GetRootComponent(),
        FAttachmentTransformRules::KeepRelativeTransform);
    LightComponent->SetRelativeLocation(Light.Location); // Set the position of the light relative to the actor
    LightComponent->SetIntensityUnits(ELightUnits::Lumens);
    LightComponent->SetIntensity(5000.f); // Set the brightness of the light
    LightComponent->SetVolumetricScatteringIntensity(0.f);
    if (FixedLightName.Contains("high_beam"))
    {
      USpotLightComponent* SpotLight =
          Cast<USpotLightComponent>(LightComponent);
      SpotLight->SetRelativeRotation(FRotator(-1.5f, 0, 0));
      SpotLight->SetAttenuationRadius(5000.f);
      SpotLight->SetInnerConeAngle(20.f);
      SpotLight->SetOuterConeAngle(30.f);
      LightComponent->SetIntensity(150000.f); // Set the brightness of the light
      LightComponent->SetVolumetricScatteringIntensity(0.025f);
    }
    else if (FixedLightName.Contains("low_beam"))
    {
      USpotLightComponent* SpotLight =
          Cast<USpotLightComponent>(LightComponent);
      LightComponent->SetRelativeRotation(FRotator(-3.f, 0, 0));
      LightComponent->SetAttenuationRadius(3000.f);
      SpotLight->SetInnerConeAngle(50.f);
      SpotLight->SetOuterConeAngle(65.f);
      LightComponent->SetIntensity(15000.f); // Set the brightness of the light
      LightComponent->SetVolumetricScatteringIntensity(0.025f);
    }
    LightComponent->SetLightColor(Light.Color);
    TemplateActor->AddInstanceComponent(LightComponent);
    UE_LOG(LogCarlaTools, Log, TEXT("Spawn Light %s, %s, %s"), *Light.Name, *Light.Location.ToString(), *Light.Color.ToString());
  }
  

  
  ACarlaWheeledVehicle* CarlaVehicle =
      Cast<ACarlaWheeledVehicle>(TemplateActor);
  if (CarlaVehicle)
  {
    auto FillSetups = [&](auto* Setups)
    {
      Setups->Empty();
      const TPair<FName, TSubclassOf<UChaosVehicleWheel>> Wheels[4] = {
        { FName(TEXT("Wheel_Front_Left")),  WheelTemplates.WheelFL },
        { FName(TEXT("Wheel_Front_Right")), WheelTemplates.WheelFR },
        { FName(TEXT("Wheel_Rear_Left")),   WheelTemplates.WheelRL },
        { FName(TEXT("Wheel_Rear_Right")),  WheelTemplates.WheelRR },
      };
      for (const auto& W : Wheels)
      {
        FChaosWheelSetup Setup;
        Setup.BoneName   = W.Key;
        Setup.WheelClass = W.Value;
        Setups->Add(Setup);
      }
    };
#if ENGINE_MAJOR_VERSION >= 5
    UChaosWheeledVehicleMovementComponent* MovementComponent =
        CarlaVehicle->FindComponentByClass<UChaosWheeledVehicleMovementComponent>();
    if (MovementComponent)
    {
      FillSetups(&MovementComponent->WheelSetups);
      UE_LOG(LogCarlaTools, Display,
             TEXT("VI.GenerateBP: WheelSetups populated with 4 wheel classes"));

      float ChassisVolM3 = 0.f;
      FVector ChassisExtentCm = FVector::ZeroVector;
      FVector ChassisCenterCm = FVector::ZeroVector;
      if (VehicleMeshes.Body)
      {
        const FBoxSphereBounds B = VehicleMeshes.Body->GetExtendedBounds();
        ChassisExtentCm = B.BoxExtent * 2.f;
        ChassisCenterCm = B.Origin;
        ChassisVolM3 =
            (ChassisExtentCm.X * ChassisExtentCm.Y * ChassisExtentCm.Z) / 1e6f;
      }

      float TunedMass = 1500.f;
      bool bSmall = false;
      if (ChassisVolM3 > 0.f && ChassisVolM3 < 0.5f)      { TunedMass = 50.f;  bSmall = true; }
      else if (ChassisVolM3 > 0.f && ChassisVolM3 < 2.f)  { TunedMass = 200.f; }
      MovementComponent->Mass = TunedMass;

      
      if (bSmall)
      {
        FRichCurve* RC = MovementComponent->EngineSetup.TorqueCurve.GetRichCurve();
        if (RC)
        {
          RC->Reset();
          RC->AddKey(0.f,    0.5f);
          RC->AddKey(1500.f, 1.0f);
          RC->AddKey(4500.f, 0.5f);
        }
        MovementComponent->EngineSetup.MaxTorque = 100.f;
        MovementComponent->EngineSetup.MaxRPM    = 4500.f;
      }

      if (SkeletalMeshComponent && ChassisExtentCm != FVector::ZeroVector)
      {
        const FVector CoM(
            ChassisCenterCm.X,
            ChassisCenterCm.Y,
            ChassisCenterCm.Z - ChassisExtentCm.Z * (1.f / 6.f));
        SkeletalMeshComponent->BodyInstance.COMNudge = CoM;
        SkeletalMeshComponent->BodyInstance.bOverrideMass = true;
        SkeletalMeshComponent->BodyInstance.SetMassOverride(TunedMass, true);
      }

      UE_LOG(LogCarlaTools, Display,
             TEXT("VI.GenerateBP: tuned mass=%.0fkg vol=%.3fm^3 small=%d CoMNudge=(%.1f,%.1f,%.1f)"),
             TunedMass, ChassisVolM3, bSmall ? 1 : 0,
             SkeletalMeshComponent ? SkeletalMeshComponent->BodyInstance.COMNudge.X : 0.f,
             SkeletalMeshComponent ? SkeletalMeshComponent->BodyInstance.COMNudge.Y : 0.f,
             SkeletalMeshComponent ? SkeletalMeshComponent->BodyInstance.COMNudge.Z : 0.f);
    }
    else
    {
      UE_LOG(LogCarlaTools, Warning,
             TEXT("VI.GenerateBP: no UChaosWheeledVehicleMovementComponent on "
                  "TemplateActor — WheelSetups not assigned, spawn will fail."));
    }
#else
    UWheeledVehicleMovementComponent4W* MovementComponent =
        Cast<UWheeledVehicleMovementComponent4W>(
            CarlaVehicle->GetVehicleMovementComponent());
    if (MovementComponent)
    {
      FillSetups(&MovementComponent->WheelSetups);
    }
#endif
  }
  else
  {
    UE_LOG(LogCarlaTools, Error, TEXT("VI.GenerateBP: TemplateActor is not an "
                                      "ACarlaWheeledVehicle — WheelSetups skipped."));
  }

  CopyCollisionToPhysicsAsset(NewPhysicsAsset, VehicleMeshes.Body);
  // assign the physics asset to the skeletal mesh
  NewSkeletalMesh->SetPhysicsAsset(NewPhysicsAsset);
  // Create the new blueprint vehicle
  FKismetEditorUtilities::FCreateBlueprintFromActorParams Params;
  Params.bReplaceActor = false;
  Params.bKeepMobility = true;
  Params.bDeferCompilation = false;
  Params.bOpenBlueprint = false;
  Params.ParentClassOverride = BaseClass;
  UBlueprint* NewBP = FKismetEditorUtilities::CreateBlueprintFromActor(
      DestPath,
      TemplateActor,
      Params);

  if (NewBP && !bBodyComponentMatched && VehicleMeshes.Body && NewBP->SimpleConstructionScript)
  {
    USimpleConstructionScript* SCS = NewBP->SimpleConstructionScript;
    USCS_Node* BodyNode = SCS->CreateNode(UStaticMeshComponent::StaticClass(), FName(TEXT("Body")));
    if (BodyNode)
    {
      if (UStaticMeshComponent* BodyTpl = Cast<UStaticMeshComponent>(BodyNode->ComponentTemplate))
      {
        BodyTpl->SetStaticMesh(VehicleMeshes.Body);
        BodyTpl->SetCollisionProfileName(FName(TEXT("NoCollision")));
        BodyTpl->SetMobility(EComponentMobility::Movable);
        BodyTpl->SetVisibility(true);
        BodyTpl->SetHiddenInGame(false);
        BodyTpl->SetRelativeLocation(FVector::ZeroVector);
      }
      USCS_Node* SkelNode = nullptr;
      for (USCS_Node* N : SCS->GetAllNodes())
      {
        if (N && N->ComponentTemplate && N->ComponentTemplate->IsA<USkeletalMeshComponent>())
        {
          SkelNode = N; break;
        }
      }
      if (SkelNode) SkelNode->AddChildNode(BodyNode);
      else SCS->AddNode(BodyNode);

      FBlueprintEditorUtils::MarkBlueprintAsStructurallyModified(NewBP);
      FKismetEditorUtilities::CompileBlueprint(NewBP);
      UE_LOG(LogCarlaTools, Display,
             TEXT("VI.GenerateBP: added Body via SCS_Node referencing %s (parent=%s)"),
             *VehicleMeshes.Body->GetName(),
             SkelNode ? *SkelNode->GetVariableName().ToString() : TEXT("root"));
    }
  }

  if (NewBP)
  {
    if (UPackage* Pkg = NewBP->GetOutermost())
    {
      Pkg->SetDirtyFlag(true);
      const FString FilePath = FPackageName::LongPackageNameToFilename(
          Pkg->GetName(), FPackageName::GetAssetPackageExtension());
      FSavePackageArgs SaveArgs;
      SaveArgs.TopLevelFlags = RF_Public | RF_Standalone;
      SaveArgs.SaveFlags     = SAVE_NoError;
      SaveArgs.Error         = GError;
      const bool bOk = UPackage::SavePackage(Pkg, NewBP, *FilePath, SaveArgs);
      UE_LOG(LogCarlaTools, Display, TEXT("GenerateNewVehicleBlueprint: SavePackage(%s) -> %s"),
             *FilePath, bOk ? TEXT("OK") : TEXT("FAILED"));
    }
  }
  else
  {
    UE_LOG(LogCarlaTools, Warning,
           TEXT("GenerateNewVehicleBlueprint: CreateBlueprintFromActor returned null for %s"),
           *DestPath);
  }
  return nullptr;
}

bool UUSDImporterWidget::EditSkeletalMeshBones(
    USkeletalMesh* NewSkeletalMesh,
    const TMap<FString, FTransform> &NewBoneTransforms)
{
  if(!NewSkeletalMesh)
  {
    UE_LOG(LogCarlaTools, Log, TEXT("Skeletal mesh invalid"));
    return false;
  }
  FReferenceSkeleton& ReferenceSkeleton = NewSkeletalMesh->GetRefSkeleton();
  FReferenceSkeletonModifier SkeletonModifier(ReferenceSkeleton, NewSkeletalMesh->GetSkeleton());
  for (auto& Element : NewBoneTransforms)
  {
    const FString& BoneName = Element.Key;
    const FTransform& BoneTransform = Element.Value;
    int32 BoneIdx = SkeletonModifier.FindBoneIndex(FName(*BoneName));
    if (BoneIdx == INDEX_NONE)
    {
      UE_LOG(LogCarlaTools, Log, TEXT("Bone %s not found"), *BoneName);
    }
    UE_LOG(LogCarlaTools, Log, TEXT("Bone %s corresponds to index %d"), *BoneName, BoneIdx);
    SkeletonModifier.UpdateRefPoseTransform(BoneIdx, BoneTransform);
  }

  NewSkeletalMesh->MarkPackageDirty();
  UPackage* Package = NewSkeletalMesh->GetOutermost();
  FSavePackageArgs SaveArgs;
  SaveArgs.TopLevelFlags =
      EObjectFlags::RF_Public | EObjectFlags::RF_Standalone;
  SaveArgs.Error = GError;
  SaveArgs.bForceByteSwapping = true;
  SaveArgs.bWarnOfLongFilename = true;
  SaveArgs.SaveFlags = SAVE_NoError;

  

  
  const FString FilePath = FPackageName::LongPackageNameToFilename(
      Package->GetName(), FPackageName::GetAssetPackageExtension());
  return UPackage::SavePackage(Package, NewSkeletalMesh, *FilePath, SaveArgs);
}

void UUSDImporterWidget::CopyCollisionToPhysicsAsset(
    UPhysicsAsset* PhysicsAssetToEdit, UStaticMesh* StaticMesh)
{
  UE_LOG(LogCarlaTools, Log, TEXT("Num bodysetups %d"), PhysicsAssetToEdit->SkeletalBodySetups.Num());
  UBodySetup* BodySetupPhysicsAsset = Cast<UBodySetup>(
      PhysicsAssetToEdit->SkeletalBodySetups[
          PhysicsAssetToEdit->FindBodyIndex(FName("Vehicle_Base"))]);
  UBodySetup* BodySetupStaticMesh = StaticMesh->GetBodySetup();
  BodySetupPhysicsAsset->AggGeom = BodySetupStaticMesh->AggGeom;

}
