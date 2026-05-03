// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB). This work is licensed under the terms of the MIT license. For a copy, see <https://opensource.org/licenses/MIT>.




#include "VehicleImporter.h"
#include "VehicleImporter_Helpers.h"
#include "USDImporterWidget.h"
#include "CarlaTools.h"
#include "Stages/Stage_Common.h"
#include "Stages/Stage_Preflight.h"
#include "Stages/Stage_Interchange.h"
#include "Stages/Stage_BuildSK.h"
#include "Stages/Stage_BuildBP.h"
#include "Stages/Stage_Persist.h"
#include "Tests/VehicleImportTestRunner.h"

#include <util/ue-header-guard-begin.h>
#include "Sockets.h"
#include "SocketSubsystem.h"
#include "IPAddress.h"
#include "Common/TcpSocketBuilder.h"
#include "Interfaces/IPv4/IPv4Address.h"
#include "Interfaces/IPv4/IPv4Endpoint.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"
#include "Async/Async.h"
#include "Containers/Ticker.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "Engine/StaticMesh.h"
#include "Engine/SkeletalMesh.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "PhysicsEngine/SkeletalBodySetup.h"
#include "PhysicsEngine/BodySetup.h"
#include "PhysicsEngine/AggregateGeom.h"
#include "Components/SkeletalMeshComponent.h"
#include "Factories/FbxImportUI.h"
#include "AssetImportTask.h"
#include "AssetToolsModule.h"
#include "IAssetTools.h"
#include "InterchangeManager.h"
#include "InterchangeProjectSettings.h"
#include "InterchangeGenericAssetsPipeline.h"
#include "InterchangeGenericMaterialPipeline.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "EditorAssetLibrary.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "ChaosVehicleWheel.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"
#include "Factories/BlueprintFactory.h"
#include "Engine/Blueprint.h"
#include "Engine/BlueprintGeneratedClass.h"
#include "HAL/FileManager.h"
#include "Misc/Paths.h"
#include "Misc/PackageName.h"
#include "UObject/SavePackage.h"
#include "GameFramework/Actor.h"
#include "Kismet2/BlueprintEditorUtils.h"
#include "Runtime/Launch/Resources/Version.h"
#include <util/ue-header-guard-end.h>

// Port is configurable via env var CARLA_VEHICLE_IMPORTER_PORT so the offline
// C++ test rig (carla-studio-vehicle-import-test) can run on a non-clashing
// port (default 18584) while Studio's editor stays on 18583.
static int32 ResolveImporterPort() {
  const FString env = FPlatformMisc::GetEnvironmentVariable(TEXT("CARLA_VEHICLE_IMPORTER_PORT"));
  if (!env.IsEmpty()) {
    const int32 p = FCString::Atoi(*env);
    if (p > 0 && p < 65536) return p;
  }
  return 18583;
}
static const int32 GImporterPort = ResolveImporterPort();

static constexpr int32 GMaxMessageBytes = 10 * 1024 * 1024;

namespace
{

  

  static constexpr float kChaosWheelRadiusFloorCm = 18.f;
  static constexpr float kStockWheelShapeRadiusCm = 35.f;
  static constexpr float kStockWheelShapeWidthCm  = 25.f;

  static UStaticMesh* MakeShrunkWheelShape(
      const FString& VehicleContentPath,
      const FString& Suffix,
      float RadiusCm,
      float WidthCm)
  {
    if (RadiusCm >= kChaosWheelRadiusFloorCm)
      return nullptr;

    static const TCHAR* const kSrcPath =
      TEXT("/Game/Carla/Blueprints/Vehicles/Wheel_Shape.Wheel_Shape");
    UStaticMesh* Src = LoadObject<UStaticMesh>(nullptr, kSrcPath);
    if (!Src)
      return nullptr;

    const FString DstPath = VehicleContentPath / FString::Printf(
        TEXT("WheelShape_%s"), *Suffix);
    UEditorAssetLibrary::DeleteAsset(DstPath);
    UObject* DupObj = UEditorAssetLibrary::DuplicateAsset(Src->GetPathName(), DstPath);
    UStaticMesh* Dup = Cast<UStaticMesh>(DupObj);
    if (!Dup || Dup->GetNumSourceModels() == 0)
      return nullptr;

    const float SafeR = FMath::Max(RadiusCm, 1.f);
    const float SafeW = FMath::Max(WidthCm,  1.f);
    const float ScaleR = SafeR / kStockWheelShapeRadiusCm;
    const float ScaleW = SafeW / kStockWheelShapeWidthCm;

    FStaticMeshSourceModel& SM = Dup->GetSourceModel(0);
    SM.BuildSettings.BuildScale3D = FVector(ScaleW, ScaleR, ScaleR);
    Dup->Build(false);
    Dup->MarkPackageDirty();
    return Dup;
  }

  

  
  
  static UBlueprint* CreateGraftedBlueprint(
      UWorld* World,
      UClass* StockParent,
      UStaticMesh* BodyMesh,
      const FWheelTemplates& WheelTemplates,
      const FString& DestPath)
  {
    if (!World || !StockParent || !BodyMesh) return nullptr;
    AActor* Template = World->SpawnActor<AActor>(StockParent);
    if (!Template) return nullptr;

    
    
    if (ACarlaWheeledVehicle* CarlaVehicle = Cast<ACarlaWheeledVehicle>(Template))
    {
      if (UChaosWheeledVehicleMovementComponent* MC =
              CarlaVehicle->FindComponentByClass<UChaosWheeledVehicleMovementComponent>())
      {
        MC->WheelSetups.Empty();
        const TPair<FName, TSubclassOf<UChaosVehicleWheel>> Wheels[4] = {
          { FName(TEXT("Wheel_Front_Left")),  WheelTemplates.WheelFL },
          { FName(TEXT("Wheel_Front_Right")), WheelTemplates.WheelFR },
          { FName(TEXT("Wheel_Rear_Left")),   WheelTemplates.WheelRL },
          { FName(TEXT("Wheel_Rear_Right")),  WheelTemplates.WheelRR },
        };
        for (const auto& W : Wheels)
        {
          FChaosWheelSetup S;
          S.BoneName   = W.Key;
          S.WheelClass = W.Value;
          MC->WheelSetups.Add(S);
        }
        UE_LOG(LogCarlaTools, Display,
               TEXT("VI.Graft: WheelSetups populated with 4 wheel classes"));
      }
      else
      {
        UE_LOG(LogCarlaTools, Warning,
               TEXT("VI.Graft: no UChaosWheeledVehicleMovementComponent on parent — "
                    "WheelSetups not populated; BP will crash in BeginPlay."));
      }
    }

    UStaticMeshComponent* BodyComp = nullptr;
    float BestVol = 0.f;
    TArray<UStaticMeshComponent*> SMs;
    Template->GetComponents(SMs);
    for (UStaticMeshComponent* C : SMs)
    {
      UStaticMesh* CurMesh = C->GetStaticMesh();
      if (!CurMesh) continue;
      const FBoxSphereBounds B = CurMesh->GetBounds();
      const float Vol = B.BoxExtent.X * B.BoxExtent.Y * B.BoxExtent.Z;
      if (Vol > BestVol) { BestVol = Vol; BodyComp = C; }
    }
    if (BodyComp)
    {
      BodyComp->SetStaticMesh(BodyMesh);
      UE_LOG(LogCarlaTools, Display,
             TEXT("VI.Graft: replaced body StaticMesh on '%s' with %s"),
             *BodyComp->GetName(), *BodyMesh->GetName());
    }
    else
    {
      
      UStaticMeshComponent* NewBody = NewObject<UStaticMeshComponent>(
          Template, UStaticMeshComponent::StaticClass(), TEXT("Body"));
      NewBody->SetStaticMesh(BodyMesh);
      NewBody->SetMobility(EComponentMobility::Movable);
      if (USceneComponent* Root = Template->GetRootComponent())
        NewBody->AttachToComponent(Root, FAttachmentTransformRules::KeepRelativeTransform);
      else
        Template->SetRootComponent(NewBody);
      NewBody->RegisterComponent();
      Template->AddInstanceComponent(NewBody);
      UE_LOG(LogCarlaTools, Display,
             TEXT("VI.Graft: parent had no StaticMesh body — added new 'Body' component with %s"),
             *BodyMesh->GetName());
    }

    FKismetEditorUtilities::FCreateBlueprintFromActorParams P;
    P.bReplaceActor       = false;
    P.bKeepMobility       = true;
    P.bDeferCompilation   = false;
    P.bOpenBlueprint      = false;
    P.ParentClassOverride = StockParent;
    UBlueprint* BP = FKismetEditorUtilities::CreateBlueprintFromActor(DestPath, Template, P);
    if (BP)
    {
      if (UPackage* Pkg = BP->GetOutermost())
      {
        Pkg->SetDirtyFlag(true);
        const FString FilePath = FPackageName::LongPackageNameToFilename(
            Pkg->GetName(), FPackageName::GetAssetPackageExtension());
        FSavePackageArgs SaveArgs;
        SaveArgs.TopLevelFlags = RF_Public | RF_Standalone;
        SaveArgs.SaveFlags     = SAVE_NoError;
        SaveArgs.Error         = GError;
        UPackage::SavePackage(Pkg, BP, *FilePath, SaveArgs);
      }
    }
    Template->Destroy();
    return BP;
  }

  static void ApplyChassisAabbResize(
      UPhysicsAsset* PA,
      const FVehicleImportSpec& Spec,
      bool bHaveAabb)
  {
    if (!PA) return;

    const float MinWheelR = FMath::Min(
        FMath::Min(Spec.WheelFL.Radius, Spec.WheelFR.Radius),
        FMath::Min(Spec.WheelRL.Radius, Spec.WheelRR.Radius));

    auto WheelRadiusForBone = [&](const FName& Bone) -> float {
      const FString S = Bone.ToString();
      if (S.Contains(TEXT("Front_Left")))  return Spec.WheelFL.Radius;
      if (S.Contains(TEXT("Front_Right"))) return Spec.WheelFR.Radius;
      if (S.Contains(TEXT("Rear_Left")))   return Spec.WheelRL.Radius;
      if (S.Contains(TEXT("Rear_Right")))  return Spec.WheelRR.Radius;
      return 0.f;
    };

    for (USkeletalBodySetup* BS : PA->SkeletalBodySetups)
    {
      if (!BS) continue;
      const FString BoneStr = BS->BoneName.ToString();
      FKAggregateGeom& Geom = BS->AggGeom;
      if (BoneStr == TEXT("Vehicle_Base"))
      {
        if (bHaveAabb)
        {
          const float DX = Spec.ChassisXMax - Spec.ChassisXMin;
          const float DY = Spec.ChassisYMax - Spec.ChassisYMin;
          const float DZ = Spec.ChassisZMax - Spec.ChassisZMin;
          const float CX = 0.5f * (Spec.ChassisXMin + Spec.ChassisXMax);
          const float CY = 0.5f * (Spec.ChassisYMin + Spec.ChassisYMax);
          const float CZ = MinWheelR + 0.5f * DZ + 1.0f;
          Geom.EmptyElements();
          FKBoxElem Box;
          Box.X = DX; Box.Y = DY; Box.Z = DZ;
          Box.Center = FVector(CX, CY, CZ);
          Geom.BoxElems.Add(Box);
        }
        BS->DefaultInstance.LinearDamping  = 0.f;
        BS->DefaultInstance.AngularDamping = 0.f;
      }
      else
      {
        const float WR = WheelRadiusForBone(BS->BoneName);
        if (WR > 0.f)
        {
          if (bHaveAabb)
          {
            Geom.EmptyElements();
            FKSphereElem Sph;
            Sph.Radius = WR;
            Sph.Center = FVector::ZeroVector;
            Geom.SphereElems.Add(Sph);
          }
          BS->PhysicsType            = PhysType_Kinematic;
          BS->CollisionReponse       = EBodyCollisionResponse::BodyCollision_Disabled;
          BS->DefaultInstance.LinearDamping  = 0.f;
          BS->DefaultInstance.AngularDamping = 0.f;
        }
      }
    }
    PA->Modify();
    PA->MarkPackageDirty();
  }
}



FVehicleImporterServer* UVehicleImporter::ServerRunnable = nullptr;
FRunnableThread*        UVehicleImporter::ServerThread   = nullptr;

void UVehicleImporter::StartServer()
{
  if (ServerRunnable)
    return;

  ServerRunnable = new FVehicleImporterServer();
  ServerThread   = FRunnableThread::Create(ServerRunnable,
                     TEXT("CarlaVehicleImporter"),
                     0, TPri_BelowNormal);
  UE_LOG(LogCarlaTools, Log, TEXT("VehicleImporter: listening on port %d"), GImporterPort);
}

void UVehicleImporter::StopServer()
{
  if (ServerRunnable)
    ServerRunnable->Stop();

  if (ServerThread)
  {
    ServerThread->WaitForCompletion();
    delete ServerThread;
    ServerThread = nullptr;
  }
  delete ServerRunnable;
  ServerRunnable = nullptr;
}



FVehicleImporterServer::FVehicleImporterServer()  = default;
FVehicleImporterServer::~FVehicleImporterServer() = default;

bool FVehicleImporterServer::Init()
{
  ISocketSubsystem* SS = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
  if (!SS)
    return false;

  ListenSocket = FTcpSocketBuilder(TEXT("CarlaVehicleImporterListen"))
    .AsReusable()
    .BoundToEndpoint(FIPv4Endpoint(FIPv4Address::Any, GImporterPort))
    .Listening(1)
    .Build();

  if (!ListenSocket)
  {
    UE_LOG(LogCarlaTools, Error,
           TEXT("VehicleImporter: failed to bind port %d"), GImporterPort);
    return false;
  }

  bRunning = true;
  return true;
}

uint32 FVehicleImporterServer::Run()
{
  while (bRunning)
  {
    bool bPending = false;
    if (ListenSocket->WaitForPendingConnection(bPending, FTimespan::FromSeconds(1.0)))
    {
      if (bPending)
      {
        FSocket* Client = ListenSocket->Accept(TEXT("CarlaStudio"));
        if (Client)
        {
          ServeClient(Client);
          Client->Close();
          ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(Client);
        }
      }
    }
  }
  return 0;
}

void FVehicleImporterServer::Stop()
{
  bRunning = false;
  if (ListenSocket)
  {
    ListenSocket->Close();
    ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(ListenSocket);
    ListenSocket = nullptr;
  }
}



static bool RecvAll(FSocket* S, uint8* Buf, int32 Len)
{
  int32 Received = 0;
  while (Received < Len)
  {
    int32 Got = 0;
    if (!S->Recv(Buf + Received, Len - Received, Got) || Got <= 0)
      return false;
    Received += Got;
  }
  return true;
}

static bool SendAll(FSocket* S, const uint8* Buf, int32 Len)
{
  int32 Sent = 0;
  while (Sent < Len)
  {
    int32 Written = 0;
    if (!S->Send(Buf + Sent, Len - Sent, Written) || Written <= 0)
      return false;
    Sent += Written;
  }
  return true;
}

void FVehicleImporterServer::ServeClient(FSocket* Client)
{
  
  uint8 LenBuf[4];
  if (!RecvAll(Client, LenBuf, 4))
    return;

  const int32 MsgLen = (int32)(LenBuf[0]
    | ((uint32)LenBuf[1] << 8)
    | ((uint32)LenBuf[2] << 16)
    | ((uint32)LenBuf[3] << 24));

  if (MsgLen <= 0 || MsgLen > GMaxMessageBytes)
  {
    UE_LOG(LogCarlaTools, Warning,
           TEXT("VehicleImporter: bad message length %d"), MsgLen);
    return;
  }

  // +1 byte for an explicit null terminator. Studio's wire protocol does not
  // send a trailing NUL — UTF8_TO_TCHAR/FString below would otherwise scan
  // into uninitialised heap memory and append garbage to the JSON string,
  // which UE's JSON parser then rejects as "JSON parse error".
  TArray<uint8> Body;
  Body.SetNumUninitialized(MsgLen + 1);
  if (!RecvAll(Client, Body.GetData(), MsgLen))
    return;
  Body[MsgLen] = 0;

  const FString JsonStr = FString(UTF8_TO_TCHAR(
    reinterpret_cast<const ANSICHAR*>(Body.GetData())));

  

  
  FString Response;
  TSharedPtr<FJsonObject> Root;
  TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(JsonStr);
  if (!FJsonSerializer::Deserialize(Reader, Root) || !Root.IsValid())
  {
    UE_LOG(LogCarlaTools, Warning,
           TEXT("VI: JSON parse failed; len=%d, full=%s"),
           JsonStr.Len(), *JsonStr);
    Response = MakeResponse(false, TEXT(""), TEXT("JSON parse error"));
  }
  else
  {
    FString Action;
    Root->TryGetStringField(TEXT("action"), Action);
    if (Action.Equals(TEXT("test_matrix"), ESearchCase::IgnoreCase))
    {
      TArray<VehicleImport::FTestMatrixSpec> Specs;
      const TArray<TSharedPtr<FJsonValue>>* Arr = nullptr;
      if (Root->TryGetArrayField(TEXT("vehicles"), Arr) && Arr)
      {
        for (const TSharedPtr<FJsonValue>& V : *Arr)
        {
          const TSharedPtr<FJsonObject>* O = nullptr;
          if (!V->TryGetObject(O) || !O || !(*O).IsValid()) continue;
          VehicleImport::FTestMatrixSpec S;
          (*O)->TryGetStringField(TEXT("name"), S.VehicleName);
          (*O)->TryGetStringField(TEXT("bp"),   S.BPPath);
          if (!S.VehicleName.IsEmpty() && !S.BPPath.IsEmpty()) Specs.Add(S);
        }
      }
      auto P = MakeShared<TPromise<FString>>();
      TFuture<FString> Future = P->GetFuture();
      FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateLambda(
        [Specs, P](float) -> bool {
          VehicleImport::FTestMatrixResult R = VehicleImport::RunMatrix(Specs);
          P->SetValue(VehicleImport::ResultsToJson(R));
          return false;
        }));
      Response = Future.Get();
    }
    else if (Action.Equals(TEXT("spawn"), ESearchCase::IgnoreCase))
    {
      FSpawnRequest Req;
      Root->TryGetStringField(TEXT("asset_path"), Req.AssetPath);
      double V = 0.0;
      if (Root->TryGetNumberField(TEXT("x"),   V)) Req.Loc.X   = (float)V;
      if (Root->TryGetNumberField(TEXT("y"),   V)) Req.Loc.Y   = (float)V;
      if (Root->TryGetNumberField(TEXT("z"),   V)) Req.Loc.Z   = (float)V;
      if (Root->TryGetNumberField(TEXT("yaw"), V)) Req.Yaw     = (float)V;

      auto P = MakeShared<TPromise<FString>>();
      TFuture<FString> Future = P->GetFuture();
      FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateLambda(
        [this, Req, P](float) -> bool {
          P->SetValue(ProcessSpawn(Req));
          return false;
        }));
      Response = Future.Get();
    }
    else
    {
      FVehicleImportSpec Spec;
      if (!ParseSpec(JsonStr, Spec))
      {
        Response = MakeResponse(false, TEXT(""), TEXT("JSON parse error"));
      }
      else
      {
        auto P = MakeShared<TPromise<FString>>();
        TFuture<FString> Future = P->GetFuture();
        FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateLambda(
          [this, Spec, P](float) -> bool {
            P->SetValue(ProcessSpec(Spec));
            return false;
          }));
        Response = Future.Get();
      }
    }
  }

  FTCHARToUTF8 ResponseUTF8(*Response);
  const int32 RespLen = ResponseUTF8.Length();
  uint8 RespLenBuf[4] = {
    (uint8)(RespLen & 0xFF),
    (uint8)((RespLen >> 8)  & 0xFF),
    (uint8)((RespLen >> 16) & 0xFF),
    (uint8)((RespLen >> 24) & 0xFF)
  };
  SendAll(Client, RespLenBuf, 4);
  SendAll(Client, reinterpret_cast<const uint8*>(ResponseUTF8.Get()), RespLen);
}



static float JF(const TSharedPtr<FJsonObject>& O, const FString& K, float Def = 0.f)
{
  double V = Def;
  O->TryGetNumberField(K, V);
  return (float)V;
}

static TSharedPtr<FJsonObject> JO(const TSharedPtr<FJsonObject>& O, const FString& K)
{
  const TSharedPtr<FJsonObject>* Sub = nullptr;
  if (O->TryGetObjectField(K, Sub))
    return *Sub;
  return nullptr;
}

static void ParseWheel(const TSharedPtr<FJsonObject>& Root,
                       const FString& Key,
                       FWheelImportSpec& Out)
{
  TSharedPtr<FJsonObject> W = JO(Root, Key);
  if (!W) return;
  Out.X              = JF(W, "x");
  Out.Y              = JF(W, "y");
  Out.Z              = JF(W, "z");
  Out.Radius         = JF(W, "radius",           33.f);
  Out.Width          = JF(W, "width",            22.f);
  Out.MaxSteerAngle  = JF(W, "max_steer_angle",  70.f);
  Out.MaxBrakeTorque = JF(W, "max_brake_torque", 1500.f);
  Out.SuspMaxRaise   = JF(W, "susp_max_raise",   10.f);
  Out.SuspMaxDrop    = JF(W, "susp_max_drop",    10.f);
}

static void ParseChassisAabb(const TSharedPtr<FJsonObject>& Root, FVehicleImportSpec& Out)
{
  TSharedPtr<FJsonObject> A = JO(Root, "chassis_aabb_cm");
  if (!A) return;
  Out.ChassisXMin = JF(A, "x_min");  Out.ChassisXMax = JF(A, "x_max");
  Out.ChassisYMin = JF(A, "y_min");  Out.ChassisYMax = JF(A, "y_max");
  Out.ChassisZMin = JF(A, "z_min");  Out.ChassisZMax = JF(A, "z_max");
  Out.HasChassisAabb = (Out.ChassisXMax > Out.ChassisXMin)
                    && (Out.ChassisYMax > Out.ChassisYMin)
                    && (Out.ChassisZMax > Out.ChassisZMin);
}

bool FVehicleImporterServer::ParseSpec(const FString& Json, FVehicleImportSpec& Out)
{
  TSharedPtr<FJsonObject> Root;
  TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Json);
  if (!FJsonSerializer::Deserialize(Reader, Root) || !Root.IsValid())
    return false;

  Root->TryGetStringField(TEXT("vehicle_name"),   Out.VehicleName);
  Root->TryGetStringField(TEXT("mesh_path"),       Out.MeshFilePath);
  Root->TryGetStringField(TEXT("content_path"),    Out.ContentPath);
  Root->TryGetStringField(TEXT("base_vehicle_bp"), Out.BaseVehicleBP);

  double V = 1500.0;
  Root->TryGetNumberField(TEXT("mass"),            V); Out.Mass = (float)V;
  Root->TryGetNumberField(TEXT("susp_damping"),    V); Out.SuspDamping = (float)V;

  V = 1.0;
  Root->TryGetNumberField(TEXT("source_scale_to_cm"), V); Out.SourceScaleToCm = (float)V;
  int32 IV = 2;
  Root->TryGetNumberField(TEXT("source_up_axis"),      IV); Out.SourceUpAxis      = IV;
  IV = 0;
  Root->TryGetNumberField(TEXT("source_forward_axis"), IV); Out.SourceForwardAxis = IV;

  V = 0.0;
  Root->TryGetNumberField(TEXT("adjust_yaw_deg"),    V); Out.UserAdjustYawDeg  = (float)V;
  V = 1.0;
  Root->TryGetNumberField(TEXT("adjust_mirror_x"),   V); Out.UserAdjustMirrorX = (float)V;
  V = 1.0;
  Root->TryGetNumberField(TEXT("adjust_mirror_y"),   V); Out.UserAdjustMirrorY = (float)V;

  ParseWheel(Root, TEXT("wheel_fl"), Out.WheelFL);
  ParseWheel(Root, TEXT("wheel_fr"), Out.WheelFR);
  ParseWheel(Root, TEXT("wheel_rl"), Out.WheelRL);
  ParseWheel(Root, TEXT("wheel_rr"), Out.WheelRR);
  ParseChassisAabb(Root, Out);

  return !Out.VehicleName.IsEmpty() && !Out.MeshFilePath.IsEmpty();
}

FString FVehicleImporterServer::MakeResponse(bool bOk, const FString& Path, const FString& Err)
{
  TSharedPtr<FJsonObject> Obj = MakeShared<FJsonObject>();
  Obj->SetBoolField(TEXT("success"),      bOk);
  Obj->SetStringField(TEXT("asset_path"), Path);
  Obj->SetStringField(TEXT("error"),      Err);

  FString Out;
  TSharedRef<TJsonWriter<>> W = TJsonWriterFactory<>::Create(&Out);
  FJsonSerializer::Serialize(Obj.ToSharedRef(), W);
  return Out;
}





static FString SanitizeAssetName(const FString& In)
{
  FString Out;
  Out.Reserve(In.Len());
  for (TCHAR C : In)
  {
    const bool bOk = (C >= 'A' && C <= 'Z') || (C >= 'a' && C <= 'z')
                  || (C >= '0' && C <= '9') || C == '_';
    Out.AppendChar(bOk ? C : TEXT('_'));
  }
  if (Out.IsEmpty()) Out = TEXT("Vehicle");
  if (Out[0] >= '0' && Out[0] <= '9') Out = TEXT("V_") + Out;
  return Out;
}

static UStaticMesh* ImportStaticMesh(const FString& FilePath,
                                     const FString& ContentPath,
                                     const FString& AssetName,
                                     const FVehicleImportSpec& Spec)
{
  UAssetImportTask* Task = NewObject<UAssetImportTask>();
  Task->Filename        = FilePath;
  Task->DestinationPath = ContentPath;
  Task->DestinationName = SanitizeAssetName(AssetName);

  

  Task->bSave           = false;
  Task->bAutomated      = true;
  Task->bReplaceExisting = true;

  UInterchangeGenericAssetsPipeline* Pipeline = nullptr;
  if (const UInterchangeProjectSettings* Settings = GetDefault<UInterchangeProjectSettings>())
  {
    if (UClass* PipelineClass = Settings->GenericPipelineClass.LoadSynchronous())
    {
      Pipeline = NewObject<UInterchangeGenericAssetsPipeline>(GetTransientPackage(), PipelineClass);
    }
  }
  if (!Pipeline)
  {
    Pipeline = NewObject<UInterchangeGenericAssetsPipeline>(GetTransientPackage());
  }
  Pipeline->ClearFlags(EObjectFlags::RF_Standalone | EObjectFlags::RF_Public);
  if (Pipeline->MaterialPipeline)
  {
    Pipeline->MaterialPipeline->bImportMaterials = true;
    Pipeline->MaterialPipeline->bIdentifyDuplicateMaterials = true;
    Pipeline->MaterialPipeline->bCreateMaterialInstanceForParent = false;
    Pipeline->MaterialPipeline->MaterialImport = EInterchangeMaterialImportOption::ImportAsMaterials;
  }

  UInterchangePipelineStackOverride* StackOverride = NewObject<UInterchangePipelineStackOverride>(GetTransientPackage());
  StackOverride->AddPipeline(Pipeline);
  Task->Options = StackOverride;

  IAssetTools& AssetTools =
    FModuleManager::GetModuleChecked<FAssetToolsModule>("AssetTools").Get();
  AssetTools.ImportAssetTasks({ Task });

  

  
  
  TArray<UObject*> Imported = Task->GetObjects();
  if (Imported.Num() == 0)
    return nullptr;
  UStaticMesh* Mesh = nullptr;
  for (UObject* Obj : Imported)
  {
    if (UStaticMesh* SM = Cast<UStaticMesh>(Obj))
    {
      Mesh = SM;
      break;
    }
  }
  if (!Mesh)
  {
    UE_LOG(LogCarlaTools, Warning,
           TEXT("VI.ImportStaticMesh: Interchange returned %d object(s) but "
                "none was a UStaticMesh"), Imported.Num());
    return nullptr;
  }

  

  
  
  if (Mesh->GetNumSourceModels() > 0)
  {
    FStaticMeshSourceModel& SM = Mesh->GetSourceModel(0);
    if (!SM.BuildSettings.BuildScale3D.Equals(FVector(1.0)))
    {
      SM.BuildSettings.BuildScale3D = FVector(1.0);
      Mesh->Build(false);
      Mesh->MarkPackageDirty();
    }
  }
  return Mesh;
}

static TSubclassOf<UChaosVehicleWheel> CreateWheelBlueprint(
    const FString& ContentPath,
    const FString& Name,
    const FString& Suffix,
    const FWheelImportSpec& W,
    UStaticMesh* ShrunkShapeOverride)
{
  IAssetTools& AssetTools =
    FModuleManager::GetModuleChecked<FAssetToolsModule>("AssetTools").Get();

  UBlueprintFactory* Factory = NewObject<UBlueprintFactory>();

  

  

  

  const FString WheelTemplatePath = FString::Printf(
    TEXT("/Game/Carla/Blueprints/Vehicles/Mustang/BP_Mustang_%s.BP_Mustang_%s_C"),
    *Suffix, *Suffix);
  UClass* WheelTemplate = LoadObject<UClass>(nullptr, *WheelTemplatePath);
  if (!WheelTemplate)
  {
    UE_LOG(LogCarlaTools, Warning,
           TEXT("VI.CreateWheel: BP_Mustang_%s not loadable at %s — "
                "falling back to UChaosVehicleWheel; throttle won't drive wheels."),
           *Suffix, *WheelTemplatePath);
    WheelTemplate = UChaosVehicleWheel::StaticClass();
  }
  Factory->ParentClass = WheelTemplate;

  UObject* Asset = AssetTools.CreateAsset(Name, ContentPath,
                                          UBlueprint::StaticClass(), Factory);
  UBlueprint* BP = Cast<UBlueprint>(Asset);
  if (!BP || !BP->GeneratedClass) return nullptr;

  

  
  
  UChaosVehicleWheel* Defaults =
    Cast<UChaosVehicleWheel>(BP->GeneratedClass->ClassDefaultObject);
  if (Defaults)
  {
    Defaults->WheelRadius        = W.Radius;
    Defaults->WheelWidth         = W.Width;
    Defaults->MaxSteerAngle      = W.MaxSteerAngle;
    Defaults->MaxBrakeTorque     = W.MaxBrakeTorque;
    Defaults->SuspensionMaxRaise = W.SuspMaxRaise;
    Defaults->SuspensionMaxDrop  = W.SuspMaxDrop;

    

    const bool bIsFront = Suffix.StartsWith(TEXT("F"));

    
    Defaults->bAffectedByEngine    = true;
    Defaults->bAffectedBySteering  = bIsFront;
    Defaults->bAffectedByHandbrake = !bIsFront;
    Defaults->bAffectedByBrake     = true;
    static const TCHAR* const kWheelShapePath =
      TEXT("/Game/Carla/Blueprints/Vehicles/Wheel_Shape.Wheel_Shape");
    UStaticMesh* Shape = ShrunkShapeOverride
        ? ShrunkShapeOverride
        : LoadObject<UStaticMesh>(nullptr, kWheelShapePath);
    if (Shape)
    {
      Defaults->CollisionMesh = Shape;
    }
    else
    {
      UE_LOG(LogCarlaTools, Warning,
             TEXT("VI.CreateWheel: Wheel_Shape not loadable at %s — wheel BP "
                  "will spawn-fail in CARLA."), kWheelShapePath);
    }
  }
  return TSubclassOf<UChaosVehicleWheel>(BP->GeneratedClass);
}

namespace VehicleImport
{
  FString SanitizeAssetName(const FString& In)
  {
    return ::SanitizeAssetName(In);
  }
  UStaticMesh* ImportStaticMesh(const FString& FilePath,
                                const FString& ContentPath,
                                const FString& AssetName,
                                const FVehicleImportSpec& Spec)
  {
    return ::ImportStaticMesh(FilePath, ContentPath, AssetName, Spec);
  }
  UStaticMesh* MakeShrunkWheelShape(const FString& VehicleContentPath,
                                    const FString& Suffix,
                                    float RadiusCm,
                                    float WidthCm)
  {
    return ::MakeShrunkWheelShape(VehicleContentPath, Suffix, RadiusCm, WidthCm);
  }
  TSubclassOf<UChaosVehicleWheel> CreateWheelBlueprint(
      const FString& ContentPath,
      const FString& Name,
      const FString& Suffix,
      const FWheelImportSpec& W,
      UStaticMesh* ShrunkShapeOverride)
  {
    return ::CreateWheelBlueprint(ContentPath, Name, Suffix, W, ShrunkShapeOverride);
  }
  void ApplyChassisAabbResize(UPhysicsAsset* PA,
                              const FVehicleImportSpec& Spec,
                              bool bHaveAabb)
  {
    ::ApplyChassisAabbResize(PA, Spec, bHaveAabb);
  }
}

FString FVehicleImporterServer::ProcessSpec(const FVehicleImportSpec& Spec)
{
  using namespace VehicleImport;

  UE_LOG(LogCarlaTools, Display, TEXT("VI.ProcessSpec: enter (vehicle=%s mesh=%s)"),
         *Spec.VehicleName, *Spec.MeshFilePath);

  FNormalizedSpec Norm;
  if (FStageError E = RunStage_Preflight(Spec, Norm); !E.IsOk())
  {
    UE_LOG(LogCarlaTools, Warning, TEXT("VI.ProcessSpec: preflight failed: %s"), *E.ToString());
    return MakeResponse(false, TEXT(""), E.ToString());
  }
  UE_LOG(LogCarlaTools, Display,
         TEXT("VI.ProcessSpec: stage 1/5 preflight ok (mesh=%s, sanitized=%s)"),
         *Norm.MeshFilePathToImport, Norm.bWasSanitized ? TEXT("yes") : TEXT("no"));

  FInterchangeOutputs IcOut;
  if (FStageError E = RunStage_Interchange(Norm, Spec, IcOut); !E.IsOk())
  {
    UE_LOG(LogCarlaTools, Warning, TEXT("VI.ProcessSpec: interchange failed: %s"), *E.ToString());
    return MakeResponse(false, TEXT(""), E.ToString());
  }
  UE_LOG(LogCarlaTools, Display, TEXT("VI.ProcessSpec: stage 2/5 interchange ok"));

  FBuildSKOutputs SkOut;
  if (FStageError E = RunStage_BuildSK(Norm, Spec, SkOut); !E.IsOk())
  {
    UE_LOG(LogCarlaTools, Warning, TEXT("VI.ProcessSpec: build SK failed: %s"), *E.ToString());
    return MakeResponse(false, TEXT(""), E.ToString());
  }
  UE_LOG(LogCarlaTools, Display, TEXT("VI.ProcessSpec: stage 3/5 build SK ok"));

  FBuildBPInputs BpIn;
  BpIn.BodyMesh  = IcOut.BodyMesh;
  BpIn.SkelMesh  = SkOut.SkelMesh;
  BpIn.PhysAsset = SkOut.PhysAsset;
  FBuildBPOutputs BpOut;
  if (FStageError E = RunStage_BuildBP(Norm, Spec, BpIn, BpOut); !E.IsOk())
  {
    UE_LOG(LogCarlaTools, Warning, TEXT("VI.ProcessSpec: build BP failed: %s"), *E.ToString());
    return MakeResponse(false, TEXT(""), E.ToString());
  }
  UE_LOG(LogCarlaTools, Display, TEXT("VI.ProcessSpec: stage 4/5 build BP ok (%s)"), *BpOut.BPPath);

  FPersistInputs PIn;
  PIn.VehicleContentPath    = Norm.VehicleContentPath;
  PIn.VehicleName           = Norm.VehicleName;
  PIn.BPPath                = BpOut.BPPath;
  PIn.PhysAssetPath         = BpOut.PhysAssetPath;
  PIn.WheelBPPaths          = BpOut.WheelBPPaths;
  PIn.ShrunkWheelShapePaths = BpOut.ShrunkWheelShapePaths;
  FPersistOutputs POut;
  if (FStageError E = RunStage_Persist(PIn, POut); !E.IsOk())
  {
    UE_LOG(LogCarlaTools, Warning, TEXT("VI.ProcessSpec: persist failed: %s"), *E.ToString());
    return MakeResponse(false, TEXT(""), E.ToString());
  }
  UE_LOG(LogCarlaTools, Display, TEXT("VI.ProcessSpec: stage 5/5 persist ok (%d/%d)"),
         POut.SavedCount, POut.RequestedCount);

  return MakeResponse(true, BpOut.BPPath, TEXT(""));
}



FString FVehicleImporterServer::ProcessSpawn(const FSpawnRequest& Req)
{
  UE_LOG(LogCarlaTools, Display, TEXT("VI.ProcessSpawn: enter (asset=%s loc=%s yaw=%.1f)"),
         *Req.AssetPath, *Req.Loc.ToString(), Req.Yaw);

  if (Req.AssetPath.IsEmpty())
    return MakeResponse(false, TEXT(""), TEXT("spawn: asset_path missing"));

  

  
  
  auto fixupClassPath = [](const FString& In) -> FString {

    
    if (In.EndsWith(TEXT("_C"))) return In;
    int32 Dot = INDEX_NONE; In.FindLastChar(TEXT('.'), Dot);
    int32 Slash = INDEX_NONE; In.FindLastChar(TEXT('/'), Slash);
    if (Dot == INDEX_NONE || Dot < Slash)
    {
      const FString Leaf = (Slash == INDEX_NONE) ? In : In.RightChop(Slash + 1);
      return In + TEXT(".") + Leaf + TEXT("_C");
    }
    return In + TEXT("_C");
  };

  UClass* Cls = nullptr;
  FString AttemptedPaths;
  
  {
    AttemptedPaths += Req.AssetPath;
    UObject* Loaded = UEditorAssetLibrary::LoadAsset(Req.AssetPath);
    if (UBlueprint* BP = Cast<UBlueprint>(Loaded))
      Cls = BP->GeneratedClass;
    else if (UClass* DirectClass = Cast<UClass>(Loaded))
      Cls = DirectClass;
  }
  
  if (!Cls)
  {
    const FString Fixed = fixupClassPath(Req.AssetPath);
    AttemptedPaths += TEXT(" | ") + Fixed;
    Cls = LoadClass<AActor>(nullptr, *Fixed);
  }

  if (!Cls)
  {
    const FString Fixed = fixupClassPath(Req.AssetPath);
    Cls = Cast<UClass>(StaticLoadObject(UClass::StaticClass(), nullptr, *Fixed));
  }

  if (!Cls || !Cls->IsChildOf(AActor::StaticClass()))
    return MakeResponse(false, TEXT(""),
      FString::Printf(TEXT("spawn: could not resolve actor class. Tried: %s"),
                      *AttemptedPaths));

  UWorld* World = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
  if (!World)
    return MakeResponse(false, TEXT(""), TEXT("spawn: no editor world available"));

  FActorSpawnParameters Params;
  Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
  AActor* Spawned = World->SpawnActor<AActor>(
      Cls, FTransform(FRotator(0.f, Req.Yaw, 0.f), Req.Loc), Params);
  if (!Spawned)
    return MakeResponse(false, TEXT(""),
      FString::Printf(TEXT("spawn: SpawnActor returned null for %s"), *Cls->GetName()));

  UE_LOG(LogCarlaTools, Display, TEXT("VI.ProcessSpawn: spawned %s (label=%s)"),
         *Cls->GetName(), *Spawned->GetActorLabel());
  return MakeResponse(true, Spawned->GetActorLabel(), TEXT(""));
}
