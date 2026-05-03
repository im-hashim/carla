#include "Stages/Stage_Preflight.h"
#include "VehicleImporter.h"
#include "CarlaTools.h"

#include <util/ue-header-guard-begin.h>
#include "HAL/FileManager.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include <util/ue-header-guard-end.h>

#include <cmath>

namespace VehicleImport
{

static bool IsSafeChar(TCHAR C)
{
  return (C >= 'A' && C <= 'Z') || (C >= 'a' && C <= 'z')
      || (C >= '0' && C <= '9') || C == '_' || C == '-' || C == '.';
}

// ---------------------------------------------------------------------------
// OBJ canonicalize
// ---------------------------------------------------------------------------
//
// Goal: rewrite the source OBJ so that, downstream, Interchange + the build
// pipeline see a mesh in CARLA's canonical convention:
//
//   +X = lateral right
//   +Y = forward
//   +Z = up
//   units = cm  (we leave in source units; ScaleAppliedToCm just records what
//                the source units appear to be — the body StaticMesh's
//                BuildScale is left to UE/Studio's existing path)
//
// Three transforms are applied in sequence per vertex:
//   1. Optional scalar from `# Scale  X  Y  Z` comment in the header
//      (CarMaker / IPG OBJ exports use this; rr.obj is the prototype).
//   2. Permutation that maps the source's longest extent → +Y, source's
//      smallest non-forward extent → +Z, the third axis → +X.
//   3. Sign flip on +Y if more vertices originally lay on the −forward side
//      (i.e. the model points the wrong way).

struct FObjScaleHint
{
  bool bFound = false;
  float X = 1.f, Y = 1.f, Z = 1.f;
};

static FObjScaleHint ParseScaleComment(const TArray<FString>& Lines)
{
  FObjScaleHint Out;
  for (const FString& L : Lines)
  {
    const FString T = L.TrimStartAndEnd();
    // CarMaker / IPG style:  "# Scale 90 90 90"
    if (T.StartsWith(TEXT("# Scale "), ESearchCase::IgnoreCase))
    {
      TArray<FString> Parts;
      T.ParseIntoArray(Parts, TEXT(" "), true);
      if (Parts.Num() >= 5)
      {
        Out.bFound = true;
        Out.X = FCString::Atof(*Parts[2]);
        Out.Y = FCString::Atof(*Parts[3]);
        Out.Z = FCString::Atof(*Parts[4]);
        return Out;
      }
    }
  }
  return Out;
}

static bool ParseVertex(const FString& Line, double& OutX, double& OutY, double& OutZ)
{
  if (!Line.StartsWith(TEXT("v "))) return false;
  TArray<FString> Parts;
  Line.ParseIntoArray(Parts, TEXT(" "), true);
  if (Parts.Num() < 4) return false;
  OutX = FCString::Atod(*Parts[1]);
  OutY = FCString::Atod(*Parts[2]);
  OutZ = FCString::Atod(*Parts[3]);
  return true;
}

struct FAxisPlan
{
  // Which source-axis index (0,1,2) maps to canonical +X / +Y / +Z.
  int32 SrcLat  = 0;   // → +X
  int32 SrcFwd  = 1;   // → +Y
  int32 SrcUp   = 2;   // → +Z
  // Sign multipliers applied AFTER the scale, BEFORE the permutation.
  float SignLat = +1.f;
  float SignFwd = +1.f;
  float SignUp  = +1.f;
  // Uniform multiplier to push the post-Scale, post-permutation result into
  // CM range (UE's native unit). Computed from the post-Scale bbox: if the
  // largest extent is < 50, we're in metres, multiply by 100; otherwise 1.0.
  float UnitToCm = 1.f;
  bool  bIsIdentity = true;
};

static FAxisPlan AnalyseAndPlan(const TArray<FString>& Lines, const FObjScaleHint& Hint)
{
  // Pass 1: bbox + per-axis vert distribution around centroid.
  double mn[3] = { 1e30, 1e30, 1e30 };
  double mx[3] = {-1e30,-1e30,-1e30 };
  int64 N = 0;
  for (const FString& L : Lines)
  {
    double x, y, z;
    if (!ParseVertex(L, x, y, z)) continue;
    if (Hint.bFound) { x *= Hint.X; y *= Hint.Y; z *= Hint.Z; }
    if (x < mn[0]) mn[0] = x; if (x > mx[0]) mx[0] = x;
    if (y < mn[1]) mn[1] = y; if (y > mx[1]) mx[1] = y;
    if (z < mn[2]) mn[2] = z; if (z > mx[2]) mx[2] = z;
    ++N;
  }
  if (N == 0) return FAxisPlan();

  const double ext[3] = { mx[0]-mn[0], mx[1]-mn[1], mx[2]-mn[2] };
  // Forward axis = longest extent.
  int fwd = (ext[0] >= ext[1] && ext[0] >= ext[2]) ? 0
          : (ext[1] >= ext[2]) ? 1 : 2;
  // Up axis = smallest extent (cars are wider than tall, way longer than wide).
  int up;
  {
    int candidates[2];
    int k = 0;
    for (int i = 0; i < 3; ++i) if (i != fwd) candidates[k++] = i;
    up = (ext[candidates[0]] <= ext[candidates[1]]) ? candidates[0] : candidates[1];
  }
  int lat = 3 - fwd - up;

  // Pass 2: forward-sign vote.
  const double ctrFwd = 0.5 * (mn[fwd] + mx[fwd]);
  int64 ahead = 0, behind = 0;
  for (const FString& L : Lines)
  {
    double v[3];
    if (!ParseVertex(L, v[0], v[1], v[2])) continue;
    if (Hint.bFound) { v[0] *= Hint.X; v[1] *= Hint.Y; v[2] *= Hint.Z; }
    if (v[fwd] > ctrFwd) ++ahead; else ++behind;
  }
  const float fwdSign = (ahead >= behind) ? +1.f : -1.f;

  FAxisPlan P;
  P.SrcLat = lat; P.SrcFwd = fwd; P.SrcUp = up;
  // The permutation already chose the source axis whose extent best matches
  // the canonical role; signs tell us if we then need to flip.
  P.SignLat = +1.f;
  P.SignFwd = fwdSign;
  P.SignUp  = +1.f;

  // Bring everything to CM. If the post-Scale extent is small (< 50), source
  // is metres-equivalent; multiply by 100. Otherwise treat as already-cm.
  const double maxPostScale = std::max({ext[0], ext[1], ext[2]});
  P.UnitToCm = (maxPostScale > 0.0 && maxPostScale < 50.0) ? 100.f : 1.f;

  P.bIsIdentity = (lat == 0 && fwd == 1 && up == 2 && fwdSign > 0.f
                   && std::fabs(P.UnitToCm - 1.f) < 1e-6f);
  return P;
}

static FString FmtV(double v)
{
  // OBJ-style fixed precision; trims trailing zeros for compactness.
  return FString::Printf(TEXT("%.6f"), v);
}

// Apply (scale * sign-flip * axis-permutation * unit-rescale * user-yaw * user-mirror)
// in that order to one (x,y,z).
static void Apply(const FObjScaleHint& Hint, const FAxisPlan& P,
                  float UserYawDeg, float UserMirrorX, float UserMirrorY,
                  double X, double Y, double Z,
                  double& OX, double& OY, double& OZ)
{
  if (Hint.bFound) { X *= Hint.X; Y *= Hint.Y; Z *= Hint.Z; }
  const double src[3] = { X, Y, Z };
  double cx = src[P.SrcLat] * P.SignLat * P.UnitToCm;
  double cy = src[P.SrcFwd] * P.SignFwd * P.UnitToCm;
  double cz = src[P.SrcUp]  * P.SignUp  * P.UnitToCm;

  if (UserMirrorX < 0.f) cx = -cx;
  if (UserMirrorY < 0.f) cy = -cy;

  if (FMath::Abs(UserYawDeg) > 1e-3f) {
    const double r = double(UserYawDeg) * (M_PI / 180.0);
    const double c = std::cos(r), s = std::sin(r);
    const double nx = cx * c - cy * s;
    const double ny = cx * s + cy * c;
    cx = nx; cy = ny;
  }
  OX = cx; OY = cy; OZ = cz;
}

static bool CanonicalizeObj(const FString& InPath, const FString& OutPath,
                            FAxisPlan& OutPlan, FObjScaleHint& OutHint,
                            float UserYawDeg, float UserMirrorX, float UserMirrorY,
                            bool& OutChanged)
{
  OutChanged = false;
  FString Body;
  if (!FFileHelper::LoadFileToString(Body, *InPath)) return false;
  TArray<FString> Lines;
  Body.ParseIntoArrayLines(Lines, false);

  OutHint = ParseScaleComment(Lines);
  OutPlan = AnalyseAndPlan(Lines, OutHint);
  const bool bUserAdjust = (FMath::Abs(UserYawDeg) > 1e-3f)
                        || (UserMirrorX < 0.f) || (UserMirrorY < 0.f);
  const bool bIdentity = OutPlan.bIsIdentity && !OutHint.bFound && !bUserAdjust;
  if (bIdentity) return true;   // nothing to rewrite

  TArray<FString> WriteLines;
  WriteLines.Reserve(Lines.Num());
  for (const FString& L : Lines)
  {
    if (L.StartsWith(TEXT("v ")))
    {
      double x, y, z;
      if (ParseVertex(L, x, y, z))
      {
        double ox, oy, oz;
        Apply(OutHint, OutPlan, UserYawDeg, UserMirrorX, UserMirrorY, x, y, z, ox, oy, oz);
        WriteLines.Add(FString::Printf(TEXT("v %s %s %s"), *FmtV(ox), *FmtV(oy), *FmtV(oz)));
        continue;
      }
    }
    else if (L.StartsWith(TEXT("vn ")))
    {
      // Normals: same permutation/sign + user yaw/mirror, NO scale.
      TArray<FString> Parts;
      L.ParseIntoArray(Parts, TEXT(" "), true);
      if (Parts.Num() >= 4)
      {
        const double n[3] = {
          FCString::Atod(*Parts[1]),
          FCString::Atod(*Parts[2]),
          FCString::Atod(*Parts[3])
        };
        double ox = n[OutPlan.SrcLat] * OutPlan.SignLat;
        double oy = n[OutPlan.SrcFwd] * OutPlan.SignFwd;
        double oz = n[OutPlan.SrcUp]  * OutPlan.SignUp;
        if (UserMirrorX < 0.f) ox = -ox;
        if (UserMirrorY < 0.f) oy = -oy;
        if (FMath::Abs(UserYawDeg) > 1e-3f) {
          const double r = double(UserYawDeg) * (M_PI / 180.0);
          const double c = std::cos(r), s = std::sin(r);
          const double nx = ox * c - oy * s;
          const double ny = ox * s + oy * c;
          ox = nx; oy = ny;
        }
        WriteLines.Add(FString::Printf(TEXT("vn %s %s %s"), *FmtV(ox), *FmtV(oy), *FmtV(oz)));
        continue;
      }
    }
    WriteLines.Add(L);
  }

  OutChanged = true;
  return FFileHelper::SaveStringToFile(FString::Join(WriteLines, TEXT("\n")), *OutPath);
}

// ---------------------------------------------------------------------------
// Header scrub (existing: strip ### BEGIN/END IPG-MOVIE-INFO blocks)
// ---------------------------------------------------------------------------

static bool ScrubObjHeaders(const FString& InPath, const FString& OutPath)
{
  FString Body;
  if (!FFileHelper::LoadFileToString(Body, *InPath)) return false;
  TArray<FString> Lines;
  Body.ParseIntoArrayLines(Lines, false);
  bool bChanged = false;
  bool bInBlock = false;
  TArray<FString> Out;
  Out.Reserve(Lines.Num());
  for (const FString& L : Lines)
  {
    const FString T = L.TrimStartAndEnd();
    if (T.StartsWith(TEXT("###")))
    {
      if (T.Contains(TEXT("BEGIN"))) { bInBlock = true; bChanged = true; continue; }
      if (T.Contains(TEXT("END")))   { bInBlock = false; bChanged = true; continue; }
      bChanged = true; continue;
    }
    if (bInBlock) { bChanged = true; continue; }
    Out.Add(L);
  }
  if (!bChanged) return false;
  return FFileHelper::SaveStringToFile(FString::Join(Out, TEXT("\n")), *OutPath);
}

FStageError RunStage_Preflight(const FVehicleImportSpec& InSpec, FNormalizedSpec& OutNorm)
{
  if (InSpec.MeshFilePath.IsEmpty())
  {
    return FStageError::Make(EStageCode::Preflight, EStageErrorKind::BadFilename,
      TEXT("MeshFilePath is empty"));
  }
  if (!IFileManager::Get().FileExists(*InSpec.MeshFilePath))
  {
    return FStageError::Make(EStageCode::Preflight, EStageErrorKind::BadFilename,
      FString::Printf(TEXT("Source mesh not found: %s"), *InSpec.MeshFilePath));
  }

  OutNorm.VehicleName = InSpec.VehicleName;
  OutNorm.OriginalMeshFilePath = InSpec.MeshFilePath;
  OutNorm.MeshFilePathToImport = InSpec.MeshFilePath;

  FString ContentRoot = InSpec.ContentPath;
  if (ContentRoot.EndsWith(TEXT("/")))
    ContentRoot.RemoveFromEnd(TEXT("/"));
  OutNorm.VehicleContentPath = ContentRoot / InSpec.VehicleName;

  // 1. Sanitize filename (rename to /tmp path with no special chars).
  const FString OrigName = FPaths::GetCleanFilename(InSpec.MeshFilePath);
  bool bNeedsSanitize = false;
  for (TCHAR C : OrigName)
  {
    if (!IsSafeChar(C)) { bNeedsSanitize = true; break; }
  }

  if (bNeedsSanitize)
  {
    const FString Ext = FPaths::GetExtension(OrigName, true);
    const FString Safe = FString::Printf(TEXT("/tmp/vi_%s_body%s"),
                                         *InSpec.VehicleName, *Ext);
    IFileManager::Get().Delete(*Safe, false, true, true);
    if (IFileManager::Get().Copy(*Safe, *InSpec.MeshFilePath, true) == COPY_OK)
    {
      OutNorm.MeshFilePathToImport = Safe;
      OutNorm.bWasSanitized = true;
    }
    else
    {
      return FStageError::Make(EStageCode::Preflight, EStageErrorKind::BadFilename,
        FString::Printf(TEXT("Failed to copy '%s' to sanitized path '%s'"),
                        *InSpec.MeshFilePath, *Safe));
    }
  }

  OutNorm.bHeadersStripped = false;
  OutNorm.ForwardAxisSign = +1;

  const bool bIsObj = FPaths::GetExtension(OutNorm.MeshFilePathToImport)
                          .Equals(TEXT("obj"), ESearchCase::IgnoreCase);
  if (bIsObj)
  {
    // 2. Canonicalize FIRST (so the # Scale comment, which often lives inside
    //    the IPG ### block, is still readable). Detects forward axis + sign +
    //    up axis, applies # Scale, rewrites verts/normals into
    //    (lateral=+X, forward=+Y, up=+Z).
    const FString Canonical = FString::Printf(TEXT("/tmp/vi_%s_body_canonical.obj"),
                                              *InSpec.VehicleName);
    FAxisPlan Plan;
    FObjScaleHint Hint;
    bool bChanged = false;
    if (CanonicalizeObj(OutNorm.MeshFilePathToImport, Canonical, Plan, Hint,
                        InSpec.UserAdjustYawDeg,
                        InSpec.UserAdjustMirrorX,
                        InSpec.UserAdjustMirrorY,
                        bChanged))
    {
      OutNorm.SrcLateralAxis = Plan.SrcLat;
      OutNorm.SrcForwardAxis = Plan.SrcFwd;
      OutNorm.SrcUpAxis      = Plan.SrcUp;
      OutNorm.ForwardAxisSign = (Plan.SignFwd >= 0.f) ? +1 : -1;
      if (Hint.bFound) OutNorm.ScaleAppliedToCm = Hint.X;   // diag-only

      if (bChanged)
      {
        OutNorm.MeshFilePathToImport = Canonical;
        OutNorm.bWasCanonicalized = true;
        UE_LOG(LogCarlaTools, Display,
               TEXT("VI.Stage.Preflight: canonicalized OBJ — src axes "
                    "(lat=%d fwd=%d up=%d, fwdSign=%s, scale=%s, unit2cm=%.0f, "
                    "userYaw=%.0f, userMirror=(%s,%s)) -> %s"),
               Plan.SrcLat, Plan.SrcFwd, Plan.SrcUp,
               Plan.SignFwd > 0 ? TEXT("+") : TEXT("-"),
               Hint.bFound ? *FString::Printf(TEXT("%.4f"), Hint.X) : TEXT("none"),
               Plan.UnitToCm,
               InSpec.UserAdjustYawDeg,
               InSpec.UserAdjustMirrorX > 0 ? TEXT("+") : TEXT("-"),
               InSpec.UserAdjustMirrorY > 0 ? TEXT("+") : TEXT("-"),
               *Canonical);
      }
    }

    // 3. Strip IPG MOVIE-INFO blocks that confuse Interchange's OBJ parser.
    //    Done AFTER canonicalize so the scale comment was available above.
    const FString Cleaned = FString::Printf(TEXT("/tmp/vi_%s_body_clean.obj"),
                                            *InSpec.VehicleName);
    if (ScrubObjHeaders(OutNorm.MeshFilePathToImport, Cleaned))
    {
      OutNorm.MeshFilePathToImport = Cleaned;
      OutNorm.bHeadersStripped = true;
    }
  }

  return FStageError::Ok();
}

}
