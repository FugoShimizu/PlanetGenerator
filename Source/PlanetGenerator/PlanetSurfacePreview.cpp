#include "PlanetSurfacePreview.h"

#include "HAL/PlatformTime.h"
#include "Materials/MaterialInterface.h"
#include "ProceduralMeshComponent.h"
#include "UObject/ConstructorHelpers.h"

namespace
{
FVector ToUnreal(PlanetCore::Vec3 V) { return FVector(V.X, V.Y, V.Z); }
}

APlanetSurfacePreview::APlanetSurfacePreview()
{
    PrimaryActorTick.bCanEverTick = false;
    PreviewMesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("DiagnosticSphere"));
    SetRootComponent(PreviewMesh);
    PreviewMesh->SetFlags(RF_Transient);
    PreviewMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    PreviewMesh->SetCanEverAffectNavigation(false);
    static ConstructorHelpers::FObjectFinder<UMaterialInterface> Material(TEXT("/Engine/EngineDebugMaterials/VertexColorMaterial.VertexColorMaterial"));
    if (Material.Succeeded()) PreviewMesh->SetMaterial(0, Material.Object);
}

void APlanetSurfacePreview::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);
    RebuildPreview();
}

bool APlanetSurfacePreview::RebuildPreview()
{
    const double Start = FPlatformTime::Seconds();
    const PlanetCore::Definition Definition{static_cast<std::uint64_t>(Seed), 1, RadiusMeters, HeightAmplitudeMeters, 0};
    PlanetCore::GeologySettings GeologySettings;
    GeologySettings.Seed = static_cast<std::uint64_t>(Seed);
    GeologySettings.RadiusMeters = RadiusMeters;
    if ((bUseGeologicalBase ? !GeologySettings.IsValid() : !Definition.IsValid()) || RadiusMeters < 1 || RadiusMeters > 1e9 || SubdivisionLevel < 0 || SubdivisionLevel > 6 ||
        !GetActorScale3D().Equals(FVector::OneVector, 1e-9))
    {
        LastBuildMessage = TEXT("Rejected invalid diagnostic parameters; existing geometry retained.");
        return false;
    }
    std::optional<PlanetCore::DiagnosticField> CandidateField;
    std::shared_ptr<const PlanetCore::GeologyField> CandidateGeology;
    double PreparationMilliseconds = 0;
    if (bUseGeologicalBase)
    {
        CandidateGeology = GeologicalField;
        if (!CandidateGeology || CandidateGeology->Settings().IdentityHash() != GeologySettings.IdentityHash())
        {
            auto Prepared = PlanetCore::GeologyField::Create(GeologySettings);
            if (!Prepared)
            {
                LastBuildMessage = TEXT("Geology preparation rejected; existing geometry retained.");
                return false;
            }
            CandidateGeology = std::make_shared<const PlanetCore::GeologyField>(std::move(*Prepared));
            PreparationMilliseconds = (FPlatformTime::Seconds() - Start) * 1000.0;
        }
    }
    else CandidateField = PlanetCore::DiagnosticField::Create(Definition);
    std::shared_ptr<const PlanetCore::OceanConnectivityField> CandidateOcean;
    double OceanPreparation = 0;
    if (CandidateGeology && bConnectOceanBasins)
    {
        CandidateOcean = OceanField;
        if (!CandidateOcean || &CandidateOcean->Base() != CandidateGeology.get())
        {
            const double OceanStart = FPlatformTime::Seconds();
            auto Prepared = PlanetCore::OceanConnectivityField::Create(CandidateGeology);
            if (!Prepared)
            {
                LastBuildMessage = TEXT("Ocean preparation rejected; existing geometry retained.");
                return false;
            }
            CandidateOcean = std::make_shared<const PlanetCore::OceanConnectivityField>(std::move(*Prepared));
            OceanPreparation = (FPlatformTime::Seconds() - OceanStart) * 1000;
        }
    }
    const auto Sphere = PlanetCore::BuildUniformSphere(static_cast<std::uint8_t>(SubdivisionLevel));
    if ((!CandidateField && !CandidateGeology) || !Sphere)
    {
        LastBuildMessage = TEXT("Core rejected the diagnostic mesh; existing geometry retained.");
        return false;
    }
    TArray<FVector> Vertices, Normals;
    TArray<int32> Indices;
    TArray<FLinearColor> Colors;
    TArray<FVector2D> UVs;
    TArray<FProcMeshTangent> Tangents;
    const int32 VertexCount = static_cast<int32>(Sphere->Directions.size());
    Vertices.Reserve(VertexCount); Normals.Reserve(VertexCount); Colors.Reserve(VertexCount); Tangents.Reserve(VertexCount);
    Indices.Reserve(static_cast<int32>(Sphere->Triangles.size() * 3));
    for (const auto& Direction : Sphere->Directions)
    {
        const auto Sample = CandidateOcean ? CandidateOcean->Evaluate(Direction) : CandidateGeology ? CandidateGeology->Evaluate(Direction).Terrain : CandidateField->Evaluate(Direction);
        const auto RadialPosition = Direction.Vector() * (RadiusMeters + Sample.ElevationMeters);
        const auto UnitNormal = PlanetCore::UnitDirection::FromVector(RadialPosition - Sample.TangentialGradientPerRadian);
        if (!UnitNormal)
        {
            LastBuildMessage = TEXT("Invalid normal rejected; existing geometry retained.");
            return false;
        }
        const FVector Position = ToUnreal(RadialPosition) * 100.0;
        const FVector Normal = ToUnreal(UnitNormal->Vector());
        if (Position.ContainsNaN() || Normal.ContainsNaN())
        {
            LastBuildMessage = TEXT("Non-finite geometry rejected; existing geometry retained.");
            return false;
        }
        Vertices.Add(Position); Normals.Add(Normal);
        const double Weight = HeightAmplitudeMeters > 0 ? FMath::Clamp(0.5 + Sample.ElevationMeters / (2 * HeightAmplitudeMeters), 0.0, 1.0) : 0.5;
        // A diagnostic height ramp. These are not ocean or biome classifications.
        if (CandidateGeology)
        {
            // Diagnostic hypsometric colors, not final biome/water materials.
            const double Unscaled = Sample.ElevationMeters * (RadiusMeters + 250000) / RadiusMeters;
            if (Unscaled < 0)
                Colors.Add(FMath::Lerp(FLinearColor(0.012f,0.045f,0.11f), FLinearColor(0.09f,0.34f,0.39f), static_cast<float>(FMath::Clamp((Unscaled + 6000) / 6000, 0.0, 1.0))));
            else
                Colors.Add(FMath::Lerp(FLinearColor(0.16f,0.26f,0.07f), FLinearColor(0.72f,0.62f,0.46f), static_cast<float>(FMath::Clamp(Unscaled / 4500, 0.0, 1.0))));
        }
        else Colors.Add(FMath::Lerp(FLinearColor(0.08f,0.18f,0.24f), FLinearColor(0.72f,0.77f,0.66f), static_cast<float>(Weight)));
        const FVector Axis = FMath::Abs(Normal.Z) < 0.8 ? FVector::UpVector : FVector::ForwardVector;
        Tangents.Emplace(FVector::CrossProduct(Axis, Normal).GetSafeNormal(), false);
    }
    for (const auto& Triangle : Sphere->Triangles)
    {
        // UE's front-face winding is clockwise; the core convention is outward CCW.
        Indices.Add(static_cast<int32>(Triangle[0]));
        Indices.Add(static_cast<int32>(Triangle[2]));
        Indices.Add(static_cast<int32>(Triangle[1]));
    }
    PreviewMesh->CreateMeshSection_LinearColor(0, Vertices, Indices, Normals, UVs, Colors, Tangents, false);
    Field = MoveTemp(CandidateField);
    GeologicalField = std::move(CandidateGeology);
    OceanField = std::move(CandidateOcean);
    ActiveOceanVersion = OceanField ? static_cast<int32>(PlanetCore::OceanConnectivityVersion) : 0;
    OceanComponentsBefore = OceanField ? static_cast<int32>(OceanField->Statistics().ComponentsBefore) : 0;
    OceanComponentsAfter = OceanField ? static_cast<int32>(OceanField->Statistics().ComponentsAfter) : 0;
    OceanCorridorCount = OceanField ? static_cast<int32>(OceanField->Statistics().CorridorCount) : 0;
    ConnectedOceanFractionEstimate = OceanField ? OceanField->Statistics().IndependentOceanFractionEstimate : 0;
    OceanMaximumSampleLoweringMeters = OceanField ? OceanField->Statistics().MaximumSampleLoweringMeters : 0;
    if (!OceanField) OceanPreparationMilliseconds = 0;
    else if (OceanPreparation > 0) OceanPreparationMilliseconds = OceanPreparation;
    ActiveGeologyVersion = GeologicalField ? static_cast<int32>(GeologicalField->Settings().Version) : 0;
    GeologicFeatureCount = GeologicalField ? static_cast<int32>(GeologicalField->Features().size()) : 0;
    InitialOceanFractionEstimate = GeologicalField ? GeologicalField->Statistics().InitialOceanFractionEstimate : 0;
    GeologyCalibrationProbeCount = GeologicalField ? static_cast<int32>(GeologicalField->Statistics().CalibrationProbeCount) : 0;
    GeologyAreaProbeCount = GeologicalField ? static_cast<int32>(GeologicalField->Statistics().AreaProbeCount) : 0;
    GeologyContinentalScale = GeologicalField ? GeologicalField->Statistics().ContinentalScale : 0;
    GeologyDatumShiftMeters = GeologicalField ? GeologicalField->Statistics().DatumShiftMeters : 0;
    GeologyTargetOceanFraction = GeologicalField ? GeologicalField->Statistics().TargetOceanFraction : 0;
    if (!GeologicalField) PreparedDefinitionMilliseconds = 0;
    else if (PreparationMilliseconds > 0) PreparedDefinitionMilliseconds = PreparationMilliseconds;
    LastPreparationMilliseconds = PreparationMilliseconds;
    GeneratedVertexCount = VertexCount;
    GeneratedTriangleCount = static_cast<int32>(Sphere->Triangles.size());
    LastBuildMilliseconds = (FPlatformTime::Seconds() - Start) * 1000.0;
    LastBuildMessage = OceanField ?
        TEXT("Ocean corridors prepared. Continuous final connectivity, drainage, erosion and runtime streaming remain incomplete.") : GeologicalField ?
        TEXT("Geologic base ready. Final ocean reconciliation, drainage, erosion and runtime streaming remain incomplete.") :
        TEXT("Diagnostic sphere ready. This fixture is separate from the geological base.");
    return true;
}

bool APlanetSurfacePreview::EvaluateDirection(FVector Direction, double& ElevationMeters, FVector& SurfaceNormal) const
{
    if (!Field && !GeologicalField) return false;
    const auto Unit = PlanetCore::UnitDirection::FromVector({Direction.X, Direction.Y, Direction.Z});
    if (!Unit) return false;
    if (!GeologicalField)
    {
        ElevationMeters = Field->Evaluate(*Unit).ElevationMeters;
        SurfaceNormal = ToUnreal(Field->SurfaceNormal(*Unit).Vector());
        return true;
    }
    const auto Sample = OceanField ? OceanField->Evaluate(*Unit) : GeologicalField->Evaluate(*Unit).Terrain;
    const double BuiltRadius = GeologicalField->Settings().RadiusMeters;
    const auto Normal = PlanetCore::UnitDirection::FromVector(Unit->Vector() * (BuiltRadius + Sample.ElevationMeters) - Sample.TangentialGradientPerRadian);
    if (!Normal) return false;
    ElevationMeters = Sample.ElevationMeters;
    SurfaceNormal = ToUnreal(Normal->Vector());
    return true;
}
