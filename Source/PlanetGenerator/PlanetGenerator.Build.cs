// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class PlanetGenerator : ModuleRules
{
	public PlanetGenerator(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		CppStandard = CppStandardVersion.Cpp20;

		PublicDependencyModuleNames.AddRange(new string[] {
			"Core",
			"CoreUObject",
			"Engine",
			"InputCore",
			"EnhancedInput",
			"AIModule",
			"StateTreeModule",
			"GameplayStateTreeModule",
			"UMG",
			"Slate",
			"ProceduralMeshComponent"
		});

		PrivateDependencyModuleNames.AddRange(new string[] { "RenderCore", "Chaos", "PhysicsCore", "RealtimeMeshComponent" });

		PublicIncludePaths.AddRange(new string[] {
			"PlanetGenerator",
			"PlanetGenerator/Variant_Horror",
			"PlanetGenerator/Variant_Horror/UI",
			"PlanetGenerator/Variant_Shooter",
			"PlanetGenerator/Variant_Shooter/AI",
			"PlanetGenerator/Variant_Shooter/UI",
			"PlanetGenerator/Variant_Shooter/Weapons"
		});

		// Uncomment if you are using Slate UI
		// PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore" });

		// Uncomment if you are using online features
		// PrivateDependencyModuleNames.Add("OnlineSubsystem");

		// To include OnlineSubsystemSteam, add it to the plugins section in your uproject file with the Enabled attribute set to true
	}
}
