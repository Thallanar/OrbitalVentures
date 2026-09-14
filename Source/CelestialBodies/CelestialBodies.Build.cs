using UnrealBuildTool;

public class CelestialBodies : ModuleRules
{
	public CelestialBodies(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "OrbitalCore" });
	}
}
