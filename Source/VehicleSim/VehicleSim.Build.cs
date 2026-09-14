using UnrealBuildTool;

public class VehicleSim : ModuleRules
{
	public VehicleSim(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "OrbitalCore" });
	}
}
