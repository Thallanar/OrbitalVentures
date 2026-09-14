using UnrealBuildTool;

public class RocketDesign : ModuleRules
{
	public RocketDesign(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "VehicleSim" });
	}
}
