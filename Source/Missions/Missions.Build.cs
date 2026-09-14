using UnrealBuildTool;

public class Missions : ModuleRules
{
	public Missions(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "OrbitalCore", "Company" });
	}
}
