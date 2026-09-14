using UnrealBuildTool;

public class GameUI : ModuleRules
{
	public GameUI(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "UMG", "Slate", "SlateCore" });
	}
}
