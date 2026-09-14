using UnrealBuildTool;

public class OrbitalVenturesTarget : TargetRules
{
	public OrbitalVenturesTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Game;
		DefaultBuildSettings = BuildSettingsVersion.Latest;
		IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
		ExtraModuleNames.Add("OrbitalVentures");
	}
}
