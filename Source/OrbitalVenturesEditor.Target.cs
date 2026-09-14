using UnrealBuildTool;

public class OrbitalVenturesEditorTarget : TargetRules
{
	public OrbitalVenturesEditorTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Editor;
		DefaultBuildSettings = BuildSettingsVersion.Latest;
		IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
		ExtraModuleNames.Add("OrbitalVentures");
	}
}
