using UnrealBuildTool;

// Núcleo de mecânica orbital em C++ puro (sem UObject). Os arquivos em Private/Orbital
// também compilam fora do motor: veja Tools/SimTests.
public class OrbitalCore : ModuleRules
{
	public OrbitalCore(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		PublicDependencyModuleNames.Add("Core");
	}
}
