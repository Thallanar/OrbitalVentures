# Orbital Ventures (nome provisório)

Jogo de gestão de empresa espacial com construção de base estilo RTS, projeto de foguetes e simulação orbital fiel em escala 1:1. Motor: Unreal Engine 5, lógica em C++.

- **Documento de design completo (GDD + arquitetura): [`docs/GDD/index.html`](docs/GDD/index.html)** &mdash; abra no navegador.

## Estrutura

```
OrbitalVentures.uproject
Config/                  configuração do projeto UE
Source/
  OrbitalVentures/       módulo principal do jogo
  OrbitalCore/           mecânica orbital em C++ puro (compila também fora do motor)
  CelestialBodies/       corpos celestes, efemérides, renderização de planetas
  VehicleSim/            simulação de veículo em estágios, guidance, falhas
  RocketDesign/          editor de foguetes
  BaseBuilding/          construção de base (RTS)
  Company/               finanças, contratos, P&D, concorrentes IA
  Missions/              objetivos de missão e campanha
  GameUI/                interface (UMG/CommonUI)
  SaveSystem/            serialização do estado da simulação
Tools/SimTests/          simulação sem o UE: testes e lançamento em linha de comando
docs/GDD/index.html      documento de design (GDD + arquitetura técnica)
```

## Simulação sem o Unreal: testes e lançamento em linha de comando

O `OrbitalCore` e o `VehicleSim` são C++ puro e compilam fora do motor.

```bash
cmake -S Tools/SimTests -B Tools/SimTests/build -DCMAKE_BUILD_TYPE=Release
cmake --build Tools/SimTests/build
./Tools/SimTests/build/SimTests      # testes automatizados
./Tools/SimTests/build/LaunchSim     # assiste a um lançamento do Kestrel 9 de Alcântara pela telemetria
```

Exemplos do `LaunchSim`:

```bash
./Tools/SimTests/build/LaunchSim --inclination 51.6              # órbita da ISS
./Tools/SimTests/build/LaunchSim --engine-out 60:1:1             # perde 1 motor do 1º estágio em T+60 s
./Tools/SimTests/build/LaunchSim --payload 25                    # perto do limite de carga
./Tools/SimTests/build/LaunchSim --site cabo                      # mesmo voo a partir do Cabo Canaveral
./Tools/SimTests/build/LaunchSim --pitchover 300 --aggressiveness 1   # giro cedo e agressivo demais
./Tools/SimTests/build/LaunchSim --help
```

## Unreal Engine 5.8 no Linux

Usamos a versão pré-compilada para Linux (https://www.unrealengine.com/linux). Ela já traz o compilador (clang) e o .NET; não é preciso instalar nada no sistema.

> ⚠️ A engine extraída ocupa **~77 GB**, e o cache de shaders cresce dezenas de GB. Por padrão, a Unreal guarda esse cache na pasta do usuário: aponte `UE_DDC_PATH` para um disco com espaço.

1. Extraia o zip num caminho **sem espaços** (alguns scripts de build quebram com espaço no caminho).
2. Copie `Tools/local.env.example` para `Tools/local.env` e ajuste `UE_ROOT` e `UE_DDC_PATH`.
3. Use o script do projeto:
   ```bash
   Tools/ue.sh build          # compila o editor com os módulos do projeto
   Tools/ue.sh editor         # abre o editor, com o cache em UE_DDC_PATH
   Tools/ue.sh projectfiles   # gera os arquivos de projeto do VS Code
   ```

O `.uproject` aponta para a UE **5.8**.

## Git

Assets binários (`.uasset`, `.umap`, texturas, áudio) vão via **Git LFS** (`.gitattributes`). Rode `git lfs install` uma vez na máquina.
