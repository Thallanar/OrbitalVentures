#!/usr/bin/env bash
# Atalhos da Unreal para este projeto. Os caminhos da máquina vêm de Tools/local.env.
#   Tools/ue.sh build          compila o editor com os módulos do projeto
#   Tools/ue.sh editor         abre o editor, com o cache no disco definido em UE_DDC_PATH
#   Tools/ue.sh projectfiles   gera os arquivos de projeto para o VS Code
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT_FILE="$PROJECT_DIR/OrbitalVentures.uproject"
LOCAL_ENV="$PROJECT_DIR/Tools/local.env"

if [[ ! -f "$LOCAL_ENV" ]]; then
	echo "Falta $LOCAL_ENV. Copie Tools/local.env.example e ajuste os caminhos." >&2
	exit 1
fi
# shellcheck source=/dev/null
source "$LOCAL_ENV"
: "${UE_ROOT:?defina UE_ROOT em Tools/local.env}"
: "${UE_DDC_PATH:?defina UE_DDC_PATH em Tools/local.env}"

if [[ ! -x "$UE_ROOT/Engine/Binaries/Linux/UnrealEditor" ]]; then
	echo "UnrealEditor não encontrado em $UE_ROOT. Confira UE_ROOT em Tools/local.env." >&2
	exit 1
fi

COMMAND="${1:-}"
shift || true

case "$COMMAND" in
	build)
		exec "$UE_ROOT/Engine/Build/BatchFiles/Linux/Build.sh" OrbitalVenturesEditor Linux Development \
			-Project="$PROJECT_FILE" -WaitMutex "$@"
		;;
	editor)
		mkdir -p "$UE_DDC_PATH"
		# -LocalDataCachePath vale para o cache em arquivos e para o servidor Zen: sem ele, os dois vão
		# para a pasta do usuário no disco do sistema.
		exec "$UE_ROOT/Engine/Binaries/Linux/UnrealEditor" "$PROJECT_FILE" -LocalDataCachePath="$UE_DDC_PATH" "$@"
		;;
	projectfiles)
		exec "$UE_ROOT/Engine/Build/BatchFiles/Linux/GenerateProjectFiles.sh" -Project="$PROJECT_FILE" -Game -VSCode "$@"
		;;
	*)
		echo "Uso: Tools/ue.sh {build|editor|projectfiles} [argumentos extras]" >&2
		exit 1
		;;
esac
