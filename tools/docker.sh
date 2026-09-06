#!/usr/bin/env bash
# Use the upstream Garage Dockerfile unchanged; no project Dockerfile or Compose.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mode="${1:-help}"
image="${GARAGE_IMAGE:-leaderboard-user:latest}"

case "$mode" in
  help|--help|-h)
    echo "Usage: bash tools/docker.sh build|run"
    echo "Required: CARLA_ROOT, CARLA_GARAGE_ROOT. run also requires TFPP_MODEL."
    echo "Optional: GARAGE_IMAGE (default leaderboard-user:latest), EXPERIMENT_OUTPUT_ROOT."
    exit 0
    ;;
  build|run) ;;
  *) echo "Unknown command: $mode" >&2; exit 2 ;;
esac

: "${CARLA_ROOT:?Set CARLA_ROOT to the host CARLA directory}"
: "${CARLA_GARAGE_ROOT:?Set CARLA_GARAGE_ROOT to the host carla_garage directory}"
for path in "$CARLA_ROOT/PythonAPI/carla" "$CARLA_GARAGE_ROOT/team_code"; do
  [[ -d "$path" ]] || { echo "Missing directory: $path" >&2; exit 2; }
done
carla_root="$(realpath "$CARLA_ROOT")"
garage_root="$(realpath "$CARLA_GARAGE_ROOT")"

if [[ "$mode" == build ]]; then
  [[ -f "$garage_root/tools/Dockerfile.master" ]] || {
    echo "Missing Garage tools/Dockerfile.master" >&2; exit 2;
  }
  [[ -d "$garage_root/leaderboard" && -d "$garage_root/scenario_runner" ]] || {
    echo "Missing Garage leaderboard or scenario_runner" >&2; exit 2;
  }
  # A private temporary context prevents copying third-party sources into this repo.
  build_context="$(mktemp -d -t ksae-garage-build.XXXXXXXX)"
  trap 'rm -rf -- "$build_context"' EXIT
  tar -C "$carla_root" --exclude='__pycache__' -cf - PythonAPI |
    tar -C "$build_context" -xf -
  tar -C "$garage_root" \
    --exclude='.git' --exclude='__pycache__' --exclude='*.pth' --exclude='*.pt' \
    --exclude='*.ckpt' --exclude='model_ckpt' --exclude='checkpoints' \
    -cf - scenario_runner leaderboard team_code | tar -C "$build_context" -xf -
  # The Dockerfile and all dependency recipes are owned by CARLA Garage.
  docker build -t "$image" -f "$garage_root/tools/Dockerfile.master" "$build_context"
  exit 0
fi

: "${TFPP_MODEL:?Set TFPP_MODEL to a host model directory containing config.json and one .pth}"
[[ -f "$TFPP_MODEL/config.json" ]] || { echo "Missing model config.json" >&2; exit 2; }
model_dir="$(realpath "$TFPP_MODEL")"
output_root="${EXPERIMENT_OUTPUT_ROOT:-$repo_root/outputs}"
mkdir -p "$output_root"
output_root="$(realpath "$output_root")"
docker image inspect "$image" >/dev/null || {
  echo "Build the Garage image with: bash tools/docker.sh build" >&2
  echo "Or set GARAGE_IMAGE to an existing image built from the same Garage checkout." >&2
  exit 2
}

exec docker run --rm -it \
  --gpus all --network host --shm-size 8g \
  --user "$(id -u):$(id -g)" \
  --mount "type=bind,source=$repo_root,target=/workspace/KSAE_2026_Autumn_ws" \
  --mount "type=bind,source=$garage_root,target=/external/carla_garage,readonly" \
  --mount "type=bind,source=$carla_root/PythonAPI,target=/external/CARLA/PythonAPI,readonly" \
  --mount "type=bind,source=$model_dir,target=/external/model,readonly" \
  --mount "type=bind,source=$output_root,target=/outputs" \
  --env CARLA_ROOT=/external/CARLA \
  --env CARLA_GARAGE_ROOT=/external/carla_garage \
  --env TFPP_MODEL=/external/model \
  --env EXPERIMENT_OUTPUT_ROOT=/outputs \
  --env PYTHONDONTWRITEBYTECODE=1 \
  --workdir /workspace/KSAE_2026_Autumn_ws \
  --entrypoint /bin/bash "$image" --noprofile --norc
