#!/bin/sh
set -euxo pipefail

# 1) Chown volumes once using sentinel files
for d in \
  /ros2_build \
  /ros2_install \
  /ros2_log \
  /platformio_cache \
  /mcu_build_artifacts \
  /mcu_build_artifacts_microros \
  /mcu_lib_external \
; do
  echo "[bootstrap] Ensuring directory exists: $d"
  mkdir -p "$d"
  if [ -f "$d/.sec26_owner_set" ]; then
    echo "[bootstrap] Ownership already set for $d; skipping"
  else
    echo "[bootstrap] Chowning recursively to 1000:1000: $d"
    chown -R 1000:1000 "$d"
    echo "owner=1000:1000" > "$d/.sec26_owner_set"
  fi
done

# 2) Seed libs_external once using sentinel file
DATA=/mcu_lib_external
MOUNT_SEED=/seed
BAKED_SEED=/seed_baked
SEED_SRC=""

echo "[bootstrap] Preparing libs_external seed"
mkdir -p "$DATA"

if [ -f "$DATA/.sec26_libs_seeded" ]; then
  echo "[bootstrap] Sentinel found: $DATA/.sec26_libs_seeded — skipping seed"
  exit 0
fi

if [ -d "$MOUNT_SEED" ] && [ -n "$(ls -A "$MOUNT_SEED" 2>/dev/null)" ]; then
  SEED_SRC="$MOUNT_SEED"
  echo "[bootstrap] Using mounted seed: $SEED_SRC"
elif [ -d "$BAKED_SEED" ] && [ -n "$(ls -A "$BAKED_SEED" 2>/dev/null)" ]; then
  SEED_SRC="$BAKED_SEED"
  echo "[bootstrap] Mounted /seed empty or missing; using baked fallback: $SEED_SRC"
else
  echo "[bootstrap] ERROR: No valid seed found. Checked: $MOUNT_SEED and $BAKED_SEED"
  exit 1
fi

echo "[bootstrap] Copying from $SEED_SRC to $DATA"
cp -a "$SEED_SRC"/. "$DATA"/

echo "[bootstrap] Setting ownership to 1000:1000 on $DATA"
chown -R 1000:1000 "$DATA"

echo "seeded_from=$SEED_SRC" > "$DATA/.sec26_libs_seeded"

echo "[bootstrap] Completed successfully"
