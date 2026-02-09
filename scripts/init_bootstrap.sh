#!/bin/sh
set -e

TARGET_UID=${TARGET_UID:-1000}
TARGET_GID=${TARGET_GID:-1000}

echo "==> init-bootstrap: chowning volumes to ${TARGET_UID}:${TARGET_GID}"

# Fix ownership on all mounted build-artifact volumes
for dir in /ros2_build /ros2_install /ros2_log /platformio_cache /mcu_build_artifacts /mcu_lib_external; do
  if [ -d "$dir" ]; then
    chown -R "${TARGET_UID}:${TARGET_GID}" "$dir"
  fi
done

# Seed libs_external if the target volume is empty
if [ -d /mcu_lib_external ]; then
  if [ -z "$(ls -A /mcu_lib_external 2>/dev/null)" ]; then
    # Prefer the runtime-mounted /seed; fall back to the baked-in copy
    if [ -d /seed ] && [ -n "$(ls -A /seed 2>/dev/null)" ]; then
      echo "==> Seeding mcu_lib_external from mounted /seed"
      cp -a /seed/. /mcu_lib_external/
    elif [ -d /seed_baked ] && [ -n "$(ls -A /seed_baked 2>/dev/null)" ]; then
      echo "==> Seeding mcu_lib_external from baked-in /seed_baked"
      cp -a /seed_baked/. /mcu_lib_external/
    fi
    chown -R "${TARGET_UID}:${TARGET_GID}" /mcu_lib_external
  fi
fi

echo "==> init-bootstrap: done"
