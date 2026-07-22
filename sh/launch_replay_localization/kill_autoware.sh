#!/usr/bin/env bash
# Delegate to the shared kill script (allowlist + bag加工/webauto exclude).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$SCRIPT_DIR/../kill_autoware.sh" "$@"
