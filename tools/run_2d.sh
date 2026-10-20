#!/usr/bin/env bash
set -eo pipefail
case "${1:-}" in txt|bag) exec bash "$(dirname "${BASH_SOURCE[0]}")/occupancy-slam" "$@";; *) echo "Usage: $0 {txt|bag} [options]"; exit 2;; esac
