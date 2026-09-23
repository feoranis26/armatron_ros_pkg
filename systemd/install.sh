#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
  echo "Run with sudo: sudo ./systemd/install.sh <pi|x86>" >&2
  exit 1
fi

if [[ $# -ne 1 || ( $1 != pi && $1 != x86 ) ]]; then
  echo "Usage: $0 <pi|x86>" >&2
  exit 2
fi

unit_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/$1" && pwd)
for unit in "${unit_dir}"/*; do
  target="/etc/systemd/system/$(basename "${unit}")"
  if [[ -e ${target} && ! -L ${target} ]]; then
    echo "Refusing to replace non-symlink unit: ${target}" >&2
    exit 1
  fi
  ln -sfn "${unit}" "${target}"
  echo "Linked ${target} -> ${unit}"
done

systemctl daemon-reload
