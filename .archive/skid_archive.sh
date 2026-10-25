#!/bin/bash
# Open, check and seal the encrypted fork archive that travels in the fork as .archive/archive.zip.
set -uo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ZIP="${HERE}/archive.zip"
WORK="${SKID_PX4_ARCHIVE_WORK:-${HOME}/.skid_archive/px4}"
KEY_FILE="${HOME}/.skid_archive_key"

die() { echo "skid_archive: $*" >&2; exit 1; }

password() {
    if [ -n "${SKID_ARCHIVE_KEY:-}" ]; then printf '%s' "${SKID_ARCHIVE_KEY}"; return; fi
    [ -r "${KEY_FILE}" ] || die "no key: set SKID_ARCHIVE_KEY or write the password to ${KEY_FILE} (chmod 600)"
    tr -d '\n' < "${KEY_FILE}"
}

cmd_open() {
    [ -f "${ZIP}" ] || die "no archive at ${ZIP}"
    # Key proven against the zip first: a wrong key must not cost the plaintext copy.
    unzip -tqq -P "$(password)" "${ZIP}" >/dev/null 2>&1 || die "unseal failed (wrong key?)"
    rm -rf "${WORK}"; mkdir -p "${WORK}"
    unzip -oq -P "$(password)" "${ZIP}" -d "${WORK}" || die "unseal failed"
    echo "skid_archive: opened -> ${WORK}"
}

cmd_check() {
    [ -f "${WORK}/check.py" ] || die "no working copy; run 'skid_archive.sh open' first"
    python3 -I "${WORK}/check.py"
}

cmd_seal() {
    [ -f "${WORK}/check.py" ] || die "no working copy to seal"
    find "${WORK}" -name '__pycache__' -type d -prune -exec rm -rf {} + 2>/dev/null
    find "${WORK}" \( -name '*.pyc' -o -name '*.part' -o -name '*.tmp' -o -name '.DS_Store' \) -delete 2>/dev/null
    python3 -I "${WORK}/check.py" >/dev/null || die "check failed; refusing to seal a broken archive"
    # Staged outside the repo so .archive/ never carries a stray; same filesystem keeps the rename atomic.
    local tmp="${WORK%/*}/px4_archive.zip.new.$$"
    ( cd "${WORK}" && zip -rq -P "$(password)" "${tmp}" . ) || { rm -f "${tmp}"; die "reseal failed"; }
    local vdir; vdir="$(mktemp -d)"
    unzip -oq -P "$(password)" "${tmp}" -d "${vdir}" && python3 -I "${vdir}/check.py" >/dev/null \
        || { rm -rf "${vdir}" "${tmp}"; die "sealed archive did not verify; original left intact"; }
    rm -rf "${vdir}"
    mv -f "${tmp}" "${ZIP}"
    echo "skid_archive: sealed $(cd "${WORK}" && find . -type f | wc -l) files -> ${ZIP}"
}

case "${1:-}" in
    open)  cmd_open ;;
    check) cmd_check ;;
    seal)  cmd_seal ;;
    *) echo "usage: skid_archive.sh {open|check|seal}" >&2; exit 2 ;;
esac
