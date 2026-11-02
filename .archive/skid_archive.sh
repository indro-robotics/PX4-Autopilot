#!/bin/bash
# Open, check and seal the encrypted fork archive that travels in the fork as .archive/archive.zip.
set -uo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ZIP="${ROOT}/.archive/archive.zip"
WORK="${SKID_PX4_ARCHIVE_WORK:-${ROOT}/.archive/archive}"
MANIFEST="${WORK}.manifest"
LEGACY="${HOME}/.skid_archive/px4"
HOOK_MARK="# skid_archive pre-push guard"
USAGE="usage: skid_archive {open [--force]|status|check|seal [--keep]|hooks}"

die() { echo "skid_archive: $*" >&2; exit 1; }
usage() { echo "${USAGE}" >&2; exit 2; }

rel() { case "$1" in "${ROOT}"/*) printf '%s' "${1#"${ROOT}"/}" ;; *) printf '%s' "$1" ;; esac; }

plural() { [ "$1" -eq 1 ] && printf '1 %s' "$2" || printf '%s %ss' "$1" "$2"; }

# The key is read inside Python from SKID_ARCHIVE_KEY, never from a file, so it reaches no process argument.
core() {
    python3 -I - "$@" <<'PY'
import hashlib, os, stat, sys, tempfile, time, zipfile, zlib

def skipped(rel):
    parts = rel.split('/')
    return '__pycache__' in parts or rel.endswith('.pyc')

def key():
    k = os.environb.get(b'SKID_ARCHIVE_KEY')
    if k:
        return k
    sys.exit('skid_archive: no key: set SKID_ARCHIVE_KEY for this one command; the key is never written to a file')

def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()

def files(root):
    out = []
    for d, dirs, names in os.walk(root):
        dirs[:] = sorted(x for x in dirs if x != '__pycache__')
        for n in names:
            p = os.path.join(d, n)
            rel = os.path.relpath(p, root).replace(os.sep, '/')
            if not skipped(rel) and os.path.isfile(p):
                out.append(rel)
    return sorted(out)

def tree(root):
    t = {}
    for rel in files(root):
        with open(os.path.join(root, rel), 'rb') as f:
            t[rel] = '%08x' % zlib.crc32(f.read())
    return t

def baseline(manifest, zpath):
    # No zip: nothing the work dir holds is sealed. No manifest: the zip's own listing is the reference.
    if not os.path.isfile(zpath):
        return {}
    if os.path.isfile(manifest):
        t = {}
        with open(manifest) as f:
            for line in f:
                crc, _, rel = line.rstrip('\n').partition(' ')
                if crc != 'zip':
                    t[rel] = crc
        return t
    with zipfile.ZipFile(zpath) as z:
        return {i.filename: '%08x' % i.CRC for i in z.infolist()
                if not i.is_dir() and not skipped(i.filename)}

def cmd_diff(work, manifest, zpath):
    base, cur = baseline(manifest, zpath), tree(work)
    for rel in sorted(set(base) | set(cur)):
        if rel not in cur:
            print('missing  ' + rel)
        elif rel not in base:
            print('added    ' + rel)
        elif base[rel] != cur[rel]:
            print('changed  ' + rel)

def cmd_manifest(root, zpath, out):
    lines = ['zip ' + sha256(zpath)] + ['%s %s' % (c, r) for r, c in sorted(tree(root).items())]
    tmp = out + '.tmp'
    with open(tmp, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    os.replace(tmp, out)

def cmd_unseal(zpath, dest):
    pwd = key()
    try:
        with zipfile.ZipFile(zpath) as z:
            for i in z.infolist():
                name = i.filename
                if name.startswith('/') or '..' in name.split('/'):
                    sys.exit('skid_archive: unseal failed - unsafe entry %s' % name)
                target = os.path.join(dest, name)
                if i.is_dir():
                    os.makedirs(target, exist_ok=True)
                    continue
                os.makedirs(os.path.dirname(target), exist_ok=True)
                data = z.read(i, pwd=pwd)
                with open(target, 'wb') as f:
                    f.write(data)
                mode = (i.external_attr >> 16) & 0o777
                if mode:
                    os.chmod(target, mode)
                t = time.mktime(i.date_time + (0, 0, -1))
                os.utime(target, (t, t))
    except (RuntimeError, zipfile.BadZipFile, zlib.error, OSError, ValueError):
        sys.exit('skid_archive: unseal failed - wrong key or damaged zip')

CRCTAB = []
for n in range(256):
    c = n
    for _ in range(8):
        c = (c >> 1) ^ 0xEDB88320 if c & 1 else c >> 1
    CRCTAB.append(c)

def encrypt(pwd, data):
    # Traditional PKWARE encryption, the scheme zip -P writes and unzip -P reads.
    k0, k1, k2 = 0x12345678, 0x23456789, 0x34567890
    tab = CRCTAB
    def upd(b, k0, k1, k2):
        k0 = (k0 >> 8) ^ tab[(k0 ^ b) & 0xff]
        k1 = ((k1 + (k0 & 0xff)) * 134775813 + 1) & 0xffffffff
        k2 = (k2 >> 8) ^ tab[(k2 ^ (k1 >> 24)) & 0xff]
        return k0, k1, k2
    for b in pwd:
        k0, k1, k2 = upd(b, k0, k1, k2)
    out = bytearray(len(data))
    for i, p in enumerate(data):
        t = (k2 | 2) & 0xffff
        out[i] = p ^ (((t * (t ^ 1)) >> 8) & 0xff)
        k0 = (k0 >> 8) ^ tab[(k0 ^ p) & 0xff]
        k1 = ((k1 + (k0 & 0xff)) * 134775813 + 1) & 0xffffffff
        k2 = (k2 >> 8) ^ tab[(k2 ^ (k1 >> 24)) & 0xff]
    return bytes(out)

def dostime(mtime):
    y, mo, d, h, mi, s = time.localtime(mtime)[:6]
    if y < 1980:
        y, mo, d, h, mi, s = 1980, 1, 1, 0, 0, 0
    return (h << 11) | (mi << 5) | (s // 2), ((y - 1980) << 9) | (mo << 5) | d

def cmd_pack(root, out):
    import struct
    pwd = key()
    entries = []
    dirs = set()
    for rel in files(root):
        parts = rel.split('/')[:-1]
        for n in range(1, len(parts) + 1):
            dirs.add('/'.join(parts[:n]) + '/')
    for d, sub, _ in os.walk(root):
        sub[:] = [x for x in sub if x != '__pycache__']
        rel = os.path.relpath(d, root).replace(os.sep, '/')
        if rel != '.':
            dirs.add(rel + '/')
    central = bytearray()
    with open(out, 'wb') as f:
        for name in sorted(dirs) + files(root):
            p = os.path.join(root, name.rstrip('/'))
            st = os.stat(p)
            t, d = dostime(st.st_mtime)
            nb = name.encode('utf-8')
            flag = 0x800 if any(c > 127 for c in nb) else 0
            if name.endswith('/'):
                crc, method, payload, usize = 0, 0, b'', 0
                attr = (stat.S_IFDIR | stat.S_IMODE(st.st_mode)) << 16 | 0x10
            else:
                with open(p, 'rb') as g:
                    data = g.read()
                crc = zlib.crc32(data)
                c = zlib.compressobj(9, zlib.DEFLATED, -15)
                comp = c.compress(data) + c.flush()
                payload = encrypt(pwd, os.urandom(11) + bytes([crc >> 24]) + comp)
                method, usize, flag = 8, len(data), flag | 1
                attr = (stat.S_IFREG | stat.S_IMODE(st.st_mode)) << 16
            off = f.tell()
            f.write(struct.pack('<4sHHHHHIIIHH', b'PK\x03\x04', 20, flag, method, t, d,
                                crc, len(payload), usize, len(nb), 0) + nb + payload)
            central += struct.pack('<4sHHHHHHIIIHHHHHII', b'PK\x01\x02', 0x0314, 20, flag, method,
                                   t, d, crc, len(payload), usize, len(nb), 0, 0, 0, 0, attr, off) + nb
            entries.append(name)
        cd = f.tell()
        f.write(central)
        f.write(struct.pack('<4sHHHHIIH', b'PK\x05\x06', 0, 0, len(entries), len(entries),
                            len(central), cd, 0))

cmd = sys.argv[1]
{'diff': cmd_diff, 'manifest': cmd_manifest, 'unseal': cmd_unseal, 'pack': cmd_pack}[cmd](*sys.argv[2:])
PY
}

# Work dir against the record of the last open or seal; one tagged line per added, changed or missing file.
work_diff() { [ -d "${WORK}" ] || return 0; core diff "${WORK}" "${MANIFEST}" "${ZIP}"; }

zip_files() {
    [ -f "${ZIP}" ] || { echo 0; return; }
    unzip -Z1 "${ZIP}" 2>/dev/null | grep -vc '/$'
}

zip_changed() {
    [ -f "${MANIFEST}" ] && [ -f "${ZIP}" ] || return 1
    [ "$(head -n1 "${MANIFEST}")" != "zip $(sha256sum < "${ZIP}" | cut -d' ' -f1)" ]
}

# The lock sits on the zip's directory, so no lock file is left behind.
take_lock() {
    local d; d="$(dirname "${ZIP}")"
    exec 9<"${d}" || die "lock failed on $(rel "${d}")"
    flock -w 60 9 || die "refused - another skid_archive open or seal holds the lock on $(rel "${d}"). Retry when it ends."
}

legacy_notice() {
    [ -d "${LEGACY}" ] && echo "skid_archive: legacy ${LEGACY} present, outside every seal; move its edits into $(rel "${WORK}") and remove it"
    return 0
}

cmd_status() {
    [ $# -eq 0 ] || usage
    local diff n=0; diff="$(work_diff)"; [ -n "${diff}" ] && n="$(grep -c . <<<"${diff}")"
    if [ -d "${WORK}" ]; then
        printf '%-12s %s present\n' "work dir" "$(rel "${WORK}")"
        printf '%-12s %s, expected 0\n' "unsealed" "$(plural "${n}" file)"
        [ -n "${diff}" ] && sed 's/^/  /' <<<"${diff}"
        if [ ! -f "${ZIP}" ]; then printf '%-12s %s\n' "zip" "absent"
        elif [ ! -f "${MANIFEST}" ]; then printf '%-12s %s\n' "zip" "no open record, compared against the zip"
        elif zip_changed; then printf '%-12s %s\n' "zip" "changed since the open"
        else printf '%-12s %s\n' "zip" "unchanged since the open"; fi
    else
        printf '%-12s %s absent\n' "work dir" "$(rel "${WORK}")"
        [ -f "${ZIP}" ] || printf '%-12s %s\n' "zip" "absent"
    fi
    printf '%-12s %s\n' "zip entries" "$(plural "$(zip_files)" file)"
    [ -d "${LEGACY}" ] && printf '%-12s %s\n' "legacy copy" "${LEGACY} present, outside every seal"
    [ "${n}" -eq 0 ]
}

cmd_open() {
    local force=0
    case "$#:${1:-}" in 0:) ;; 1:--force) force=1 ;; *) usage ;; esac
    [ -f "${ZIP}" ] || die "no archive at $(rel "${ZIP}")"
    take_lock
    local diff n=0; diff="$(work_diff)"; [ -n "${diff}" ] && n="$(grep -c . <<<"${diff}")"
    if [ "${n}" -gt 0 ] && [ "${force}" -eq 0 ]; then
        die "refused - $(rel "${WORK}") holds unsealed edits. Run skid_archive seal, or skid_archive open --force to discard them."
    fi
    install_git_hook || true
    # Unsealed into a staging dir first: a wrong key must not cost the plaintext copy.
    local stage; stage="$(mktemp -d "${WORK}.open.XXXXXX")" || die "unseal failed"
    core unseal "${ZIP}" "${stage}" || { rm -rf "${stage}"; exit 1; }
    [ "${n}" -gt 0 ] && echo "skid_archive: discarded $(plural "${n}" "unsealed file") in $(rel "${WORK}")"
    rm -rf "${WORK}" && mv "${stage}" "${WORK}" || die "unseal failed - $(rel "${stage}") holds the unsealed copy"
    core manifest "${WORK}" "${ZIP}" "${MANIFEST}" || die "manifest write failed"
    echo "skid_archive: opened -> $(rel "${WORK}")"
    legacy_notice
}

# A pre-push hook already in place is kept as pre-push.chained and runs after the guard.
# Returns 1 outside a git checkout.
install_git_hook() {
    local dir; dir="$(git -C "${ROOT}" rev-parse --path-format=absolute --git-path hooks 2>/dev/null)" || return 1
    local hook="${dir}/pre-push"
    mkdir -p "${dir}"
    if [ -f "${hook}" ] && ! grep -qF "${HOOK_MARK}" "${hook}"; then
        if [ -e "${hook}.chained" ] && ! cmp -s "${hook}" "${hook}.chained"; then
            die "refused - $(rel "${hook}") and $(rel "${hook}.chained") are two different hooks. Merge pre-push into pre-push.chained, then run skid_archive hooks."
        fi
        mv "${hook}" "${hook}.chained" || die "pre-push hook install failed"
    fi
    cat > "${hook}" <<HOOK || die "pre-push hook install failed"
#!/bin/sh
${HOOK_MARK}
top="\$(git rev-parse --show-toplevel)" || exit 1
work="\${SKID_PX4_ARCHIVE_WORK:-\${top}/.archive/archive}"
if [ -e "\${work}" ]; then
    echo "push refused - \${work#"\${top}"/} is open. Run skid_archive seal first." >&2
    exit 1
fi
[ -x "\$0.chained" ] && exec "\$0.chained" "\$@"
exit 0
HOOK
    chmod 755 "${hook}"
}

cmd_hooks() {
    [ $# -eq 0 ] || usage
    install_git_hook || die "refused - $(rel "${ROOT}") is not a git checkout"
    echo "skid_archive: pre-push hook installed"
}

cmd_check() {
    [ $# -eq 0 ] || usage
    [ -f "${WORK}/check.py" ] || die "no work dir at $(rel "${WORK}"). Run skid_archive open first."
    ( cd "${WORK}" && python3 -I check.py )
}

cmd_seal() {
    local keep=0
    case "$#:${1:-}" in 0:) ;; 1:--keep) keep=1 ;; *) usage ;; esac
    [ -f "${WORK}/check.py" ] || die "no work dir at $(rel "${WORK}"). Run skid_archive open first."
    take_lock
    zip_changed && die "refused - $(rel "${ZIP}") changed since the open. Copy the edits aside, run skid_archive open --force, then reapply them."
    find "${WORK}" -name '__pycache__' -type d -prune -exec rm -rf {} + 2>/dev/null
    find "${WORK}" \( -name '*.pyc' -o -name '*.part' -o -name '*.tmp' -o -name '.DS_Store' \) -delete 2>/dev/null
    ( cd "${WORK}" && python3 -I check.py >/dev/null ) \
        || die "refused - check.py reports failures. Run skid_archive check and fix them."
    # Same filesystem as the zip keeps the rename atomic; .gitignore covers the staging names.
    local tmp="${ZIP}.new.$$" newman="${MANIFEST}.new.$$"
    core pack "${WORK}" "${tmp}" || { rm -f "${tmp}"; die "reseal failed"; }
    local vdir; vdir="$(mktemp -d)"
    core unseal "${tmp}" "${vdir}" && ( cd "${vdir}" && python3 -I check.py >/dev/null ) \
        && core manifest "${vdir}" "${tmp}" "${newman}" \
        || { rm -rf "${vdir}" "${tmp}" "${newman}"; die "sealed archive did not verify; original left intact"; }
    rm -rf "${vdir}"
    mv -f "${tmp}" "${ZIP}" && mv -f "${newman}" "${MANIFEST}" || die "reseal failed"
    local n; n="$(plural "$(zip_files)" file)"
    # An edit landing while the zip was built is absent from it: the work dir stays.
    local diff m=0; diff="$(work_diff)"; [ -n "${diff}" ] && m="$(grep -c . <<<"${diff}")"
    if [ "${m}" -gt 0 ]; then
        echo "skid_archive: sealed ${n} -> $(rel "${ZIP}"); work dir kept, $(plural "${m}" file) changed during the seal. Run skid_archive seal again." >&2
        exit 1
    fi
    if [ "${keep}" -eq 1 ]; then
        echo "skid_archive: sealed ${n} -> $(rel "${ZIP}"); work dir kept"
    else
        rm -rf "${WORK}" "${MANIFEST}"
        echo "skid_archive: sealed ${n} -> $(rel "${ZIP}"); work dir removed"
    fi
}

cmd="${1:-}"; [ $# -gt 0 ] && shift
case "${cmd}" in
    open)   cmd_open "$@" ;;
    status) cmd_status "$@" ;;
    check)  cmd_check "$@" ;;
    seal)   cmd_seal "$@" ;;
    hooks)  cmd_hooks "$@" ;;
    *) usage ;;
esac
