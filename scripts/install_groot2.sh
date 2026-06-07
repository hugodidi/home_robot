#!/bin/bash
set -e

GROOT2_VERSION="${GROOT2_VERSION:-1.9.0}"
GROOT2_URL="${GROOT2_URL:-https://pub-32cef6782a9e411e82222dee82af193e.r2.dev/Groot2-v${GROOT2_VERSION}-x86_64.AppImage}"
INSTALL_DIR="${GROOT2_INSTALL_DIR:-/opt/groot2}"
APPIMAGE="${INSTALL_DIR}/Groot2.AppImage"
WRAPPER="/usr/local/bin/Groot2"

if [ "$(id -u)" -ne 0 ]; then
    echo "Run as root: sudo ./scripts/install_groot2.sh"
    exit 1
fi

mkdir -p "${INSTALL_DIR}"

echo "Downloading Groot2 ${GROOT2_VERSION}..."
curl -L "${GROOT2_URL}" -o "${APPIMAGE}"
chmod +x "${APPIMAGE}"

cat > "${WRAPPER}" <<'EOF'
#!/bin/bash
set -e

APPIMAGE="/opt/groot2/Groot2.AppImage"
EXTRACTED="/opt/groot2/squashfs-root/AppRun"

if [ -x "${APPIMAGE}" ]; then
    if "${APPIMAGE}" "$@"; then
        exit 0
    fi
fi

# Some containers cannot run AppImages through FUSE. Extract once and run AppRun.
if [ ! -x "${EXTRACTED}" ]; then
    cd /opt/groot2
    "${APPIMAGE}" --appimage-extract >/dev/null
fi

exec "${EXTRACTED}" "$@"
EOF

chmod +x "${WRAPPER}"
ln -sf "${WRAPPER}" /usr/local/bin/groot2

echo "Groot2 installed: ${WRAPPER}"
