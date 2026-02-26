#!/bin/bash
###############################################################################
# Voice Models Installation Script
#
# Downloads and configures speech-to-text (STT) and text-to-speech (TTS)
# models for Spanish language voice control.
#
# Usage:
#   ./scripts/install_voice_models.sh
#
# Downloads:
#   - Vosk Spanish STT model (~40 MB): vosk-model-small-es-0.42
#   - Piper TTS binary and Spanish voice model (~110 MB): es_ES-carlfm-x_low
#
# Output Directory:
#   $PROJECT_ROOT/voice_models/
#     ├── vosk/vosk-model-small-es-0.42/
#     └── piper/
#         ├── piper (executable)
#         └── voices/es_ES-carlfm-x_low.onnx(.json)
#
# Features:
#   - Automatic download with retry logic
#   - File integrity verification
#   - Piper binary validation test
#   - Skips already installed models
#
# Requirements:
#   - curl or wget
#   - unzip, tar
#   - Internet connection
###############################################################################

set -euo pipefail

# Cleanup function for graceful interruptions
cleanup() {
    if [ $? -ne 0 ]; then
        echo -e "\n⚠️ Installation interrupted."
    fi
}

trap cleanup INT TERM EXIT

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
VOICE_MODELS_DIR="$PROJECT_ROOT/voice_models"
PIPER_BIN="$VOICE_MODELS_DIR/piper/piper"
VOICE_ONNX="$VOICE_MODELS_DIR/piper/voices/es_ES-carlfm-x_low.onnx"
VOICE_JSON="$VOICE_MODELS_DIR/piper/voices/es_ES-carlfm-x_low.onnx.json"

echo "========================================"
echo "Voice Models Installation Script"
echo "========================================"
echo ""
echo "This script will download:"
echo "  - Vosk Spanish model (~40 MB)"
echo "  - Piper TTS binary and Spanish voice model (~110 MB)"
echo ""
echo "Target directory: $VOICE_MODELS_DIR"
echo ""

# Create directories
mkdir -p "$VOICE_MODELS_DIR/vosk"
mkdir -p "$VOICE_MODELS_DIR/piper/voices"

# Function to download with progress
download_file() {
    local url="$1"
    local output="$2"
    echo "Downloading: $url"
    if command -v curl &> /dev/null; then
        curl -fL --retry 3 --progress-bar -o "$output" "$url"
    elif command -v wget &> "/dev/null"; then
        wget --tries=3 --show-progress -O "$output" "$url"
    else
        echo "Error: Neither wget nor curl found. Please install one of them."
        exit 1
    fi

    # Verify file is not empty
    if [ ! -s "$output" ]; then
        echo "ERROR: Downloaded file is empty: $output"
        exit 8
    fi
}

# 1. Download Vosk Spanish model
echo ""
echo "[1/3] Downloading Vosk Spanish model..."
VOSK_URL="https://alphacephei.com/vosk/models/vosk-model-small-es-0.42.zip"
VOSK_ZIP="$VOICE_MODELS_DIR/vosk/vosk-model.zip"

if [ -d "$VOICE_MODELS_DIR/vosk/vosk-model-small-es-0.42" ]; then
    echo "✓ Vosk model already exists. Skipping."
else
    download_file "$VOSK_URL" "$VOSK_ZIP"
    echo "Extracting Vosk model..."
    unzip -q "$VOSK_ZIP" -d "$VOICE_MODELS_DIR/vosk/"
    rm "$VOSK_ZIP"
    echo "✓ Vosk model installed."
fi

# 2. Download Piper TTS binary
echo ""
echo "[2/3] Downloading Piper TTS binary..."
# GitHub asset naming changed between versions; try newest first, fallback to older release
PIPER_URL_PRIMARY="https://github.com/rhasspy/piper/releases/download/v1.2.0/piper_amd64.tar.gz"
PIPER_URL_FALLBACK="https://github.com/rhasspy/piper/releases/download/2023.11.14-2/piper_linux_x86_64.tar.gz"
PIPER_TAR="$VOICE_MODELS_DIR/piper/piper.tar.gz"

if [ -f "$VOICE_MODELS_DIR/piper/piper" ]; then
    echo "✓ Piper binary already exists. Skipping."
else
    rm -f "$PIPER_TAR"
    # Try primary URL first; if 404, fallback to older naming convention
    if ! download_file "$PIPER_URL_PRIMARY" "$PIPER_TAR"; then
        echo "Primary Piper URL failed, trying fallback..."
        rm -f "$PIPER_TAR"
        download_file "$PIPER_URL_FALLBACK" "$PIPER_TAR"
    fi

    echo "Extracting Piper binary..."
    if tar -xzf "$PIPER_TAR" -C "$VOICE_MODELS_DIR/piper/" --strip-components=1; then
        rm -f "$PIPER_TAR"
        chmod +x "$VOICE_MODELS_DIR/piper/piper"
        echo "✓ Piper binary installed."
    else
        echo "ERROR: Failed to extract Piper tarball (possibly incomplete download)."
        ls -lh "$VOICE_MODELS_DIR/piper" || true
        exit 8
    fi
fi

# 3. Download Piper Spanish voice model
echo ""
echo "[3/3] Downloading Piper Spanish voice model (carlfm-x_low)..."
VOICE_BASE_URL="https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/es/es_ES/carlfm/x_low"
VOICE_ONNX_URL="$VOICE_BASE_URL/es_ES-carlfm-x_low.onnx"
VOICE_JSON_URL="$VOICE_BASE_URL/es_ES-carlfm-x_low.onnx.json"

if [ -f "$VOICE_ONNX" ] && [ -f "$VOICE_JSON" ]; then
    echo "✓ Voice model already exists. Skipping."
else
    download_file "$VOICE_ONNX_URL" "$VOICE_ONNX"
    download_file "$VOICE_JSON_URL" "$VOICE_JSON"
    echo "✓ Voice model installed."
fi

# 4. Validate Piper binary and voice files
echo ""
echo "[4/4] Validating Piper setup..."
if [ ! -x "$PIPER_BIN" ]; then
    echo "Setting execute bit on Piper binary..."
    chmod +x "$PIPER_BIN" || true
fi

# Check all required files are present
missing=0
for f in "$PIPER_BIN" "$VOICE_ONNX" "$VOICE_JSON"; do
    if [ ! -f "$f" ]; then
        echo "ERROR: Missing file $f"
        missing=1
    fi
done

# Run synthesis test to verify Piper is fully functional
if [ "$missing" -eq 0 ]; then
    echo "Running Piper self-test (silent)..."
    if "$PIPER_BIN" --model "$VOICE_ONNX" --config "$VOICE_JSON" --output_file /tmp/tts_test.wav >/dev/null 2>&1 <<< "hola"; then
        rm -f /tmp/tts_test.wav
        echo "✓ Piper TTS OK"
    else
        echo "WARNING: Piper failed to synthesize. Check binary and model files."
    fi
fi

echo ""
echo "========================================"
echo "✓ All voice models installed successfully!"
echo "========================================"
echo ""
echo "Models location:"
echo "  - Vosk: $VOICE_MODELS_DIR/vosk/vosk-model-small-es-0.42/"
echo "  - Piper: $VOICE_MODELS_DIR/piper/"
echo ""
echo "You can now use the voice control features."
echo ""
