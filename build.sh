#!/bin/bash
# Script para compilar e fazer upload do ESP32

# Verificar se o PlatformIO está instalado
if ! command -v platformio &> /dev/null; then
    echo "🔧 Instalando PlatformIO..."
    python3 -m venv /tmp/pio_env
    source /tmp/pio_env/bin/activate
    pip install platformio --quiet
    echo "✅ PlatformIO instalado!"
else
    echo "✅ PlatformIO já está instalado"
fi

# Fazer build e upload
echo "🔨 Compilando firmware..."
cd "$(dirname "$0")"

source /tmp/pio_env/bin/activate 2>/dev/null || true

platformio run --target upload -v

if [ $? -eq 0 ]; then
    echo "✅ Upload concluído com sucesso!"
    echo "📱 Abrindo monitor serial..."
    sleep 2
    platformio device monitor --baud 115200
else
    echo "❌ Erro no upload"
    exit 1
fi
