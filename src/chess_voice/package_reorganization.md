# Reorganización del Paquete ROS 2 `chess_voice`

Se ha analizado la estructura de `chess_voice` y se han realizado mejoras en la organización y configuración siguiendo los estándares recomendados para paquetes ROS 2 en Python.

## Estructura Actualizada del Paquete

El paquete contiene los siguientes directorios y archivos:

```text
chess_voice/
├── package.xml             # Declaración de dependencias y metadatos
├── setup.cfg               # Configuración de instalación
├── setup.py                # Script de compilación e instalación
├── config/
│   └── params.yaml         # Archivo central de parámetros (Nuevo)
├── launch/
│   └── voice_pipeline.launch.py # Lanzador modular del pipeline (Nuevo)
└── chess_voice/            # Directorio del módulo Python
    ├── __init__.py
    ├── audio_capture.py    # Captura de audio desde el micrófono
    ├── whisper_asr_node.py # Transcripción usando Whisper
    ├── voice_parser_node.py # Procesamiento y análisis de la jugada
    └── parsers.py          # Clases para parsing (LLM y Regex Fallback)
```

---

## Archivo de Configuración Central: `config/params.yaml`

Se ha creado un archivo centralizado [params.yaml](file:///home/juanpe/Escritorio/Proyecto_Robotica/src/chess_voice/config/params.yaml) para unificar la parametrización de los tres nodos del pipeline:

- **`chess_audio_capture`**: Configuración de muestreo, umbrales de detección de voz y dispositivo de captura.
- **`chess_whisper_asr`**: Parámetros de modelo de lenguaje, idioma y dispositivo de ejecución (CPU/CUDA).
- **`chess_voice_parser`**: Modos de parseo (LLM/Regex), modelos, tamaño de casilla, geometría del tablero y configuración de temperatura.

### Gestión de Tokens API (Hugging Face)
Para evitar tener que exportar variables de entorno manualmente en la terminal (`export HUGGINGFACEHUB_API_TOKEN=...`), ahora se puede configurar directamente en `params.yaml`:

```yaml
chess_voice_parser:
  ros__parameters:
    huggingface_api_token: "tu_token_aqui"
```

El nodo `voice_parser_node.py` detectará automáticamente este parámetro e inyectará la variable de entorno necesaria en tiempo de ejecución.

---

## Modificaciones de Código en Scripts

Se modificó [voice_parser_node.py](file:///home/juanpe/Escritorio/Proyecto_Robotica/src/chess_voice/chess_voice/voice_parser_node.py) para permitir la parametrización de la temperatura del modelo LLM y del token de Hugging Face mediante ROS 2.

### Cambios Aplicados en el Constructor de `VoiceParserNode`

```python
        self.declare_parameter("use_llm",         True)
        self.declare_parameter("llm_model_id",    "meta-llama/Meta-Llama-3-8B-Instruct")
        self.declare_parameter("llm_temperature", 0.0)
        self.declare_parameter("huggingface_api_token", "")
        
        # ...
        
        hf_token = self.get_parameter("huggingface_api_token").get_parameter_value().string_value
        if hf_token:
            import os
            os.environ["HUGGINGFACEHUB_API_TOKEN"] = hf_token
```

---

## Lanzamiento del Entorno Simplificado

Se ha modificado el archivo de lanzamiento general del sistema en [chess_full.launch.py](file:///home/juanpe/Escritorio/Proyecto_Robotica/src/chess_bringup/launch/chess_full.launch.py) para que:
1. Cargue automáticamente por defecto el archivo central de configuración de voz [params.yaml](file:///home/juanpe/Escritorio/Proyecto_Robotica/src/chess_voice/config/params.yaml).
2. Resuelva automáticamente la ruta absoluta al modelo YOLOv8 de estimación del tablero (`best.pt`) utilizando la ruta de share del paquete `chess_perception`, en lugar de rutas relativas con `~` o variables de entorno locales.

### Comando Simplificado
Gracias a esto, ahora se puede iniciar todo el stack del robot de ajedrez simplemente ejecutando:

```bash
ros2 launch chess_bringup chess_full.launch.py
```

Sin necesidad de pasar argumentos extensos por línea de comandos ni exportar variables.
