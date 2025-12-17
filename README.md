# sintesis-bird
# 🐦 Bird Sound Synthesis & Analysis
### Síntesis, procesamiento y análisis espectral de cantos de aves en Python

Este repositorio presenta un sistema modular para la **síntesis de sonido inspirada en cantos de aves (bird / chirp synthesis)**, acompañado de **procesamiento digital de señales (DSP)**, análisis espectral y control paramétrico mediante **MIDI**.

El proyecto está orientado a la experimentación en **audio digital, síntesis sonora y modelado de señales no estacionarias**, y sirve como base conceptual para futuras implementaciones en **FPGA o sistemas embebidos DSP**.

---

## 🎯 Objetivo del proyecto

Desarrollar un entorno experimental que permita:

- Generar sonidos tipo *bird / chirp* mediante síntesis matemática
- Aplicar filtrado digital y post-procesamiento
- Analizar el contenido espectral de las señales generadas
- Controlar parámetros del sintetizador mediante MIDI
- Sentar las bases para una futura migración a hardware (FPGA / DSP)

---

## 🧱 Arquitectura general del sistema

El sistema sigue una **arquitectura modular y secuencial**, donde cada componente cumple una función clara dentro del flujo de audio:
## Síntesis → Filtrado → Análisis → Control


Cada módulo puede ejecutarse de forma independiente o integrarse dentro del flujo completo.

---

## 📂 Estructura del repositorio

```text
sintesis-bird/
│
├── bird.py
├── bird_2.py
├── bird_Filter.py
├── midi.py
│
├── *.wav
│
├── spectrogram.png
├── welch_psd.png
│
└── README.md


🧩 Descripción detallada de los módulos
🔹 bird.py — Núcleo de síntesis

Script principal encargado de la síntesis del canto de ave.
Este archivo representa el core del sintetizador.

Funciones principales:

Generación de señal base (senoidal / FM / chirp)

Control de frecuencia portadora (fc)

Modulación en frecuencia (fm, índice de modulación)

Aplicación de envolvente temporal (ataque, decaimiento)

Exportación del audio generado a formato .wav

📌 Es el punto de partida del sistema de síntesis.

🔹 bird_2.py — Variantes y experimentación

Versión alternativa del generador de sonido, utilizada como entorno de pruebas y exploración.

Permite:

Probar modelos de síntesis distintos

Ajustar parámetros no lineales

Comparar resultados auditivos y espectrales

Experimentar con nuevas ecuaciones o envolventes

📌 Funciona como sandbox de investigación sonora.

🔹 bird_Filter.py — Procesamiento digital (DSP)

Módulo dedicado al post-procesamiento del audio generado.

Incluye:

Lectura de archivos .wav

Diseño y aplicación de filtros digitales (pasabajo, pasabanda, etc.)

Eliminación de componentes no deseadas

Preparación del audio para análisis o escucha final

📌 Representa la etapa de DSP clásico del sistema.

🔹 midi.py — Control MIDI

Implementa una interfaz de entrada MIDI para control paramétrico del sintetizador.

Permite:

Mapear perillas o sliders MIDI a parámetros del sintetizador

Controlar frecuencia, modulación o amplitud

Preparar el sistema para ejecución en tiempo real

Simular interacción hardware-software

📌 Pensado como puente hacia control físico o implementación embebida.

📊 Análisis y visualización

El proyecto incluye herramientas de análisis para validar el comportamiento de las señales generadas:

🔸 Espectrograma (spectrogram.png)

Análisis tiempo-frecuencia

Visualización de chirps y modulaciones

🔸 Densidad espectral de potencia – PSD (welch_psd.png)

Estimación energética por bandas

Verificación del contenido armónico

📌 Estas visualizaciones permiten comparar el diseño teórico con el resultado real.

🔊 Archivos de audio (.wav)

Los archivos .wav incluidos corresponden a:

Señales sintetizadas

Señales filtradas

Versiones de prueba para evaluación auditiva

📌 Útiles tanto para validación subjetiva (escucha) como objetiva (DSP).

⚙️ Requisitos del sistema

Python 3.9 o superior

Dependencias principales
pip install numpy scipy matplotlib soundfile

Soporte MIDI
pip install mido python-rtmidi

🚀 Ejecución básica

Síntesis principal:

python bird.py


Procesamiento y filtrado:

python bird_Filter.py


Control MIDI:

python midi.py

🔬 Proyección y trabajo futuro

Migración del núcleo de síntesis a FPGA (Verilog / VHDL)

Implementación en DSP embebido

Desarrollo de una interfaz gráfica de control

Creación de un banco de presets de cantos reales

Exportación y control en tiempo real (streaming de audio)

👤 Autor

Santiago Alexander Zambrano
Ingeniería Electrónica

Áreas de interés:
Síntesis de audio · DSP · FPGA · Audio digital

📄 Licencia

Este proyecto puede adaptarse a licencia MIT o GPL, según necesidades académicas o comerciales.

🔥 Siguiente nivel (opcional)

Este proyecto puede evolucionar hacia:

README formal de proyecto de grado

Enfoque completo en FPGA / audio digital

Inclusión de ecuaciones matemáticas de la síntesis

Preparación para empresas de audio profesional (ej. Antelope Audio)


---

Si quieres, en el próximo paso puedo:
- Ajustarlo **exactamente al formato UNAL / trabajo de grado**
- Agregar **ecuaciones matemáticas** de la síntesis
- Convertir esto en un **paper técnico**
- Prepararlo como **repositorio demostrable para empresas de audio**

Dime **para qué lo vas a usar** y lo afinamos al 100%. 🚀






