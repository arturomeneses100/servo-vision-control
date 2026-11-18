# servo-vision-control

Interfaz gráfica en Python para **controlar un servomotor usando visión artificial**. La cámara detecta un objeto por color (HSV), calcula el error respecto al centro de la imagen y un controlador (On-Off, P, PI o PID) decide el ángulo del servo, que se envía por puerto serial a un microcontrolador PIC.

Proyecto didáctico para la unidad **"Desarrollo de aplicaciones de control de procesos utilizando sistemas embebidos"** (lazo abierto/cerrado, On-Off, P, PI, PID).

> Firmware del PIC: [servo-pic-firmware](https://github.com/TU_USUARIO/servo-pic-firmware)

---

## Arquitectura

```
Cámara ──► OpenCV (Python) ──► Controlador (Python) ──► Serial ──► PIC ──► PWM 50 Hz ──► Servo
           segmentación HSV     On-Off / P / PI / PID     "90\n"
           centroide → error
```

**Variable medida (error):** posición horizontal del objeto respecto al centro de la imagen, normalizada de **−100 %** (borde izquierdo) a **+100 %** (borde derecho).

**Variable manipulada:** ángulo del servo, de 0° a 180°.

## Características

- Video en vivo con caja del objeto, centroide, línea central y un servo virtual que muestra el ángulo.
- Vista de la máscara binaria para depurar la segmentación.
- 6 deslizadores HSV con colores predefinidos (rojo, verde, azul y amarillo). El rojo se maneja aunque su tono pase por 0.
- Filtro por área mínima para ignorar ruido.
- Seis modos de operación (ver tabla).
- Ajuste en vivo de Kp, Ki, Kd, banda muerta y paso del On-Off.
- Gráfica en tiempo real del error y del ángulo (ventana de 10 s).
- Grabación a CSV (`t_s, modo, error_pct, angulo_deg, P, I, D`) para analizar la respuesta.
- **Modo simulación:** funciona sin PIC conectado.

## Modos de control

| Modo | Montaje | Ley de control | Qué se observa |
|---|---|---|---|
| Manual | — | ángulo = deslizador | Calibración pulso ↔ ángulo |
| Lazo abierto | Cámara fija, servo apunta al objeto | ángulo = 180 · x / ancho | Sin realimentación: no corrige perturbaciones |
| On-Off | Cámara sobre el servo | ±paso si \|e\| > banda | Oscilación, efecto de la banda muerta |
| P | Cámara sobre el servo | 90 + Kp·e | Error en estado estable |
| PI | Cámara sobre el servo | 90 + Kp·e + Ki·∫e dt | Elimina el error; sobreimpulso. Anti-windup |
| PID | Cámara sobre el servo | 90 + Kp·e + Ki·∫e dt + Kd·de/dt | Amortiguamiento y sensibilidad al ruido |

El PID usa el `dt` real entre cuadros y satura la integral (anti-windup). Si se pierde el objetivo, el servo mantiene su última posición.

## Instalación

Requiere Python 3.9 o superior.

```bash
git clone <url-del-repo>
cd servo-vision-control
python -m venv .venv
# Windows
.venv\Scripts\activate
# Linux / macOS
source .venv/bin/activate
pip install -r requirements.txt
```

## Uso

```bash
python servo_vision.py
```

1. Elige el número de cámara y pulsa **Iniciar cámara**.
2. En **Detección (HSV)** elige un color y ajusta los rangos hasta que la máscara muestre solo el objeto.
3. Selecciona el puerto COM y pulsa **Conectar** (u omítelo para trabajar en simulación).
4. En **Control** elige el modo y ajusta las ganancias.
5. Si en lazo cerrado el servo se aleja del objeto en vez de seguirlo, marca **Invertir sentido del servo**.
6. Usa **Grabar CSV** para registrar un experimento.

## Protocolo serial

| Parámetro | Valor |
|---|---|
| Velocidad | 9600 baudios (configurable en la GUI) |
| Formato | 8N1 |
| Trama | ángulo entero en grados seguido de `\n`, p. ej. `90\n` |
| Frecuencia | Máximo ~33 envíos/s, solo cuando cambia el ángulo; reenvío cada 0.5 s como señal de vida |

## Hardware sugerido

- Webcam USB.
- Servomotor tipo SG90 o MG996R.
- PIC16F877A con cristal de 20 MHz (ver repositorio del firmware).
- Convertidor USB-serial (TTL 5 V), por ejemplo CH340, FT232 o PL2303.
- Fuente de 5 V **independiente** para el servo, con tierra común con el PIC y el convertidor.

## Prácticas sugeridas

1. **Calibración:** en modo Manual, relacionar el ángulo con el ancho de pulso.
2. **Lazo abierto vs. cerrado:** mover la base y comparar cómo responde cada modo.
3. **On-Off:** variar la banda muerta y el paso; medir la amplitud de la oscilación.
4. **P:** medir el error en estado estable para distintos Kp.
5. **PI / PID:** sintonizar (por ejemplo, Ziegler-Nichols) y comparar sobreimpulso y tiempo de establecimiento con los CSV.

## Estructura

```
servo-vision-control/
├── servo_vision.py     # Interfaz gráfica, visión y control
├── requirements.txt
└── README.md
```

## Autor

**Arturo Elfego Meneses Flores** — Universidad Tecnológica de la Huasteca Hidalguense (UTHH)
