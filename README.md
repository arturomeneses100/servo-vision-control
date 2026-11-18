# servo-vision-control

Interfaz gráfica en Python para **controlar un servomotor mediante visión artificial**. La cámara detecta un objeto por su color (HSV), calcula el error respecto al centro de la imagen y un controlador (On-Off, P, PI o PID) determina el ángulo del servomotor, que se envía por puerto serial a un microcontrolador PIC.

Proyecto didáctico para la unidad **"Desarrollo de aplicaciones de control de procesos utilizando sistemas embebidos"**, que permite trabajar con conceptos de control en lazo abierto y lazo cerrado, así como con controladores On-Off, P, PI y PID.

**Firmware del PIC:** [servo-pic-firmware](https://github.com/arturomeneses100/servo-pic-firmware)

---

## Arquitectura

```text
Cámara → OpenCV (Python) → Controlador (Python) → Serial → PIC → PWM 50 Hz → Servomotor
              │                    │
        Segmentación HSV      On-Off / P / PI / PID
        Centroide → Error
```

**Variable medida (error):** posición horizontal del objeto respecto al centro de la imagen, normalizada de **−100 %** (borde izquierdo) a **+100 %** (borde derecho).

**Variable manipulada:** ángulo del servomotor, de 0° a 180°.

## Características

- Video en vivo con cuadro delimitador del objeto, centroide, línea central y representación virtual del servomotor que muestra el ángulo.
- Visualización de la máscara binaria para depurar la segmentación.
- Seis deslizadores HSV con colores predefinidos (rojo, verde, azul y amarillo). La detección del color rojo contempla el cruce del tono por el valor 0.
- Filtro por área mínima para ignorar ruido.
- Seis modos de operación (consulta la tabla de modos de control).
- Ajuste en tiempo real de Kp, Ki, Kd, banda muerta y paso del controlador On-Off.
- Gráfica en tiempo real del error y del ángulo (ventana de 10 segundos).
- Grabación de datos en formato CSV (`t_s, modo, error_pct, angulo_deg, P, I, D`) para analizar la respuesta del sistema.
- **Modo de simulación:** permite ejecutar el programa sin necesidad de conectar el PIC.

## Modos de control

| Modo | Montaje | Ley de control | Qué se observa |
|---|---|---|---|
| Manual | — | Ángulo = deslizador | Calibración entre pulso y ángulo |
| Lazo abierto | Cámara fija, servomotor apuntando al objeto | Ángulo = 180 · x / ancho | Sin realimentación: no corrige perturbaciones |
| On-Off | Cámara sobre el servomotor | ±paso si \|e\| > banda | Oscilación y efecto de la banda muerta |
| P | Cámara sobre el servomotor | 90 + Kp · e | Error en estado estable |
| PI | Cámara sobre el servomotor | 90 + Kp · e + Ki · ∫e dt | Eliminación del error, sobreimpulso y anti-windup |
| PID | Cámara sobre el servomotor | 90 + Kp · e + Ki · ∫e dt + Kd · de/dt | Amortiguamiento y sensibilidad al ruido |

El controlador PID utiliza el intervalo de tiempo real (`dt`) entre cuadros y limita la acumulación de la integral mediante una estrategia anti-windup.

Si se pierde el objetivo, el servomotor mantiene su última posición.

## Instalación

Requiere Python 3.9 o superior.

```bash
git clone <url-del-repositorio>
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

1. Selecciona el número de cámara y pulsa **Iniciar cámara**.
2. En **Detección (HSV)**, elige un color y ajusta los rangos hasta conseguir que la máscara muestre únicamente el objeto.
3. Selecciona el puerto COM y pulsa **Conectar**. También puedes omitir este paso para trabajar en modo de simulación.
4. En **Control**, selecciona el modo de operación y ajusta las ganancias correspondientes.
5. Si en el modo de lazo cerrado el servomotor se aleja del objeto en lugar de seguirlo, activa la opción **Invertir sentido del servo**.
6. Utiliza **Grabar CSV** para registrar los datos del experimento.

## Protocolo serial

| Parámetro | Valor |
|---|---|
| Velocidad | 9600 baudios (configurable desde la interfaz gráfica) |
| Formato | 8N1 |
| Trama | Ángulo entero en grados seguido de `\n`, por ejemplo: `90\n` |
| Frecuencia | Máximo aproximado de 33 envíos por segundo, únicamente cuando cambia el ángulo. Se realiza un reenvío cada 0.5 segundos como señal de actividad. |

## Hardware sugerido

- Webcam USB.
- Servomotor tipo SG90 o MG996R.
- PIC16F877A con cristal de 20 MHz (consulta el repositorio del firmware).
- Convertidor USB-serial (TTL de 5 V), por ejemplo, CH340, FT232 o PL2303.
- Fuente de alimentación independiente de 5 V para el servomotor, con tierra común entre el PIC, el convertidor y la fuente.

## Prácticas sugeridas

1. **Calibración:** utilizar el modo Manual para relacionar el ángulo del servomotor con el ancho del pulso.
2. **Lazo abierto vs. lazo cerrado:** mover la base del sistema y comparar la respuesta de ambos modos.
3. **On-Off:** variar la banda muerta y el paso de control para medir la amplitud de las oscilaciones.
4. **Control P:** medir el error en estado estable para distintos valores de Kp.
5. **Control PI / PID:** realizar la sintonización de los controladores (por ejemplo, mediante el método de Ziegler-Nichols) y comparar el sobreimpulso y el tiempo de establecimiento utilizando los registros CSV.

## Estructura del proyecto

```text
servo-vision-control/
├── servo_vision.py     # Interfaz gráfica, visión artificial y control
├── requirements.txt   # Dependencias del proyecto
└── README.md          # Documentación del proyecto
```

## Autor

**Arturo Elfego Meneses Flores**  
Universidad Tecnológica de la Huasteca Hidalguense (UTHH)
