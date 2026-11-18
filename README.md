# servo-vision-control

Interfaz grafica en Python para **controlar un servomotor usando visión artificial**. La cÃ¡mara detecta un objeto por color (HSV), calcula el error respecto al centro de la imagen y un controlador (On-Off, P, PI o PID) decide el Angulo del servo, que se envi­a por puerto serial a un microcontrolador PIC.

Proyecto didactico para la unidad **"Desarrollo de aplicaciones de control de procesos utilizando sistemas embebidos"** (lazo abierto/cerrado, On-Off, P, PI, PID).

> Firmware del PIC: [servo-pic-firmware](https://github.com/arturomeneses100/servo-pic-firmware)

---

## Arquitectura

```
CÃ¡mara â”€â”€â–º OpenCV (Python) â”€â”€â–º Controlador (Python) â”€â”€â–º Serial â”€â”€â–º PIC â”€â”€â–º PWM 50 Hz â”€â”€â–º Servo
           segmentaciÃ³n HSV     On-Off / P / PI / PID     "90\n"
           centroide â†’ error
```

**Variable medida (error):** posiciÃ³n horizontal del objeto respecto al centro de la imagen, normalizada de **âˆ’100 %** (borde izquierdo) a **+100 %** (borde derecho).

**Variable manipulada:** Ã¡ngulo del servo, de 0Â° a 180Â°.

## CaracterÃ­sticas

- Video en vivo con caja del objeto, centroide, lÃ­nea central y un servo virtual que muestra el Ã¡ngulo.
- Vista de la mÃ¡scara binaria para depurar la segmentaciÃ³n.
- 6 deslizadores HSV con colores predefinidos (rojo, verde, azul y amarillo). El rojo se maneja aunque su tono pase por 0.
- Filtro por Ã¡rea mÃ­nima para ignorar ruido.
- Seis modos de operaciÃ³n (ver tabla).
- Ajuste en vivo de Kp, Ki, Kd, banda muerta y paso del On-Off.
- GrÃ¡fica en tiempo real del error y del Ã¡ngulo (ventana de 10 s).
- GrabaciÃ³n a CSV (`t_s, modo, error_pct, angulo_deg, P, I, D`) para analizar la respuesta.
- **Modo simulaciÃ³n:** funciona sin PIC conectado.

## Modos de control

| Modo | Montaje | Ley de control | QuÃ© se observa |
|---|---|---|---|
| Manual | â€” | Ã¡ngulo = deslizador | CalibraciÃ³n pulso â†” Ã¡ngulo |
| Lazo abierto | CÃ¡mara fija, servo apunta al objeto | Ã¡ngulo = 180 Â· x / ancho | Sin realimentaciÃ³n: no corrige perturbaciones |
| On-Off | CÃ¡mara sobre el servo | Â±paso si \|e\| > banda | OscilaciÃ³n, efecto de la banda muerta |
| P | CÃ¡mara sobre el servo | 90 + KpÂ·e | Error en estado estable |
| PI | CÃ¡mara sobre el servo | 90 + KpÂ·e + KiÂ·âˆ«e dt | Elimina el error; sobreimpulso. Anti-windup |
| PID | CÃ¡mara sobre el servo | 90 + KpÂ·e + KiÂ·âˆ«e dt + KdÂ·de/dt | Amortiguamiento y sensibilidad al ruido |

El PID usa el `dt` real entre cuadros y satura la integral (anti-windup). Si se pierde el objetivo, el servo mantiene su Ãºltima posiciÃ³n.

## InstalaciÃ³n

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

1. Elige el nÃºmero de cÃ¡mara y pulsa **Iniciar cÃ¡mara**.
2. En **DetecciÃ³n (HSV)** elige un color y ajusta los rangos hasta que la mÃ¡scara muestre solo el objeto.
3. Selecciona el puerto COM y pulsa **Conectar** (u omÃ­telo para trabajar en simulaciÃ³n).
4. En **Control** elige el modo y ajusta las ganancias.
5. Si en lazo cerrado el servo se aleja del objeto en vez de seguirlo, marca **Invertir sentido del servo**.
6. Usa **Grabar CSV** para registrar un experimento.

## Protocolo serial

| ParÃ¡metro | Valor |
|---|---|
| Velocidad | 9600 baudios (configurable en la GUI) |
| Formato | 8N1 |
| Trama | Ã¡ngulo entero en grados seguido de `\n`, p. ej. `90\n` |
| Frecuencia | MÃ¡ximo ~33 envÃ­os/s, solo cuando cambia el Ã¡ngulo; reenvÃ­o cada 0.5 s como seÃ±al de vida |

## Hardware sugerido

- Webcam USB.
- Servomotor tipo SG90 o MG996R.
- PIC16F877A con cristal de 20 MHz (ver repositorio del firmware).
- Convertidor USB-serial (TTL 5 V), por ejemplo CH340, FT232 o PL2303.
- Fuente de 5 V **independiente** para el servo, con tierra comÃºn con el PIC y el convertidor.

## PrÃ¡cticas sugeridas

1. **CalibraciÃ³n:** en modo Manual, relacionar el Ã¡ngulo con el ancho de pulso.
2. **Lazo abierto vs. cerrado:** mover la base y comparar cÃ³mo responde cada modo.
3. **On-Off:** variar la banda muerta y el paso; medir la amplitud de la oscilaciÃ³n.
4. **P:** medir el error en estado estable para distintos Kp.
5. **PI / PID:** sintonizar (por ejemplo, Ziegler-Nichols) y comparar sobreimpulso y tiempo de establecimiento con los CSV.

## Estructura

```
servo-vision-control/
â”œâ”€â”€ servo_vision.py     # Interfaz grÃ¡fica, visiÃ³n y control
â”œâ”€â”€ requirements.txt
â””â”€â”€ README.md
```

## Autor

**Arturo Elfego Meneses Flores** â€” Universidad TecnolÃ³gica de la Huasteca Hidalguense (UTHH)
