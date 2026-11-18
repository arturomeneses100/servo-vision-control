"""
Seguimiento visual con servo — GUI didáctica
Unidad: Desarrollo de aplicaciones de control de procesos utilizando sistemas embebidos
Arquitectura:  Cámara -> OpenCV (Python) -> controlador (Python) -> Serial -> PIC -> PWM -> Servo

Requisitos:
    pip install opencv-python pillow pyserial matplotlib numpy

Protocolo serial (PC -> PIC): el ángulo entero en grados seguido de salto de línea, p. ej. "90\n"
"""
import time
import csv
from collections import deque
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

import cv2
import numpy as np
from PIL import Image, ImageTk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

# (H min, S min, V min, H max, S max, V max). Si H min > H max, el rango "da la vuelta" (rojo).
PRESETS = {
    "Rojo": (170, 120, 70, 10, 255, 255),
    "Verde": (40, 70, 50, 80, 255, 255),
    "Azul": (100, 120, 50, 130, 255, 255),
    "Amarillo": (20, 100, 100, 35, 255, 255),
}
MODOS = ["Manual", "Lazo abierto", "On-Off", "P", "PI", "PID"]
VIDEO_W, VIDEO_H = 640, 480
HIST = 300  # muestras en la gráfica (~10 s a 30 fps)


class PID:
    """PID discreto con dt real y anti-windup por saturación de la integral."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.e_prev = None
        self.t_prev = None

    def calcular(self, e, kp, ki, kd, usar_i, usar_d, lim_i=300.0):
        t = time.perf_counter()
        dt = 0.0 if self.t_prev is None else t - self.t_prev
        d = 0.0
        if dt > 0:
            if usar_i:
                self.integral += e * dt
                self.integral = max(-lim_i, min(lim_i, self.integral))
            if usar_d and self.e_prev is not None:
                d = (e - self.e_prev) / dt
        self.e_prev, self.t_prev = e, t
        p = kp * e
        i = ki * self.integral if usar_i else 0.0
        dd = kd * d if usar_d else 0.0
        return p, i, dd


class App:
    def __init__(self, root):
        self.root = root
        root.title("Seguimiento visual con servo — Control de procesos")
        self.cap = None
        self.ser = None
        self.pid = PID()
        self.terminos = (0.0, 0.0, 0.0)
        self.angulo = 90.0
        self.ultimo_envio = 0.0
        self.ultimo_ang = None
        self.t0 = time.perf_counter()
        self.h_t = deque(maxlen=HIST)
        self.h_e = deque(maxlen=HIST)
        self.h_a = deque(maxlen=HIST)
        self.grabando = False
        self.registro = []
        self.n = 0
        self._gui()
        self._refrescar_puertos()
        root.protocol("WM_DELETE_WINDOW", self.cerrar)
        self.bucle()

    # ------------------------------------------------------------------ GUI
    def _slider(self, parent, texto, desde, hasta, inicial, res=1.0):
        var = tk.DoubleVar(value=inicial)
        tk.Scale(parent, label=texto, from_=desde, to=hasta, resolution=res,
                 orient="horizontal", variable=var, length=260).pack(fill="x")
        return var

    def _gui(self):
        izq = ttk.Frame(self.root, padding=5)
        izq.grid(row=0, column=0, sticky="n")
        der = ttk.Frame(self.root, padding=5)
        der.grid(row=0, column=1, sticky="n")

        # Video + máscara + gráfica
        self.lbl_video = ttk.Label(izq)
        self.lbl_video.pack()
        fila = ttk.Frame(izq)
        fila.pack(fill="x", pady=4)
        self.lbl_mask = ttk.Label(fila)
        self.lbl_mask.pack(side="left")
        fig = Figure(figsize=(5.3, 2.1), dpi=80)
        self.ax = fig.add_subplot(111)
        self.ax.set_xlabel("t [s]")
        self.ax.grid(True, alpha=0.3)
        self.ln_e, = self.ax.plot([], [], label="error [%]")
        self.ln_a, = self.ax.plot([], [], label="ángulo − 90 [°]")
        self.ax.legend(loc="upper left", fontsize=8)
        fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(fig, master=fila)
        self.canvas.get_tk_widget().pack(side="left", padx=5)

        # Conexión
        f = ttk.LabelFrame(der, text="Conexión", padding=5)
        f.pack(fill="x")
        self.cb_puerto = ttk.Combobox(f, width=12, state="readonly")
        self.cb_puerto.grid(row=0, column=0)
        ttk.Button(f, text="↻", width=3, command=self._refrescar_puertos).grid(row=0, column=1)
        self.cb_baud = ttk.Combobox(f, width=7, state="readonly",
                                    values=["9600", "19200", "57600", "115200"])
        self.cb_baud.set("9600")
        self.cb_baud.grid(row=0, column=2)
        self.btn_ser = ttk.Button(f, text="Conectar", command=self.toggle_serial)
        self.btn_ser.grid(row=0, column=3)
        ttk.Label(f, text="Cámara #").grid(row=1, column=0, sticky="e", pady=4)
        self.sp_cam = ttk.Spinbox(f, from_=0, to=5, width=4)
        self.sp_cam.set(0)
        self.sp_cam.grid(row=1, column=1)
        self.btn_cam = ttk.Button(f, text="Iniciar cámara", command=self.toggle_camara)
        self.btn_cam.grid(row=1, column=2, columnspan=2, sticky="ew")

        nb = ttk.Notebook(der)
        nb.pack(fill="x", pady=4)

        # Pestaña Detección
        f = ttk.Frame(nb, padding=5)
        nb.add(f, text="Detección (HSV)")
        self.cb_preset = ttk.Combobox(f, values=list(PRESETS), state="readonly")
        self.cb_preset.set("Rojo")
        self.cb_preset.bind("<<ComboboxSelected>>", self._aplicar_preset)
        self.cb_preset.pack(fill="x")
        rej = ttk.Frame(f)
        rej.pack()
        nombres = ["H min", "S min", "V min", "H max", "S max", "V max"]
        maximos = [179, 255, 255, 179, 255, 255]
        self.hsv = []
        for i, (nom, mx) in enumerate(zip(nombres, maximos)):
            v = tk.IntVar()
            tk.Scale(rej, label=nom, from_=0, to=mx, orient="horizontal",
                     variable=v, length=130).grid(row=i % 3, column=i // 3)
            self.hsv.append(v)
        self.area_min = self._slider(f, "Área mínima [px²]", 50, 5000, 400, 50)
        self._aplicar_preset()

        # Pestaña Control
        f = ttk.Frame(nb, padding=5)
        nb.add(f, text="Control")
        self.cb_modo = ttk.Combobox(f, values=MODOS, state="readonly")
        self.cb_modo.set("Manual")
        self.cb_modo.bind("<<ComboboxSelected>>", lambda _e: self.pid.reset())
        self.cb_modo.pack(fill="x")
        self.ang_manual = self._slider(f, "Ángulo manual [°]", 0, 180, 90)
        self.kp = self._slider(f, "Kp [°/%]", 0, 3, 0.5, 0.05)
        self.ki = self._slider(f, "Ki [°/(%·s)]", 0, 3, 0.0, 0.05)
        self.kd = self._slider(f, "Kd [°·s/%]", 0, 0.5, 0.0, 0.01)
        self.banda = self._slider(f, "Banda muerta On-Off [%]", 0, 30, 5)
        self.paso = self._slider(f, "Paso On-Off [°/ciclo]", 0.5, 10, 2, 0.5)
        self.invertir = tk.BooleanVar(value=False)
        ttk.Checkbutton(f, text="Invertir sentido del servo",
                        variable=self.invertir).pack(anchor="w")

        # Estado
        f = ttk.LabelFrame(der, text="Estado", padding=5)
        f.pack(fill="x")
        self.lbl_estado = ttk.Label(f, text="—", font=("Consolas", 10), justify="left")
        self.lbl_estado.pack(anchor="w")
        b = ttk.Frame(f)
        b.pack(fill="x", pady=4)
        ttk.Button(b, text="Centrar (90°)", command=self.centrar).pack(side="left")
        self.btn_rec = ttk.Button(b, text="● Grabar CSV", command=self.toggle_grabar)
        self.btn_rec.pack(side="left", padx=5)

    def _aplicar_preset(self, _e=None):
        for var, val in zip(self.hsv, PRESETS[self.cb_preset.get()]):
            var.set(val)

    # ------------------------------------------------------------- Hardware
    def _refrescar_puertos(self):
        puertos = [p.device for p in serial.tools.list_ports.comports()] if serial else []
        self.cb_puerto["values"] = puertos
        if puertos:
            self.cb_puerto.set(puertos[0])

    def toggle_serial(self):
        if self.ser:
            self.desconectar_serial()
            return
        if serial is None:
            messagebox.showwarning("Serial", "Instala pyserial:  pip install pyserial")
            return
        try:
            self.ser = serial.Serial(self.cb_puerto.get(), int(self.cb_baud.get()), timeout=0)
            self.btn_ser.config(text="Desconectar")
        except Exception as ex:
            self.ser = None
            messagebox.showerror("Serial", str(ex))

    def desconectar_serial(self):
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.btn_ser.config(text="Conectar")

    def toggle_camara(self):
        if self.cap is None:
            idx = int(self.sp_cam.get())
            self.cap = cv2.VideoCapture(idx)
            if not self.cap.isOpened():
                self.cap = None
                messagebox.showerror("Cámara", f"No se pudo abrir la cámara {idx}")
                return
            self.btn_cam.config(text="Detener cámara")
        else:
            self.cap.release()
            self.cap = None
            self.btn_cam.config(text="Iniciar cámara")

    def enviar(self, ang):
        a = int(round(ang))
        ahora = time.perf_counter()
        if not (self.ser and self.ser.is_open):
            return
        # máx. ~33 envíos/s; reenvía cada 0.5 s aunque no cambie (keep-alive)
        if ahora - self.ultimo_envio > 0.03 and (a != self.ultimo_ang or ahora - self.ultimo_envio > 0.5):
            try:
                self.ser.write(f"{a}\n".encode())
                self.ultimo_ang, self.ultimo_envio = a, ahora
            except Exception as ex:
                self.desconectar_serial()
                messagebox.showerror("Serial", f"Se perdió la conexión: {ex}")

    # ------------------------------------------------------- Visión + control
    def bucle(self):
        if self.cap is not None:
            ok, frame = self.cap.read()
            if ok:
                self.procesar(frame)
        self.root.after(15, self.bucle)

    def segmentar(self, frame):
        hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (7, 7), 0), cv2.COLOR_BGR2HSV)
        hmin, smin, vmin, hmax, smax, vmax = [int(v.get()) for v in self.hsv]
        if hmin <= hmax:
            mask = cv2.inRange(hsv, (hmin, smin, vmin), (hmax, smax, vmax))
        else:  # rango de tono que cruza 0 (rojo)
            mask = cv2.inRange(hsv, (hmin, smin, vmin), (179, smax, vmax)) | \
                   cv2.inRange(hsv, (0, smin, vmin), (hmax, smax, vmax))
        k = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        return mask

    def procesar(self, frame):
        frame = cv2.resize(frame, (VIDEO_W, VIDEO_H))
        mask = self.segmentar(frame)
        contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cx = None
        if contornos:
            c = max(contornos, key=cv2.contourArea)
            if cv2.contourArea(c) >= self.area_min.get():
                m = cv2.moments(c)
                if m["m00"] > 0:
                    cx, cy = int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"])
                    x, y, w, h = cv2.boundingRect(c)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)

        # Error normalizado: −100 % (borde izq.) … +100 % (borde der.)
        e = None if cx is None else (cx - VIDEO_W / 2) / (VIDEO_W / 2) * 100
        self.controlar(e, cx)
        self.overlay(frame, e)
        self.mostrar(frame, mask)
        self.registrar(e, cx)

    def controlar(self, e, cx):
        modo = self.cb_modo.get()
        s = -1 if self.invertir.get() else 1
        if modo == "Manual":
            self.angulo = self.ang_manual.get()
        elif e is None:
            self.pid.t_prev = None  # objetivo perdido: mantener posición, evitar salto de dt
        elif modo == "Lazo abierto":
            # Cámara fija: el servo "apunta" al objeto con un mapeo directo píxel -> grado
            r = cx / VIDEO_W
            self.angulo = 180 * (r if s > 0 else 1 - r)
        elif modo == "On-Off":
            b = self.banda.get()
            if e > b:
                self.angulo += s * self.paso.get()
            elif e < -b:
                self.angulo -= s * self.paso.get()
        else:
            # Cámara montada en el servo (lazo cerrado): llevar el objeto al centro
            self.terminos = self.pid.calcular(e, self.kp.get(), self.ki.get(), self.kd.get(),
                                              modo in ("PI", "PID"), modo == "PID")
            self.angulo = 90 + s * sum(self.terminos)
        self.angulo = max(0.0, min(180.0, self.angulo))
        self.enviar(self.angulo)

    def overlay(self, frame, e):
        c0 = VIDEO_W // 2
        cv2.line(frame, (c0, 0), (c0, VIDEO_H), (255, 255, 255), 1)
        modo = self.cb_modo.get()
        if modo == "On-Off":
            db = int(self.banda.get() / 100 * VIDEO_W / 2)
            for xx in (c0 - db, c0 + db):
                cv2.line(frame, (xx, 0), (xx, VIDEO_H), (0, 200, 255), 1)
        cv2.putText(frame, f"{modo}  |  {self.angulo:5.1f} deg", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)
        if e is None and modo != "Manual":
            cv2.putText(frame, "OBJETIVO NO DETECTADO", (10, VIDEO_H - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        # Servo virtual (indicador de 0° a 180°)
        cen, r = (VIDEO_W - 65, 75), 45
        cv2.ellipse(frame, cen, (r, r), 0, 180, 360, (220, 220, 220), 1)
        th = np.radians(self.angulo)
        punta = (int(cen[0] - r * np.cos(th)), int(cen[1] - r * np.sin(th)))
        cv2.line(frame, cen, punta, (0, 255, 255), 3)

    def mostrar(self, frame, mask):
        self._img = ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)))
        self.lbl_video.configure(image=self._img)
        self._imgm = ImageTk.PhotoImage(Image.fromarray(cv2.resize(mask, (213, 160))))
        self.lbl_mask.configure(image=self._imgm)

    def registrar(self, e, cx):
        t = time.perf_counter() - self.t0
        self.h_t.append(t)
        self.h_e.append(np.nan if e is None else e)
        self.h_a.append(self.angulo - 90)
        if self.grabando:
            self.registro.append((round(t, 3), self.cb_modo.get(),
                                  "" if e is None else round(e, 2), round(self.angulo, 2),
                                  *[round(v, 3) for v in self.terminos]))
        self.n += 1
        if self.n % 5 == 0:
            self.ln_e.set_data(self.h_t, self.h_e)
            self.ln_a.set_data(self.h_t, self.h_a)
            self.ax.set_xlim(max(0, t - 10), max(10, t))
            self.ax.set_ylim(-100, 100)
            self.canvas.draw_idle()
            p, i, d = self.terminos
            estado_ser = "conectado" if self.ser else "simulación (sin PIC)"
            self.lbl_estado.config(text=(
                f"Centroide x : {'—' if cx is None else cx}\n"
                f"Error       : {'—' if e is None else f'{e:+6.1f} %'}\n"
                f"Ángulo      : {self.angulo:6.1f} °\n"
                f"P / I / D   : {p:+.1f} / {i:+.1f} / {d:+.1f}\n"
                f"Serial      : {estado_ser}"))

    # ------------------------------------------------------------ Utilidades
    def centrar(self):
        self.angulo = 90.0
        self.ang_manual.set(90)
        self.pid.reset()

    def toggle_grabar(self):
        if not self.grabando:
            self.registro, self.grabando = [], True
            self.btn_rec.config(text="■ Detener y guardar")
            return
        self.grabando = False
        self.btn_rec.config(text="● Grabar CSV")
        ruta = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV", "*.csv")])
        if ruta:
            with open(ruta, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(["t_s", "modo", "error_pct", "angulo_deg", "P", "I", "D"])
                w.writerows(self.registro)

    def cerrar(self):
        if self.cap is not None:
            self.cap.release()
        self.desconectar_serial()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()
