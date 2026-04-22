#!/usr/bin/env python3

import subprocess
import re
import tkinter as tk

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


TOUCH_OUTPUT_NAME = "HDMI-1-0"
FALLBACK_GEOMETRY = "1024x600+0+0"

MIN_HZ = 0.0
MAX_HZ = 60.0


def obtener_geometria_salida(nombre_salida: str) -> str:
    try:
        salida = subprocess.check_output(["xrandr"], text=True)
    except Exception as e:
        print(f"No se pudo ejecutar xrandr: {e}")
        return FALLBACK_GEOMETRY

    patron = re.compile(
        rf"^{re.escape(nombre_salida)} connected(?: primary)? (\d+)x(\d+)\+(\d+)\+(\d+)",
        re.MULTILINE,
    )

    match = patron.search(salida)
    if match:
        ancho, alto, pos_x, pos_y = match.groups()
        return f"{ancho}x{alto}+{pos_x}+{pos_y}"

    print(f"No se encontró la salida {nombre_salida}. Usando geometría por defecto.")
    return FALLBACK_GEOMETRY


class TouchHMINode(Node):
    def __init__(self):
        super().__init__("touch_hmi_node")
        self.pub_cmd = self.create_publisher(String, "/l510_cmd", 10)

    def enviar_comando(self, texto: str):
        msg = String()
        msg.data = texto
        self.pub_cmd.publish(msg)
        self.get_logger().info(f"Publicado: {texto}")


class PanelTouchApp:
    def __init__(self, root: tk.Tk, geometry: str, ros_node: TouchHMINode):
        self.root = root
        self.geometry = geometry
        self.ros_node = ros_node
        self.fullscreen = True

        self.root.title("Panel Touch Industrial")
        self.root.configure(bg="#1e1e1e")
        self.root.geometry(self.geometry)

        self.root.after(250, self.activar_fullscreen)
        self.root.bind("<Escape>", self.salir)
        self.root.bind("<F11>", self.toggle_fullscreen)
        self.root.bind("<Button-1>", self.on_touch)

        self.construir_ui()

    def construir_ui(self):
        contenedor = tk.Frame(self.root, bg="#1e1e1e")
        contenedor.pack(fill="both", expand=True, padx=30, pady=20)

        titulo = tk.Label(
            contenedor,
            text="CONTROL DE BANDA",
            font=("Arial", 32, "bold"),
            bg="#1e1e1e",
            fg="white",
        )
        titulo.pack(pady=(10, 20))

        self.estado = tk.Label(
            contenedor,
            text="Sistema en espera",
            font=("Arial", 24, "bold"),
            bg="#1e1e1e",
            fg="lime",
        )
        self.estado.pack(pady=(0, 20))

        self.coords = tk.Label(
            contenedor,
            text=f"Pantalla detectada: {self.geometry}",
            font=("Arial", 14),
            bg="#1e1e1e",
            fg="#bbbbbb",
        )
        self.coords.pack(pady=(0, 20))

        fila_botones = tk.Frame(contenedor, bg="#1e1e1e")
        fila_botones.pack(pady=20)

        btn_run = tk.Button(
            fila_botones,
            text="RUN",
            font=("Arial", 24, "bold"),
            width=10,
            height=2,
            bg="#1f6f3f",
            fg="white",
            command=self.run_accion,
        )
        btn_run.grid(row=0, column=0, padx=15, pady=10)

        btn_reverse = tk.Button(
            fila_botones,
            text="REVERSE",
            font=("Arial", 24, "bold"),
            width=10,
            height=2,
            bg="#a86f00",
            fg="white",
            command=self.reverse_accion,
        )
        btn_reverse.grid(row=0, column=1, padx=15, pady=10)

        btn_stop = tk.Button(
            fila_botones,
            text="STOP",
            font=("Arial", 24, "bold"),
            width=10,
            height=2,
            bg="#8b1e1e",
            fg="white",
            command=self.stop_accion,
        )
        btn_stop.grid(row=0, column=2, padx=15, pady=10)

        frame_vel = tk.Frame(contenedor, bg="#1e1e1e")
        frame_vel.pack(pady=20, fill="x")

        etiqueta_vel = tk.Label(
            frame_vel,
            text="Velocidad",
            font=("Arial", 24, "bold"),
            bg="#1e1e1e",
            fg="white",
        )
        etiqueta_vel.pack()

        self.valor_vel = tk.Label(
            frame_vel,
            text="30.0 Hz  |  50 %",
            font=("Arial", 22, "bold"),
            bg="#1e1e1e",
            fg="cyan",
        )
        self.valor_vel.pack(pady=(5, 15))

        self.slider = tk.Scale(
            frame_vel,
            from_=0,
            to=100,
            orient="horizontal",
            length=1000,
            width=40,
            sliderlength=80,
            font=("Arial", 18),
            highlightthickness=0,
            bd=4,
            troughcolor="#444444",
            fg="white",
            bg="#1e1e1e",
            command=self.cambio_velocidad,
        )
        self.slider.set(50)
        self.slider.pack(pady=10)

        fila_presets = tk.Frame(contenedor, bg="#1e1e1e")
        fila_presets.pack(pady=15)

        presets = [10, 20, 30, 40, 50, 60]
        for i, hz in enumerate(presets):
            btn = tk.Button(
                fila_presets,
                text=f"{hz} Hz",
                font=("Arial", 18, "bold"),
                width=7,
                height=2,
                command=lambda valor=hz: self.set_preset_hz(valor),
            )
            btn.grid(row=0, column=i, padx=8, pady=5)

        pie = tk.Label(
            contenedor,
            text="ESC: salir | F11: fullscreen on/off",
            font=("Arial", 12),
            bg="#1e1e1e",
            fg="#888888",
        )
        pie.pack(side="bottom", pady=10)

    def activar_fullscreen(self):
        self.root.attributes("-fullscreen", True)
        self.fullscreen = True

    def toggle_fullscreen(self, event=None):
        self.fullscreen = not self.fullscreen
        self.root.attributes("-fullscreen", self.fullscreen)

    def salir(self, event=None):
        self.root.destroy()

    def on_touch(self, event):
        self.coords.config(
            text=f"Pantalla: {self.geometry} | Touch en x={event.x}, y={event.y}"
        )

    def porcentaje_a_hz(self, porcentaje: float) -> float:
        return MIN_HZ + (MAX_HZ - MIN_HZ) * (porcentaje / 100.0)

    def hz_a_porcentaje(self, hz: float) -> int:
        return int(round((hz - MIN_HZ) * 100.0 / (MAX_HZ - MIN_HZ)))

    def run_accion(self):
        self.ros_node.enviar_comando("run")
        self.estado.config(text="RUN enviado", fg="lime")

    def reverse_accion(self):
        self.ros_node.enviar_comando("reverse")
        self.estado.config(text="REVERSE enviado", fg="orange")

    def stop_accion(self):
        self.ros_node.enviar_comando("stop")
        self.estado.config(text="STOP enviado", fg="red")

    def cambio_velocidad(self, valor):
        porcentaje = float(valor)
        hz = self.porcentaje_a_hz(porcentaje)
        self.valor_vel.config(text=f"{hz:.1f} Hz  |  {int(porcentaje)} %")
        self.ros_node.enviar_comando(f"freq {hz:.1f}")
        self.estado.config(text=f"Velocidad enviada: {hz:.1f} Hz", fg="cyan")

    def set_preset_hz(self, hz: float):
        porcentaje = self.hz_a_porcentaje(hz)
        self.slider.set(porcentaje)
        self.ros_node.enviar_comando(f"freq {hz:.1f}")
        self.estado.config(text=f"Preset enviado: {hz:.1f} Hz", fg="cyan")


def main(args=None):
    rclpy.init(args=args)
    ros_node = TouchHMINode()

    geometry = obtener_geometria_salida(TOUCH_OUTPUT_NAME)
    print(f"Geometría usada: {geometry}")

    root = tk.Tk()
    app = PanelTouchApp(root, geometry, ros_node)

    def poll_ros():
        rclpy.spin_once(ros_node, timeout_sec=0.0)
        root.after(50, poll_ros)

    root.after(50, poll_ros)

    try:
        root.mainloop()
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
