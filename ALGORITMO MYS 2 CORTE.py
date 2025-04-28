import tkinter as tk
from tkinter import messagebox
import math
import time
import threading
import numpy as np
from scipy.interpolate import splprep, splev
from PIL import Image, ImageTk


class SimuladorRobot:
    def __init__(self, root):
        self.root = root
        self.root.title("Simulador de Robot Diferencial")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        
        # Ruta global para la imagen del robot
        self.robot_image_path = "Robot.png"  # Asegúrate de que el archivo "robot.png" esté en la misma carpeta
        self.original_robot_image = Image.open(self.robot_image_path).resize((40,40))  # Redimensionar a 50x50 píxeles
        self.robot_image = tk.PhotoImage(file=self.robot_image_path)

        # Configuración inicial del robot
        self.robot_x = 600
        self.robot_y = 350
        self.robot_theta = 0
        self.radio_rueda = 1.0
        self.distancia_entre_ruedas = 40.0
        self.en_movimiento = False
        self.ultimo_dibujo = time.time()
        self.active_threads = []
        
        # Inicializar IDs de elementos del canvas
        self.robot_id = None
        self.linea_id = None
        self.trayectoria_id = None
        
        # Límites del área de movimiento
        self.min_x = 20
        self.max_x = 1180
        self.min_y = 20
        self.max_y = 680

        # Parámetros del controlador
        self.Kp_linear = 0.8
        self.Kp_angular = 1.2
        self.velocidad_maxima = 15
        self.tolerancia_posicion = 5
        self.tolerancia_angulo = 0.1

        # Configuración de la interfaz
        self._configurar_interfaz()
        self._dibujar_tablero()
        self._dibujar_robot()

        # Variables para el dibujo
        self.dibujando = False
        self.puntos_recorrido = []
        self.puntos = []

    def on_close(self):
        """Maneja el cierre seguro de la aplicación"""
        self.en_movimiento = False
        for thread in self.active_threads:
            if thread.is_alive():
                thread.join(timeout=0.5)
        self.root.destroy()

    def _configurar_interfaz(self):
        # Panel izquierdo
        self.panel_izquierdo = tk.Frame(self.root, width=250, bg="white")
        self.panel_izquierdo.pack(side=tk.LEFT, fill=tk.Y)

        # Sección de voltajes
        tk.Label(self.panel_izquierdo, text="VOLTAJES:", font=("Arial", 10, "bold"), bg="white").pack(pady=(10, 0))
        self.v_izq_entry = self._crear_input("V(IZQ):", validar_numero=True)
        self.v_der_entry = self._crear_input("V(DER):", validar_numero=True)
        self._crear_botonera(self.enviar_voltajes, self.resetear_voltajes)

        # Sección de dibujo
        tk.Label(self.panel_izquierdo, text="DIBUJA EL RECORRIDO", bg="white", font=("Arial", 10, "bold")).pack(pady=(20, 0))
        self.canvas_dibujo = tk.Canvas(self.panel_izquierdo, width=150, height=150, bg="lightgray")
        self.canvas_dibujo.pack()
        self._crear_botonera(self.enviar_recorrido, self.resetear_recorrido)

        # Sección de posición
        tk.Label(self.panel_izquierdo, text="INGRESE POSICION:", bg="white", font=("Arial", 10, "bold")).pack(pady=(20, 0))
        self.x_entry = self._crear_input("VALOR DE X", validar_numero=True)
        self.y_entry = self._crear_input("VALOR DE Y", validar_numero=True)
        self.theta_entry = self._crear_input("VALOR DE θ", validar_numero=True)
        self._crear_botonera(self.enviar_posicion, self.resetear_posicion)

        # Canvas principal para el robot
        self.canvas_robot = tk.Canvas(self.root, bg="lightgray")
        self.canvas_robot.pack(side=tk.LEFT, expand=True, fill=tk.BOTH)

        # Configurar eventos de dibujo
        self.canvas_dibujo.bind("<Button-1>", self.iniciar_dibujo)
        self.canvas_dibujo.bind("<B1-Motion>", self.dibujar_ruta)
        self.canvas_dibujo.bind("<ButtonRelease-1>", self.terminar_dibujo)

    def _crear_input(self, label, validar_numero=False):
        tk.Label(self.panel_izquierdo, text=label, bg="white").pack()
        
        if validar_numero:
            validate_cmd = (self.root.register(self._validar_numero), '%P')
            entry = tk.Entry(self.panel_izquierdo, validate="key", validatecommand=validate_cmd)
        else:
            entry = tk.Entry(self.panel_izquierdo)
            
        entry.pack(pady=5)
        return entry

    def _validar_numero(self, nuevo_valor):
        if nuevo_valor == "" or nuevo_valor == "-":
            return True
        try:
            float(nuevo_valor)
            return True
        except ValueError:
            return False

    def _crear_botonera(self, send_func, reset_func):
        frame = tk.Frame(self.panel_izquierdo, bg="white")
        frame.pack(pady=5)
        # Crear botón SEND y almacenar referencia
        send_button = tk.Button(frame, text="SEND", width=8, command=lambda: self._bloquear_boton(send_button, send_func), bg="black", fg="white")
        send_button.pack(side=tk.LEFT, padx=5)
        # Crear botón RESET y almacenar referencia
        reset_button = tk.Button(frame, text="RESET", width=8, command=lambda: self._desbloquear_boton(send_button, reset_func), bg="black", fg="white")
        reset_button.pack(side=tk.LEFT)

    def _bloquear_boton(self, button, func):
        button.config(state=tk.DISABLED)
        func()

    def _desbloquear_boton(self, button, func):
        button.config(state=tk.NORMAL)
        func()

    def _dibujar_tablero(self):
        self.canvas_robot.create_text(300, 13, text="TABLERO DE RECORRIDO Y VISUALIZACION DEL ROBOT", font=("Arial", 12, "bold"))
        # Dibujar borde del área permitida con línea roja más visible
        self.canvas_robot.create_rectangle(self.min_x, self.min_y, self.max_x, self.max_y, 
                                         outline="red", dash=(2,2), width=2)

    def _dibujar_robot(self):
        ahora = time.time()
        if ahora - self.ultimo_dibujo < 0.05:
            return

        self.ultimo_dibujo = ahora

        # Eliminar elementos existentes solo si existen
        if self.robot_id is not None:
            self.canvas_robot.delete(self.robot_id)
        if self.linea_id is not None:
            self.canvas_robot.delete(self.linea_id)
        if self.trayectoria_id is not None:
            self.canvas_robot.delete(self.trayectoria_id)

        # Dibujar trayectoria si hay puntos
        if len(self.puntos) > 1:
            self.trayectoria_id = self.canvas_robot.create_line(self.puntos, fill="red", width=2, smooth=True)

        # Rotar la imagen según la orientación del robot
        rotated_image = self.original_robot_image.rotate(-math.degrees(self.robot_theta), resample=Image.BICUBIC)
        self.robot_image = ImageTk.PhotoImage(rotated_image)

        # Dibujar robot con imagen
        x, y = self.robot_x, self.robot_y
        self.robot_id = self.canvas_robot.create_image(x, y, image=self.robot_image)

        # Dibujar línea de orientación del robot
        r = 20  # Longitud de la línea de orientación
        punta_x = x + r * math.cos(self.robot_theta)
        punta_y = y + r * math.sin(self.robot_theta)
        self.linea_id = self.canvas_robot.create_line(x, y, punta_x, punta_y, fill="white", width=2)

        # Agregar punto actual a la trayectoria
        self.puntos.append((x, y))

    def _limitar_posicion(self):
        """Asegura que el robot no salga de los límites establecidos"""
        self.robot_x = max(self.min_x, min(self.max_x, self.robot_x))
        self.robot_y = max(self.min_y, min(self.max_y, self.robot_y))
        
        # Si estamos en el límite, agregar un marcador visual
        if (self.robot_x <= self.min_x or self.robot_x >= self.max_x or
            self.robot_y <= self.min_y or self.robot_y >= self.max_y):
            self.canvas_robot.create_text(self.robot_x, self.robot_y-20, 
                                        text="LÍMITE", fill="red", font=("Arial", 10))

    def _calcular_voltajes_control(self, x_objetivo, y_objetivo, theta_objetivo):
        """Calcula los voltajes necesarios para llegar a la posición deseada"""
        # Calcular vector hacia el objetivo
        dx = x_objetivo - self.robot_x
        dy = y_objetivo - self.robot_y
        distancia = math.sqrt(dx**2 + dy**2)
        
        # Ángulo hacia el objetivo
        theta_deseado = math.atan2(dy, dx)
        
        # Error angular (normalizado entre -pi y pi)
        error_angular = (theta_deseado - self.robot_theta + math.pi) % (2 * math.pi) - math.pi
        
        # Error de orientación final
        error_theta_final = (theta_objetivo - self.robot_theta + math.pi) % (2 * math.pi) - math.pi
        
        # Control proporcional para movimiento
        if distancia > self.tolerancia_posicion:
            # Priorizar llegar a la posición
            vel_lineal = min(self.Kp_linear * distancia, self.velocidad_maxima)
            vel_angular = self.Kp_angular * error_angular
        else:
            # Luego ajustar la orientación
            vel_lineal = 0
            vel_angular = self.Kp_angular * error_theta_final
        
        # Convertir a velocidades de rueda
        v_izq = vel_lineal - vel_angular * self.distancia_entre_ruedas / 2
        v_der = vel_lineal + vel_angular * self.distancia_entre_ruedas / 2
        
        return v_izq, v_der, distancia, abs(error_theta_final)

    def _navegar_a_posicion(self, x_objetivo, y_objetivo, theta_objetivo):
        """Mueve el robot a la posición deseada usando control proporcional"""
        def mover():
            if not self.en_movimiento:
                return

            v_izq, v_der, distancia, error_theta = self._calcular_voltajes_control(
                x_objetivo, y_objetivo, theta_objetivo)

            # Condición de terminación
            if distancia < self.tolerancia_posicion and error_theta < self.tolerancia_angulo:
                self.en_movimiento = False
                return

            # Aplicar velocidades
            v_l = v_izq * self.radio_rueda
            v_r = v_der * self.radio_rueda
            v = (v_r + v_l) / 2.0
            w = (v_r - v_l) / self.distancia_entre_ruedas

            self.robot_x += v * math.cos(self.robot_theta) * 0.1
            self.robot_y += v * math.sin(self.robot_theta) * 0.1
            self.robot_theta += w * 0.1

            # Normalizar ángulo entre -pi y pi
            self.robot_theta = (self.robot_theta + math.pi) % (2 * math.pi) - math.pi

            # Aplicar límites
            self._limitar_posicion()

            # Dibujar el robot
            self._dibujar_robot()

            # Programar la siguiente iteración
            self.root.after(50, mover)

        # Iniciar el movimiento
        mover()

    def enviar_voltajes(self):
        try:
            v_izq = float(self.v_izq_entry.get())
            v_der = float(self.v_der_entry.get())
        except ValueError:
            messagebox.showerror("Error", "Voltajes inválidos")
            return

        self.en_movimiento = True
        thread = threading.Thread(target=self.simular_movimiento, args=(v_izq, v_der), daemon=True)
        thread.start()
        self.active_threads.append(thread)

    def simular_movimiento(self, v_izq, v_der):
        tiempo_total = 2
        dt = 0.05
        pasos = int(tiempo_total / dt)
        self.paso_actual = 0

        def actualizar_movimiento():
            if not self.en_movimiento or self.paso_actual >= pasos:
                self.en_movimiento = False
                return

            v_l = v_izq * self.radio_rueda
            v_r = v_der * self.radio_rueda
            v = (v_l + v_r) / 2.0
            w = (v_l - v_r) / self.distancia_entre_ruedas

            self.robot_x += v * math.cos(self.robot_theta) * dt * 15
            self.robot_y += v * math.sin(self.robot_theta) * dt * 15
            self.robot_theta += w * dt

            # Aplicar límites
            self._limitar_posicion()

            self._dibujar_robot()
            self.paso_actual += 1

            # Programar la siguiente actualización
            self.root.after(int(dt * 1000), actualizar_movimiento)

        # Iniciar la simulación
        actualizar_movimiento()

    def resetear_voltajes(self):
        self.en_movimiento = False
        self.v_izq_entry.delete(0, tk.END)
        self.v_der_entry.delete(0, tk.END)
        self.reiniciar_robot()

    def enviar_recorrido(self):
        if not self.puntos_recorrido:
            messagebox.showerror("Error", "No hay recorrido dibujado")
            return

        # Verificar que el dibujo esté completamente dentro del área de dibujo
        for x, y in self.puntos_recorrido:
            if x < 0 or x > self.canvas_dibujo.winfo_width() or \
               y < 0 or y > self.canvas_dibujo.winfo_height():
                messagebox.showerror("Error", "El recorrido debe estar completamente dentro del área de dibujo")
                return

        # Usar dimensiones fijas como respaldo
        canvas_dibujo_w = self.canvas_dibujo.winfo_width() or 150
        canvas_dibujo_h = self.canvas_dibujo.winfo_height() or 150
        canvas_robot_w = self.canvas_robot.winfo_width() or 1000
        canvas_robot_h = self.canvas_robot.winfo_height() or 650

        escala_x = canvas_robot_w / canvas_dibujo_w
        escala_y = canvas_robot_h / canvas_dibujo_h
        puntos_escalados = [(x * escala_x, y * escala_y) for x, y in self.puntos_recorrido]

        # Filtrar puntos para que estén dentro de los límites permitidos
        puntos_escalados = [(max(self.min_x, min(self.max_x, x)), 
                           max(self.min_y, min(self.max_y, y))) for x, y in puntos_escalados]

        puntos_suavizados = self._suavizar_recorrido(puntos_escalados)
        
        self.en_movimiento = True
        thread = threading.Thread(target=self.seguir_recorrido_suave, args=(puntos_suavizados,), daemon=True)
        thread.start()
        self.active_threads.append(thread)

    def _suavizar_recorrido(self, puntos):
        if len(puntos) < 3:
            return puntos

        try:
    # Asegurar que los puntos estén dentro de los límites antes de suavizar
            puntos = [(max(self.min_x, min(self.max_x, x)), 
              max(self.min_y, min(self.max_y, y))) for x, y in puntos]
                     
            
            x = np.array([p[0] for p in puntos])
            y = np.array([p[1] for p in puntos])
            tck, u = splprep([x, y], s=0, per=False)
            u_new = np.linspace(u.min(), u.max(), 100)
            x_new, y_new = splev(u_new, tck)
            
            # Asegurar que los puntos suavizados también estén dentro de los límites
            puntos_suavizados = [(max(self.min_x, min(self.max_x, x)), 
                     max(self.min_y, min(self.max_y, y))) 
                    for x, y in zip(x_new, y_new)]
            return puntos_suavizados
        except Exception as e:
            print(f"Error al suavizar recorrido: {e}")
            return puntos
            

    def seguir_recorrido_suave(self, puntos):
        try:
            if not puntos:
                return

            # Filtrar puntos para que estén dentro de los límites
            puntos = [(max(self.min_x, min(self.max_x, x)), 
                       max(self.min_y, min(self.max_y, y))) for x, y in puntos]

            # Reiniciar posición y trayectoria
            self.robot_x, self.robot_y = puntos[0]
            self.robot_theta = 0
            self.puntos = []
            self._dibujar_robot()

            velocidad = 8
            tolerancia = 5
            self.punto_actual = 1

            def mover_al_siguiente_punto():
                if self.punto_actual >= len(puntos) or not self.en_movimiento:
                    self.en_movimiento = False
                    return

                x_objetivo, y_objetivo = puntos[self.punto_actual]
                dx = x_objetivo - self.robot_x
                dy = y_objetivo - self.robot_y
                distancia = math.sqrt(dx**2 + dy**2)

                if distancia > tolerancia:
                    dx_norm = dx / distancia if distancia > 0 else 0
                    dy_norm = dy / distancia if distancia > 0 else 0

                    # Verificar que no nos salgamos de los límites
                    if ((self.robot_x <= self.min_x and dx_norm < 0) or
                        (self.robot_x >= self.max_x and dx_norm > 0) or
                        (self.robot_y <= self.min_y and dy_norm < 0) or
                        (self.robot_y >= self.max_y and dy_norm > 0)):
                        self.en_movimiento = False
                        return

                    self.robot_x += dx_norm * min(velocidad, distancia)
                    self.robot_y += dy_norm * min(velocidad, distancia)
                    self.robot_theta = math.atan2(dy, dx)
                    self._limitar_posicion()
                    self._dibujar_robot()

                    # Programar la siguiente actualización
                    self.root.after(30, mover_al_siguiente_punto)
                else:
                    self.punto_actual += 1
                    mover_al_siguiente_punto()

            # Iniciar el movimiento
            mover_al_siguiente_punto()

        except Exception as e:
            print(f"Error en seguimiento de recorrido: {e}")
            self.en_movimiento = False

    def enviar_posicion(self):
        try:
            x = float(self.x_entry.get())
            y = float(self.y_entry.get())
            theta = float(self.theta_entry.get())
            
            if (self.min_x <= x <= self.max_x and 
                self.min_y <= y <= self.max_y):
                self.en_movimiento = True
                thread = threading.Thread(target=self._navegar_a_posicion, args=(x, y, theta), daemon=True)
                thread.start()
                self.active_threads.append(thread)
            else:
                messagebox.showerror("Error", "Posición fuera de los límites permitidos")
            
        except ValueError:
            messagebox.showerror("Error", "Valores inválidos")

    def resetear_posicion(self):
        self.en_movimiento = False
        self.x_entry.delete(0, tk.END)
        self.y_entry.delete(0, tk.END)
        self.theta_entry.delete(0, tk.END)
        self.reiniciar_robot()

    def resetear_recorrido(self):
        self.en_movimiento = False
        self.canvas_dibujo.delete("all")
        self.puntos_recorrido = []
        self.reiniciar_robot()

    def reiniciar_robot(self):
        self.canvas_robot.delete("all")
        self.robot_x = 600
        self.robot_y = 350
        self.robot_theta = 0
        self.puntos = []
        self.robot_id = None
        self.linea_id = None
        self.trayectoria_id = None
        self._dibujar_tablero()
        self._dibujar_robot()

    def iniciar_dibujo(self, event):
        self.dibujando = True
        self.puntos_recorrido = [(event.x, event.y)]

    def dibujar_ruta(self, event):
        if self.dibujando:
            x, y = event.x, event.y
            self.puntos_recorrido.append((x, y))

            def actualizar_dibujo():
                if len(self.puntos_recorrido) > 1:
                    self.canvas_dibujo.create_line(
                        self.puntos_recorrido[-2][0], self.puntos_recorrido[-2][1],
                        self.puntos_recorrido[-1][0], self.puntos_recorrido[-1][1],
                        fill="blue", width=2
                    )

            # Usar after para programar la actualización del dibujo
            self.root.after(10, actualizar_dibujo)

    def terminar_dibujo(self, event):
        self.dibujando = False


if __name__ == "__main__":
    root = tk.Tk()
    app = SimuladorRobot(root)
    root.geometry("1350x700")
    root.mainloop()
    