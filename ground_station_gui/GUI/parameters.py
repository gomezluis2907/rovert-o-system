import customtkinter as ctk
import colors as c

class App(ctk.CTk):
    def __init__(self):
        super().__init__(fg_color=c.white[0])
        self.title("test")
        self.geometry("1920x1080")
        
        parameters = Parameters(master=self)
        parameters.place(relx=0.05, rely=0.05, relwidth=0.9, relheight=0.9)

# === clase padre (frame) ===
class Parameters(ctk.CTkFrame):
    # parametros de la clase
    def __init__(self, master):
        # inicializacion del frame
        super().__init__(master, corner_radius=32, fg_color= c.gray[3], border_width=4, border_color=c.black[1])
        self.border_color = c.red[2]
        self.grid_columnconfigure((0, 1, 2), weight=1)
        self.grid_rowconfigure((0, 1, 2, 3), weight=1)
        
        # fuentes de la interfaz
        self.fonts = [ctk.CTkFont(family="Noto Sans Mono", size=18, weight="bold"), ctk.CTkFont(family="Noto Sans Mono", size=18, weight="normal"), ctk.CTkFont(family="Noto Sans Mono", size=18, slant="italic")]
        
        # ===== instancia de seccion (frame) de parametros del rover =====
        self.rover_params = self.RoverParams(master=self)
        self.rover_params.grid(row=0, column=2, sticky="nsew", padx=16, pady=16)
        self.rover_params.grid_columnconfigure((0, 1, 2), weight=1)
        self.rover_params.grid_rowconfigure((0, 1, 2, 3), weight=1)
        
        # cambio de color del texto
        for widget in self.rover_params.winfo_children():
            if isinstance(widget, ctk.CTkLabel):
                widget.configure(text_color=c.black[1])
                
        # ===== instancia de seccion (frame) de parametros de rasputin =====
        self.rasp_params = self.RaspParams(master=self)
        self.rasp_params.grid(row=1, column=2, sticky="nsew", padx=16, pady=16)
        self.rasp_params.grid_columnconfigure((0, 1, 2), weight=1)
        self.rasp_params.grid_rowconfigure((0, 1, 2, 3), weight=1)
        
        # ===== instancia de seccion (frame) de instrucciones =====
        self.instructions = self.Instructions(master=self)
        self.instructions.grid(row=2, column=2, rowspan=2, sticky="nsew", padx=16, pady=16)
        
        # cambio de color del texto
        for widget in self.instructions.winfo_children():
            if isinstance(widget, ctk.CTkLabel):
                widget.configure(text_color=c.black[1])
        
        # ===== instancia de seccion (frame) de parametros del mapa =====
        self.map_params = self.MapParams(master=self)
        self.map_params.grid(row=3, column=0, columnspan=2, sticky="nsew", padx=16, pady=16)
        self.map_params.grid_columnconfigure((0, 1, 2), weight=1)
        self.map_params.grid_rowconfigure((0, 1, 2, 3), weight=1)
        
        # cambio de color del texto
        for widget in self.map_params.winfo_children():
            if isinstance(widget, ctk.CTkLabel):
                widget.configure(text_color=c.black[1])
        
        # ===== instancia de seccion (frame) de mapa =====
        self.map = self.Map(master=self)
        self.map.grid(row=0, column=0, rowspan=3, columnspan=2, sticky="nsew", padx=16, pady=16)

    # === clase de seccion de instrucciones (frame) ===
    class Instructions(ctk.CTkFrame):
        # parametros de la clase
        def __init__(self, master):
            # inicializacion del frame
            super().__init__(master, corner_radius=32, fg_color= c.white[0], border_width=4, border_color=c.black[1])
            
            # variables de texto
            self.text_state = ctk.StringVar(value="Detenido")
            
            # distancia
            self.label_distance = ctk.CTkLabel(master=self, text="Distancia [m]:", font=master.fonts[0])
            self.label_distance.place(anchor="nw", relx=0.1, rely=0.1)
            self.entry_distance = ctk.CTkEntry(master=self, corner_radius=8, placeholder_text="Ingresa distancia...", font=master.fonts[2])
            self.entry_distance.place(anchor="nw", relx=0.1, rely=0.2, relwidth=0.5, relheight=0.1)
            
            # direccion
            self.label_direction = ctk.CTkLabel(master=self, text="Dirección [\u00b0]:", font=master.fonts[0])
            self.label_direction.place(anchor="nw", relx=0.1, rely=0.35)
            self.entry_direction = ctk.CTkEntry(master=self, corner_radius=8, placeholder_text="Ingresa dirección...", font=master.fonts[2])
            self.entry_direction.place(anchor="nw", relx=0.1, rely=0.45, relwidth=0.5, relheight=0.1)
            
            # boton envio
            self.button_send = ctk.CTkButton(master=self, corner_radius=8, text="Enviar avance", font=master.fonts[0], border_width=4, border_color=c.yellow[0], fg_color=c.yellow[1], hover_color=c.yellow[2], command=self.button_send_pressed)
            self.button_send.place(anchor="nw", relx=0.1, rely=0.6, relwidth=0.35, relheight=0.1)
            
            # boton detener
            self.button_stop = ctk.CTkButton(master=self, corner_radius=8, text="Detener avance", font=master.fonts[0], border_width=4, border_color=c.red[1], fg_color=c.red[2], hover_color=c.red[3], command=self.button_stop_pressed)
            self.button_stop.place(anchor="nw", relx=0.55, rely=0.6, relwidth=0.35, relheight=0.1)
            
            # accion
            self.label_action = ctk.CTkLabel(master=self, text="Acción en proceso:", font=master.fonts[0])
            self.label_action.place(anchor="nw", relx=0.1, rely=0.75)
            self.label_state = ctk.CTkLabel(master=self, textvariable=self.text_state, font=master.fonts[2])
            self.label_state.place(anchor="nw", relx=0.15, rely=0.85)
        
        # funcion de boton de envio
        def button_send_pressed(self):
            pass
        
        # funcion de boton de detener
        def button_stop_pressed(self):
            pass
            
    # === clase de seccion de parametros del rover (frame) ===
    class RoverParams(ctk.CTkFrame):
        # parametros de la clase
        def __init__(self, master):
            # inicializacion del frame
            super().__init__(master, corner_radius=32, fg_color= c.white[0], border_width=4, border_color=c.black[1])
            
            # variables de texto
            self.text_env_temp = ctk.StringVar(value="(min, max)") # === modificar por valores de electronica ===
            self.text_bat_temp = ctk.StringVar(value="(5 \u00b0C, 30 \u00b0C)")
            self.text_lidar_temp = ctk.StringVar(value="(min, max)")
            
            #variables de valor
            self.val_env_temp = ctk.StringVar(value="null")
            self.val_bat_temp = ctk.StringVar(value="null")
            self.val_lidar_temp = ctk.StringVar(value="null")
            
            # temperatura
            self.label_param = ctk.CTkLabel(master=self, text="Temperatura", font=master.fonts[0])
            self.label_param.grid(row=0, column=0)
            
            # actual
            self.label_current = ctk.CTkLabel(master=self, text="Actual", font=master.fonts[0])
            self.label_current.grid(row=0, column=1)
            
            # normal
            self.label_normal = ctk.CTkLabel(master=self, text="Normal", font=master.fonts[0])
            self.label_normal.grid(row=0, column=2)
            
            # ambiente
            self.label_env_temp = ctk.CTkLabel(master=self, text="Ambiente [\u00b0C]", font=master.fonts[1])
            self.label_env_temp.grid(row=1, column=0)
            self.label_env_temp_current = ctk.CTkLabel(master=self, textvariable=self.val_env_temp, font=master.fonts[2])
            self.label_env_temp_current.grid(row=1, column=1)
            self.label_env_temp_normal = ctk.CTkLabel(master=self, textvariable=self.text_env_temp, font=master.fonts[1])
            self.label_env_temp_normal.grid(row=1, column=2)
            
            # bateria
            self.label_bat_temp = ctk.CTkLabel(master=self, text="Batería [\u00b0C]", font=master.fonts[1])
            self.label_bat_temp.grid(row=2, column=0)
            self.label_bat_temp_current = ctk.CTkLabel(master=self, textvariable=self.val_bat_temp, font=master.fonts[2])
            self.label_bat_temp_current.grid(row=2, column=1)
            self.label_bat_temp_normal = ctk.CTkLabel(master=self, textvariable=self.text_bat_temp, font=master.fonts[1])
            self.label_bat_temp_normal.grid(row=2, column=2)
            
            # lidar
            self.label_lidar_temp = ctk.CTkLabel(master=self, text="LiDAR [\u00b0C]", font=master.fonts[1])
            self.label_lidar_temp.grid(row=3, column=0)
            self.label_lidar_temp_current = ctk.CTkLabel(master=self, textvariable=self.val_lidar_temp, font=master.fonts[2])
            self.label_lidar_temp_current.grid(row=3, column=1)
            self.label_lidar_temp_normal = ctk.CTkLabel(master=self, textvariable=self.text_lidar_temp, font=master.fonts[1])
            self.label_lidar_temp_normal.grid(row=3, column=2)
        
        # funcion para establecer el valor de ambiente
        def set_ambiente(self, valor="null"):
            if type(valor) != "str":
                str(valor)
            self.val_env_temp.set(valor)
        
        # funcion para establecer el valor de bateria
        def set_bateria(self, valor="null"):
            if type(valor) != "str":
                str(valor)
            self.val_bat_temp.set(valor)
    
    # === clase de seccion de parametros de rasputin (frame) ===        
    class RaspParams(ctk.CTkFrame):
        # parametros de la clase
        def __init__(self, master):
            # inicializacion del frame
            super().__init__(master, corner_radius=32, fg_color=c.white[0], border_width=4, border_color=c.black[1])
            
            # variables de texto
            self.text_cpu = ctk.StringVar(value="(0%, 70%)")
            self.text_ram = ctk.StringVar(value="(1250 MB, 3000 MB)")
            self.text_temp = ctk.StringVar(value="(20 \u00b0C, 70 \u00b0C)")
            
            # variables de valor
            self.val_cpu = ctk.StringVar(value="null")
            self.val_ram = ctk.StringVar(value="null")
            self.val_temp = ctk.StringVar(value="null")
            
            # parametro
            self.label_param = ctk.CTkLabel(master=self, text="Parámetro", font=master.fonts[0], text_color=c.black[1])
            self.label_param.grid(row=0, column=0)
            
            # actual
            self.label_param = ctk.CTkLabel(master=self, text="Actual", font=master.fonts[0], text_color=c.black[1])
            self.label_param.grid(row=0, column=1)
            
            # normal
            self.label_param = ctk.CTkLabel(master=self, text="Normal", font=master.fonts[0], text_color=c.black[1])
            self.label_param.grid(row=0, column=2)
            
            # cpu
            self.label_cpu = ctk.CTkLabel(master=self, text="CPU [%]", font=master.fonts[1], text_color=c.black[1])
            self.label_cpu.grid(row=2, column=0)
            self.label_cpu_current = ctk.CTkLabel(master=self, textvariable=self.val_cpu, font=master.fonts[2], text_color=c.black[1])
            self.label_cpu_current.grid(row=2, column=1)
            self.label_cpu_normal = ctk.CTkLabel(master=self, textvariable=self.text_cpu, font=master.fonts[1], text_color=c.black[1])
            self.label_cpu_normal.grid(row=2, column=2)
            
            # ram
            self.label_ram = ctk.CTkLabel(master=self, text="RAM [MB]", font=master.fonts[1], text_color=c.black[1])
            self.label_ram.grid(row=3, column=0)
            self.label_ram_current = ctk.CTkLabel(master=self, textvariable=self.val_ram, font=master.fonts[2], text_color=c.black[1])
            self.label_ram_current.grid(row=3, column=1)
            self.label_ram_normal = ctk.CTkLabel(master=self, textvariable=self.text_ram, font=master.fonts[1], text_color=c.black[1])
            self.label_ram_normal.grid(row=3, column=2)
            
            # temperatura
            self.label_temp = ctk.CTkLabel(master=self, text="Temperatura [\u00b0C]", font=master.fonts[1], text_color=c.black[1])
            self.label_temp.grid(row=1, column=0)
            self.label_temp_current = ctk.CTkLabel(master=self, textvariable=self.val_temp, font=master.fonts[2], text_color=c.black[1])
            self.label_temp_current.grid(row=1, column=1)
            self.label_temp_normal = ctk.CTkLabel(master=self, textvariable=self.text_temp, font=master.fonts[1], text_color=c.black[1])
            self.label_temp_normal.grid(row=1, column=2)
            
         # funcion para establecer el valor de ambiente
        def set_cpu(self, valor="null"):
            if type(valor) != "str":
                str(valor)
            self.val_cpu.set(valor)
        
        # funcion para establecer el valor de bateria
        def set_ram(self, valor="null"):
            if type(valor) != "str":
                str(valor)
            self.val_ram.set(valor)
    
    # === clase de seccion de parametros del mapa (frame) ===        
    class MapParams(ctk.CTkFrame):
        # parametros de la clase
        def __init__(self, master):
            # inicializacion del frame
            super().__init__(master, corner_radius=32, fg_color= c.white[0], border_width=4, border_color=c.black[1])
            
            # variables de valor
            self.val_speed = ctk.StringVar(value="null")
            self.val_angular_ac_x = ctk.StringVar(value="null")
            self.val_angular_ac_y = ctk.StringVar(value="null")
            self.val_angular_ac_z = ctk.StringVar(value="null")
            
            # velocidad
            self.label_speed = ctk.CTkLabel(master=self, text="Velocidad [m/s]:", font=master.fonts[0])
            self.label_speed.grid(row=0, column=0)
            self.label_speed_current = ctk.CTkLabel(master=self, textvariable=self.val_speed, font=master.fonts[2])
            self.label_speed_current.grid(row=0, column=1, sticky="w")
            
            # aceleracion angular
            self.label_angular_ac = ctk.CTkLabel(master=self, text="Ac. angular [rad/s\u00b2]", font=master.fonts[0])
            self.label_angular_ac.grid(row=1, column=1)
            
            # eje x
            self.label_angular_ac_x = ctk.CTkLabel(master=self, text="Eje x", font=master.fonts[1])
            self.label_angular_ac_x.grid(row=2, column=0)
            self.label_angular_ac_x_current = ctk.CTkLabel(master=self, textvariable=self.val_angular_ac_x, font=master.fonts[2])
            self.label_angular_ac_x_current.grid(row=3, column=0)
            
            # eje y
            self.label_angular_ac_y = ctk.CTkLabel(master=self, text="Eje y", font=master.fonts[1])
            self.label_angular_ac_y.grid(row=2, column=1)
            self.label_angular_ac_y_current = ctk.CTkLabel(master=self, textvariable=self.val_angular_ac_y, font=master.fonts[2])
            self.label_angular_ac_y_current.grid(row=3, column=1)
            
            # eje z
            self.label_angular_ac_z = ctk.CTkLabel(master=self, text="Eje z", font=master.fonts[1])
            self.label_angular_ac_z.grid(row=2, column=2)
            self.label_angular_ac_z_current = ctk.CTkLabel(master=self, textvariable=self.val_angular_ac_z, font=master.fonts[2])
            self.label_angular_ac_z_current.grid(row=3, column=2)
        
        # funcion para establecer valor de velocidad
        def set_velocidad(self, valor="null"):
            if type(valor) != "str":
                str(valor)
            self.val_speed.set(valor)
        
        # funcion para establecer valor de aceleracion en x
        def set_aceleracion_x(self, valor="null"):
            if type(valor) != "str":
                str(valor)
            self.val_angular_ac_x.set(valor)
        
        # funcion para establecer valor de aceleracion en y
        def set_aceleracion_y(self, valor="null"):
            if type(valor) != "str":
                str(valor)
            self.val_angular_ac_y.set(valor)
        
        # funcion para establecer valor de aceleracion en z
        def set_aceleracion_z(self, valor="null"):
            if type(valor) != "str":
                str(valor)
            self.val_angular_ac_z.set(valor)
            
    class Map(ctk.CTkFrame):
        def __init__(self, master):
            super().__init__(master, corner_radius=32,  fg_color= c.white[0], border_width=4, border_color=c.black[1])
            
            # variables de valor
            self.val_time = ctk.StringVar(value="null")
            self.val_voltage = ctk.StringVar(value="null")
            self.val_current = ctk.StringVar(value="null")
            
            # boton iniciar mision
            self.button_start_mission = ctk.CTkButton(master=self, corner_radius=8, text="Iniciar misión", font=master.fonts[0], border_width=4, border_color=c.green[1], fg_color=c.green[2], hover_color=c.green[3], command=self.button_start_pressed)
            self.button_start_mission.place(anchor="nw", relx=0.04, rely=0.04, relwidth=0.25, relheight=0.08)
            
            # boton detener mision
            self.button_stop_mission = ctk.CTkButton(master=self, corner_radius=8, text="Detener misión", font=master.fonts[0], border_width=4, border_color=c.red[1], fg_color=c.red[2], hover_color=c.red[3], command=self.button_stop_pressed)
            self.button_stop_mission.place(anchor="nw", relx=0.33, rely=0.04, relwidth=0.25, relheight=0.08)
            
            # tiempo de ejecucion
            self.label_time = ctk.CTkLabel(master=self, text="Tiempo [s]:", font=master.fonts[0], text_color=c.black[1])
            self.label_time.place(relx=0.65, rely=0.9)
            self.label_time_current = ctk.CTkLabel(master=self, textvariable=self.val_time, font=master.fonts[2], text_color=c.black[1], justify="left")
            self.label_time_current.place(relx=0.8, rely=0.9)
            
            # voltaje de bateria
            self.label_voltage = ctk.CTkLabel(master=self, text="Voltaje [V]:", font=master.fonts[0], text_color=c.black[1])
            self.label_voltage.place(relx=0.65, rely=0.05)
            self.label_voltage_current = ctk.CTkLabel(master=self, textvariable=self.val_time, font=master.fonts[2], text_color=c.black[1], justify="left")
            self.label_voltage_current.place(relx=0.8, rely=0.05)
            
            # corriente de bateria
            self.label_current = ctk.CTkLabel(master=self, text="Corriente [A]:", font=master.fonts[0], text_color=c.black[1])
            self.label_current.place(relx=0.65, rely=0.1)
            self.label_current_current = ctk.CTkLabel(master=self, textvariable=self.val_time, font=master.fonts[2], text_color=c.black[1], justify="left")
            self.label_current_current.place(relx=0.82, rely=0.1)
            
        def button_start_pressed(self):
            pass
            
        def button_stop_pressed(self):
            pass
            
        # funcion para establecer valor de tiempo
        def set_time(self, valor="null"):
            if type(valor) != "str":
                str(valor)
            self.val_time.set(valor)
            
        # funcion para establecer valor de voltaje
        def set_time(self, valor="null"):
            if type(valor) != "str":
                str(valor)
            self.val_voltage.set(valor)
            
        # funcion para establecer valor de corriente
        def set_time(self, valor="null"):
            if type(valor) != "str":
                str(valor)
            self.val_current.set(valor)

app = App()
app.mainloop()
