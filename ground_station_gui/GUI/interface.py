import customtkinter as ctk
import parameters as p
import graphics as g
import colors as c

class App(ctk.CTk):
    # parametros de la clase
    def __init__(self, title: str="Window", size: tuple=(512, 512)):
        # inicializacion de la aplicacion
        super().__init__(fg_color=c.white[0])
        self.title(title)

        # tamano "restaurado": el pedido, pero sin exceder el 85% de la pantalla actual
        scale = ctk.ScalingTracker.get_window_scaling(self)
        screen_w = int(self.winfo_screenwidth() / scale)
        screen_h = int(self.winfo_screenheight() / scale)
        width = min(size[0], int(screen_w * 0.85))
        height = min(size[1], int(screen_h * 0.85))
        self.geometry(f"{width}x{height}+{(screen_w - width) // 2}+{(screen_h - height) // 2}")
        self.minsize(min(960, width), min(600, height))
        self.resizable(True, True)

        # arranca maximizada para ocupar la pantalla en la que se abra
        # (con retraso: customtkinter reacomoda la ventana al iniciar y anularia el maximizado)
        self.after(300, self._maximize)


        self.view = self.View(master=self)
        self.view.place(relx=0, rely=0, relwidth=1, relheight=1)

    def _maximize(self):
        try:
            self.state("zoomed")  # Windows / macOS
        except Exception:
            try:
                self.attributes("-zoomed", True)  # Linux (X11)
            except Exception:
                pass

    class View(ctk.CTkTabview):
        # parametros de la clase
        def __init__(self, master):
            # inicializacion del tabview
            super().__init__(master, corner_radius=32, anchor="nw", fg_color=c.white[0], border_width=4, border_color=c.black[1])
            
            # creacion de pestanas
            self.add("Parámetros")
            self.add("Gráficas")
            self.set("Parámetros")
            
            # agregado de seccion (frame) de parametros
            self.parameters = p.Parameters(master=self.tab("Parámetros"))
            self.parameters.place(relx=0.05, rely=0.05, relwidth=0.9, relheight=0.85)
            
            # agregado de seccion (frame) de graficas
            self.graphics = g.Graphics(master=self.tab("Gráficas"), parameters=self.parameters)
            self.graphics.place(relx=0, rely=0, relwidth=1, relheight=1)  # el dashboard ocupa toda la pestaña
