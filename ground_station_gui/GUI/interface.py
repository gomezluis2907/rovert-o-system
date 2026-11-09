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
        self.geometry(f"{size[0]}x{size[1]}")
        self.resizable(False, False)
        
        self.view = self.View(master=self)
        self.view.place(relx=0, rely=0, relwidth=1, relheight=1)

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
            self.graphics = g.Graphics(master=self.tab("Gráficas"))
            self.graphics.place(relx=0.05, rely=0.05, relwidth=0.9, relheight=0.85)
