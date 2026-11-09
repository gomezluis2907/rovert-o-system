import customtkinter as ctk
import colors as c

# === clase padre (frame) === 
class Graphics(ctk.CTkFrame):
    # parametros de la clase
    def __init__(self, master):
        # inicializacion del frame 
        super().__init__(master, corner_radius=32, fg_color= c.gray[3], border_width=4, border_color=c.black[1])
        
