import os
import sys
import threading
import time

import customtkinter as ctk
import numpy as np
from PIL import Image

import interface
from gs_serial_sender import GroundStationSerial

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from gs_map_decoder import HDR_LEN, MAX_PAYLOAD, MapDecoder, xor_checksum  # noqa: E402

GS_PORT = os.environ.get("GS_PORT", "/dev/ttyUSB0")  # puerto USB del ESP32-GS
MAP_BOX = (900, 560)                                  # área máxima del mapa [px]
SHADE = np.array([128, 255, 0], dtype=np.uint8)       # desconocido=gris, libre=blanco, ocupado=negro


def _frame_end(buf, i):
    """Fin del Packet 3 que empieza en i: >0 válido, 0 incompleto, -1 inválido."""
    if len(buf) - i < HDR_LEN:
        return 0
    plen = buf[i + 5]
    if plen > MAX_PAYLOAD:
        return -1
    end = i + HDR_LEN + plen + 1
    if end > len(buf):
        return 0
    return end if xor_checksum(buf[i:end - 1]) == buf[end - 1] else -1


def extract_frames(buf: bytearray):
    """Saca de buf los Packet 3 completos y válidos (0xAA 0x77 ...)."""
    frames = []
    while True:
        i = buf.find(b"\xAA\x77")
        if i < 0:
            del buf[:max(0, len(buf) - 1)]  # conservar posible 0xAA final
            return frames
        end = _frame_end(buf, i)
        if end == 0:
            # Incompleto: si más adelante hay un frame válido, i era un falso sync
            j = buf.find(b"\xAA\x77", i + 1)
            while j >= 0 and _frame_end(buf, j) <= 0:
                j = buf.find(b"\xAA\x77", j + 1)
            if j < 0:
                del buf[:i]
                return frames
            del buf[:j]
            continue
        if end < 0:
            del buf[:i + 1]  # falso sync (p. ej. texto de depuración): resincronizar
            continue
        frames.append(bytes(buf[i:end]))
        del buf[:end]


def show_map(label, grid, meta):
    img = Image.fromarray(np.flipud(SHADE[np.clip(grid, 0, 2)]), "L")  # fila 0 = y máx
    h, w = grid.shape
    k = max(1, int(min(MAP_BOX[0] / w, MAP_BOX[1] / h)))
    img = img.resize((w * k, h * k), Image.NEAREST)
    label.image = ctk.CTkImage(light_image=img, dark_image=img, size=img.size)
    label.configure(image=label.image,
                    text=f"{w}x{h} @ {meta['resolution_mm']} mm/celda", compound="top")


def serial_reader(gs, app, label):
    dec, buf = MapDecoder(), bytearray()
    while gs.ser is not None:
        try:
            data = gs.ser.read(512)
        except Exception:
            app.after(0, lambda: label.configure(text="Puerto serial desconectado"))
            return
        if not data:
            time.sleep(0.01)
            dec.check_timeout()
            continue
        buf.extend(data)
        for frame in extract_frames(buf):
            res = dec.parse_packet(frame)
            if res:
                app.after(0, show_map, label, *res)
        # TODO: decodificar aquí los paquetes de telemetría cuando Telecom defina su formato


if __name__ == "__main__":
    # instancia de aplicacion
    app = interface.App(title="Interface", size=(1920, 1080))
    # PARA DIEGO que realizara la simulacion e inyeccion de datos a la interfaz
    # Utiliza las siguientes funciones y sustituye "a" por el valor que quieras asignar a la variable de la interfaz, ya sea int, float o str.
    # === app.rover_params.set_ambiente(a) ===
    # === app.rover_params.set_bateria(a) ===
    # === app.map_params.set_velocidad(a) ===
    # === app.map_params.set_aceleracion_x(a) ===
    # === app.map_params.set_aceleracion_y(a) ===
    # === app.map_params.set_aceleracion_z(a) ===
    # TODO: asignar temperaturas (BME688/DS18B20), velocidad y aceleración (BNO085) al llegar su telemetría

    gs = GroundStationSerial(port=GS_PORT)
    ins = app.instructions

    # Etiqueta del mapa dentro del frame existente
    map_label = ctk.CTkLabel(master=app.map, text="Esperando mapa...", font=app.fonts[1])
    map_label.pack(expand=True, fill="both", padx=24, pady=24)

    def on_send():
        try:
            d = float(ins.entry_distance.get())
        except ValueError:
            d = 0.0
        if d <= 0:
            ins.text_state.set("Distancia inválida")
            return
        gs.send_advance(d)  # TODO: la dirección (entry_direction) aún no existe en Packet 4
        ins.text_state.set(f"Avanzar {d:.2f} m")

    def on_stop():
        gs.send_stop()
        ins.text_state.set("Detenido")

    ins.button_send.configure(command=on_send)
    ins.button_stop.configure(command=on_stop)

    if gs.ser is None:
        map_label.configure(text=f"Sin ESP32-GS en {GS_PORT} (dry-run)")
    else:
        threading.Thread(target=serial_reader, args=(gs, app, map_label), daemon=True).start()

    app.mainloop()
