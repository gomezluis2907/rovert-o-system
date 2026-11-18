import math
import os
import struct
import sys
import threading
import time

import customtkinter as ctk
import numpy as np
from PIL import Image

import interface
from gs_serial_sender import GroundStationSerial

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import gs_map_decoder  # noqa: E402
from gs_map_decoder import HDR_LEN, MAX_PAYLOAD, MapDecoder, xor_checksum  # noqa: E402

GS_PORT = os.environ.get("GS_PORT", "/dev/ttyUSB0")  # puerto USB del ESP32-GS
MAP_BOX = (900, 560)                                  # área máxima del mapa [px]
SHADE = np.array([128, 255, 0], dtype=np.uint8)       # desconocido=gris, libre=blanco, ocupado=negro

# Identificadores y formatos según docs/ICD.md
SYNC = 0xAA
ID_NAV = 0x02
ID_POWER = 0x03
ID_THERMAL = 0x04
ID_HEALTH = 0x05
ID_MAP = 0x06
ID_MAP_LEGACY = 0x77

NAV_FMT = "<BBBIfffffffB"     # 36 B
POWER_FMT = "<BBBIffBB"       # 17 B
THERMAL_FMT = "<BBBIfffBB"    # 21 B
HEALTH_FMT = "<BBBIBBBBHBB"   # 15 B

FIXED_FRAME_LENS = {
    ID_NAV: struct.calcsize(NAV_FMT),
    ID_POWER: struct.calcsize(POWER_FMT),
    ID_THERMAL: struct.calcsize(THERMAL_FMT),
    ID_HEALTH: struct.calcsize(HEALTH_FMT),
}

ROVER_STATUS_TEXT = {
    0: "En espera (Standby)",
    1: "Ejecutando recorrido",
    2: "Evadiendo obstáculo",
    3: "Meta alcanzada",
}

# Regla de "3 periodos" de docs/ICD.md (sección 2):
# NAV (10 Hz -> 0.3 s), POWER (1 Hz -> 3.0 s), HEALTH (0.5 Hz -> 6.0 s), THERMAL (0.1 Hz -> 30.0 s)
STALE_TIMEOUT_S = {
    ID_NAV: 0.3,
    ID_POWER: 3.0,
    ID_HEALTH: 6.0,
    ID_THERMAL: 30.0,
}
LINK_LOST_TIMEOUT_S = 6.0  # > 3 periodos de HEALTH (o sin ninguna trama del rover)
MAX_DISTANCE_M = 100.0
MAX_ANGLE_DEG = 360.0


def _frame_end(buf, i):
    """Fin de la trama ICD que empieza en i: >0 válido, 0 incompleto, -1 inválido."""
    rem = len(buf) - i
    if rem < 2:
        return 0
    if buf[i] != SYNC:
        return -1
    pkt_id = buf[i + 1]
    if pkt_id in FIXED_FRAME_LENS:
        end = i + FIXED_FRAME_LENS[pkt_id]
        if end > len(buf):
            return 0
        return end if xor_checksum(buf[i:end - 1]) == buf[end - 1] else -1
    if pkt_id in (ID_MAP, ID_MAP_LEGACY):
        if rem < HDR_LEN:
            return 0
        plen = buf[i + 5]
        if plen == 0 or plen > MAX_PAYLOAD:
            return -1
        end = i + HDR_LEN + plen + 1
        if end > len(buf):
            return 0
        return end if xor_checksum(buf[i:end - 1]) == buf[end - 1] else -1
    return -1


def extract_frames(buf: bytearray):
    """Extrae de buf todas las tramas ICD completas y válidas (0x02..0x06 y 0x77)."""
    frames = []
    while True:
        i = buf.find(b"\xAA")
        if i < 0:
            buf.clear()
            return frames
        if len(buf) - i < 2:
            del buf[:i]
            return frames
        end = _frame_end(buf, i)
        if end == 0:
            # Incompleto: si más adelante hay una trama válida, i era un falso sync (p. ej. texto debug)
            j = buf.find(b"\xAA", i + 1)
            while j >= 0 and _frame_end(buf, j) <= 0:
                j = buf.find(b"\xAA", j + 1)
            if j < 0:
                del buf[:i]
                return frames
            del buf[:j]
            continue
        if end < 0:
            del buf[:i + 1]
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


def handle_telemetry_frame(frame: bytes, params, thermal_state: dict, now: float = None):
    if now is None:
        now = time.monotonic()
    pkt_id = frame[1]
    by_id = thermal_state.setdefault("by_id", {})
    stale = thermal_state.setdefault("stale", set())
    by_id[pkt_id] = now
    stale.discard(pkt_id)
    thermal_state["last_any_rx"] = now
    thermal_state["link_lost"] = False

    if pkt_id == ID_NAV:
        _, _, _seq, _ts, _yaw, _pitch, _roll, ax, ay, az, vel, _ = struct.unpack(NAV_FMT, frame)
        params.map_params.val_speed.set(f"{vel:.3f}")
        params.map_params.val_angular_ac_x.set(f"{ax:.3f}")
        params.map_params.val_angular_ac_y.set(f"{ay:.3f}")
        params.map_params.val_angular_ac_z.set(f"{az:.3f}")
    elif pkt_id == ID_POWER:
        _, _, _seq, _ts, voltage_v, current_a, _flags, _ = struct.unpack(POWER_FMT, frame)
        params.map.val_voltage.set(f"{voltage_v:.2f}")
        params.map.val_current.set(f"{current_a:.2f}")
    elif pkt_id == ID_THERMAL:
        _, _, _seq, _ts, t_bat, t_pcb, t_lidar, _flags, _ = struct.unpack(THERMAL_FMT, frame)
        params.rover_params.val_bat_temp.set(f"{t_bat:.2f}")
        params.rover_params.val_env_temp.set(f"{t_pcb:.2f}")
        params.rover_params.val_lidar_temp.set(f"{t_lidar:.2f}")
        thermal_state["last_rx"] = now
    elif pkt_id == ID_HEALTH:
        _, _, _seq, _ts, r_status, _obs, cpu_t, cpu_p, ram_mb, h_flags, _ = struct.unpack(
            HEALTH_FMT, frame
        )
        params.rasp_params.val_temp.set(f"{cpu_t}")
        params.rasp_params.val_cpu.set(f"{cpu_p}")
        params.rasp_params.val_ram.set(f"{ram_mb}")
        if h_flags & 0x20:
            params.instructions.text_state.set("Enlace perdido (Standby)")
        elif r_status in ROVER_STATUS_TEXT:
            params.instructions.text_state.set(ROVER_STATUS_TEXT[r_status])


def check_telemetry_staleness(params, thermal_state: dict, now: float = None):
    """Aplica la regla de '3 periodos' (docs/ICD.md) y detecta pérdida de enlace en la GUI."""
    if now is None:
        now = time.monotonic()
    by_id = thermal_state.setdefault("by_id", {})
    stale = thermal_state.setdefault("stale", set())

    # Mantener canales DS18B20 (0.1 Hz -> ventana de 3 periodos = 30 s) activos en Gráficas
    t_th = by_id.get(ID_THERMAL, thermal_state.get("last_rx", 0.0))
    if t_th > 0.0 and (now - t_th) <= STALE_TIMEOUT_S[ID_THERMAL]:
        if (now - thermal_state.get("last_thermal_refresh", 0.0)) >= 1.0:
            thermal_state["last_thermal_refresh"] = now
            params.rover_params.val_env_temp.set(params.rover_params.val_env_temp.get())
            params.rover_params.val_bat_temp.set(params.rover_params.val_bat_temp.get())
            params.rover_params.val_lidar_temp.set(params.rover_params.val_lidar_temp.get())

    # Marcar como "null" los parámetros cuyo paquete supere 3 periodos sin actualizarse
    for pkt_id, timeout_s in STALE_TIMEOUT_S.items():
        ts = by_id.get(pkt_id, 0.0)
        if ts > 0.0 and (now - ts) > timeout_s and pkt_id not in stale:
            stale.add(pkt_id)
            if pkt_id == ID_NAV:
                params.map_params.val_speed.set("null")
                params.map_params.val_angular_ac_x.set("null")
                params.map_params.val_angular_ac_y.set("null")
                params.map_params.val_angular_ac_z.set("null")
            elif pkt_id == ID_POWER:
                params.map.val_voltage.set("null")
                params.map.val_current.set("null")
            elif pkt_id == ID_THERMAL:
                params.rover_params.val_bat_temp.set("null")
                params.rover_params.val_env_temp.set("null")
                params.rover_params.val_lidar_temp.set("null")
            elif pkt_id == ID_HEALTH:
                params.rasp_params.val_temp.set("null")
                params.rasp_params.val_cpu.set("null")
                params.rasp_params.val_ram.set("null")

    # Pérdida de enlace si transcurren > 6 s (3 periodos de HEALTH) sin señal o sin HEALTH
    last_any = thermal_state.get("last_any_rx", 0.0)
    t_health = by_id.get(ID_HEALTH, 0.0)
    link_expired = (
        (last_any > 0.0 and (now - last_any) > LINK_LOST_TIMEOUT_S)
        or (t_health > 0.0 and (now - t_health) > STALE_TIMEOUT_S[ID_HEALTH])
    )
    if link_expired and not thermal_state.get("link_lost", False):
        thermal_state["link_lost"] = True
        params.instructions.text_state.set("Enlace perdido (Standby)")


def serial_reader(gs, app, params, label, thermal_state):
    dec, buf = MapDecoder(), bytearray()
    while gs.ser is not None:
        try:
            data = gs.ser.read(512)
        except Exception:
            gs.link_lost = True  # el dashboard de "Gráficas" lo muestra como desconectado
            app.after(0, lambda: label.configure(text="Puerto serial desconectado"))
            return
        if not data:
            time.sleep(0.01)
            dec.check_timeout()
            continue
        buf.extend(data)
        for frame in extract_frames(buf):
            pkt_id = frame[1]
            if pkt_id in (ID_MAP, ID_MAP_LEGACY):
                thermal_state["last_any_rx"] = time.monotonic()
                gs_map_decoder.SYNC2 = pkt_id
                res = dec.parse_packet(frame)
                if res:
                    app.after(0, show_map, label, *res)
            else:
                app.after(0, handle_telemetry_frame, frame, params, thermal_state)


def start_demo(app, period_ms=200):
    """Inyecta datos simulados en los parámetros (para probar gráficas y grabación sin el rover)."""
    import math
    params, t0 = app.view.parameters, time.time()

    def step():
        t = time.time() - t0
        params.rover_params.val_env_temp.set(f"{24 + 2 * math.sin(t / 9):.2f}")
        params.rover_params.val_bat_temp.set(f"{28 + 3 * math.sin(t / 15 + 1):.2f}")
        params.rover_params.val_lidar_temp.set(f"{26 + 1.5 * math.cos(t / 12):.2f}")
        params.rasp_params.val_temp.set(f"{52 + int(4 * math.sin(t / 10))}")
        params.rasp_params.val_cpu.set(f"{34 + int(8 * math.sin(t / 5))}")
        params.rasp_params.val_ram.set(f"{1480 + int(40 * math.cos(t / 8))}")
        params.map.val_voltage.set(f"{12.1 - 0.1 * math.sin(t / 20):.2f}")
        params.map.val_current.set(f"{0.85 + 0.15 * math.sin(t / 4):.2f}")
        params.map_params.val_speed.set(f"{0.04 + 0.01 * math.sin(t / 3):.3f}")
        params.map_params.val_angular_ac_x.set(f"{0.20 * math.sin(t / 2):.3f}")
        params.map_params.val_angular_ac_y.set(f"{0.15 * math.cos(t / 2.7):.3f}")
        params.map_params.val_angular_ac_z.set(f"{0.05 * math.sin(t / 1.3):.3f}")
        app.after(period_ms, step)

    step()


if __name__ == "__main__":
    app = interface.App(title="Interface", size=(1920, 1080))
    gs = GroundStationSerial(port=GS_PORT)
    params = app.view.parameters  # los frames viven dentro de la pestaña "Parámetros"
    ins = params.instructions

    # Ajustes en tiempo de ejecución sin modificar parameters.py
    params.map.label_voltage_current.configure(textvariable=params.map.val_voltage)
    params.map.label_current_current.configure(textvariable=params.map.val_current)
    params.map_params.label_angular_ac.configure(text="Ac. lineal [m/s\u00b2]")

    # Etiqueta del mapa dentro del frame existente (debajo de los botones/etiquetas de Map)
    map_label = ctk.CTkLabel(master=params.map, text="Esperando mapa...", font=params.fonts[1])
    map_label.pack(expand=True, fill="both", padx=24, pady=(80, 56))
    map_label.lower()

    mission_state = {"running": False, "t0": 0.0}
    thermal_state = {"last_rx": 0.0, "by_id": {}, "stale": set(), "last_any_rx": 0.0, "link_lost": False}

    def _tick_ui():
        now = time.monotonic()
        if mission_state["running"]:
            params.map.val_time.set(f"{int(now - mission_state['t0'])}")
        check_telemetry_staleness(params, thermal_state, now)
        app.after(100, _tick_ui)

    app.after(100, _tick_ui)

    def _parse_float_entry(entry_widget, default=0.0):
        raw = entry_widget.get().strip().replace(",", ".")
        if not raw:
            return default
        val = float(raw)
        if not math.isfinite(val):
            raise ValueError("non-finite float")
        return val

    def on_send():
        try:
            d = _parse_float_entry(ins.entry_distance, 0.0)
            ang = _parse_float_entry(ins.entry_direction, 0.0)
        except ValueError:
            ins.text_state.set("Valor inválido")
            return
        if (
            not (math.isfinite(d) and math.isfinite(ang))
            or d < 0.0
            or d > MAX_DISTANCE_M
            or abs(ang) > MAX_ANGLE_DEG
            or (d == 0.0 and abs(ang) < 1e-3)
        ):
            ins.text_state.set("Distancia/dirección inválida")
            return
        if gs.send_move(d, ang) is None:
            ins.text_state.set("Distancia/dirección inválida")
            return
        if d > 0.0 and abs(ang) >= 1e-3:
            ins.text_state.set(f"Girar {ang:+.1f}° y avanzar {d:.2f} m")
        elif d > 0.0:
            ins.text_state.set(f"Avanzar {d:.2f} m")
        else:
            ins.text_state.set(f"Girar {ang:+.1f}°")

    def on_stop():
        gs.send_stop()
        ins.text_state.set("Detenido (misión activa)")

    def on_start_mission():
        gs.send_start_mission()
        mission_state["running"] = True
        mission_state["t0"] = time.monotonic()
        params.map.val_time.set("0")
        map_label.configure(image=None, text="Misión iniciada. Esperando mapa...")
        ins.text_state.set("Misión iniciada (Standby)")
        if app.view.graphics.session is None or not app.view.graphics.session.active:
            app.view.graphics._start()

    def on_stop_mission():
        gs.send_stop_mission()
        mission_state["running"] = False
        ins.text_state.set("Misión finalizada")
        if app.view.graphics.session is not None and app.view.graphics.session.active:
            app.view.graphics._stop()

    ins.button_send.configure(command=on_send)
    ins.button_stop.configure(command=on_stop)
    params.map.button_start_mission.configure(command=on_start_mission)
    params.map.button_stop_mission.configure(command=on_stop_mission)

    # modo demo: `python main.py --demo` (o GS_DEMO=1) genera lecturas simuladas
    demo = "--demo" in sys.argv or os.environ.get("GS_DEMO") == "1"
    if demo:
        start_demo(app)
    app.view.graphics.set_link(gs, GS_PORT, demo)  # solo para mostrar el estado del enlace

    if gs.ser is None:
        map_label.configure(text=f"Sin ESP32-GS en {GS_PORT} (dry-run)")
    else:
        threading.Thread(
            target=serial_reader,
            args=(gs, app, params, map_label, thermal_state),
            daemon=True,
        ).start()

    app.mainloop()
