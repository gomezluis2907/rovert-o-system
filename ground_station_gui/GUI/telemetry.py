"""Canales de telemetria del dashboard y lectura de los valores de la pestana "Parametros".

La GUI todavia no recibe un paquete de telemetria de sensores (ver TODO en main.py): el
punto de entrada de los datos son los set_*() de parameters.py. SensorHub observa esas
mismas variables, anota cuando llega cada valor y entrega la lectura vigente, de modo que
el dashboard funciona con lo que llegue por ahi (telemetria real o `--demo`).
"""
import math
import time
from dataclasses import dataclass

STALE_S = 5.0  # sin valor nuevo durante este tiempo -> el sensor se considera sin senal

OK, STALE, NO_DATA, SETPOINT = "activo", "sin señal", "sin lectura", "consigna"


@dataclass(frozen=True)
class Channel:
    key: str        # columna en el archivo de la sesion
    label: str      # nombre completo
    short: str      # nombre corto (tarjetas, leyendas)
    column: str     # encabezado al exportar
    unit: str
    color: str
    setpoint: bool = False  # True: valor escrito por el operador, no medido
    normal: tuple = None    # rango "Normal" de la pestana Parametros (min, max); None = sin definir


# Mismas variables y unidades que la pestana "Parametros" (orden: por grupo de la pestana).
CHANNELS = (
    # temperaturas (DS18B20 del rover y CPU de la Raspberry)
    Channel("temp_ambiente", "Temperatura ambiente", "Temp. ambiente", "Temp. ambiente", "°C", "#2563eb"),
    Channel("temp_bateria", "Temperatura de batería", "Temp. batería", "Temp. batería", "°C", "#06b6d4", normal=(5, 30)),
    Channel("temp_lidar", "Temperatura del LiDAR", "Temp. LiDAR", "Temp. LiDAR", "°C", "#f97316"),
    Channel("temp_rasp", "Temperatura de la Raspberry Pi", "Temp. Raspberry", "Temp. Raspberry", "°C", "#e11d48", normal=(20, 70)),
    # bateria (INA226)
    Channel("voltaje", "Voltaje de batería", "Voltaje", "Voltaje", "V", "#ca8a04"),
    Channel("corriente", "Corriente de batería", "Corriente", "Corriente", "A", "#dc2626"),
    # Raspberry Pi
    Channel("cpu", "Uso de CPU de la Raspberry Pi", "CPU", "CPU", "%", "#7c3aed", normal=(0, 70)),
    Channel("ram", "RAM usada de la Raspberry Pi", "RAM", "RAM", "MB", "#db2777", normal=(1250, 3000)),
    # movimiento (odometria y BNO085)
    Channel("velocidad", "Velocidad", "Velocidad", "Velocidad", "m/s", "#10b981"),
    Channel("acel_x", "Aceleración lineal X", "Acel. X", "Aceleración X", "m/s²", "#0ea5e9"),
    Channel("acel_y", "Aceleración lineal Y", "Acel. Y", "Aceleración Y", "m/s²", "#14b8a6"),
    Channel("acel_z", "Aceleración lineal Z", "Acel. Z", "Aceleración Z", "m/s²", "#84cc16"),
    # consignas escritas por el operador
    Channel("distancia", "Distancia (consigna)", "Distancia", "Distancia", "m", "#6366f1", True),
    Channel("direccion", "Dirección (consigna)", "Dirección", "Dirección", "°", "#8b5cf6", True),
)
INDEX = {ch.key: i for i, ch in enumerate(CHANNELS)}  # clave -> posicion en CHANNELS

# Grupos para graficar (pestana "Graficas" y hoja "Graficas" del Excel): (titulo, claves)
GROUPS = (
    ("Temperaturas", ("temp_ambiente", "temp_bateria", "temp_lidar", "temp_rasp")),
    ("Batería: voltaje y corriente", ("voltaje", "corriente")),
    ("Raspberry Pi: CPU y RAM", ("cpu", "ram")),
    ("Velocidad", ("velocidad",)),
    ("Aceleración lineal", ("acel_x", "acel_y", "acel_z")),
    ("Distancia y dirección (consignas)", ("distancia", "direccion")),
)


def to_float(text):
    """Texto de un parametro -> float; None si no hay dato ("null", vacio, no numerico)."""
    try:
        value = float(str(text).strip().replace(",", "."))
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


class SensorHub:
    """Lee los valores vigentes de "Parametros" sin modificar esa pestana."""

    def __init__(self, parameters=None):
        n = len(CHANNELS)
        self._vars = [None] * n      # StringVar de sensores medidos
        self._entries = [None] * n   # campos de consigna
        self.last_write = [None] * n  # hora de llegada del ultimo valor valido
        self.last_rx = None           # hora de llegada del ultimo valor de cualquier sensor
        if parameters is None:
            return
        rover, mapp, ins = parameters.rover_params, parameters.map_params, parameters.instructions
        rasp, power = parameters.rasp_params, parameters.map
        sources = {
            "temp_ambiente": rover.val_env_temp, "temp_bateria": rover.val_bat_temp,
            "temp_lidar": rover.val_lidar_temp, "temp_rasp": rasp.val_temp,
            "voltaje": power.val_voltage, "corriente": power.val_current,
            "cpu": rasp.val_cpu, "ram": rasp.val_ram,
            "velocidad": mapp.val_speed, "acel_x": mapp.val_angular_ac_x,
            "acel_y": mapp.val_angular_ac_y, "acel_z": mapp.val_angular_ac_z,
        }
        entries = {"distancia": ins.entry_distance, "direccion": ins.entry_direction}
        for i, ch in enumerate(CHANNELS):
            if ch.key in sources:
                self._vars[i] = sources[ch.key]
                sources[ch.key].trace_add("write", lambda *_a, i=i: self._written(i))
            else:
                self._entries[i] = entries.get(ch.key)

    def _written(self, i):
        if to_float(self._vars[i].get()) is not None:
            self.last_write[i] = self.last_rx = time.time()

    def status(self, i, now=None):
        """OK / STALE / NO_DATA para sensores; SETPOINT para consignas."""
        if CHANNELS[i].setpoint:
            return SETPOINT
        if self._vars[i] is None or self.last_write[i] is None:
            return NO_DATA
        now = time.time() if now is None else now
        return OK if now - self.last_write[i] <= STALE_S else STALE

    def read(self, now=None):
        """Lectura vigente de cada canal. None = sin lectura (nunca se confunde con 0)."""
        now = time.time() if now is None else now
        values = []
        for i in range(len(CHANNELS)):
            if self._entries[i] is not None:
                values.append(to_float(self._entries[i].get()))
            elif self.status(i, now) == OK:
                values.append(to_float(self._vars[i].get()))
            else:
                values.append(None)  # sin lectura o sin senal: no se repite un valor viejo
        return values

    def counts(self, now=None):
        """(sensores activos, sin senal, total de sensores medidos)."""
        states = [self.status(i, now) for i, ch in enumerate(CHANNELS) if not ch.setpoint]
        return states.count(OK), states.count(STALE), len(states)
