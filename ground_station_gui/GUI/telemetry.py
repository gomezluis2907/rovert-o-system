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


CHANNELS = (
    Channel("temp_ambiente", "Temperatura ambiente", "Temp. ambiente", "Temp. ambiente", "°C", "#2563eb"),
    Channel("temp_bateria", "Temperatura de batería", "Temp. batería", "Temp. batería", "°C", "#06b6d4"),
    Channel("distancia", "Distancia (consigna)", "Distancia", "Distancia", "m", "#6366f1", True),
    Channel("direccion", "Dirección (consigna)", "Dirección", "Dirección", "°", "#8b5cf6", True),
    Channel("velocidad", "Velocidad", "Velocidad", "Velocidad", "m/s", "#10b981"),
    Channel("acel_x", "Aceleración angular X", "Acel. X", "Aceleración X", "rad/s²", "#0ea5e9"),
    Channel("acel_y", "Aceleración angular Y", "Acel. Y", "Aceleración Y", "rad/s²", "#14b8a6"),
    Channel("acel_z", "Aceleración angular Z", "Acel. Z", "Aceleración Z", "rad/s²", "#84cc16"),
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
        sources = {
            "temp_ambiente": rover.val_env_temp, "temp_bateria": rover.val_bat_temp,
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
