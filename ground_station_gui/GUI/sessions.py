"""Sesiones de adquisicion: registro en disco, consulta y exportacion (CSV / Excel).

Cada sesion vive en su propia carpeta dentro de `ground_station_gui/lecturas/`:
    <id>/meta.json   nombre, fechas, estado, duracion, numero de lecturas
    <id>/datos.csv   una fila por lectura; se escribe y se vuelca a disco al momento

Este modulo no depende de la interfaz grafica (se puede probar sin abrir la ventana).
"""
import csv
import json
import os
import re
import time
from datetime import datetime

import numpy as np

from telemetry import CHANNELS

DEFAULT_DIR = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "lecturas"))
EXPORT_DIRNAME = "exportados"

RECORDING, PAUSED, FINISHED, INTERRUPTED = "registrando", "pausada", "finalizada", "interrumpida"
META_EVERY = 25  # filas entre actualizaciones de meta.json mientras se registra


def stamp(epoch, millis=False):
    text = datetime.fromtimestamp(epoch).strftime("%Y-%m-%d %H:%M:%S.%f")
    return text[:-3] if millis else text[:-7]


def fmt_duration(seconds):
    seconds = int(max(0, seconds or 0))
    return f"{seconds // 3600:02d}:{seconds % 3600 // 60:02d}:{seconds % 60:02d}"


def slug(name):
    """Nombre valido para archivos: sin caracteres prohibidos y con guiones bajos."""
    name = re.sub(r'[<>:"/\\|?*\x00-\x1f]', " ", name)
    return re.sub(r"\s+", "_", name.strip(" ._")) or "sesion"


class Stats:
    """Estadisticas acumuladas de una variable (ignora las lecturas sin dato)."""
    __slots__ = ("n", "total", "lo", "hi")

    def __init__(self):
        self.n, self.total, self.lo, self.hi = 0, 0.0, None, None

    def add(self, value):
        if value is None:
            return
        self.n += 1
        self.total += value
        self.lo = value if self.lo is None else min(self.lo, value)
        self.hi = value if self.hi is None else max(self.hi, value)

    @property
    def mean(self):
        return self.total / self.n if self.n else None


class DataSet:
    """Lecturas en memoria: tiempo relativo, hora real y una columna por canal."""

    def __init__(self, n_channels=len(CHANNELS)):
        self.n_channels = n_channels
        self.clear()

    def clear(self):
        self.t0 = None
        self.t, self.epoch = [], []
        self.cols = [[] for _ in range(self.n_channels)]
        self.stats = [Stats() for _ in range(self.n_channels)]
        self._np = np.empty((self.n_channels + 1, 1024))  # copia para graficar: fila 0 = tiempo, NaN = sin dato

    def __len__(self):
        return len(self.t)

    def append(self, t, epoch, values):
        n = len(self.t)
        if n == self._np.shape[1]:
            self._np = np.concatenate((self._np, np.empty_like(self._np)), axis=1)  # duplica la capacidad
        self._np[0, n] = t
        self._np[1:, n] = [np.nan if v is None else v for v in values]
        self.t.append(t)
        self.epoch.append(epoch)
        for col, stat, value in zip(self.cols, self.stats, values):
            col.append(value)
            stat.add(value)

    def trim(self, count):
        """Descarta las `count` lecturas mas antiguas (solo para el buffer en vivo)."""
        n = len(self.t)
        self._np[:, :n - count] = self._np[:, count:n].copy()
        del self.t[:count], self.epoch[:count]
        for col in self.cols:
            del col[:count]

    def times(self):
        """Tiempos como arreglo de numpy (para las graficas)."""
        return self._np[0, :len(self.t)]

    def values(self, i):
        """Lecturas del canal i como arreglo de numpy; NaN donde no hubo dato."""
        return self._np[i + 1, :len(self.t)]

    def last(self, i):
        for value in reversed(self.cols[i][-500:]):
            if value is not None:
                return value
        return None


class Session(DataSet):
    """Una sesion de medicion con su carpeta propia."""

    def __init__(self, folder, meta):
        super().__init__()
        self.folder, self.meta = folder, meta
        self._file = self._writer = None
        self._active_from = None  # inicio del tramo activo en curso
        self._active_done = meta.get("active_s") or 0.0  # tiempo activo de los tramos ya cerrados

    id = property(lambda self: self.meta["id"])
    name = property(lambda self: self.meta["name"])
    state = property(lambda self: self.meta["state"])
    period_ms = property(lambda self: self.meta.get("period_ms", 500))
    data_path = property(lambda self: os.path.join(self.folder, "datos.csv"))
    active = property(lambda self: self._file is not None)

    # --- ciclo de vida -------------------------------------------------------
    def start(self, now=None):
        now = time.time() if now is None else now
        self._file = open(self.data_path, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(["t_s", "epoch", "timestamp"] + [ch.key for ch in CHANNELS])
        self._file.flush()
        self.meta.update(state=RECORDING, started_epoch=now, started_at=stamp(now),
                         ended_epoch=None, ended_at=None, active_s=0.0, n_rows=0)
        self._active_from, self._active_done = now, 0.0
        self.save_meta()

    def pause(self, now=None):
        if self.active and self.state == RECORDING:
            now = time.time() if now is None else now
            self._close_stretch(now)
            self.meta["state"] = PAUSED
            self.save_meta()

    def resume(self, now=None):
        if self.active and self.state == PAUSED:
            self._active_from = time.time() if now is None else now
            self.meta["state"] = RECORDING
            self.save_meta()

    def stop(self, now=None):
        if not self.active:
            return
        now = time.time() if now is None else now
        self._close_stretch(now)
        try:
            self._file.close()
        finally:
            self._file = self._writer = None
        self.meta.update(state=FINISHED, ended_epoch=now, ended_at=stamp(now), n_rows=len(self))
        self.save_meta()

    def append(self, now, values):
        """Registra una lectura (solo mientras se esta registrando). True si se guardo."""
        if not self.active or self.state != RECORDING:
            return False
        t = now - self.meta["started_epoch"]
        self._writer.writerow([f"{t:.3f}", f"{now:.3f}", stamp(now, millis=True)]
                              + ["" if v is None else repr(v) for v in values])
        self._file.flush()  # cada lectura queda en disco aunque la aplicacion se cierre
        super().append(t, now, values)
        self.meta["n_rows"] = len(self)
        if len(self) % META_EVERY == 0:
            self.meta["active_s"] = self.active_seconds(now)
            self.save_meta()
        return True

    # --- consulta ------------------------------------------------------------
    def active_seconds(self, now=None):
        """Tiempo efectivo de adquisicion (no cuenta las pausas)."""
        total = self._active_done
        if self._active_from is not None:
            total += (time.time() if now is None else now) - self._active_from
        return total

    def _close_stretch(self, now):
        self._active_done = self.meta["active_s"] = self.active_seconds(now)
        self._active_from = None

    def total_seconds(self, now=None):
        """Duracion total entre inicio y fin (o hasta ahora si sigue abierta)."""
        start = self.meta.get("started_epoch")
        if start is None:
            return 0.0
        end = self.meta.get("ended_epoch")
        if end is None:
            end = (time.time() if now is None else now) if self.active else (self.epoch[-1] if self.epoch else start)
        return end - start

    def save_meta(self):
        tmp = os.path.join(self.folder, "meta.json.tmp")
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(self.meta, f, ensure_ascii=False, indent=2)
        os.replace(tmp, os.path.join(self.folder, "meta.json"))

    def summary(self):
        """Pares (campo, valor) con la informacion general de la sesion."""
        m = self.meta
        return [
            ("Sesión", m["name"]), ("Identificador", m["id"]), ("Estado", m["state"]),
            ("Inicio", m.get("started_at") or ""),
            ("Fin", m.get("ended_at") or (stamp(self.epoch[-1]) if self.epoch and not self.active else "")),
            ("Duración total", fmt_duration(self.total_seconds())),
            ("Tiempo efectivo de adquisición", fmt_duration(self.active_seconds())),
            ("Lecturas", len(self)),
            ("Periodo de registro (ms)", self.period_ms),
        ]

    def load_data(self):
        """Lee datos.csv completo. Tolera una ultima fila truncada por un corte."""
        self.clear()
        if not os.path.exists(self.data_path):
            return
        with open(self.data_path, newline="", encoding="utf-8") as f:
            lines = f.read().split("\n")
        lines.pop()  # tras el ultimo salto de linea no hay nada, o hay una fila a medio escribir
        reader = csv.reader(lines)
        header = next(reader, None)
        if header:
            index = [header.index(ch.key) if ch.key in header else None for ch in CHANNELS]
            for row in reader:
                if len(row) != len(header):
                    continue
                try:
                    t, epoch = float(row[0]), float(row[1])
                except ValueError:
                    continue
                values = []
                for i in index:
                    cell = row[i] if i is not None and i < len(row) else ""
                    try:
                        values.append(float(cell) if cell != "" else None)
                    except ValueError:
                        values.append(None)
                super().append(t, epoch, values)


class SessionStore:
    """Carpeta con todas las sesiones guardadas."""

    def __init__(self, folder=DEFAULT_DIR):
        self.folder = os.path.abspath(folder)

    @property
    def export_dir(self):
        return os.path.join(self.folder, EXPORT_DIRNAME)

    def create(self, name="", period_ms=500, now=None):
        """Crea una sesion nueva en una carpeta propia; nunca reutiliza una existente."""
        now = time.time() if now is None else now
        name = name.strip() or "Sesión " + stamp(now)
        base = datetime.fromtimestamp(now).strftime("%Y%m%d_%H%M%S_") + slug(name)[:40]
        session_id, n = base, 2
        os.makedirs(self.folder, exist_ok=True)
        while os.path.exists(os.path.join(self.folder, session_id)):
            session_id, n = f"{base}_{n}", n + 1
        folder = os.path.join(self.folder, session_id)
        os.makedirs(folder)
        meta = {
            "id": session_id, "name": name, "state": RECORDING, "period_ms": period_ms,
            "channels": [{"key": ch.key, "label": ch.label, "unit": ch.unit} for ch in CHANNELS],
        }
        session = Session(folder, meta)
        session.start(now)
        return session

    def _read_meta(self, session_id):
        with open(os.path.join(self.folder, session_id, "meta.json"), encoding="utf-8") as f:
            return json.load(f)

    def list(self, active_id=None):
        """Metadatos de las sesiones guardadas, de la mas reciente a la mas antigua."""
        found = []
        if not os.path.isdir(self.folder):
            return found
        for entry in os.listdir(self.folder):
            try:
                meta = self._read_meta(entry)
            except (OSError, ValueError):
                continue
            if meta.get("state") in (RECORDING, PAUSED) and meta.get("id") != active_id:
                meta["state"] = INTERRUPTED  # la aplicacion se cerro sin detener la sesion
            found.append(meta)
        found.sort(key=lambda m: m.get("started_epoch") or 0, reverse=True)
        return found

    def load(self, session_id):
        """Abre una sesion guardada con todas sus lecturas (solo consulta)."""
        folder = os.path.join(self.folder, session_id)
        session = Session(folder, self._read_meta(session_id))
        session.load_data()
        meta = session.meta
        if meta.get("state") in (RECORDING, PAUSED):
            meta["state"] = INTERRUPTED
        meta["n_rows"] = len(session)
        if meta.get("ended_epoch") is None and session.epoch:
            meta["ended_epoch"], meta["ended_at"] = session.epoch[-1], stamp(session.epoch[-1])
        return session


# === exportacion ==============================================================
def export_name(session, extension):
    """Ej.: Prueba_Temperatura_Ambiental_2026-11-10.csv"""
    date = (session.meta.get("started_at") or stamp(time.time()))[:10]
    return f"{slug(session.name)}_{date}{extension}"


def _headers():
    return ["Timestamp", "Tiempo (s)"] + [f"{ch.column} ({ch.unit})" for ch in CHANNELS]


def export_csv(session, path):
    """Todas las lecturas de la sesion. Las lineas de informacion general empiezan con '#'."""
    with open(path, "w", newline="", encoding="utf-8-sig") as f:  # utf-8-sig: Excel lee los acentos
        writer = csv.writer(f)
        for field, value in session.summary():
            writer.writerow([f"# {field}", value])
        writer.writerow(_headers())
        for row in range(len(session)):
            writer.writerow([stamp(session.epoch[row], millis=True), f"{session.t[row]:.3f}"]
                            + ["" if col[row] is None else repr(col[row]) for col in session.cols])
    return path


def export_xlsx(session, path):
    """Libro con tres hojas: informacion de la sesion, datos y estadisticas."""
    from openpyxl import Workbook  # se importa aqui: solo hace falta al exportar a Excel
    from openpyxl.styles import Font

    bold = Font(bold=True)
    book = Workbook()

    info = book.active
    info.title = "Sesión"
    for field, value in session.summary():
        info.append([field, value])
    for cell in info["A"]:
        cell.font = bold
    info.column_dimensions["A"].width, info.column_dimensions["B"].width = 34, 44

    data = book.create_sheet("Datos")
    data.append(_headers())
    for row in range(len(session)):
        data.append([stamp(session.epoch[row], millis=True), round(session.t[row], 3)]
                    + [col[row] for col in session.cols])  # None -> celda vacia
    data.freeze_panes = "A2"

    stats = book.create_sheet("Estadísticas")
    stats.append(["Variable", "Unidad", "Lecturas válidas", "Sin dato", "Mínimo", "Máximo", "Promedio"])
    for ch, st in zip(CHANNELS, session.stats):
        stats.append([ch.label, ch.unit, st.n, len(session) - st.n, st.lo, st.hi, st.mean])

    for sheet in (data, stats):
        for cell in sheet[1]:
            cell.font = bold
        for column in sheet.columns:
            sheet.column_dimensions[column[0].column_letter].width = max(14, len(str(column[0].value)) + 3)
    book.save(path)
    return path
