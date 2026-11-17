"""Pestana "Graficas": dashboard de telemetria, adquisicion, sesiones y exportacion.

Fuente de datos: las mismas variables que muestra la pestana "Parametros" (telemetry.SensorHub).
El muestreo (adquisicion) y el redibujado (visualizacion) corren en ciclos separados: la
adquisicion no depende de la frecuencia con que se actualizan las graficas.
"""
import os
import subprocess
import sys
import time
from tkinter import filedialog

import customtkinter as ctk

import sessions as ss
from charts import Series, TimeChart, fmt_value
from telemetry import CHANNELS, NO_DATA, OK, SETPOINT, STALE, SensorHub

# paleta propia del dashboard (la de "Parametros" vive en colors.py y no se toca)
BG, CARD, BORDER, SOFT = "#e9eef5", "#ffffff", "#d6deea", "#f1f5f9"
TEXT, MUTED = "#1f2a37", "#6b7a90"
BLUE, BLUE_D, GREEN, AMBER, RED, RED_D, GRAY = "#2563eb", "#1d4ed8", "#16a34a", "#d97706", "#dc2626", "#b91c1c", "#94a3b8"
FAMILY = "Segoe UI"

RENDER_MS = 250                 # refresco visual
LIVE_MAX = 36000                # lecturas que conserva el buffer en vivo (no afecta a las sesiones)
RATES = {"1 Hz": 1000, "2 Hz": 500, "5 Hz": 200, "10 Hz": 100}
WINDOWS = {"30 s": 30.0, "Todo": None}
SMALL_CHARTS = (                # (titulo, canales)
    ("Temperatura", (0, 1)),
    ("Velocidad", (4,)),
    ("Aceleración angular", (5, 6, 7)),
    ("Distancia y dirección (consignas)", (2, 3)),
)
STATUS_COLOR = {OK: GREEN, STALE: RED, NO_DATA: GRAY, SETPOINT: MUTED}

_fonts = {}


def font(size, bold=False):
    key = (size, bold)
    if key not in _fonts:
        _fonts[key] = ctk.CTkFont(family=FAMILY, size=size, weight="bold" if bold else "normal")
    return _fonts[key]


def card(master, **kw):
    return ctk.CTkFrame(master, fg_color=CARD, corner_radius=14, border_width=1, border_color=BORDER, **kw)


def label(master, text="", size=12, bold=False, color=TEXT, **kw):
    return ctk.CTkLabel(master, text=text, font=font(size, bold), text_color=color, **kw)


def button(master, text, command, width=92, color=BLUE, hover=BLUE_D, ghost=False):
    if ghost:
        return ctk.CTkButton(master, text=text, command=command, width=width, height=30, corner_radius=8,
                             font=font(12), fg_color=SOFT, hover_color="#e2e8f0", text_color=TEXT,
                             border_width=1, border_color=BORDER)
    return ctk.CTkButton(master, text=text, command=command, width=width, height=32, corner_radius=8,
                         font=font(12, True), fg_color=color, hover_color=hover, text_color="#ffffff",
                         text_color_disabled="#e2e8f0")


def set_text(widget, text, **kw):
    """Cambia el texto solo si es distinto (evita parpadeos y trabajo innecesario)."""
    if widget.cget("text") != text or kw:
        widget.configure(text=text, **kw)


class KpiCard(ctk.CTkFrame):
    """Tarjeta de indicadores de una variable: actual, minimo, maximo y promedio."""

    def __init__(self, master, channel):
        super().__init__(master, fg_color=CARD, corner_radius=14, border_width=1, border_color=BORDER)
        self.channel = channel
        head = ctk.CTkFrame(self, fg_color="transparent")
        head.pack(fill="x", padx=12, pady=(8, 0))
        label(head, "●", 11, color=channel.color).pack(side="left")
        label(head, " " + channel.short, 11, True, MUTED).pack(side="left")
        row = ctk.CTkFrame(self, fg_color="transparent")
        row.pack(fill="x", padx=12)
        self.value = label(row, "—", 22, True)
        self.value.pack(side="left")
        label(row, " " + channel.unit, 11, color=MUTED).pack(side="left", pady=(8, 0))
        self.range = label(self, "mín —  ·  máx —", 10, color=MUTED, anchor="w", height=16)
        self.range.pack(fill="x", padx=12)
        foot = ctk.CTkFrame(self, fg_color="transparent")
        foot.pack(fill="x", padx=12, pady=(0, 8))
        self.mean = label(foot, "prom —", 10, color=MUTED, height=16)
        self.mean.pack(side="left")
        self.status = label(foot, "", 10, True, GRAY, height=16)
        self.status.pack(side="right")

    def update_values(self, value, stats, status):
        set_text(self.value, fmt_value(value))
        set_text(self.range, f"mín {fmt_value(stats.lo)}  ·  máx {fmt_value(stats.hi)}")
        set_text(self.mean, f"prom {fmt_value(stats.mean)}")
        if self.status.cget("text") != status:
            self.status.configure(text=status, text_color=STATUS_COLOR.get(status, GRAY))


# === clase padre (frame) ===
class Graphics(ctk.CTkFrame):
    def __init__(self, master, parameters=None, store=None):
        super().__init__(master, fg_color=BG, corner_radius=18)
        self.hub = SensorHub(parameters)
        self.store = store or ss.SessionStore()
        self.live = ss.DataSet()      # lo que se ve "en vivo" cuando no hay sesion; no se guarda
        self.session = None           # sesion en curso o la ultima terminada
        self.viewing = None           # sesion abierta para consulta historica
        self.selected = [0, 1]        # canales visibles en la grafica principal
        self.floors = {}              # "limpiar vista": instante desde el que se dibuja cada fuente
        self.period_ms = 500
        self.link = self.link_port = None
        self.demo = False
        self.error = ""
        self._values = [None] * len(CHANNELS)
        self._chart_key = self._ui_key = None
        self._draw_after = 0.0

        # distribucion: barra de adquisicion, franja de estado/exportacion y, ocupando el resto,
        # la grafica principal (izquierda) junto a las cuatro graficas por grupo (derecha, 2 x 2)
        self._msg_until = 0.0
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(3, weight=1)
        self._build_header()
        self._build_strip()
        self._build_kpis()
        body = ctk.CTkFrame(self, fg_color="transparent")
        body.grid(row=3, column=0, sticky="nsew", padx=14, pady=(6, 14))
        body.grid_columnconfigure(0, weight=11, uniform="body")
        body.grid_columnconfigure(1, weight=9, uniform="body")
        body.grid_rowconfigure(0, weight=1)
        self._build_main(body)
        self._build_small(body)
        self._paint_chips()

        self.bind("<Destroy>", self._on_destroy)
        self._due = time.monotonic()
        self.after(self.period_ms, self._sample)
        self.after(RENDER_MS, self._render)

    # === construccion de la interfaz =========================================
    def _build_header(self):
        """Panel de adquisicion: nombre de la sesion, frecuencia y controles."""
        box = card(self)
        box.grid(row=0, column=0, columnspan=2, sticky="ew", padx=14, pady=(14, 6))
        box.grid_columnconfigure(1, weight=1)
        title = ctk.CTkFrame(box, fg_color="transparent")
        title.grid(row=0, column=0, padx=(16, 12), pady=10)
        label(title, "Adquisición de datos", 15, True, anchor="w").pack(anchor="w")
        label(title, "Nombre de la sesión y control de registro", 10, color=MUTED, anchor="w", height=14).pack(anchor="w")

        self.entry_name = ctk.CTkEntry(box, height=32, corner_radius=8, font=font(12), fg_color=SOFT,
                                       border_color=BORDER, text_color=TEXT, placeholder_text_color=GRAY,
                                       placeholder_text="Nombre de la prueba (ej. Prueba de temperatura ambiental)")
        self.entry_name.grid(row=0, column=1, sticky="ew", padx=6)
        self.entry_name.bind("<Return>", lambda _e: self._start())

        self.rate = ctk.CTkOptionMenu(box, values=list(RATES), command=self._set_rate, width=78, height=32,
                                      corner_radius=8, font=font(12), fg_color=SOFT, button_color="#e2e8f0",
                                      button_hover_color=BORDER, text_color=TEXT, dropdown_font=font(12))
        self.rate.set("2 Hz")
        self.rate.grid(row=0, column=2, padx=6)

        self.btn_start = button(box, "▶  Iniciar", self._start, 104)
        self.btn_start.grid(row=0, column=3, padx=4)
        self.btn_pause = button(box, "Pausar", self._pause, 104, AMBER, "#b45309")
        self.btn_pause.grid(row=0, column=4, padx=4)
        self.btn_stop = button(box, "■  Detener", self._stop, 104, RED, RED_D)
        self.btn_stop.grid(row=0, column=5, padx=4)

        self.pill = ctk.CTkLabel(box, text="DETENIDO", font=font(11, True), text_color="#ffffff",
                                 fg_color=GRAY, corner_radius=10, width=112, height=24)
        self.pill.grid(row=0, column=6, padx=(12, 6))
        clock = ctk.CTkFrame(box, fg_color="transparent")
        clock.grid(row=0, column=7, padx=(6, 16))
        self.timer = label(clock, "00:00:00", 18, True, height=22)
        self.timer.pack(anchor="e")
        self.count = label(clock, "0 lecturas", 10, color=MUTED, height=14)
        self.count.pack(anchor="e")

    def _build_kpis(self):
        # ocultas por defecto para dar espacio a las graficas; se muestran con el boton "Indicadores"
        self.kpi_row = row = ctk.CTkFrame(self, fg_color="transparent")
        self.kpis_visible = False
        self.kpis = []
        for i, channel in enumerate(CHANNELS):
            row.grid_columnconfigure(i, weight=1, uniform="kpi")
            kpi = KpiCard(row, channel)
            kpi.grid(row=0, column=i, sticky="nsew", padx=4, pady=6)
            self.kpis.append(kpi)

    def _toggle_kpis(self):
        self.kpis_visible = not self.kpis_visible
        if self.kpis_visible:
            self.kpi_row.grid(row=2, column=0, sticky="ew", padx=10, pady=0)
        else:
            self.kpi_row.grid_remove()
        self.btn_kpis.configure(text="Ocultar indicadores" if self.kpis_visible else "Indicadores")

    def _build_main(self, parent):
        """Panel principal: grafica con seleccion de variables, intervalo, zoom y comparacion."""
        box = card(parent)
        box.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        head = ctk.CTkFrame(box, fg_color="transparent")
        head.pack(fill="x", padx=14, pady=(10, 0))
        label(head, "Análisis en el tiempo", 14, True).pack(side="left")
        self.mode = ctk.CTkLabel(head, text="EN VIVO", font=font(10, True), text_color="#ffffff",
                                 fg_color=BLUE, corner_radius=9, height=20)
        self.mode.pack(side="left", padx=10)
        self.btn_live = button(head, "Volver a en vivo", self._back_to_live, 118, ghost=True)
        button(head, "Limpiar vista", self._clear_view, 96, ghost=True).pack(side="right")
        button(head, "Restablecer", self._reset_view, 92, ghost=True).pack(side="right", padx=6)
        self.window = ctk.CTkSegmentedButton(head, values=list(WINDOWS), command=self._set_window, height=30,
                                             font=font(11), fg_color=SOFT, selected_color=BLUE,
                                             selected_hover_color=BLUE_D, unselected_color=SOFT,
                                             unselected_hover_color="#e2e8f0", text_color=TEXT)
        self.window.set("30 s")
        self.window.pack(side="right", padx=6)

        chips = ctk.CTkFrame(box, fg_color="transparent")
        chips.pack(fill="x", padx=14, pady=(8, 0))
        self.chips = []
        for i, channel in enumerate(CHANNELS):
            chip = ctk.CTkButton(chips, text=channel.short, width=20, height=24, corner_radius=12, font=font(11),
                                 border_width=1, command=lambda i=i: self._toggle_channel(i))
            chip.pack(side="left", padx=(0, 5))
            self.chips.append(chip)

        label(box, "Rueda: zoom  ·  Arrastrar: desplazar  ·  Doble clic: volver a seguir  ·  Cursor: valor en ese instante",
              10, color=GRAY, height=16).pack(side="bottom", pady=(0, 6))
        self.main_chart = TimeChart(box, legend=True)
        self.main_chart.pack(fill="both", expand=True, padx=12, pady=(4, 2))

    def _build_small(self, parent):
        """Graficas individuales por grupo de sensores, en cuadricula de 2 x 2."""
        grid = ctk.CTkFrame(parent, fg_color="transparent")
        grid.grid(row=0, column=1, sticky="nsew", padx=(6, 0))
        self.small = []
        for n, (title, indices) in enumerate(SMALL_CHARTS):
            r, col = divmod(n, 2)
            grid.grid_rowconfigure(r, weight=1, uniform="small")
            grid.grid_columnconfigure(col, weight=1, uniform="small")
            box = card(grid)
            box.grid(row=r, column=col, sticky="nsew", padx=(0, 6) if col == 0 else (6, 0),
                     pady=(0, 6) if r == 0 else (6, 0))
            units = " · ".join(dict.fromkeys(CHANNELS[i].unit for i in indices))
            label(box, f"{title}  [{units}]", 11, True, anchor="w", height=18).pack(fill="x", padx=12, pady=(8, 0))
            chart = TimeChart(box, compact=True, legend=True, interactive=False)
            chart.pack(fill="both", expand=True, padx=8, pady=(0, 8))
            self.small.append((chart, indices))

    def _build_strip(self):
        """Franja delgada: estado del sistema a la izquierda, exportacion a la derecha."""
        box = card(self)
        box.grid(row=1, column=0, sticky="ew", padx=14, pady=6)
        self.status = {}
        for n, name in enumerate(("ESP32-GS", "Recepción", "Sensores", "Adquisición")):
            label(box, name, 11, color=MUTED).pack(side="left", padx=(16 if n == 0 else 18, 5), pady=7)
            value = label(box, "—", 11, True)
            value.pack(side="left")
            self.status[name] = value

        self.btn_kpis = button(box, "Indicadores", self._toggle_kpis, 130, ghost=True)
        self.btn_kpis.pack(side="right", padx=(6, 12))
        button(box, "Abrir carpeta", self._open_folder, 104, ghost=True).pack(side="right", padx=3)
        button(box, "Excel", lambda: self._export(".xlsx"), 70, GREEN, "#15803d").pack(side="right", padx=3)
        button(box, "CSV", lambda: self._export(".csv"), 70).pack(side="right", padx=3)
        self.export_msg = label(box, "", 10, color=MUTED, anchor="e")  # sesion a exportar o resultado
        self.export_msg.pack(side="right", padx=8)

    def _say(self, text, color):
        """Mensaje temporal junto a los botones de exportacion."""
        self.export_msg.configure(text=text[:70], text_color=color)
        self._msg_until = time.monotonic() + 8


    # === enlace con main.py ====================================================
    def set_link(self, gs, port=None, demo=False):
        """main.py entrega el enlace serial para mostrar su estado (no se usa para enviar nada)."""
        self.link, self.link_port, self.demo = gs, port, demo

    # === adquisicion (independiente del refresco visual) =======================
    def _sample(self):
        if not self.winfo_exists():
            return
        now = time.time()
        try:
            self._values = self.hub.read(now)
            if self.live.t0 is None:
                self.live.t0 = now
            self.live.append(now - self.live.t0, now, self._values)
            if len(self.live) > LIVE_MAX:
                self.live.trim(LIVE_MAX // 10)
            if self.session is not None and self.session.active:
                self.session.append(now, self._values)
        except OSError as error:
            # fallo de disco: se cierra la sesion con lo ya guardado para no corromperla
            self.error = f"Error al guardar: {error}"
            try:
                self.session.stop()
            except OSError:
                pass
        period = self.period_ms / 1000.0
        self._due += period
        mono = time.monotonic()
        if self._due < mono - period:
            self._due = mono  # se atraso (equipo ocupado): no acumular disparos
        self.after(max(1, int((self._due - mono) * 1000)), self._sample)

    def _set_rate(self, choice):
        self.period_ms = RATES[choice]

    def _start(self):
        if self.session is not None and self.session.active:
            return
        try:
            self.session = self.store.create(self.entry_name.get(), self.period_ms)
        except OSError as error:
            self.error = f"No se pudo crear la sesión: {error}"
            return
        self.error = ""
        self.viewing = None
        self._show_window("30 s")

    def _pause(self):
        if self.session is None or not self.session.active:
            return
        if self.session.state == ss.PAUSED:
            self.session.resume()
        else:
            self.session.pause()

    def _stop(self):
        if self.session is None or not self.session.active:
            return
        self.session.stop()
        self.viewing = self.session  # queda a la vista para consultarla y exportarla
        self.entry_name.configure(state="normal")
        self.entry_name.delete(0, "end")  # la siguiente sesion empieza con su propio nombre
        self._show_window("Todo")         # la sesion terminada se muestra completa
        self._refresh()

    def _on_destroy(self, event):
        if event.widget is self and self.session is not None and self.session.active:
            try:
                self.session.stop()  # cerrar la ventana no deja la sesion a medias
            except OSError:
                pass

    # === visualizacion =========================================================
    def _source(self):
        if self.viewing is not None:
            return self.viewing
        if self.session is not None and self.session.active:
            return self.session
        return self.live

    def _toggle_channel(self, i):
        if i in self.selected:
            if len(self.selected) > 1:
                self.selected.remove(i)
        else:
            self.selected.append(i)
        self._paint_chips()

    def _paint_chips(self):
        for i, chip in enumerate(self.chips):
            on = i in self.selected
            color = CHANNELS[i].color
            chip.configure(fg_color=color if on else CARD, hover_color=color if on else SOFT,
                           text_color="#ffffff" if on else MUTED, border_color=color if on else BORDER)

    def _set_window(self, choice):
        for chart in [self.main_chart] + [c for c, _i in self.small]:
            chart.set_window(WINDOWS[choice])

    def _clear_view(self):
        """Limpia solo lo que se ve; los registros de la sesion no se tocan."""
        src = self._source()
        if src is self.live:
            self.live.clear()
        elif src.t:
            self.floors[id(src)] = src.t[-1]
        self.main_chart.manual = None
        self._chart_key = None

    def _reset_view(self):
        self.floors.pop(id(self._source()), None)
        self.main_chart.manual = None
        self._chart_key = None

    def _back_to_live(self):
        self.viewing = None
        self._show_window("30 s")

    def _show_window(self, choice):
        self.window.set(choice)
        self._set_window(choice)

    def _render(self):
        if not self.winfo_exists():
            return
        try:
            self._refresh()
        finally:
            self.after(RENDER_MS, self._render)

    def _refresh(self):
        now = time.time()
        session = self.session
        active = session is not None and session.active
        paused = active and session.state == ss.PAUSED
        src = self._source()
        historic = self.viewing is not None

        # controles de adquisicion
        key = (active, paused, historic, src is self.live)
        if key != self._ui_key:
            self._ui_key = key
            off = "#cbd5e1"
            self.btn_start.configure(state="disabled" if active else "normal", fg_color=off if active else BLUE)
            self.btn_pause.configure(state="normal" if active else "disabled",
                                     text="Reanudar" if paused else "Pausar",
                                     fg_color=off if not active else GREEN if paused else AMBER,
                                     hover_color="#15803d" if paused else "#b45309")
            self.btn_stop.configure(state="normal" if active else "disabled", fg_color=RED if active else off)
            self.entry_name.configure(state="disabled" if active else "normal")
            self.rate.configure(state="disabled" if active else "normal")
            text, color = ("PAUSADO", AMBER) if paused else ("REGISTRANDO", GREEN) if active else ("DETENIDO", GRAY)
            self.pill.configure(text=text, fg_color=color)
            if historic:
                self.btn_live.pack(side="left")
            else:
                self.btn_live.pack_forget()

        shown = self.viewing if isinstance(self.viewing, ss.Session) else session  # la sesion a la vista
        elapsed = shown.active_seconds(now) if shown is not None else 0
        readings = len(shown) if shown is not None else 0
        set_text(self.timer, ss.fmt_duration(elapsed))
        set_text(self.count, f"{readings} lecturas")

        # modo de la grafica principal
        if historic:
            text, color = "SESIÓN FINALIZADA", AMBER
        elif active:
            text, color = "SESIÓN EN CURSO", GREEN
        else:
            text, color = "EN VIVO · sin registrar", BLUE
        if self.mode.cget("text") != text:
            self.mode.configure(text=text, fg_color=color)

        # tarjetas de indicadores
        for i, kpi in enumerate(self.kpis if self.kpis_visible else ()):
            if historic:
                value, status = src.last(i), ""
            else:
                value, status = self._values[i], self.hub.status(i, now)
                if status == SETPOINT and value is None:
                    status = "sin consigna"
            kpi.update_values(value, src.stats[i], status)

        self._refresh_status(now, active, paused)
        if time.monotonic() >= self._msg_until:  # sin mensaje reciente: indica que sesion se exportaria
            target = self.viewing or self.session
            text = f"Exportar: {target.name[:44]}" if target else "Inicia una sesión para poder exportar"
            if self.export_msg.cget("text") != text:
                self.export_msg.configure(text=text, text_color=MUTED)

        # graficas: solo si la pestana esta visible y algo cambio
        if not self.winfo_ismapped():
            return
        key = (id(src), len(src), tuple(self.selected), self.floors.get(id(src)))
        if key == self._chart_key or time.monotonic() < self._draw_after:
            return
        self._chart_key = key
        started = time.monotonic()
        period = (src.period_ms if isinstance(src, ss.Session) else self.period_ms) / 1000.0
        gap = max(1.5, 3 * period)
        floor = self.floors.get(id(src))
        empty = "Sin lecturas en esta sesión" if historic else "Esperando lecturas de los sensores"

        times = src.times()

        def series(indices):
            return [Series(CHANNELS[i].short, CHANNELS[i].color, times, src.values(i), CHANNELS[i].unit, gap=gap)
                    for i in indices]

        main = series(self.selected)
        self.main_chart.floor, self.main_chart.empty_text = floor, empty
        self.main_chart.set_series(main)
        for chart, indices in self.small:
            chart.floor, chart.empty_text = floor, empty
            chart.set_series(series(indices))
        # si dibujar llegara a tardar, se espacian los redibujados para no frenar la adquisicion
        self._draw_after = time.monotonic() + 3 * (time.monotonic() - started)

    def _refresh_status(self, now, active, paused):
        def show(name, text, color=TEXT):
            widget = self.status[name]
            if widget.cget("text") != text:
                widget.configure(text=text, text_color=color)

        if self.link is None:
            show("ESP32-GS", "Sin información", GRAY)
        elif getattr(self.link, "link_lost", False):
            show("ESP32-GS", "Desconectado", RED)
        elif self.link.ser is None:
            show("ESP32-GS", "Sin conexión", AMBER)
        else:
            show("ESP32-GS", f"Conectado · {self.link_port or ''}", GREEN)

        suffix = " (simulados)" if self.demo else ""
        if self.hub.last_rx is None:
            show("Recepción", "Sin datos", GRAY)
        elif now - self.hub.last_rx <= 3:
            show("Recepción", "Recibiendo" + suffix, AMBER if self.demo else GREEN)
        else:
            show("Recepción", f"Sin datos hace {int(now - self.hub.last_rx)} s", RED)

        ok, stale, total = self.hub.counts(now)
        text = f"{ok} de {total} activos" + (f" · {stale} sin señal" if stale else "")
        show("Sensores", text, GREEN if ok == total else RED if stale else GRAY if ok == 0 else AMBER)

        if self.error:
            show("Adquisición", self.error[:40], RED)
        else:
            show("Adquisición", "Pausada" if paused else "Registrando" if active else "Detenida",
                 AMBER if paused else GREEN if active else GRAY)

    # === exportacion ===========================================================
    def export_to(self, session, path):
        """Exporta `session` a `path` (.csv o .xlsx). Devuelve la ruta escrita."""
        if path.lower().endswith(".xlsx"):
            return ss.export_xlsx(session, path)
        return ss.export_csv(session, path)

    def _export(self, extension):
        session = self.viewing or self.session
        if session is None:
            self._say("No hay sesión para exportar: inicia una primero.", AMBER)
            return
        os.makedirs(self.store.export_dir, exist_ok=True)
        kind = ("Excel", "*.xlsx") if extension == ".xlsx" else ("CSV", "*.csv")
        path = filedialog.asksaveasfilename(parent=self, title="Exportar sesión", initialdir=self.store.export_dir,
                                            initialfile=ss.export_name(session, extension),
                                            defaultextension=extension, filetypes=[kind])
        if not path:
            return
        try:
            self.export_to(session, path)
        except ImportError:
            self._say("Falta la librería openpyxl para Excel: pip install openpyxl", RED)
        except OSError as error:
            self._say(f"No se pudo guardar (¿archivo abierto?): {error}", RED)
        else:
            self._say(f"Exportado: {os.path.basename(path)} ({len(session)} lecturas)", GREEN)

    def _open_folder(self):
        folder = self.store.folder
        os.makedirs(folder, exist_ok=True)
        if sys.platform == "win32":
            os.startfile(folder)
        else:
            subprocess.Popen(["open" if sys.platform == "darwin" else "xdg-open", folder])
