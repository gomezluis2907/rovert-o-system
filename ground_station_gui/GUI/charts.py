"""Grafica de series de tiempo sobre tk.Canvas (sin dependencias externas).

- Varias series a la vez; con dos unidades distintas usa eje izquierdo y derecho, con mas
  de dos muestra cada unidad en porcentaje de su propio rango.
- Rueda del raton: zoom en el tiempo. Arrastrar: desplazar. Doble clic: volver a seguir.
- Al pasar el cursor muestra el valor de cada serie en ese instante.
- Solo dibuja los puntos visibles y los reduce (min/max por tramo) cuando hay demasiados;
  los datos originales nunca se modifican. Las series llegan como arreglos de numpy
  (NaN = sin dato) para que una sesion de horas se dibuje en milisegundos.
"""
import math
import tkinter as tk

import numpy as np

BG, GRID, FRAME, TEXT, MUTED = "#ffffff", "#edf1f6", "#d5dde8", "#1f2a37", "#6b7a90"
FONT, FONT_BOLD = ("Segoe UI", 9), ("Segoe UI", 9, "bold")
TIME_STEPS = (1, 2, 5, 10, 15, 30, 60, 120, 300, 600, 900, 1800, 3600, 7200, 14400, 28800, 86400)
MIN_SPAN = 2.0  # zoom maximo: 2 s visibles


class Series:
    __slots__ = ("label", "color", "t", "v", "unit", "dash", "gap")

    def __init__(self, label, color, t, v, unit="", dash=None, gap=None):
        self.label, self.color = label, color
        self.t, self.v = np.asarray(t, dtype=float), np.asarray(v, dtype=float)  # None -> NaN
        self.unit, self.dash, self.gap = unit, dash, gap  # gap: corte de linea si faltan datos [s]


def fmt_value(value):
    if value is None:
        return "—"
    a = abs(value)
    if a >= 1000:
        return f"{value:.0f}"
    if a >= 100:
        return f"{value:.1f}"
    if a >= 1:
        return f"{value:.2f}"
    return f"{value:.3f}"


def fmt_time(seconds, decimals=False):
    sign = "-" if seconds < 0 else ""
    seconds = abs(seconds)
    whole = int(seconds)
    h, m, s = whole // 3600, whole % 3600 // 60, whole % 60
    text = f"{h}:{m:02d}:{s:02d}" if h else f"{m:02d}:{s:02d}"
    return sign + text + (f".{int((seconds - whole) * 10)}" if decimals else "")


def nice_ticks(lo, hi, count=4):
    span = hi - lo
    if span <= 0:
        return [lo]
    raw = span / count
    mag = 10 ** math.floor(math.log10(raw))
    step = next(m * mag for m in (1, 2, 5, 10) if m * mag >= raw)
    first = math.ceil(lo / step - 1e-9) * step
    return [first + i * step for i in range(int((hi - first) / step + 1e-9) + 1)]


class TimeChart(tk.Canvas):
    def __init__(self, master, compact=False, legend=True, interactive=True):
        super().__init__(master, bg=BG, highlightthickness=0)
        self.compact, self.legend, self.interactive = compact, legend, interactive
        self.series = []
        self.window = 30.0   # segundos visibles al seguir en vivo; None = todo
        self.floor = None    # oculta lo anterior a este instante ("limpiar vista")
        self.manual = None   # (t_inicio, t_fin) elegido con zoom / arrastre
        self.empty_text = "Sin datos"
        self._geom = None
        self._drag = None
        self.bind("<Configure>", lambda _e: self.redraw())
        self.bind("<Motion>", self._on_motion)
        self.bind("<Leave>", lambda _e: self.delete("hover"))
        if interactive:
            self.bind("<MouseWheel>", lambda e: self._zoom(e.x, 0.8 if e.delta > 0 else 1.25))
            self.bind("<Button-4>", lambda e: self._zoom(e.x, 0.8))
            self.bind("<Button-5>", lambda e: self._zoom(e.x, 1.25))
            self.bind("<ButtonPress-1>", self._drag_start)
            self.bind("<B1-Motion>", self._drag_move)
            self.bind("<ButtonRelease-1>", lambda _e: setattr(self, "_drag", None))
            self.bind("<Double-Button-1>", lambda _e: self.reset_view())

    # --- API -----------------------------------------------------------------
    def set_series(self, series):
        self.series = series
        self.redraw()

    def set_window(self, seconds):
        self.window, self.manual = seconds, None
        self.redraw()

    def reset_view(self):
        self.manual = None
        self.redraw()

    # --- dibujo ----------------------------------------------------------------
    def _time_range(self, series):
        tmin, tmax = min(float(s.t[0]) for s in series), max(float(s.t[-1]) for s in series)
        if self.floor is not None:
            tmin = min(max(tmin, self.floor), tmax)
        if self.manual:
            return self.manual
        if self.window is None:
            return tmin, max(tmax, tmin + MIN_SPAN)
        if tmax - tmin > self.window:
            return tmax - self.window, tmax
        return tmin, tmin + self.window

    def redraw(self):
        self.delete("all")
        self._geom = None
        w, h = self.winfo_width(), self.winfo_height()
        series = [s for s in self.series if s.t.size]
        units = list(dict.fromkeys(s.unit for s in series))
        two_axes, percent = len(units) == 2, len(units) > 2
        x0 = 44 if self.compact else 56
        x1 = w - (x0 if two_axes else 14)
        y0 = 28 if self.legend else 10
        y1 = h - 22
        if x1 - x0 < 60 or y1 - y0 < 30:
            return
        if not series:
            self._empty(x0, x1, y0, y1)
            return
        xa, xb = self._time_range(series)

        # tramo visible de cada serie y rango vertical por unidad
        visible, ranges = [], {}
        for s in series:
            i0 = max(0, int(np.searchsorted(s.t, xa)) - 1)
            if self.floor is not None:
                i0 = max(i0, int(np.searchsorted(s.t, self.floor)))
            i1 = min(s.t.size, int(np.searchsorted(s.t, xb, side="right")) + 1)
            visible.append((s, i0, i1))
            chunk = s.v[i0:i1]
            if chunk.size and not np.isnan(chunk).all():
                lo, hi = ranges.get(s.unit, (math.inf, -math.inf))
                ranges[s.unit] = (min(lo, float(np.nanmin(chunk))), max(hi, float(np.nanmax(chunk))))
        if not ranges:
            self._empty(x0, x1, y0, y1)
            return
        for unit, (lo, hi) in ranges.items():
            pad = (hi - lo) * 0.12 or max(abs(hi) * 0.1, 0.5)
            ranges[unit] = (lo - pad, hi + pad)

        def px(t):
            return x0 + (t - xa) / (xb - xa) * (x1 - x0)

        def py(v, unit):
            lo, hi = ranges[unit]
            return y1 - (v - lo) / (hi - lo) * (y1 - y0)

        # rejilla horizontal (las etiquetas del eje Y se dibujan al final, sobre los margenes)
        if percent:
            for frac in (0, 0.5, 1):
                y = y1 - frac * (y1 - y0)
                self.create_line(x0, y, x1, y, fill=GRID)
        else:
            first = next((u for u in units if u in ranges), None)
            for tick in nice_ticks(*ranges[first], count=3 if self.compact else 5):
                self.create_line(x0, py(tick, first), x1, py(tick, first), fill=GRID)

        # rejilla vertical y etiquetas de tiempo
        span = xb - xa
        step = next((s for s in TIME_STEPS if s >= span / (5 if self.compact else 10)), TIME_STEPS[-1])
        tick = math.ceil(xa / step) * step
        while tick <= xb + 1e-9:
            x = px(tick)
            self.create_line(x, y0, x, y1, fill=GRID)
            self.create_text(min(x, x1), y1 + 11, text=fmt_time(tick), font=FONT, fill=MUTED,
                             anchor="e" if x > x1 - 16 else "center")
            tick += step

        # series
        max_points = max(200, int(x1 - x0))
        for s, i0, i1 in visible:
            if s.unit not in ranges:
                continue
            index = self._decimate(s.v, i0, i1, max_points)
            stride = (i1 - i0) / max(1, len(index))
            gap = None if s.gap is None else s.gap * max(1.0, stride * 2)
            segment, prev_t, last = [], None, None
            for t, v in zip(s.t[index].tolist() + [None], s.v[index].tolist() + [None]):
                missing = t is None or v != v  # NaN
                if missing or (gap and prev_t is not None and t - prev_t > gap):
                    self._stroke(segment, s)
                    segment = []
                if missing:
                    prev_t = None
                    continue
                last = (px(t), py(v, s.unit))
                segment += last
                prev_t = t
            if last and not self.manual and x0 <= last[0] <= x1:
                self.create_oval(last[0] - 3.5, last[1] - 3.5, last[0] + 3.5, last[1] + 3.5,
                                 fill=s.color, outline=BG, width=1.5)

        # margenes (tapan lo que sobresale del area) y marco
        self.create_rectangle(0, 0, x0 - 1, h, fill=BG, outline="")
        self.create_rectangle(x1 + 1, 0, w, h, fill=BG, outline="")
        self.create_rectangle(x0, y0, x1, y1, outline=FRAME)
        self._axis_labels(units, ranges, percent, x0, x1, y0, y1, py)
        if self.legend:
            self._legend(series, units, x0, x1)
        self._geom = (x0, x1, y0, y1, xa, xb, visible, ranges)

    def _axis_labels(self, units, ranges, percent, x0, x1, y0, y1, py):
        """Etiquetas del eje Y: primera unidad a la izquierda, segunda a la derecha."""
        units = [u for u in units if u in ranges]
        if percent:
            for frac in (0, 0.5, 1):
                self.create_text(x0 - 6, y1 - frac * (y1 - y0), text=f"{int(frac * 100)}%",
                                 anchor="e", font=FONT, fill=MUTED)
        else:
            for side, unit in enumerate(units[:2]):
                for tick in nice_ticks(*ranges[unit], count=3 if self.compact else 5):
                    if side == 0:
                        self.create_text(x0 - 6, py(tick, unit), text=f"{tick:.6g}", anchor="e", font=FONT, fill=MUTED)
                    else:
                        self.create_text(x1 + 6, py(tick, unit), text=f"{tick:.6g}", anchor="w", font=FONT, fill=MUTED)

    def _legend(self, series, units, x0, x1):
        x = x0
        for s in series:
            tail = s.v[-200:]
            tail = tail[~np.isnan(tail)]
            last = float(tail[-1]) if tail.size else None
            self.create_rectangle(x, 9, x + 10, 19, fill=s.color, outline="")
            item = self.create_text(x + 15, 14, anchor="w", font=FONT, fill=TEXT,
                                    text=f"{s.label}  {fmt_value(last)} {s.unit}")
            x = self.bbox(item)[2] + 16
            if x > x1 - 60:
                break

    def _empty(self, x0, x1, y0, y1):
        self.create_rectangle(x0, y0, x1, y1, outline=FRAME)
        self.create_text((x0 + x1) / 2, (y0 + y1) / 2, text=self.empty_text,
                         font=("Segoe UI", 10, "italic"), fill=MUTED)

    def _stroke(self, segment, s):
        if len(segment) >= 4:
            self.create_line(*segment, fill=s.color, width=2, dash=s.dash, joinstyle="round")
        elif len(segment) == 2:
            self.create_oval(segment[0] - 2, segment[1] - 2, segment[0] + 2, segment[1] + 2,
                             fill=s.color, outline="")

    @staticmethod
    def _decimate(values, i0, i1, max_points):
        """Indices a dibujar: todos, o el minimo y maximo de cada tramo si son demasiados."""
        n = i1 - i0
        if n <= max_points:
            return np.arange(i0, i1)
        bucket = math.ceil(n / (max_points / 2))
        rows = math.ceil(n / bucket)
        block = np.full(rows * bucket, np.nan)
        block[:n] = values[i0:i1]
        block = block.reshape(rows, bucket)
        empty = np.isnan(block)
        lo = np.where(empty, np.inf, block).argmin(axis=1)
        hi = np.where(empty, -np.inf, block).argmax(axis=1)
        base = i0 + np.arange(rows) * bucket
        pairs = np.stack((base + np.minimum(lo, hi), base + np.maximum(lo, hi)), axis=1)
        index = pairs[~empty.all(axis=1)].ravel()
        if index.size == 0:
            return index
        return index[np.concatenate(([True], index[1:] != index[:-1]))]  # sin repetidos

    # --- interaccion -----------------------------------------------------------
    def _on_motion(self, event):
        self.delete("hover")
        if not self._geom or self._drag:
            return
        x0, x1, y0, y1, xa, xb, visible, ranges = self._geom
        if not (x0 <= event.x <= x1 and y0 <= event.y <= y1):
            return
        t = xa + (event.x - x0) / (x1 - x0) * (xb - xa)
        lines, snap = [], None
        for s, i0, i1 in visible:
            if i1 <= i0 or s.unit not in ranges:
                continue
            j = min(i0 + int(np.searchsorted(s.t[i0:i1], t)), i1 - 1)
            if j > i0 and abs(s.t[j - 1] - t) <= abs(s.t[j] - t):
                j -= 1
            if abs(s.t[j] - t) > (xb - xa) * 0.05:
                continue  # no hay lectura cerca del cursor
            v = float(s.v[j])
            x = x0 + (float(s.t[j]) - xa) / (xb - xa) * (x1 - x0)
            snap = x if snap is None else snap
            if v != v:
                lines.append((s, "sin dato"))
                continue
            lo, hi = ranges[s.unit]
            y = y1 - (v - lo) / (hi - lo) * (y1 - y0)
            self.create_oval(x - 4, y - 4, x + 4, y + 4, fill=s.color, outline=BG, width=2, tags="hover")
            lines.append((s, f"{fmt_value(v)} {s.unit}"))
        if not lines:
            return
        self.create_line(snap, y0, snap, y1, fill="#94a3b8", dash=(3, 3), tags="hover")
        t_snap = xa + (snap - x0) / (x1 - x0) * (xb - xa)
        rows = [("", f"t = {fmt_time(t_snap, decimals=True)}")] + [(s.color, f"{s.label}: {text}") for s, text in lines]
        width = max(len(text) for _c, text in rows) * 6.4 + 34
        height = len(rows) * 17 + 10
        bx = snap + 12 if snap + 12 + width < x1 else snap - 12 - width
        by = min(max(event.y - height / 2, y0 + 2), y1 - height - 2)
        self.create_rectangle(bx, by, bx + width, by + height, fill="#1f2a37", outline="", tags="hover")
        for n, (color, text) in enumerate(rows):
            y = by + 13 + n * 17
            if color:
                self.create_rectangle(bx + 8, y - 4, bx + 16, y + 4, fill=color, outline="", tags="hover")
            self.create_text(bx + (22 if color else 8), y, text=text, anchor="w", fill="#ffffff",
                             font=FONT if color else FONT_BOLD, tags="hover")

    def _current_range(self):
        return (self._geom[4], self._geom[5]) if self._geom else None

    def _zoom(self, x, factor):
        rng = self._current_range()
        if not rng:
            return
        x0, x1 = self._geom[0], self._geom[1]
        xa, xb = rng
        pivot = xa + min(max((x - x0) / (x1 - x0), 0), 1) * (xb - xa)
        span = max(MIN_SPAN, (xb - xa) * factor)
        ratio = (pivot - xa) / (xb - xa)
        self.manual = (pivot - ratio * span, pivot + (1 - ratio) * span)
        self.redraw()

    def _drag_start(self, event):
        rng = self._current_range()
        if rng:
            self._drag = (event.x, rng)

    def _drag_move(self, event):
        if not self._drag or not self._geom:
            return
        start_x, (xa, xb) = self._drag
        shift = (start_x - event.x) / (self._geom[1] - self._geom[0]) * (xb - xa)
        if abs(start_x - event.x) < 3 and not self.manual:
            return
        self.manual = (xa + shift, xb + shift)
        self.redraw()
