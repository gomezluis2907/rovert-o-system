"""Pruebas de sesiones y exportacion (sin abrir la ventana).

    python -m unittest discover -s ground_station_gui/tests
"""
import csv
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "GUI"))

import sessions as ss  # noqa: E402
from telemetry import CHANNELS, to_float  # noqa: E402

N = len(CHANNELS)


def row(k):
    """Lectura de prueba: canal 0 con muchos decimales, canal 1 en cero, canal 2 sin dato."""
    return [20.123456789 + k, 0.0, None] + [float(k)] * (N - 3)


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.store = ss.SessionStore(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def record(self, name="Prueba de temperatura ambiental", t0=1_800_000_000.0):
        s = self.store.create(name, 100, now=t0)
        for k in range(10):
            self.assertTrue(s.append(t0 + k * 0.1, row(k)))
        return s, t0

    def test_pause_keeps_data_and_resume_continues_same_session(self):
        s, t0 = self.record()
        s.pause(t0 + 1.0)
        self.assertFalse(s.append(t0 + 5.0, row(99)))       # en pausa no se registra
        self.assertEqual(len(s), 10)                         # ni se pierde lo anterior
        s.resume(t0 + 10.0)
        for k in range(40):                                  # suficientes para varios guardados de meta.json
            self.assertTrue(s.append(t0 + 10.0 + k * 0.02, row(10 + k)))
        self.assertAlmostEqual(s.active_seconds(t0 + 10.9), 1.9, places=6)
        s.stop(t0 + 11.0)
        self.assertEqual(len(s), 50)
        self.assertEqual(s.state, ss.FINISHED)
        self.assertAlmostEqual(s.active_seconds(), 2.0, places=6)   # 1 s + 1 s, sin la pausa
        self.assertAlmostEqual(s.total_seconds(), 11.0, places=6)
        self.assertFalse(s.append(t0 + 12, row(0)))          # detenida: ya no admite lecturas

    def test_new_session_never_overwrites(self):
        a, t0 = self.record("Ensayo")
        a.stop(t0 + 1)
        b = self.store.create("Ensayo", 100, now=t0)         # mismo nombre y misma hora
        b.stop(t0 + 1)
        self.assertNotEqual(a.id, b.id)
        self.assertEqual(len(self.store.load(a.id)), 10)
        self.assertEqual(len(self.store.load(b.id)), 0)
        self.assertEqual(len(self.store.list()), 2)

    def test_reload_preserves_values_nulls_and_zero(self):
        s, t0 = self.record()
        s.stop(t0 + 1)
        again = self.store.load(s.id)
        self.assertEqual(again.cols, s.cols)                 # precision completa
        self.assertEqual(again.cols[1][0], 0.0)              # cero es cero...
        self.assertIsNone(again.cols[2][0])                  # ...y sin dato es sin dato
        self.assertEqual(again.stats[2].n, 0)
        self.assertEqual(again.meta["name"], "Prueba de temperatura ambiental")

    def test_interrupted_session_is_recoverable(self):
        s, _t0 = self.record()
        s._file.close()                                       # simula un cierre abrupto
        with open(s.data_path, "a", encoding="utf-8") as f:
            f.write("1.0,18000")                              # ultima fila truncada
        self.assertEqual(self.store.list()[0]["state"], ss.INTERRUPTED)
        again = self.store.load(s.id)
        self.assertEqual(again.state, ss.INTERRUPTED)
        self.assertEqual(len(again), 10)

    def test_export_csv(self):
        s, t0 = self.record()
        s.stop(t0 + 1)
        self.assertEqual(ss.export_name(s, ".csv"), "Prueba_de_temperatura_ambiental_2027-01-15.csv")
        path = ss.export_csv(s, os.path.join(self.tmp.name, "out.csv"))
        with open(path, newline="", encoding="utf-8-sig") as f:
            rows = list(csv.reader(f))
        info = {r[0]: r[1] for r in rows if r[0].startswith("#")}
        self.assertEqual(info["# Sesión"], "Prueba de temperatura ambiental")
        self.assertTrue(info["# Inicio"] and info["# Fin"])
        data = [r for r in rows if not r[0].startswith("#")]
        self.assertEqual(data[0][:3], ["Timestamp", "Tiempo (s)", "Temp. ambiente (°C)"])
        self.assertEqual(len(data), 11)                      # encabezado + todas las lecturas
        self.assertEqual(float(data[1][2]), 20.123456789)    # precision original
        self.assertEqual(data[1][4], "")                     # sin dato -> celda vacia

    def test_export_xlsx(self):
        from openpyxl import load_workbook
        s, t0 = self.record()
        s.stop(t0 + 1)
        path = ss.export_xlsx(s, os.path.join(self.tmp.name, "out.xlsx"))
        book = load_workbook(path)
        self.assertEqual(book.sheetnames, ["Sesión", "Datos", "Estadísticas"])
        data = list(book["Datos"].iter_rows(values_only=True))
        self.assertEqual(len(data), 11)
        self.assertEqual(data[1][2], 20.123456789)
        self.assertEqual(data[1][3], 0)
        self.assertIsNone(data[1][4])
        stats = {r[0]: r for r in book["Estadísticas"].iter_rows(values_only=True)}
        self.assertEqual(stats[CHANNELS[0].label][2:4], (10, 0))
        self.assertEqual(stats[CHANNELS[2].label][2:4], (0, 10))

    def test_to_float(self):
        self.assertIsNone(to_float("null"))
        self.assertIsNone(to_float(""))
        self.assertEqual(to_float("0"), 0.0)
        self.assertEqual(to_float(" 2,5 "), 2.5)


if __name__ == "__main__":
    unittest.main()
