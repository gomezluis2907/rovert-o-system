import struct
import serial

SYNC = 0xAA
CMD_ID = 0x01
CMD_FMT = "<BBBBff"

CMD_STOP = 0
CMD_MOVE = 1
CMD_START_MISSION = 2
CMD_STOP_MISSION = 3


class GroundStationSerial:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200):
        self.seq = 0
        self.link_lost = False
        # Abre el puerto; si no hay ESP32 queda en modo prueba en seco
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0, write_timeout=0.1)
        except (serial.SerialException, OSError):
            self.ser = None
            print(f"[GS] Sin hardware en {port}: modo dry-run")

    def _send(self, cmd_id: int, distance_m: float = 0.0, angle_deg: float = 0.0) -> bytes:
        # Trama 0x01 COMMAND (13 B, <BBBBffB): 0xAA, 0x01, seq, cmd_id, distance_m, angle_deg, xor
        body = struct.pack(
            CMD_FMT,
            SYNC,
            CMD_ID,
            self.seq & 0xFF,
            cmd_id & 0xFF,
            float(distance_m),
            float(angle_deg),
        )
        self.seq = (self.seq + 1) & 0xFF
        chk = 0
        for b in body:
            chk ^= b
        frame = body + bytes([chk])
        print("[GS] TX:", frame.hex(" "))
        if self.ser:
            try:
                self.ser.write(frame)
            except (serial.SerialException, OSError):
                self.link_lost = True
                print("[GS] Error de escritura serial")
        return frame

    def send_move(self, distance_m: float = 0.0, angle_deg: float = 0.0):
        if distance_m > 0.0 or abs(angle_deg) > 1e-3:
            self._send(CMD_MOVE, max(0.0, float(distance_m)), float(angle_deg))

    def send_advance(self, distance_m: float, angle_deg: float = 0.0):
        self.send_move(distance_m, angle_deg)

    def send_stop(self):
        self._send(CMD_STOP, 0.0, 0.0)

    def send_start_mission(self):
        self._send(CMD_START_MISSION, 0.0, 0.0)

    def send_stop_mission(self):
        self._send(CMD_STOP_MISSION, 0.0, 0.0)
