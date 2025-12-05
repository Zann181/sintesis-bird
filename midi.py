#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
F1 HID -> salida CC 7-bit con IMPRESIÓN binaria y LATCH de la GRID.
NO se cierra si el F1 no está conectado: espera y reconecta en caliente.
Controla LEDs del F1 (Output Report 0x80) para reflejar el latch.

Formato de impresión:
  "ID VAL    TX: <status_bin8> <d1_bin7> <d2_bin7>"
Ej.: "6 127    TX: 10110000 0000110 1111111"

Dependencias (Debian/Ubuntu):
  sudo apt update
  sudo apt install -y libhidapi-hidraw0 python3-pip
  pip3 install --break-system-packages hid pyserial
"""

import time, struct, sys, argparse
import hid  # pyhidapi: enumerate, device, read/write con timeout_ms (ver docs)

try:
    import sim_bridge
except ImportError:
    sim_bridge = None

F1_NAME = "Traktor Kontrol F1"
F1_VENDOR_ID = 0x17CC  # opcional (Native Instruments)
# F1_PRODUCT_ID = 0x1120  # si quieres filtrar también por PID

# ---------- Utilidades ----------
def clamp(x, lo, hi): return lo if x < lo else hi if x > hi else x
def _bin7(x): return format(x & 0x7F, "07b")
def _bin8(x): return format(x & 0xFF, "08b")

def map_axis_to_7bit(v_raw):
    # F1 ~0..4095 -> 0..127
    return clamp(int(round(v_raw / 32.0)), 0, 127)

# ---------- HID conexión con espera/reconexión ----------
def find_f1_path():
    # Buscar por Vendor ID y/o nombre de producto
    for d in hid.enumerate():  # ver enumerate() en docs
        prod = (d.get("product_string") or "")
        vid = d.get("vendor_id")
        if (F1_NAME in prod) or (vid == F1_VENDOR_ID):
            return d["path"]
    return None

def wait_for_f1(poll_s=0.5):
    """Bloquea hasta que aparezca el F1; no sale del programa."""
    announced = False
    while True:
        path = find_f1_path()
        if path:
            try:
                dev = hid.device()
                dev.open_path(path)               # abrir por path (robusto)
                dev.set_nonblocking(False)       # usaremos read con timeout_ms
                print("✅ F1 conectado.")
                return dev
            except OSError:
                # Si falló abrir justo al conectar, reintentar
                time.sleep(poll_s)
                continue
        if not announced:
            print("🔌 Esperando Traktor Kontrol F1... (conéctalo)")
            announced = True
        time.sleep(poll_s)

# ---------- LEDS (Output Report 0x80) ----------
# Frame: 1 byte ReportID (0x80) + 80 bytes payload (Mixxx wiki)
def led_frame_new():
    frame = bytearray(1 + 80)
    frame[0] = 0x80
    return frame

def _grid_rgb_offset_in_payload(pad_idx_1based):
    # grid pads 1..16, 3 bytes c/u (R,G,B) empezando en payload[24] (bytes 25..72 1-based)
    return 24 + (pad_idx_1based - 1) * 3

def led_set_grid_rgb(frame, pad_idx_1based, r, g, b):
    off = _grid_rgb_offset_in_payload(pad_idx_1based)
    frame[1 + off + 0] = r & 0x7F
    frame[1 + off + 1] = g & 0x7F
    frame[1 + off + 2] = b & 0x7F

def led_send(dev, frame):
    if dev is None:
        return
    try:
        dev.write(frame)  # primer byte del buffer es el Report ID 0x80
    except OSError:
        # Si se desconectó mientras escribíamos, lo detectará el loop principal
        pass

# Mapear bit (0..15) a pad 1..16 (orden documentado en la wiki)
def bit_to_pad_1based(bit):
    if 0 <= bit <= 7:
        return 8 - bit
    if 8 <= bit <= 15:
        return 24 - bit
    return None

# ---------- Salida / UART ----------
def print_and_maybe_send(ser, status, id7, val7):
    print(f"{id7} {val7}    TX: {_bin8(status)} {_bin7(id7)} {_bin7(val7)}")
    if ser:
        try:
            ser.write(bytes([status & 0xFF, id7 & 0x7F, val7 & 0x7F]))
        except Exception:
            pass

# ---------- Main ----------
def main():
    ap = argparse.ArgumentParser(description="F1 latch + LEDs (HID) con reconexión y impresión binaria")
    ap.add_argument("--serial", help="UART para enviar los 3 bytes (p.ej. /dev/ttyUSB0). Si se omite, solo imprime.")
    ap.add_argument("--baud", type=int, default=31250, help="Baudios UART (31250 MIDI; 115200 pruebas)")
    ap.add_argument("--channel", type=int, default=0, help="Canal MIDI 0..15 (0 = CH1)")
    ap.add_argument("--on-color", default="0,127,0", help="RGB (0..127) LED encendido, ej. 0,127,0")
    ap.add_argument("--off-color", default="0,0,0", help="RGB (0..127) LED apagado")
    args = ap.parse_args()

    # UART opcional
    ser = None
    if args.serial:
        if sim_bridge is None:
            print("⚠ pyserial no instalado; se imprimirá pero no se enviará.", file=sys.stderr)
        else:
            try:
                ser = sim_bridge.Serial(args.serial, args.baud, timeout=0)
                print(f"→ UART {args.serial} @ {args.baud}")
            except Exception as e:
                print(f"⚠ No pude abrir {args.serial}: {e}\nSe imprimirá pero no se enviará.", file=sys.stderr)
                ser = None

    # Parse colores
    def parse_rgb(s):
        try:
            r, g, b = (int(x) for x in s.split(","))
            return (clamp(r,0,127), clamp(g,0,127), clamp(b,0,127))
        except Exception:
            return (0,127,0)
    ON_RGB  = parse_rgb(args.on_color)
    OFF_RGB = parse_rgb(args.off_color)

    status_base = 0xB0 | (args.channel & 0x0F)
    prev_btn_bits = 0
    prev_cc_val7 = {}          # id7 -> último valor emitido (para NO duplicar)
    latch_on = {}              # pad(1..16) -> bool (True = “presionado/127”)

    dev = None
    ledframe = led_frame_new()

    # Bucle eterno con auto-(re)conexión
    while True:
        if dev is None:
            # Esperar hasta conectar
            dev = wait_for_f1(poll_s=0.5)
            # Al conectar: apagar grid y limpiar estado de LEDs
            for pad in range(1, 17):
                led_set_grid_rgb(ledframe, pad, *OFF_RGB)
            led_send(dev, ledframe)
            prev_btn_bits = 0

        try:
            # Leer con timeout para no ocupar CPU (pyhidapi read admite timeout_ms)
            data = dev.read(64, timeout_ms=50)  # ver API read(timeout_ms) en docs
            if not data:
                continue  # timeout sin datos

            # Alinear (input report 0x01, 22 bytes útiles)
            if data[0] != 0x01:
                if len(data) >= 23 and data[1] == 0x01:
                    data = data[1:23]
                else:
                    continue
            if len(data) < 22:
                continue

            rep = bytes(data[:22])

            # Botones (bits)
            btn_bits = rep[1] | (rep[2] << 8) | (rep[3] << 16) | (rep[4] << 24)
            changed = btn_bits ^ prev_btn_bits
            if changed:
                for bit in range(29):  # 0..15 grid, luego otros
                    if not (changed & (1 << bit)):
                        continue

                    is_pressed_now = bool(btn_bits & (1 << bit))
                    pad = bit_to_pad_1based(bit)

                    if pad is not None:
                        # LATCH en flanco de subida
                        if is_pressed_now:
                            new_state = not latch_on.get(pad, False)
                            latch_on[pad] = new_state
                            logical_val = 127 if new_state else 0

                            # LED del pad
                            if new_state:
                                led_set_grid_rgb(ledframe, pad, *ON_RGB)
                            else:
                                led_set_grid_rgb(ledframe, pad, *OFF_RGB)
                            led_send(dev, ledframe)

                            # Emitir (sin duplicar)
                            if prev_cc_val7.get(bit) != logical_val:
                                print_and_maybe_send(ser, status_base, bit & 0x7F, logical_val & 0x7F)
                                prev_cc_val7[bit] = logical_val
                        # flanco de bajada: ignorar (se queda “presionado”)
                    else:
                        # Botón fuera de grid: comportamiento normal
                        val7 = 127 if is_pressed_now else 0
                        if prev_cc_val7.get(bit) != val7:
                            print_and_maybe_send(ser, status_base, bit & 0x7F, val7 & 0x7F)
                            prev_cc_val7[bit] = val7

                prev_btn_bits = btn_bits

            # Encoder relativo (ID 29) – sin latch; salida normal
            browse_delta = struct.unpack("<b", rep[5:6])[0]
            if browse_delta != 0:
                val7 = clamp(64 + browse_delta, 0, 127)
                if prev_cc_val7.get(29) != val7:
                    print_and_maybe_send(ser, status_base, 29, val7)
                    prev_cc_val7[29] = val7

            # Ejes (IDs 30..37)
            axes_raw = struct.unpack("<8H", rep[6:22])
            for i, v in enumerate(axes_raw):
                id7 = (30 + i) & 0x7F
                val7 = map_axis_to_7bit(v) & 0x7F
                if prev_cc_val7.get(id7) != val7:
                    print_and_maybe_send(ser, status_base, id7, val7)
                    prev_cc_val7[id7] = val7

        except (OSError, ValueError):
            # Probable desconexión caliente; cerrar y volver a esperar
            try:
                dev.close()
            except Exception:
                pass
            dev = None
            print("⚠ F1 desconectado. Volveré a intentar en cuanto lo conectes…")
            time.sleep(0.5)

if __name__ == "__main__":
    main()
