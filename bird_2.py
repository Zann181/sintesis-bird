#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Bird FM Synth + FX — Modo Doble Velocidad + Espejo (8 compases en tiempo de 4)
Versión con mapeo de frecuencias a partir de la rueda Camelot
-------------------------------------------------------------
- fc ya no es un valor libre de 435..2800 Hz, sino que se toma de una tabla de
  frecuencias fundamentales derivada de la rueda de Camelot.
- Un control (CC30 del F1) recorre las 24 entradas (1A..12A, 1B..12B) y fija
  la fc según la tonalidad seleccionada.
- El resto del motor (FM, FX, secuenciador, GUI) se mantiene igual.
"""

import sys, time, math, threading, struct
from collections import deque
import numpy as np
import sounddevice as sd
import tkinter as tk
from tkinter import ttk
from queue import Queue

# ========= HID import =========
hid = None
try:
    import hid  # pyhidapi
except Exception:
    try:
        import hidapi as hid
    except Exception:
        hid = None

# ========= Constantes =========
FS = 44100
BLOCK = 512
F1_NAME = "Traktor Kontrol F1"
F1_VENDOR_ID = 0x17CC
SLICE_LOW_MIN = 1000.0
SLICE_LOW_MAX = FS * 0.45
SLICE_HIGH_MIN = 2000.0
SLICE_HIGH_MAX = FS * 0.49

# ========= Camelot → nota → frecuencia (Hz) =========
# Frecuencias de referencia (octava 4) con A4 = 440 Hz
NOTE_FREQ = {
    "C": 261.63,
    "C#": 277.18,
    "Db": 277.18,
    "D": 293.66,
    "D#": 311.13,
    "Eb": 311.13,
    "E": 329.63,
    "F": 349.23,
    "F#": 369.99,
    "Gb": 369.99,
    "G": 392.00,
    "G#": 415.30,
    "Ab": 415.30,
    "A": 440.00,
    "A#": 466.16,
    "Bb": 466.16,
    "B": 493.88,
}

# Orden lógico para recorrer con un potenciómetro (0..23)
CAMELOT_ORDER = [
    "1A", "1B",
    "2A", "2B",
    "3A", "3B",
    "4A", "4B",
    "5A", "5B",
    "6A", "6B",
    "7A", "7B",
    "8A", "8B",
    "9A", "9B",
    "10A", "10B",
    "11A", "11B",
    "12A", "12B",
]

# Mapeo de la rueda de la imagen → nota raíz (octava 4)
CAMELOT_FREQ = {
    # 1A = A♭ minor → A♭ = G#
    "1A": NOTE_FREQ["Ab"],
    # 1B = B major → B
    "1B": NOTE_FREQ["B"],
    # 2A = E♭ minor → Eb
    "2A": NOTE_FREQ["Eb"],
    # 2B = F# major
    "2B": NOTE_FREQ["F#"],
    # 3A = B♭ minor → Bb
    "3A": NOTE_FREQ["Bb"],
    # 3B = D♭ major → Db
    "3B": NOTE_FREQ["Db"],
    # 4A = F minor
    "4A": NOTE_FREQ["F"],
    # 4B = A♭ major
    "4B": NOTE_FREQ["Ab"],
    # 5A = C minor
    "5A": NOTE_FREQ["C"],
    # 5B = E♭ major
    "5B": NOTE_FREQ["Eb"],
    # 6A = G minor
    "6A": NOTE_FREQ["G"],
    # 6B = B♭ major
    "6B": NOTE_FREQ["Bb"],
    # 7A = D minor
    "7A": NOTE_FREQ["D"],
    # 7B = F major
    "7B": NOTE_FREQ["F"],
    # 8A = A minor
    "8A": NOTE_FREQ["A"],
    # 8B = C major
    "8B": NOTE_FREQ["C"],
    # 9A = E minor
    "9A": NOTE_FREQ["E"],
    # 9B = G major
    "9B": NOTE_FREQ["G"],
    # 10A = B minor
    "10A": NOTE_FREQ["B"],
    # 10B = D major
    "10B": NOTE_FREQ["D"],
    # 11A = F# minor
    "11A": NOTE_FREQ["F#"],
    # 11B = A major
    "11B": NOTE_FREQ["A"],
    # 12A = D♭ minor (C# minor)
    "12A": NOTE_FREQ["Db"],
    # 12B = E major
    "12B": NOTE_FREQ["E"],
}

# ========= Util =========
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def map_axis_to_7bit(v_raw):
    return clamp(int(round(v_raw/32.0)), 0, 127)

def map_7bit(v, lo, hi):
    return lo + (hi-lo) * (clamp(int(v),0,127)/127.0)

def db_to_lin(db):
    return 10.0 ** (db / 20.0)

def find_f1_path():
    if hid is None: return None
    try:
        for d in hid.enumerate():
            if (d.get("vendor_id") == F1_VENDOR_ID) or (F1_NAME in (d.get("product_string") or "")):
                return d["path"]
    except Exception:
        pass
    return None

# ========= Parámetros =========
class Params:
    def __init__(self):
        self.lock = threading.Lock()
        # Síntesis (fc será dictado por la clave Camelot)
        self.fm, self.beta, self.decay = 10.0, 80.0, 4.0
        self.gain, self.bpm = 0.5, 138.0
        self.muted, self.seq_on = False, True
        self.trigger = False
        # Índice Camelot (por defecto 8A = A minor)
        self.camelot_idx = CAMELOT_ORDER.index("8A")
        # FX
        self.fx_echo_on = True
        self.fx_echo_mix = 0.25
        self.fx_echo_time_ms = 350.0
        self.fx_slice_on = True
        self.fx_slice_low = 5000.0
        self.fx_slice_high = 16000.0
        self.fx_slice_gain_db = 0.0
        self.fx_comp_thresh_db = -12.0
        self.fx_comp_ratio = 2.0
        # PRE-BP
        self.prebp_on = True
        self.prebp_low = 200.0
        self.prebp_high = 6000.0
        self.prebp_gain_db = 0.0
        # DoubleTime + Mirror (nuevo)
        self.double_time = False
    def snapshot(self):
        with self.lock:
            return (self.camelot_idx, self.fm, self.beta, self.decay, self.gain,
                    self.bpm, self.muted, self.seq_on, self.trigger,
                    self.fx_echo_on, self.fx_echo_mix, self.fx_echo_time_ms,
                    self.fx_slice_on, self.fx_slice_low, self.fx_slice_high, self.fx_slice_gain_db,
                    self.fx_comp_thresh_db, self.fx_comp_ratio,
                    self.prebp_on, self.prebp_low, self.prebp_high, self.prebp_gain_db,
                    self.double_time)

P = Params()

# ========= Suavizado =========
class Smooth:
    def __init__(self, tau_ms=25):
        self.tau = tau_ms/1000.0
        self.state = dict(fc=440.0, fm=P.fm, beta=P.beta, decay=P.decay, gain=P.gain, bpm=P.bpm)
    def step(self, fc,fm,beta,decay,gain,bpm):
        a = 1.0 - math.exp(-BLOCK/(FS*self.tau))
        for k, tgt in {"fc":fc, "fm":fm, "beta":beta, "decay":decay, "gain":gain, "bpm":bpm}.items():
            self.state[k] += a*(tgt - self.state[k])
        s = self.state
        return s["fc"], s["fm"], s["beta"], s["decay"], s["gain"], s["bpm"]

SM = Smooth()

# ========= Motor FM =========
class FMOsc:
    def __init__(self):
        self.ph_car=0.0; self.ph_mod=0.0; self.env=0.0
    def process(self, fc,fm,beta,decay,gain,trig,n):
        if trig: self.env = 1.0
        i = np.arange(n, dtype=np.float32)
        phm = self.ph_mod + (2*np.pi*fm/FS)*i
        mod = np.sin(phm, dtype=np.float32)
        phc = self.ph_car + (2*np.pi*fc/FS)*i + beta*mod
        x = np.sin(phc, dtype=np.float32)
        env = self.env*np.exp(-decay*i/FS).astype(np.float32)
        self.env = float(env[-1] * math.exp(-decay/FS))
        y = gain * x * env
        self.ph_mod = float((phm[-1] + 2*np.pi*fm/FS) % (2*np.pi))
        self.ph_car = float((phc[-1] + 2*np.pi*fc/FS) % (2*np.pi))
        return y

OSC = FMOsc()

# ========= Biquads (se dejan igual) =========
class RBJ_Biquad:
    def __init__(self, fs):
        self.fs = fs
        self.b0=self.b1=self.b2=self.a1=self.a2=0.0
        self.z1=self.z2=0.0
    def set_lpf(self, fc, Q=0.707):
        w0 = 2*math.pi*fc/self.fs
        c = math.cos(w0); s = math.sin(w0)
        alpha = s/(2*Q)
        b0=(1-c)/2; b1=1-c; b2=(1-c)/2
        a0=1+alpha; a1=-2*c; a2=1-alpha
        self._set(b0,b1,b2,a1,a2,a0)
    def set_hpf(self, fc, Q=0.707):
        w0=2*math.pi*fc/self.fs
        c=math.cos(w0); s=math.sin(w0)
        alpha=s/(2*Q)
        b0=(1+c)/2; b1=-(1+c); b2=(1+c)/2
        a0=1+alpha; a1=-2*c; a2=1-alpha
        self._set(b0,b1,b2,a1,a2,a0)
    def _set(self,b0,b1,b2,a1,a2,a0):
        self.b0=b0/a0; self.b1=b1/a0; self.b2=b2/a0
        self.a1=a1/a0; self.a2=a2/a0
        self.z1=self.z2=0.0
    def process(self, x: np.ndarray):
        y = np.empty_like(x, dtype=np.float32)
        z1,z2 = self.z1, self.z2
        b0,b1,b2,a1,a2 = self.b0,self.b1,self.b2,self.a1,self.a2
        for n in range(x.size):
            xn = float(x[n])
            yn = b0*xn + z1
            z1 = b1*xn + z2 - a1*yn
            z2 = b2*xn - a2*yn
            y[n] = yn
        self.z1, self.z2 = z1, z2
        return y

class LPFCascade:
    def __init__(self, fs, fc):
        self.s1 = RBJ_Biquad(fs); self.s1.set_lpf(fc, 0.707)
        self.s2 = RBJ_Biquad(fs); self.s2.set_lpf(fc, 0.707)
    def set_fc(self, fc):
        self.s1.set_lpf(fc, 0.707); self.s2.set_lpf(fc, 0.707)
    def process(self, x):
        return self.s2.process(self.s1.process(x))

# ========= PRE-BP =========
class PreBandpass:
    def __init__(self, fs, low=200.0, high=6000.0, gain_db=0.0):
        self.fs = float(fs)
        self.low = float(low)
        self.high = float(high)
        self.gain_db = float(gain_db)
        self.on = True
        self._hpf = RBJ_Biquad(fs); self._hpf.set_hpf(self.low, 0.707)
        self._lpf = RBJ_Biquad(fs); self._lpf.set_lpf(self.high, 0.707)
    def set_params(self, on=None, low=None, high=None, gain_db=None):
        if on is not None: self.on = bool(on)
        if low is not None:
            self.low = float(clamp(low, 20.0, self.fs*0.45))
            self._hpf.set_hpf(self.low, 0.707)
        if high is not None:
            self.high = float(clamp(high, self.low+100.0, self.fs*0.49))
            self._lpf.set_lpf(self.high, 0.707)
        if gain_db is not None:
            self.gain_db = float(clamp(gain_db, -24.0, 24.0))
    def process(self, x: np.ndarray):
        if not self.on: return x
        band = self._lpf.process(self._hpf.process(x))
        g = db_to_lin(self.gain_db)
        return band * g

PRE = PreBandpass(FS, low=P.prebp_low, high=P.prebp_high, gain_db=P.prebp_gain_db)

# ========= FX =========
class FinalFX:
    def __init__(self, fs):
        self.fs = float(fs)
        # Echo
        self.echo_on = True
        self.echo_mix = 0.25
        self.echo_time_ms = 350.0
        self._max_delay_s = 2.0
        self._max_delay_n = int(self._max_delay_s * self.fs)
        self._buf = np.zeros(self._max_delay_n, dtype=np.float32)
        self._widx = 0
        # Compresor
        self.comp_thresh_db = -12.0
        self.comp_ratio = 2.0
        self._env = 0.0
        self._alpha_a = math.exp(-1.0/(self.fs*0.005))
        self._alpha_r = math.exp(-1.0/(self.fs*0.050))
        # Slice (bandpass 5–16 kHz)
        self.slice_on = True
        self.slice_low = 5000.0
        self.slice_high = 16000.0
        self.slice_gain_db = 0.0
        self._hpf = RBJ_Biquad(fs); self._hpf.set_hpf(self.slice_low, 0.707)
        self._lpf = RBJ_Biquad(fs); self._lpf.set_lpf(self.slice_high, 0.707)
        # LPF final 16 kHz
        self._lpf_out = LPFCascade(fs, 16000.0)
    def set_echo_on(self, on: bool): self.echo_on = bool(on)
    def set_echo_mix(self, mix): self.echo_mix = float(clamp(mix,0.0,1.0))
    def set_echo_time_ms(self, ms):
        self.echo_time_ms = float(clamp(ms, 50.0, 1200.0))
        self._delay_n = int(self.echo_time_ms*1e-3*self.fs)
    def set_comp(self, thresh_db=None, ratio=None):
        if thresh_db is not None: self.comp_thresh_db = float(clamp(thresh_db,-60.0,0.0))
        if ratio is not None: self.comp_ratio = float(clamp(ratio,1.0,12.0))
    def set_slice(self, on=None, low=None, high=None, gain_db=None):
        if on is not None: self.slice_on = bool(on)
        if low is not None:
            self.slice_low = float(clamp(low, 20.0, self.fs*0.45))
            self._hpf.set_hpf(self.slice_low, 0.707)
        if high is not None:
            self.slice_high = float(clamp(high, self.slice_low+100.0, self.fs*0.49))
            self._lpf.set_lpf(self.slice_high, 0.707)
        if gain_db is not None:
            self.slice_gain_db = float(clamp(gain_db, -24.0, 24.0))
    def _compress(self, x: np.ndarray):
        thr = db_to_lin(self.comp_thresh_db)
        env = self._env
        a_a, a_r = self._alpha_a, self._alpha_r
        out = np.empty_like(x)
        for i in range(x.size):
            s = abs(float(x[i]))
            env = a_a*env + (1-a_a)*s if s>env else a_r*env + (1-a_r)*s
            if env > thr:
                xdb = 20*math.log10(max(env,1e-12))
                gdb = (self.comp_thresh_db + (xdb - self.comp_thresh_db)/self.comp_ratio) - xdb
                g = db_to_lin(gdb)
            else:
                g = 1.0
            out[i] = x[i]*g
        self._env = env
        return out
    def _echo(self, x: np.ndarray):
        if not self.echo_on or self.echo_mix<=1e-6: return x
        N = x.size; y = np.empty_like(x); fb = 0.35
        w = self._widx; buf = self._buf
        dn = getattr(self, "_delay_n", int(self.echo_time_ms*1e-3*self.fs))
        for i in range(N):
            ridx = (w - dn) % self._max_delay_n
            d = buf[ridx]
            y[i] = (1.0 - self.echo_mix)*x[i] + self.echo_mix*d
            buf[w] = x[i] + d*fb
            w = (w+1) % self._max_delay_n
        self._widx = w
        return y
    def _slice(self, x: np.ndarray):
        if not self.slice_on: return x
        band = self._lpf.process(self._hpf.process(x))
        gain = db_to_lin(self.slice_gain_db)
        return x + (gain - 1.0)*band
    def process(self, x_mono: np.ndarray):
        y = self._compress(x_mono)
        y = self._echo(y)
        y = self._slice(y)
        y = self._lpf_out.process(y)
        return y

FX = FinalFX(FS)

# ========= Secuenciador (igual) =========
class FourBarSeqDouble:
    def __init__(self, bpm=138.0):
        self.sample_pos = 0
        self.bpm = float(bpm)
        self.double_time = False
        self._update_lengths()
        self.base_idx = 0
        self.sub_idx = 0
        self.pattern = [True, True, True, True]
    def _update_lengths(self):
        self.bar_len = int((60.0/self.bpm) * 4.0 * FS)
        self.sub_bar_len = max(1, self.bar_len // 2)
    def set_bpm(self, bpm):
        self.bpm = float(bpm)
        self._update_lengths()
    def set_double_time(self, on: bool):
        self.double_time = bool(on)
    def toggle_bar(self, i):
        self.pattern[i] = not self.pattern[i]
    def reset_to_start(self):
        self.sample_pos = 0
        self.base_idx = 0
        self.sub_idx = 0
    def _mirror_base_index(self, sub_idx):
        if sub_idx < 4:
            return sub_idx
        mirror_map = [3, 2, 1, 0]
        return mirror_map[sub_idx-4]
    def tick(self, n):
        s = self.sample_pos; e = s + n; self.sample_pos = e
        if not self.double_time:
            if (s // self.bar_len) != (e // self.bar_len):
                self.base_idx = (self.base_idx + 1) & 3
                self.sub_idx = self.base_idx
                return True
            return False
        else:
            if (s // self.sub_bar_len) != (e // self.sub_bar_len):
                self.sub_idx = (self.sub_idx + 1) % 8
                self.base_idx = self._mirror_base_index(self.sub_idx)
                return True
            return False
    def is_active_now(self):
        if not self.double_time:
            return self.pattern[self.base_idx]
        base_for_sub = self._mirror_base_index(self.sub_idx)
        return self.pattern[base_for_sub]

SEQ = FourBarSeqDouble(P.bpm)

# ========= UI =========
root = tk.Tk()
root.title("Bird FM – Camelot fc + DoubleTime+Mirror + FX + PRE + Salida")
root.geometry("1320x820")

main = ttk.Frame(root, padding=8); main.pack(fill="both", expand=True)
for c in (0,1): main.columnconfigure(c, weight=1)
for r in (2,): main.rowconfigure(r, weight=1)

audio_frame = ttk.LabelFrame(main, text="Salida de sonido", padding=8)
audio_frame.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0,8))

sel_dev_var = tk.StringVar(value="")
status_audio_var = tk.StringVar(value="Audio detenido")
combo = ttk.Combobox(audio_frame, textvariable=sel_dev_var, state="readonly", width=70)
combo.grid(row=0, column=0, padx=(0,6), sticky="ew")

startstop_btn = ttk.Button(audio_frame, text="Iniciar audio")
startstop_btn.grid(row=0, column=2, padx=6)
status_lbl = ttk.Label(audio_frame, textvariable=status_audio_var)
status_lbl.grid(row=0, column=3, padx=6)

audio_frame.columnconfigure(0, weight=1)

# etiqueta para Camelot
camelot_var = tk.StringVar(value=f"Clave Camelot: {CAMELOT_ORDER[P.camelot_idx]}")
camelot_lbl = ttk.Label(audio_frame, textvariable=camelot_var)
camelot_lbl.grid(row=0, column=4, padx=10)

# DoubleTime state label
double_lbl_var = tk.StringVar(value="DoubleTime: OFF (BTN 25)")
double_lbl = ttk.Label(audio_frame, textvariable=double_lbl_var)
double_lbl.grid(row=0, column=5, padx=10)

# Helpers dispositivos
def list_output_devices():
    devices = sd.query_devices()
    out = []
    for idx, d in enumerate(devices):
        if d.get('max_output_channels', 0) >= 1:
            api = sd.query_hostapis()[d['hostapi']]['name']
            name = f"{idx}: {d['name']} [{api}]"
            out.append((idx, name))
    return out

def refresh_devices():
    items = list_output_devices()
    names = [n for _,n in items]
    combo["values"] = names
    if names:
        try:
            def_out = sd.default.device[1]
        except Exception:
            def_out = items[0][0]
        pick = None
        for idx, name in items:
            if idx == def_out:
                pick = name; break
        sel_dev_var.set(pick or names[0])

refresh_btn = ttk.Button(audio_frame, text="Refrescar", command=lambda: refresh_devices())
refresh_btn.grid(row=0, column=1, padx=6)

# ----- Valores síntesis -----
vals_frame = ttk.LabelFrame(main, text="Valores actuales (síntesis)", padding=8)
vals_frame.grid(row=1, column=0, sticky="nsew", padx=(0,8))

val_fc   = tk.StringVar(value="fc: 440.0 Hz (8A)")
val_fm   = tk.StringVar(value="fm: 10.0 Hz")
val_beta = tk.StringVar(value="beta: 80.0")
val_decay= tk.StringVar(value="decay: 4.0 s^-1")
val_bpm  = tk.StringVar(value="BPM: 138.0")
val_gain = tk.StringVar(value="Gain: 0.50")

for r,(lbl,var) in enumerate([
    ("fc",val_fc),("fm",val_fm),("beta",val_beta),("decay",val_decay),("BPM",val_bpm),("Gain",val_gain)
]):
    ttk.Label(vals_frame, text=lbl+":").grid(row=r, column=0, sticky="w", pady=1)
    ttk.Label(vals_frame, textvariable=var).grid(row=r, column=1, sticky="w", pady=1)

bars_frame = ttk.LabelFrame(vals_frame, text="Patrón 4 compases (botones 7..4)")
bars_frame.grid(row=6, column=0, columnspan=2, sticky="ew", pady=(8,0))
bar_labels=[]
for i in range(4):
    lbl = tk.Label(bars_frame, text=f"Bar {i+1}", width=10, bd=0, fg="#fff")
    lbl.grid(row=0, column=i, padx=4, pady=2)
    bar_labels.append(lbl)

# ----- PRE-BP Panel -----
prebp_frame = ttk.LabelFrame(main, text="PRE-Bandpass (antes de FX)", padding=8)
prebp_frame.grid(row=1, column=1, sticky="nsew")

prebp_on_var = tk.BooleanVar(value=P.prebp_on)
prebp_low_var = tk.DoubleVar(value=P.prebp_low)
prebp_high_var = tk.DoubleVar(value=P.prebp_high)
prebp_gain_var = tk.DoubleVar(value=P.prebp_gain_db)

rowp = 0

ttk.Checkbutton(prebp_frame, text="PRE-BP ON", variable=prebp_on_var).grid(row=rowp, column=0, sticky="w")
rowp += 1

ttk.Label(prebp_frame, text="Low Cut (Hz)").grid(row=rowp, column=0, sticky="w")
sc_pre_low = ttk.Scale(prebp_frame, from_=20, to=int(FS*0.45), orient="horizontal", variable=prebp_low_var)
sc_pre_low.grid(row=rowp, column=1, sticky="ew", padx=6)
rowp += 1

ttk.Label(prebp_frame, text="High Cut (Hz)").grid(row=rowp, column=0, sticky="w")
sc_pre_high = ttk.Scale(prebp_frame, from_=200, to=int(FS*0.49), orient="horizontal", variable=prebp_high_var)
sc_pre_high.grid(row=rowp, column=1, sticky="ew", padx=6)
rowp += 1

ttk.Label(prebp_frame, text="BP Gain (dB)").grid(row=rowp, column=0, sticky="w")
sc_pre_gain = ttk.Scale(prebp_frame, from_=-24, to=24, orient="horizontal", variable=prebp_gain_var)
sc_pre_gain.grid(row=rowp, column=1, sticky="ew", padx=6)

prebp_frame.columnconfigure(1, weight=1)

# ----- FX Panel -----
fx_frame = ttk.LabelFrame(main, text="FX Final (Comp + Echo + Slice + LPF)", padding=8)
fx_frame.grid(row=2, column=0, sticky="nsew", padx=(0,8))

echo_on_var = tk.BooleanVar(value=P.fx_echo_on)
echo_mix_var = tk.DoubleVar(value=P.fx_echo_mix*100)
echo_time_var = tk.DoubleVar(value=P.fx_echo_time_ms)

ttk.Checkbutton(fx_frame, text="Echo ON", variable=echo_on_var).grid(row=0, column=0, sticky="w")

ttk.Label(fx_frame, text="Echo Mix (%)").grid(row=1, column=0, sticky="w")
sc_echo_mix = ttk.Scale(fx_frame, from_=0, to=100, orient="horizontal", variable=echo_mix_var)
sc_echo_mix.grid(row=1, column=1, sticky="ew", padx=6)

ttk.Label(fx_frame, text="Echo Time (ms)").grid(row=2, column=0, sticky="w")
sc_echo_time = ttk.Scale(fx_frame, from_=50, to=1200, orient="horizontal", variable=echo_time_var)
sc_echo_time.grid(row=2, column=1, sticky="ew", padx=6)

slice_on_var = tk.BooleanVar(value=P.fx_slice_on)
slice_low_var = tk.DoubleVar(value=P.fx_slice_low)
slice_high_var = tk.DoubleVar(value=P.fx_slice_high)
slice_gain_var = tk.DoubleVar(value=P.fx_slice_gain_db)

row0 = 3

ttk.Checkbutton(fx_frame, text="Slice ON (5–16k por defecto)", variable=slice_on_var).grid(row=row0, column=0, sticky="w")

ttk.Label(fx_frame, text="Low Cut (Hz)").grid(row=row0+1, column=0, sticky="w")
sc_slice_low = ttk.Scale(fx_frame, from_=1000, to=int(FS*0.45), orient="horizontal", variable=slice_low_var)
sc_slice_low.grid(row=row0+1, column=1, sticky="ew", padx=6)

ttk.Label(fx_frame, text="High Cut (Hz)").grid(row=row0+2, column=0, sticky="w")
sc_slice_high = ttk.Scale(fx_frame, from_=2000, to=int(FS*0.49), orient="horizontal", variable=slice_high_var)
sc_slice_high.grid(row=row0+2, column=1, sticky="ew", padx=6)

ttk.Label(fx_frame, text="Slice Gain (dB)").grid(row=row0+3, column=0, sticky="w")
sc_slice_gain = ttk.Scale(fx_frame, from_=-24, to=24, orient="horizontal", variable=slice_gain_var)
sc_slice_gain.grid(row=row0+3, column=1, sticky="ew", padx=6)

comp_thresh_var = tk.DoubleVar(value=P.fx_comp_thresh_db)
comp_ratio_var  = tk.DoubleVar(value=P.fx_comp_ratio)

row1 = row0+5

ttk.Label(fx_frame, text="Comp Threshold (dB)").grid(row=row1, column=0, sticky="w")
sc_cth = ttk.Scale(fx_frame, from_=-40, to=0, orient="horizontal", variable=comp_thresh_var)
sc_cth.grid(row=row1, column=1, sticky="ew", padx=6)

ttk.Label(fx_frame, text="Comp Ratio").grid(row=row1+1, column=0, sticky="w")
sc_cr = ttk.Scale(fx_frame, from_=1, to=8, orient="horizontal", variable=comp_ratio_var)
sc_cr.grid(row=row1+1, column=1, sticky="ew", padx=6)

fx_frame.columnconfigure(1, weight=1)

# ----- Osciloscopio -----
scope_frame = ttk.LabelFrame(main, text="Osciloscopio", padding=6)
scope_frame.grid(row=2, column=1, sticky="nsew")
scope_canvas = tk.Canvas(scope_frame, bg="#111"); scope_canvas.pack(fill="both", expand=True)
ctrl = ttk.Frame(scope_frame); ctrl.pack(fill="x", pady=6)
normalize_var = tk.BooleanVar(value=True); ttk.Checkbutton(ctrl, text="Normalizar", variable=normalize_var).pack(side="left")
time_ms_var = tk.IntVar(value=500); ttk.Label(ctrl, text="Ventana (ms):").pack(side="left", padx=(10,2))
slider_t = ttk.Scale(ctrl, from_=50, to=2000, orient="horizontal", variable=time_ms_var); slider_t.pack(side="left", fill="x", expand=True)
scale_v_var = tk.DoubleVar(value=1.0); ttk.Label(ctrl, text="±V:").pack(side="left", padx=(10,2))
slider_v = ttk.Scale(ctrl, from_=0.1, to=2.5, orient="horizontal", variable=scale_v_var); slider_v.pack(side="left", fill="x", expand=True)

# ====== Audio stream mgmt ======
audio_stream = None
audio_lock = threading.Lock()

SCOPE_SECONDS=5
SCOPE_LEN=int(FS*SCOPE_SECONDS)
scope_ring=np.zeros(SCOPE_LEN, dtype=np.float32)
scope_idx=0
scope_lock=threading.Lock()
current_base_idx_atomic=0
current_sub_idx_atomic=0

ACTIVE_COL = "#3a86ff"; OFF_COL = "#444"; IDLE_COL = "#222"

def get_selected_output_index():
    name = sel_dev_var.get()
    if not name: return None
    try: return int(name.split(":",1)[0])
    except Exception: return None

def start_audio():
    global audio_stream
    with audio_lock:
        if audio_stream is not None: return
        device_index = get_selected_output_index()
        try:
            audio_stream = sd.OutputStream(samplerate=FS, blocksize=BLOCK, channels=2,
                                           dtype="float32", callback=audio_cb, device=device_index)
            audio_stream.start()
            status_audio_var.set(f"Audio en marcha (device={device_index if device_index is not None else 'default'})")
            startstop_btn.configure(text="Detener audio")
        except Exception as e:
            status_audio_var.set(f"Error al iniciar: {e}")
            audio_stream = None

def stop_audio():
    global audio_stream
    with audio_lock:
        if audio_stream is None: return
        try:
            audio_stream.stop(); audio_stream.close()
        except Exception:
            pass
        audio_stream = None
        startstop_btn.configure(text="Iniciar audio")
        status_audio_var.set("Audio detenido")

def restart_audio():
    was_running = (audio_stream is not None)
    stop_audio()
    if was_running: start_audio()

startstop_btn.configure(command=lambda: start_audio() if audio_stream is None else stop_audio())
combo.bind("<<ComboboxSelected>>", lambda e: restart_audio())

# ===== Scope buffers =====

def scope_write(block):
    global scope_idx
    n = block.size
    with scope_lock:
        end = scope_idx + n
        if end <= SCOPE_LEN:
            scope_ring[scope_idx:end] = block
        else:
            k = SCOPE_LEN - scope_idx
            scope_ring[scope_idx:] = block[:k]
            scope_ring[:end % SCOPE_LEN] = block[k:]
        scope_idx = (scope_idx + n) % SCOPE_LEN

def scope_read_window(win_samples):
    with scope_lock:
        idx = scope_idx
        if win_samples >= SCOPE_LEN:
            data = np.concatenate((scope_ring[idx:], scope_ring[:idx]))
        else:
            start = (idx - win_samples) % SCOPE_LEN
            if start < idx:
                data = scope_ring[start:idx].copy()
            else:
                data = np.concatenate((scope_ring[start:], scope_ring[:idx]))
    return data

def draw_scope_from_ring():
    W=scope_canvas.winfo_width(); H=scope_canvas.winfo_height()
    if W<10 or H<10: return
    win_ms = clamp(int(time_ms_var.get()), 50, 2000)
    buf = scope_read_window(int(FS*(win_ms/1000.0)))
    if normalize_var.get():
        m = max(1e-6, float(np.max(np.abs(buf)))); y = buf/m; vlabel = "norm"
    else:
        vmax = max(1e-6, float(scale_v_var.get())); y = np.clip(buf/vmax, -1.0, 1.0); vlabel=f"±{vmax:.2f}"
    scope_canvas.delete("all")
    scope_canvas.create_line(0, H//2, W, H//2, fill="#333")
    n=y.size; last=None
    for x in range(W):
        idx=int(x/max(1,W-1)*(n-1)); v=0.5*(1-y[idx]); ypix=int(v*(H-1))
        if last is not None:
            scope_canvas.create_line(last[0], last[1], x, ypix, fill="#55ccff")
        last=(x, ypix)
    scope_canvas.create_text(8, 12, anchor="w", fill="#aaa", text=f"{win_ms} ms | {vlabel}")

# ========= Conexión + hilos =========
evт_queue: Queue = Queue(maxsize=256)

# Actualiza UI de valores síntesis
def set_value_labels(fc,fm,beta,decay,gain,bpm,camelot_key):
    val_fc.set(f"fc: {fc:.2f} Hz ({camelot_key})"); val_fm.set(f"fm: {fm:.2f} Hz"); val_beta.set(f"beta: {beta:.1f}")
    val_decay.set(f"decay: {decay:.2f} s^-1"); val_bpm.set(f"BPM: {bpm:.1f}"); val_gain.set(f"Gain: {gain:.2f}")
    camelot_var.set(f"Clave Camelot: {camelot_key}")

# Sync GUI -> Params/módulos
def sync_gui_to_engine():
    with P.lock:
        P.prebp_on = bool(prebp_on_var.get())
        lo = float(prebp_low_var.get()); hi = float(prebp_high_var.get())
        if hi <= lo + 100.0: hi = lo + 100.0; prebp_high_var.set(hi)
        P.prebp_low, P.prebp_high = lo, hi
        P.prebp_gain_db = float(prebp_gain_var.get())
        P.fx_echo_on = bool(echo_on_var.get())
        P.fx_echo_mix = float(echo_mix_var.get()/100.0)
        P.fx_echo_time_ms = float(echo_time_var.get())
        P.fx_slice_on = bool(slice_on_var.get())
        slo = float(slice_low_var.get()); shi = float(slice_high_var.get())
        if shi <= slo + 100.0: shi = slo + 100.0; slice_high_var.set(shi)
        P.fx_slice_low, P.fx_slice_high = slo, shi
        P.fx_slice_gain_db = float(slice_gain_var.get())
        P.fx_comp_thresh_db = float(comp_thresh_var.get())
        P.fx_comp_ratio = float(comp_ratio_var.get())
    PRE.set_params(P.prebp_on, P.prebp_low, P.prebp_high, P.prebp_gain_db)
    FX.set_echo_on(P.fx_echo_on)
    FX.set_echo_mix(P.fx_echo_mix)
    FX.set_echo_time_ms(P.fx_echo_time_ms)
    FX.set_slice(P.fx_slice_on, P.fx_slice_low, P.fx_slice_high, P.fx_slice_gain_db)
    FX.set_comp(P.fx_comp_thresh_db, P.fx_comp_ratio)
    root.after(120, sync_gui_to_engine)

# HID reader
def hid_thread():
    dev=None; prev_btn_bits=0; prev_cc={}
    while True:
        try:
            if dev is None:
                path = find_f1_path()
                if path:
                    dev = hid.device(); dev.open_path(path); dev.set_nonblocking(False)
                else:
                    time.sleep(0.5); continue
            data = dev.read(64, timeout_ms=80)
            if not data: continue
            if data[0] != 0x01:
                if len(data)>=23 and data[1]==0x01: data=data[1:23]
                else: continue
            if len(data) < 22: continue
            rep = bytes(data[:22])
            btn_bits = rep[1] | (rep[2]<<8) | (rep[3]<<16) | (rep[4]<<24)
            changed = btn_bits ^ prev_btn_bits
            if changed:
                for bit in range(0, 32):
                    if changed & (1<<bit):
                        if bool(btn_bits & (1<<bit)):
                            evt_queue.put(("BTN", bit))
                prev_btn_bits = btn_bits
            axes = struct.unpack("<8H", rep[6:22])
            for i, vr in enumerate(axes):
                id7 = 30+i
                val7 = map_axis_to_7bit(vr)
                if prev_cc.get(id7) != val7:
                    prev_cc[id7] = val7
                    evt_queue.put(("CC", id7, val7))
        except Exception:
            try:
                if dev: dev.close()
            except Exception:
                pass
            dev=None; time.sleep(0.5)

# Dispatcher
def dispatcher_thread():
    while True:
        try:
            item = evt_queue.get()
            if not item: continue
            if item[0] == "BTN":
                bit = item[1]
                if bit in (7,6,5,4):
                    idx = 7 - bit
                    SEQ.toggle_bar(idx)
                elif bit == 26:
                    with P.lock:
                        P.fx_echo_on = not P.fx_echo_on
                        echo_on_var.set(P.fx_echo_on)
                elif bit == 27:
                    with P.lock:
                        P.seq_on = not P.seq_on
                        playing = P.seq_on
                    if playing:
                        SEQ.reset_to_start()
                        if SEQ.pattern[0]:
                            with P.lock: P.trigger = True
                elif bit == 25:
                    with P.lock:
                        P.double_time = not P.double_time
                        SEQ.set_double_time(P.double_time)
                    double_lbl_var.set(f"DoubleTime: {'ON' if P.double_time else 'OFF'} (BTN 25)")
            else:
                cc, val7 = item[1], item[2]
                with P.lock:
                    if cc==30:
                        # recorre 0..23 → 24 claves Camelot
                        P.camelot_idx = int(round(map_7bit(val7, 0, len(CAMELOT_ORDER)-1)))
                    elif cc==31:
                        P.fm   = map_7bit(val7, 8.0, 435.0)
                    elif cc==34:
                        P.beta = map_7bit(val7, 1.0, 100.0)
                    elif cc==35:
                        P.decay= map_7bit(val7, 0.1, 7.0)
                    elif cc==32:
                        P.bpm  = map_7bit(val7, 128.0, 150.0)
                    elif cc==37:
                        P.gain = map_7bit(val7, 0.0, 0.70)
                    elif cc==33:
                        P.fx_echo_mix = map_7bit(val7, 0.0, 1.0); echo_mix_var.set(P.fx_echo_mix*100)
                    elif cc==36:
                        P.fx_echo_time_ms = map_7bit(val7, 50.0, 1200.0); echo_time_var.set(P.fx_echo_time_ms)
                # Refrescar label con nueva clave
                with P.lock:
                    key = CAMELOT_ORDER[P.camelot_idx]
                    fc_now = CAMELOT_FREQ[key]
                    vals = (fc_now, P.fm, P.beta, P.decay, P.gain, P.bpm, key)
                root.after(0, lambda v=vals: set_value_labels(*v))
        except Exception:
            time.sleep(0.005)

# ========= Audio callback =========
def audio_cb(outdata, frames, time_info, status):
    global current_base_idx_atomic, current_sub_idx_atomic
    vals = P.snapshot()
    (camelot_idx,fm,beta,decay,gain,bpm,muted,seq_on,trig,
     e_on,e_mix,e_ms,s_on,s_lo,s_hi,s_db,c_th,c_ra,
     pb_on,pb_lo,pb_hi,pb_g,
     dbl) = vals
    key = CAMELOT_ORDER[camelot_idx]
    fc = CAMELOT_FREQ[key]
    if status:
        print(status, file=sys.stderr)
    fc,fm,beta,decay,gain,bpm = SM.step(fc,fm,beta,decay,gain,bpm)
    SEQ.set_bpm(bpm)
    SEQ.set_double_time(dbl)
    trig_gate=False
    if trig:
        with P.lock: P.trigger=False
        trig_gate=True
    if seq_on and SEQ.tick(frames):
        current_base_idx_atomic = SEQ.base_idx
        current_sub_idx_atomic = SEQ.sub_idx
        if SEQ.is_active_now():
            trig_gate=True
    y = OSC.process(fc,fm,beta,decay,gain,trig_gate,frames)
    if muted: y *= 0.0
    PRE.set_params(pb_on, pb_lo, pb_hi, pb_g)
    y = PRE.process(y)
    out_m = FX.process(y)
    outdata[:,0]=out_m
    if outdata.shape[1]>1: outdata[:,1]=out_m
    scope_write(out_m)

# ===== UI loops =====
def refresh_bars():
    idx = current_base_idx_atomic
    for i,lbl in enumerate(bar_labels):
        on = SEQ.pattern[i]
        is_current = (i==idx)
        if is_current and on:
            lbl.config(bg=ACTIVE_COL, fg="#fff")
        elif on:
            lbl.config(bg=IDLE_COL, fg="#fff")
        else:
            lbl.config(bg=OFF_COL, fg="#bbb")
    root.after(40, refresh_bars)

def draw_scope_from_ring_loop():
    draw_scope_from_ring(); root.after(30, draw_scope_from_ring_loop)

# Lanzar hilos auxiliares
threading.Thread(target=hid_thread, daemon=True).start()
threading.Thread(target=dispatcher_thread, daemon=True).start()

# Inicializaciones
refresh_devices()
refresh_bars(); draw_scope_from_ring_loop(); sync_gui_to_engine()

# Mainloop
root.mainloop()
