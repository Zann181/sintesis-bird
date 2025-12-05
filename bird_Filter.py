#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Bird FM Synth — Doble Velocidad + Espejo (8 compases en tiempo de 4)
--------------------------------------------------------------------
- Mantiene el mismo BPM, pero permite que en el mismo lapso de 4 compases entren 8 sub‑compases.
- MODO: "DoubleTime + Mirror" (doble velocidad + espejo):
    * Internamente se divide cada compás en 2 sub‑compases.
    * Los sub‑compases 0..3 (primera mitad) usan el patrón de 4 compases del usuario.
    * Los sub‑compases 4..7 (segunda mitad) son el espejo del patrón: [Bar4, Bar3, Bar2, Bar1].
- Resultado: se oye el doble de eventos en el mismo tiempo (BPM invariable), con la segunda mitad espejada.
- Control:
    * BTN 27 (F1): Divide BPM por 2
    * BTN 26 (F1): Multiplica BPM por 2
    * BTN 19 (F1): Play/Pause del secuenciador
    * BTN 25 (F1): Activa/Desactiva DoubleTime+Mirror
    * Botones 7..4 siguen alternando las 4 barras base.
- UI: Etiqueta de estado "DoubleTime", osciloscopio y ecualizador PRE-Bandpass.
"""

import sys, time, math, threading, struct, argparse, os
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
FS = 48000
BLOCK = 512
F1_NAME = "Traktor Kontrol F1"
F1_VENDOR_ID = 0x17CC
BPM_MIN = 60.0
BPM_MAX = 300.0
PREBP_LOW_MIN = 20.0
PREBP_LOW_MAX = FS * 0.45
PREBP_HIGH_MIN = 200.0
PREBP_HIGH_MAX = FS * 0.49

# ========= Util =========
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def map_axis_to_7bit(v_raw):
    return clamp(int(round(v_raw/32.0)), 0, 127)

def map_7bit(v, lo, hi):
    return lo + (hi-lo) * (clamp(int(v),0,127)/127.0)

def freq_to_log_slider(freq, fmin, fmax):
    freq = clamp(float(freq), fmin, fmax)
    span = math.log(fmax/fmin)
    if span <= 0:
        return 0.0
    return math.log(freq/fmin) / span

def log_slider_to_freq(norm, fmin, fmax):
    t = clamp(float(norm), 0.0, 1.0)
    return fmin * math.exp(t * math.log(fmax/fmin))

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
        # Síntesis
        self.fc, self.fm, self.beta, self.decay = 2343.75, 10.0, 80.0, 4.0
        self.gain, self.bpm = 0.5, 138.0
        self.muted, self.seq_on = False, True
        self.trigger = False
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
            return (self.fc, self.fm, self.beta, self.decay, self.gain,
                    self.bpm, self.muted, self.seq_on, self.trigger,
                    self.fx_echo_on, self.fx_echo_mix, self.fx_echo_time_ms,
                    self.fx_slice_on, self.fx_slice_low, self.fx_slice_high, self.fx_slice_gain_db,
                    self.fx_comp_thresh_db, self.fx_comp_ratio,
                    self.prebp_on, self.prebp_low, self.prebp_high, self.prebp_gain_db,
                    self.double_time)
    def fast_snapshot(self):
        # Versión sin lock pensada para el callback de audio, para evitar
        # que el hilo de audio se bloquee si otro hilo está actualizando P.
        return (self.fc, self.fm, self.beta, self.decay, self.gain,
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
        self.state = dict(fc=P.fc, fm=P.fm, beta=P.beta, decay=P.decay, gain=P.gain, bpm=P.bpm)
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

# ========= Ventaneo =========
_WINDOW_CACHE = {}

def get_hann_window(n: int, normalize: bool = False) -> np.ndarray:
    """Devuelve una ventana Hann de longitud n, opcionalmente normalizada a media=1."""
    n_int = max(1, int(n))
    key = (n_int, bool(normalize))
    w = _WINDOW_CACHE.get(key)
    if w is None:
        w = np.hanning(n_int).astype(np.float32)
        if normalize:
            mean = float(np.mean(w))
            if mean > 1e-9:
                w = w / mean
        _WINDOW_CACHE[key] = w
    return w

def apply_window_inplace(block: np.ndarray, normalize: bool = False):
    """Multiplica un bloque por la ventana Hann pedida sin asignar memoria extra."""
    if block is None or block.size == 0:
        return block
    w = get_hann_window(block.size, normalize=normalize)
    block *= w
    return block

# ========= Biquads =========
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
        # Avoid reinitializing filters unless a parameter actually changed.
        if on is not None:
            self.on = bool(on)
        if low is not None:
            new_low = float(clamp(low, 20.0, self.fs*0.45))
            if abs(new_low - self.low) > 1e-6:
                self.low = new_low
                self._hpf.set_hpf(self.low, 0.707)
        if high is not None:
            new_high = float(clamp(high, self.low+100.0, self.fs*0.49))
            if abs(new_high - self.high) > 1e-6:
                self.high = new_high
                self._lpf.set_lpf(self.high, 0.707)
        if gain_db is not None:
            new_gain = float(clamp(gain_db, -24.0, 24.0))
            if abs(new_gain - self.gain_db) > 1e-6:
                self.gain_db = new_gain
    def process(self, x: np.ndarray):
        if not self.on: return x
        band = self._lpf.process(self._hpf.process(x))
        g = db_to_lin(self.gain_db)
        return band * g

PRE = PreBandpass(FS, low=P.prebp_low, high=P.prebp_high, gain_db=P.prebp_gain_db)

# ========= Echo =========
class SimpleEcho:
    def __init__(self, fs, max_delay_ms=1500.0, feedback=0.35,
                 init_on=True, init_mix=0.25, init_time_ms=350.0):
        self.fs = float(fs)
        self.max_delay_n = max(2, int(self.fs * (max_delay_ms/1000.0)))
        self.buf = np.zeros(self.max_delay_n, dtype=np.float32)
        self.widx = 0
        self.feedback = float(feedback)
        self.mix_alpha = 0.02          # suave pero rápido
        self.delay_alpha = 0.005       # cambio de tiempo más lento para evitar clicks
        self.on = bool(init_on)
        self._mix_target = clamp(init_mix, 0.0, 1.0)
        self._mix_state = self._mix_target
        init_delay = self._ms_to_samples(init_time_ms)
        self._delay_target = init_delay
        self._delay_state = float(init_delay)

    def _ms_to_samples(self, time_ms):
        samples = int(self.fs * float(time_ms) / 1000.0)
        return clamp(samples, 1, self.max_delay_n - 1)

    def set_params(self, on=None, mix=None, time_ms=None, feedback=None):
        if on is not None:
            self.on = bool(on)
        if mix is not None:
            self._mix_target = clamp(float(mix), 0.0, 1.0)
        if time_ms is not None:
            self._delay_target = self._ms_to_samples(time_ms)
        if feedback is not None:
            self.feedback = float(clamp(feedback, -0.95, 0.95))

    def process(self, x_block: np.ndarray):
        if x_block is None or x_block.size == 0:
            return x_block
        buf = self.buf
        w = self.widx
        max_n = self.max_delay_n
        fb = self.feedback if self.on else 0.0
        mix_target = self._mix_target if self.on else 0.0
        mix_state = self._mix_state
        delay_state = self._delay_state
        delay_target = float(self._delay_target)
        mix_alpha = self.mix_alpha
        delay_alpha = self.delay_alpha

        for i in range(x_block.size):
            mix_state += mix_alpha * (mix_target - mix_state)
            delay_state += delay_alpha * (delay_target - delay_state)
            delay_n = int(delay_state)
            if delay_n < 1:
                delay_n = 1
            elif delay_n >= max_n:
                delay_n = max_n - 1
            ridx = w - delay_n
            if ridx < 0:
                ridx += max_n
            d = buf[ridx]
            xn = float(x_block[i])
            buf[w] = xn + d * fb
            x_block[i] = (1.0 - mix_state) * xn + mix_state * d
            w += 1
            if w == max_n:
                w = 0

        self.widx = w
        self._mix_state = mix_state
        self._delay_state = delay_state
        return x_block

ECHO = SimpleEcho(FS,
                  max_delay_ms=1500.0,
                  feedback=0.4,
                  init_on=P.fx_echo_on,
                  init_mix=P.fx_echo_mix,
                  init_time_ms=P.fx_echo_time_ms)

# ========= Secuenciador: 4 compases base + sub‑compases (8) =========
class FourBarSeqDouble:
    """Secuenciador con 4 compases base. Cuando `double_time=True` divide cada compás en 2 sub‑compases,
    de modo que en el tiempo de 4 compases caben 8 sub‑compases. Los sub‑compases 4..7 son espejo.
    """
    def __init__(self, bpm=138.0):
        self.sample_pos = 0
        self.bpm = float(bpm)
        self.double_time = False
        self._update_lengths()
        self.base_idx = 0        # 0..3 (compás base)
        self.sub_idx = 0         # 0..7 (sub‑compás cuando double_time)
        self.pattern = [True, True, True, True]
    def _update_lengths(self):
        # Un compás = 4 negras (4/4). Duración compás base (4 beats):
        self.bar_len = int((60.0/self.bpm) * 4.0 * FS)
        # Sub‑compás en doble velocidad: la mitad
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
        # Mapeo espejo para sub_idx 0..7 -> índice base 0..3
        # 0..3: usa 0..3, 4..7: usa 3,2,1,0
        if sub_idx < 4:
            return sub_idx
        mirror_map = [3, 2, 1, 0]  # espejo de [0,1,2,3]
        return mirror_map[sub_idx-4]
    def tick(self, n):
        """Avanza n samples. Devuelve True si hay *borde de disparo* (inicio de compás o sub‑compás),
        y actualiza los índices actuales. Usa `self.double_time` para decidir la granularidad.
        """
        s = self.sample_pos; e = s + n; self.sample_pos = e
        if not self.double_time:
            # Granularidad compás base
            if (s // self.bar_len) != (e // self.bar_len):
                self.base_idx = (self.base_idx + 1) & 3
                self.sub_idx = (self.base_idx)  # mantener coherencia visual 0..3
                return True
            return False
        else:
            # Granularidad sub‑compás (8 en total por ciclo de 4 compases)
            if (s // self.sub_bar_len) != (e // self.sub_bar_len):
                self.sub_idx = (self.sub_idx + 1) % 8
                # Deriva base_idx para UI (0..3) desde sub_idx
                self.base_idx = self._mirror_base_index(self.sub_idx)
                return True
            return False
    def is_active_now(self):
        """Devuelve si el sub‑compás/compás actual debe disparar según patrón y modo."""
        if not self.double_time:
            return self.pattern[self.base_idx]
        base_for_sub = self._mirror_base_index(self.sub_idx)
        return self.pattern[base_for_sub]

SEQ = FourBarSeqDouble(P.bpm)

# ========= UI =========
root = tk.Tk()
root.title("Bird FM – DoubleTime+Mirror (8 en 4) + EQ PRE + Salida")
root.geometry("1200x780")

main = ttk.Frame(root, padding=8); main.pack(fill="both", expand=True)
for c in (0,1): main.columnconfigure(c, weight=1)
for r in (2,): main.rowconfigure(r, weight=1)

# ----- Panel de AUDIO OUT -----
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

# DoubleTime state label
double_lbl_var = tk.StringVar(value="DoubleTime: OFF (BTN 25)")
double_lbl = ttk.Label(audio_frame, textvariable=double_lbl_var)
double_lbl.grid(row=0, column=4, padx=10)

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

refresh_btn = ttk.Button(audio_frame, text="Refrescar", command=lambda: refresh_devices())
refresh_btn.grid(row=0, column=1, padx=6)

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

# ----- Valores síntesis -----
vals_frame = ttk.LabelFrame(main, text="Valores actuales (síntesis)", padding=8)
vals_frame.grid(row=1, column=0, sticky="nsew", padx=(0,8))

val_fc   = tk.StringVar(value=f"fc: {P.fc:.1f} Hz")
val_fm   = tk.StringVar(value=f"fm: {P.fm:.2f} Hz")
val_beta = tk.StringVar(value=f"beta: {P.beta:.1f}")
val_decay= tk.StringVar(value=f"decay: {P.decay:.2f} s^-1")
val_bpm  = tk.StringVar(value=f"BPM: {P.bpm:.1f}")
val_gain = tk.StringVar(value=f"Gain: {P.gain:.2f}")
val_echo_mix = tk.StringVar(value=f"Echo Mix: {P.fx_echo_mix:.2f}")
val_echo_time = tk.StringVar(value=f"Echo Time: {P.fx_echo_time_ms:.0f} ms")
seq_play_var = tk.BooleanVar(value=P.seq_on)

for r,(lbl,var) in enumerate([
    ("fc",val_fc),("fm",val_fm),("beta",val_beta),("decay",val_decay),
    ("BPM",val_bpm),("Gain",val_gain),("Echo Mix", val_echo_mix),("Echo Time", val_echo_time)
]):
    ttk.Label(vals_frame, text=lbl+":").grid(row=r, column=0, sticky="w", pady=1)
    ttk.Label(vals_frame, textvariable=var).grid(row=r, column=1, sticky="w", pady=1)

bpm_btn_frame = ttk.Frame(vals_frame)
bpm_btn_frame.grid(row=8, column=0, columnspan=2, sticky="ew", pady=(6,0))
ttk.Button(bpm_btn_frame, text="BPM ÷2", command=lambda: apply_bpm_multiplier(0.5)).pack(side="left", expand=True, fill="x", padx=(0,4))
ttk.Button(bpm_btn_frame, text="BPM ×2", command=lambda: apply_bpm_multiplier(2.0)).pack(side="left", expand=True, fill="x")

ttk.Checkbutton(vals_frame, text="Secuencia ON", variable=seq_play_var,
                command=lambda: set_seq_play_state(seq_play_var.get())
               ).grid(row=9, column=0, columnspan=2, sticky="w", pady=(6,0))

bars_frame = ttk.LabelFrame(vals_frame, text="Patrón 4 compases (botones 7..4)")
bars_frame.grid(row=10, column=0, columnspan=2, sticky="ew", pady=(8,0))
bar_labels=[]
for i in range(4):
    lbl = tk.Label(bars_frame, text=f"Bar {i+1}", width=10, bd=0, fg="#fff")
    lbl.grid(row=0, column=i, padx=4, pady=2)
    bar_labels.append(lbl)

# ----- EQ PRE-BP Panel -----
prebp_frame = ttk.LabelFrame(main, text="Ecualizador PRE-Bandpass", padding=8)
prebp_frame.grid(row=1, column=1, sticky="nsew")

prebp_on_var = tk.BooleanVar(value=P.prebp_on)
prebp_low_var = tk.DoubleVar(value=freq_to_log_slider(P.prebp_low, PREBP_LOW_MIN, PREBP_LOW_MAX))
prebp_high_var = tk.DoubleVar(value=freq_to_log_slider(P.prebp_high, PREBP_HIGH_MIN, PREBP_HIGH_MAX))
prebp_gain_var = tk.DoubleVar(value=P.prebp_gain_db)
prebp_low_readout = tk.StringVar(value=f"{P.prebp_low:.0f} Hz")
prebp_high_readout = tk.StringVar(value=f"{P.prebp_high:.0f} Hz")

rowp = 0

ttk.Checkbutton(prebp_frame, text="PRE-BP ON", variable=prebp_on_var).grid(row=rowp, column=0, sticky="w")
rowp += 1

ttk.Label(prebp_frame, text="Low Cut (Hz)").grid(row=rowp, column=0, sticky="w")
sc_pre_low = ttk.Scale(prebp_frame, from_=0.0, to=1.0, orient="horizontal", variable=prebp_low_var)
sc_pre_low.grid(row=rowp, column=1, sticky="ew", padx=6)
ttk.Label(prebp_frame, textvariable=prebp_low_readout, width=10).grid(row=rowp, column=2, sticky="e")
rowp += 1

ttk.Label(prebp_frame, text="High Cut (Hz)").grid(row=rowp, column=0, sticky="w")
sc_pre_high = ttk.Scale(prebp_frame, from_=0.0, to=1.0, orient="horizontal", variable=prebp_high_var)
sc_pre_high.grid(row=rowp, column=1, sticky="ew", padx=6)
ttk.Label(prebp_frame, textvariable=prebp_high_readout, width=10).grid(row=rowp, column=2, sticky="e")
rowp += 1

ttk.Label(prebp_frame, text="BP Gain (dB)").grid(row=rowp, column=0, sticky="w")
sc_pre_gain = ttk.Scale(prebp_frame, from_=-24, to=24, orient="horizontal", variable=prebp_gain_var)
sc_pre_gain.grid(row=rowp, column=1, sticky="ew", padx=6)

prebp_frame.columnconfigure(1, weight=1)
prebp_frame.columnconfigure(2, weight=0)

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

def get_selected_output_index():
    name = sel_dev_var.get()
    if not name: return None
    try: return int(name.split(":",1)[0])
    except Exception: return None

audio_lock = threading.Lock()

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
        except Exception: pass
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
SCOPE_SECONDS=5
SCOPE_LEN=int(FS*SCOPE_SECONDS)
scope_ring=np.zeros(SCOPE_LEN, dtype=np.float32)
scope_idx=0
scope_lock=threading.Lock()
current_base_idx_atomic=0
current_sub_idx_atomic=0

ACTIVE_COL = "#3a86ff"; OFF_COL = "#444"; IDLE_COL = "#222"

def scope_write(block):
    global scope_idx
    n = block.size
    # No bloquear nunca el hilo de audio: si el lock está ocupado (UI
    # dibujando el osciloscopio), simplemente saltamos esta escritura.
    if not scope_lock.acquire(blocking=False):
        return
    try:
        end = scope_idx + n
        if end <= SCOPE_LEN:
            scope_ring[scope_idx:end] = block
        else:
            k = SCOPE_LEN - scope_idx
            scope_ring[scope_idx:] = block[:k]
            scope_ring[:end % SCOPE_LEN] = block[k:]
        scope_idx = (scope_idx + n) % SCOPE_LEN
    finally:
        scope_lock.release()

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

def save_figures_from_buffer(buf: np.ndarray, sr: float, out_dir: str = None):
    """Guarda un espectrograma y un PSD por Welch en PNG."""
    try:
        import matplotlib
        if "matplotlib.pyplot" not in sys.modules:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        print(f"[warn] matplotlib no disponible: {e}", file=sys.stderr)
        return

    out_dir = out_dir or os.getcwd()
    try:
        os.makedirs(out_dir, exist_ok=True)
    except Exception:
        pass

    buf = np.asarray(buf, dtype=np.float32)
    spec_path = os.path.join(out_dir, "spectrogram.png")
    welch_path = os.path.join(out_dir, "welch_psd.png")
    try:
        spec_nfft = 1024
        spec_overlap = spec_nfft // 2
        spec_window = get_hann_window(spec_nfft, normalize=False)
        # Espectrograma
        fig, ax = plt.subplots(figsize=(8, 4))
        if buf.size == 0:
            ax.text(0.5, 0.5, "Sin datos", ha="center", va="center")
            ax.axis("off")
        else:
            ax.specgram(buf, NFFT=spec_nfft, Fs=sr,
                        noverlap=spec_overlap, window=spec_window, cmap="magma")
            ax.set_xlabel("Tiempo (s)"); ax.set_ylabel("Frecuencia (Hz)")
            ax.set_ylim(0, sr/2); ax.set_title("Espectrograma")
        fig.tight_layout(); fig.savefig(spec_path, dpi=150); plt.close(fig)

        # PSD por Welch
        welch_nfft = 2048
        welch_overlap = welch_nfft // 2
        welch_window = get_hann_window(welch_nfft, normalize=False)
        from matplotlib import mlab
        fig2, ax2 = plt.subplots(figsize=(8, 4))
        if buf.size == 0:
            ax2.text(0.5, 0.5, "Sin datos", ha="center", va="center")
            ax2.axis("off")
        else:
            pxx, freqs = mlab.psd(buf, NFFT=welch_nfft, Fs=sr,
                                  noverlap=welch_overlap, window=welch_window,
                                  scale_by_freq=True)
            pxx_db = 10.0 * np.log10(np.maximum(pxx, 1e-12))
            ax2.plot(freqs, pxx_db, color="#3a86ff", linewidth=1.0)
            ax2.set_xlim(0, sr/2)
            ax2.set_ylabel("dB/Hz")
            ax2.set_xlabel("Frecuencia (Hz)")
            ax2.set_title("PSD (Welch, dB)")
            ax2.grid(True, alpha=0.3)
        fig2.tight_layout(); fig2.savefig(welch_path, dpi=150); plt.close(fig2)

        print(f"[info] Guardado espectrograma en {spec_path}", file=sys.stderr)
        print(f"[info] Guardado PSD (Welch) en {welch_path}", file=sys.stderr)
    except Exception as e:
        print(f"[warn] No se pudieron guardar las figuras: {e}", file=sys.stderr)

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
evt_queue: Queue = Queue(maxsize=256)

# Actualiza UI de valores síntesis

def set_value_labels(fc,fm,beta,decay,gain,bpm,echo_mix,echo_time_ms):
    val_fc.set(f"fc: {fc:.1f} Hz"); val_fm.set(f"fm: {fm:.2f} Hz"); val_beta.set(f"beta: {beta:.1f}")
    val_decay.set(f"decay: {decay:.2f} s^-1"); val_bpm.set(f"BPM: {bpm:.1f}"); val_gain.set(f"Gain: {gain:.2f}")
    val_echo_mix.set(f"Echo Mix: {echo_mix:.2f}")
    val_echo_time.set(f"Echo Time: {echo_time_ms:.0f} ms")

def apply_bpm_multiplier(factor: float):
    if factor <= 0.0:
        return
    with P.lock:
        new_bpm = clamp(P.bpm * float(factor), BPM_MIN, BPM_MAX)
        P.bpm = new_bpm
        vals = (P.fc, P.fm, P.beta, P.decay, P.gain, P.bpm,
                P.fx_echo_mix, P.fx_echo_time_ms)
    root.after(0, lambda v=vals: set_value_labels(*v))

def set_seq_play_state(on: bool):
    on = bool(on)
    with P.lock:
        prev = P.seq_on
        P.seq_on = on
    if on and not prev:
        SEQ.reset_to_start()
        if SEQ.pattern[0]:
            with P.lock:
                P.trigger = True
    root.after(0, lambda state=on: seq_play_var.set(state))

# Sync GUI -> Params/módulos

def sync_gui_to_engine():
    with P.lock:
        # PRE-BP (Ecualizador)
        P.prebp_on = bool(prebp_on_var.get())
        lo_norm = clamp(float(prebp_low_var.get()), 0.0, 1.0)
        hi_norm = clamp(float(prebp_high_var.get()), 0.0, 1.0)
        lo = log_slider_to_freq(lo_norm, PREBP_LOW_MIN, PREBP_LOW_MAX)
        hi = log_slider_to_freq(hi_norm, PREBP_HIGH_MIN, PREBP_HIGH_MAX)
        if hi <= lo + 100.0:
            lo = clamp(lo, PREBP_LOW_MIN, PREBP_HIGH_MAX - 100.0)
            hi = lo + 100.0
            hi_norm = freq_to_log_slider(hi, PREBP_HIGH_MIN, PREBP_HIGH_MAX)
            prebp_high_var.set(hi_norm)
        P.prebp_low, P.prebp_high = lo, hi
        P.prebp_gain_db = float(prebp_gain_var.get())
        prebp_low_readout.set(f"{lo:.0f} Hz")
        prebp_high_readout.set(f"{hi:.0f} Hz")
    # El callback de audio aplicará estos parámetros al módulo PRE.
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
                elif bit == 27:
                    apply_bpm_multiplier(0.5)
                elif bit == 26:
                    apply_bpm_multiplier(2.0)
                elif bit == 19:
                    with P.lock:
                        current = P.seq_on
                    set_seq_play_state(not current)
                elif bit == 25:
                    with P.lock:
                        P.double_time = not P.double_time
                        SEQ.set_double_time(P.double_time)
                    double_lbl_var.set(f"DoubleTime: {'ON' if P.double_time else 'OFF'} (BTN 25)")
            else:
                cc, val7 = item[1], item[2]
                with P.lock:
                    if cc==30: P.fc   = map_7bit(val7, 435.0, 7000.0)
                    elif cc==31: P.fm   = map_7bit(val7, 8.0, 435.0)
                    elif cc==33: P.fx_echo_mix = map_7bit(val7, 0.0, 1.0)
                    elif cc==34: P.beta = map_7bit(val7, 1.0, 100.0)
                    elif cc==35: P.decay= map_7bit(val7, 0.1, 30.0)
                    elif cc==32: P.bpm  = map_7bit(val7, 128.0, 150.0)
                    elif cc==36: P.fx_echo_time_ms = map_7bit(val7, 80.0, 1200.0)
                    elif cc==37: P.gain = map_7bit(val7, 0.0, 0.70)
                vals = (P.fc, P.fm, P.beta, P.decay, P.gain, P.bpm,
                        P.fx_echo_mix, P.fx_echo_time_ms)
                root.after(0, lambda v=vals: set_value_labels(*v))
        except Exception:
            time.sleep(0.005)

# ========= Audio callback =========

def audio_cb(outdata, frames, time_info, status):
    global current_base_idx_atomic, current_sub_idx_atomic
    # Usar una snapshot sin lock para no bloquear al hilo de audio.
    vals = P.fast_snapshot()
    (fc,fm,beta,decay,gain,bpm,muted,seq_on,trig,
     e_on,e_mix,e_ms,s_on,s_lo,s_hi,s_db,c_th,c_ra,
     pb_on,pb_lo,pb_hi,pb_g,
     dbl) = vals
    # Evitar prints dentro del callback: pueden provocar más cortes.
    # if status:
    #     print(status, file=sys.stderr)
    fc,fm,beta,decay,gain,bpm = SM.step(fc,fm,beta,decay,gain,bpm)
    SEQ.set_bpm(bpm)
    SEQ.set_double_time(dbl)

    trig_gate=False
    if trig:
        # No usar lock aquí para no bloquear el hilo de audio;
        # un write simple de bool es suficientemente seguro en este contexto.
        P.trigger = False
        trig_gate=True

    if seq_on and SEQ.tick(frames):
        current_base_idx_atomic = SEQ.base_idx
        current_sub_idx_atomic = SEQ.sub_idx
        if SEQ.is_active_now():
            trig_gate=True

    y = OSC.process(fc,fm,beta,decay,gain,trig_gate,frames)
    apply_window_inplace(y, normalize=True)
    if muted: y *= 0.0

    # Aplicar parámetros del ecualizador PRE-BP en el propio hilo de audio
    # para que su estado interno no se mezcle con otros hilos.
    PRE.set_params(pb_on, pb_lo, pb_hi, pb_g)
    out_m = PRE.process(y)

    # Eco insertado entre PRE y el limitador final.
    ECHO.set_params(on=e_on, mix=e_mix, time_ms=e_ms)
    out_m = ECHO.process(out_m)

    # Limitador sencillo en salida para evitar clipping duro.
    peak = float(np.max(np.abs(out_m)))
    if peak > 0.99:
        out_m = (0.99 / peak) * out_m

    outdata[:,0]=out_m
    if outdata.shape[1]>1: outdata[:,1]=out_m
    scope_write(out_m)

# ===== UI loops =====

def refresh_bars():
    # Pinta los 4 labels base según el índice base actual
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

# Osciloscopio

def draw_scope_from_ring_loop():
    draw_scope_from_ring(); root.after(30, draw_scope_from_ring_loop)

def on_close():
    """Al cerrar: detener audio y guardar espectrograma + PSD (Welch)."""
    try:
        stop_audio()
    except Exception as e:
        print(f"[warn] Error al detener audio: {e}", file=sys.stderr)
    try:
        buf = scope_read_window(SCOPE_LEN)
        save_figures_from_buffer(buf, FS)
    except Exception as e:
        print(f"[warn] Error al guardar figuras: {e}", file=sys.stderr)
    root.destroy()

# Lanzar hilos auxiliares
threading.Thread(target=hid_thread, daemon=True).start()
threading.Thread(target=dispatcher_thread, daemon=True).start()

# Inicializaciones
refresh_devices()
refresh_bars(); draw_scope_from_ring_loop(); sync_gui_to_engine()
root.protocol("WM_DELETE_WINDOW", on_close)

# Mainloop
root.mainloop()
