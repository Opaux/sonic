import serial
import threading
import time
import math
import numpy as np
import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import plotly.graph_objs as go
from collections import deque

# ─────────────────────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────────────────────
SERIAL_PORT = 'COM6'
BAUD_RATE = 115200
WINDOW_N = 512
UPDATE_MS = 200

# We are now capturing true contiguous 48kHz blocks!
EFFECTIVE_FS = 48000

# ─────────────────────────────────────────────────────────────────────────────
# Shared State
# ─────────────────────────────────────────────────────────────────────────────
_lock = threading.Lock()

# These now hold perfect 512-sample snapshots
_ref_history = []
_err_history = []

_metrics = deque(maxlen=150)
_start_time = time.time()
_anc_status = "WAITING"


# ─────────────────────────────────────────────────────────────────────────────
# Serial Listener Thread
# ─────────────────────────────────────────────────────────────────────────────
def _serial_listener():
    global _anc_status, _ref_history, _err_history
    print(f"[Serial] Opening {SERIAL_PORT}...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print("[Serial] Connected! Waiting for data...")

        in_snap = False
        temp_ref = []
        temp_err = []

        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()

            if "SPM converged ANC ON" in line:
                _anc_status = "ON"
            elif "TRAINING" in line:
                _anc_status = "TRAINING"

            if line == "SNAP_START":
                in_snap = True
                temp_ref = []
                temp_err = []
                continue

            elif line == "SNAP_END":
                in_snap = False
                # Only update the main arrays if we got exactly 512 samples
                if len(temp_ref) == WINDOW_N:
                    with _lock:
                        _ref_history = temp_ref.copy()
                        _err_history = temp_err.copy()
                continue

            if in_snap and line.startswith("DATA:"):
                try:
                    parts = line.replace("DATA:", "").split(",")
                    temp_ref.append(float(parts[0]))
                    temp_err.append(float(parts[1]))
                except (ValueError, IndexError):
                    pass
    except Exception as e:
        print(f"[Serial] Error: {e}")


# ─────────────────────────────────────────────────────────────────────────────
# Dash Layout & Plot Styling
# ─────────────────────────────────────────────────────────────────────────────
_BASE_LAYOUT = dict(
    paper_bgcolor="#161b22", plot_bgcolor="#0d1117",
    font={"color": "#e6edf3", "size": 12},
    margin={"l": 60, "r": 20, "t": 40, "b": 40},
    legend={"bgcolor": "rgba(0,0,0,0)", "font": {"color": "#e6edf3"}},
)

_AXIS_STYLE = {
    "gridcolor": "#21262d", "zerolinecolor": "#21262d",
    "title_font": {"size": 13, "color": "#8b949e"}, "tickfont": {"color": "#8b949e"},
}


def _xax(title: str, **extra): return {**_AXIS_STYLE, "title": {"text": title}, **extra}


def _yax(title: str, **extra): return {**_AXIS_STYLE, "title": {"text": title}, **extra}


app = dash.Dash(__name__, title="SONIC FxLMS Dashboard")
app.layout = html.Div(
    style={"fontFamily": "Arial, sans-serif", "backgroundColor": "#0d1117", "minHeight": "100vh", "padding": "18px"},
    children=[
        html.Div(
            style={"display": "flex", "alignItems": "center", "marginBottom": "14px"},
            children=[
                html.H2("SONIC Active Noise Cancellation Dashboard", style={"color": "#e6edf3", "margin": "0 24px 0 0"}),
                html.Div(id="anc-indicator",
                         style={"padding": "8px 20px", "borderRadius": "12px", "fontWeight": "bold"})
            ],
        ),
        html.Div(
            style={"display": "grid", "gridTemplateColumns": "1fr 1fr", "gap": "12px", "marginBottom": "12px"},
            children=[dcc.Graph(id="wave-plot"), dcc.Graph(id="spectrum-plot")],
        ),
        html.Div(
            style={"display": "grid", "gridTemplateColumns": "1fr 1fr", "gap": "12px"},
            children=[dcc.Graph(id="atten-plot"), dcc.Graph(id="rms-plot")],
        ),
        dcc.Interval(id="interval", interval=UPDATE_MS, n_intervals=0),
    ],
)


# ─────────────────────────────────────────────────────────────────────────────
# Callbacks
# ─────────────────────────────────────────────────────────────────────────────
@app.callback(
    Output("anc-indicator", "children"),
    Output("anc-indicator", "style"),
    Output("wave-plot", "figure"),
    Output("spectrum-plot", "figure"),
    Output("atten-plot", "figure"),
    Output("rms-plot", "figure"),
    Input("interval", "n_intervals"),
)
def update_dashboard(_n):
    with _lock:
        ref_arr = np.array(_ref_history, dtype=np.float32)
        err_arr = np.array(_err_history, dtype=np.float32)
        anc_stat = _anc_status
        current_time = time.time() - _start_time

    # 1. Status Indicator
    indicator_bg = "#238636" if anc_stat == "ON" else "#da3633" if anc_stat == "TRAINING" else "#6e7681"
    indicator_style = {"padding": "6px 18px", "borderRadius": "14px", "fontWeight": "bold",
                       "backgroundColor": indicator_bg, "color": "white"}

    # If we don't have enough data yet, return empty plots
    if len(ref_arr) < 64:
        empty = go.Figure(layout=go.Layout(**_BASE_LAYOUT, annotations=[
            {"text": "Waiting for Snapshot...", "showarrow": False, "font": {"size": 16, "color": "#8b949e"}}]))
        return f"Status: {anc_stat}", indicator_style, empty, empty, empty, empty

    # 2. Calculations
    ref_rms = float(np.sqrt(np.mean(ref_arr ** 2)))
    err_rms = float(np.sqrt(np.mean(err_arr ** 2)))

    if ref_rms > 1e-8 and err_rms > 1e-8:
        atten_db = 20.0 * math.log10(ref_rms / err_rms)
    else:
        atten_db = 0.0

    _metrics.append({"t": current_time, "ref_rms": ref_rms, "err_rms": err_rms, "atten_db": atten_db})

    # FFT (Hann Windowed)
    n = len(ref_arr)
    win = np.hanning(n)
    ref_fft = np.abs(np.fft.rfft(ref_arr * win)) * (2.0 / n)
    err_fft = np.abs(np.fft.rfft(err_arr * win)) * (2.0 / n)
    freqs = np.fft.rfftfreq(n, d=1.0 / EFFECTIVE_FS)

    # --- NEW: SMOOTHING FILTER ---
    # Create a simple moving average window to smooth out the jagged FFT spikes
    kernel_size = 2  # Increase this number (e.g., to 7 or 9) for more smoothing
    kernel = np.ones(kernel_size) / kernel_size
    ref_fft_smooth = np.convolve(ref_fft, kernel, mode='same')
    err_fft_smooth = np.convolve(err_fft, kernel, mode='same')

    # 3. Build Figures
    wave_fig = go.Figure(layout=go.Layout(**_BASE_LAYOUT, title="Time-Domain Waveform", xaxis=_xax("Recent Samples"),
                                          yaxis=_yax("Amplitude")))
    wave_fig.add_trace(go.Scatter(y=ref_arr, name="Ref", line=dict(color="#58a6ff", width=1.5)))
    wave_fig.add_trace(go.Scatter(y=err_arr, name="Err", line=dict(color="#f78166", width=1.5)))

    # --- NEW: ZOOMED X-AXIS ---
    # Notice the range=[0, 3000] here to focus on your speaker's physical capabilities
    spec_fig = go.Figure(layout=go.Layout(**_BASE_LAYOUT, title="Frequency Spectrum (Smoothed, 0-3kHz)",
                                          xaxis=_xax("Frequency (Hz)", range=[0, 3000]),
                                          yaxis=_yax("Magnitude")))
    # Plotting the smoothed variables instead of the raw FFT
    spec_fig.add_trace(go.Scatter(x=freqs, y=ref_fft_smooth, name="Ref", line=dict(color="#58a6ff", width=1.5)))
    spec_fig.add_trace(go.Scatter(x=freqs, y=err_fft_smooth, name="Err", line=dict(color="#f78166", width=1.5)))

    m_t = [m["t"] for m in _metrics]

    atten_fig = go.Figure(
        layout=go.Layout(**_BASE_LAYOUT, title="ANC Attenuation vs Time", xaxis=_xax("Elapsed Time (s)"),
                         yaxis=_yax("Attenuation (dB)")))
    atten_fig.add_trace(
        go.Scatter(x=m_t, y=[m["atten_db"] for m in _metrics], name="Atten", line=dict(color="#d2a8ff", width=2)))

    rms_fig = go.Figure(layout=go.Layout(**_BASE_LAYOUT, title="RMS Amplitude vs Time", xaxis=_xax("Elapsed Time (s)"),
                                         yaxis=_yax("RMS")))
    rms_fig.add_trace(
        go.Scatter(x=m_t, y=[m["ref_rms"] for m in _metrics], name="Ref RMS", line=dict(color="#58a6ff", width=2)))
    rms_fig.add_trace(
        go.Scatter(x=m_t, y=[m["err_rms"] for m in _metrics], name="Err RMS", line=dict(color="#f78166", width=2)))

    return f"Status: {anc_stat}", indicator_style, wave_fig, spec_fig, atten_fig, rms_fig


if __name__ == "__main__":
    threading.Thread(target=_serial_listener, daemon=True).start()
    app.run(debug=False, port=8050)