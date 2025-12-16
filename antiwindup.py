from dash import Dash, dcc, html, Input, Output, State
import plotly.graph_objects as go
import numpy as np

app = Dash(__name__)

# --- KONFIG ---
PRESETS = {
    "compact": {"label": "Samochód kompaktowy", "m": 1200, "Kt": 300.0, "A": 2.1, "cw": 0.32},
    "sport": {"label": "Samochód sportowy", "m": 1400, "Kt": 360.0, "A": 2.0, "cw": 0.28},
    "suv": {"label": "SUV", "m": 1800, "Kt": 450.0, "A": 2.5, "cw": 0.38},
}

RHO = 1.225
B_CONST = 11.0        # stały współczynnik tłumienia [N·s/m]
U_MIN, U_MAX = 0.0, 10.0
G = 9.81
TSIN_DEFAULT = 180.0
TP_DEFAULT = 0.1


def symulacja(
    vd=60,
    v0=0,
    Kp=10,
    Ti=5,
    Tsin=TSIN_DEFAULT,
    Tp=TP_DEFAULT,
    Kt=300.0,
    m=1200.0,
    A=2.1,
    cw=0.32,
    nachylenia=None,
    zmienna_pred=False,
    vd2=80,
    t_change=60,
):
    # Konwersje km/h -> m/s
    vd_ms = vd * 1000.0 / 3600.0
    vd2_ms = vd2 * 1000.0 / 3600.0
    v0_ms = v0 * 1000.0 / 3600.0

    vmax = 200.0 * 1000.0 / 3600.0

    ant_windup_param = 0.5
    Ki = Kp * (Tp / Ti) if Ti != 0 else 0.0
    integral_term = 0.0

    num_steps = int(Tsin / Tp)

    time = [0.0]
    speed = [v0_ms]

    F_drive = []
    F_aero = []
    F_slope = []
    F_damp = []

    vd_traj = []
    angle_traj = []

    if nachylenia is None:
        nachylenia = np.zeros(num_steps)

    for step in range(num_steps):
        t = step * Tp

        # zadana prędkość (zmienna)
        if zmienna_pred and t > t_change:
            vd_current = vd2_ms
        else:
            vd_current = vd_ms

        vd_traj.append(vd_current * 3.6)

        alpha_deg = float(nachylenia[step]) if step < len(nachylenia) else 0.0
        angle_traj.append(alpha_deg)

        v = speed[-1]

        # opór aerodynamiczny (zawsze przeciwdziała ruchowi)
        f_aero = 0.5 * cw * RHO * A * v * v

        # siła zbocza: m * g * sin(alpha)
        alpha_rad = np.deg2rad(alpha_deg)
        f_slope = m * G * np.sin(alpha_rad)  # dodatnia -> przeciwdziała ruchowi (pod górę)

        # tłumienie (viscous): b * v (przeciw ruchowi)
        f_damp_mag = B_CONST * v

        # PI
        e = vd_current - v
        u_unclamped = Kp * e + integral_term
        u_actual = float(np.clip(u_unclamped, U_MIN, U_MAX))
        u_error = u_actual - u_unclamped
        integral_term += Ki * e + ant_windup_param * u_error

        # siła napędu (dodatnia = napędzająca)
        f_drive = Kt * u_actual

        # zapisz siły (wartości z konwencją: dodatnie = napęd, ujemne = hamujące)
        F_drive.append(f_drive)         # dodatnie
        F_aero.append(-f_aero)          # opór powietrza jako ujemny (hamujący)
        F_slope.append(-f_slope)        # siła zbocza: pod górę -> ujemna (hamuje); z górki (f_slope<0) -> -f_slope>0 (wspomaga)
        F_damp.append(-f_damp_mag)      # tłumienie jako ujemne

        # dynamika: m*dv/dt = f_drive - f_damp_mag - f_aero - f_slope
        acc = (f_drive - f_damp_mag - f_aero - f_slope) / m
        v_next = v + Tp * acc
        v_next = float(np.clip(v_next, 0.0, vmax))

        time.append(t + Tp)
        speed.append(v_next)

    # dopełnij trajektorie do len = num_steps+1
    if len(vd_traj) == num_steps:
        vd_traj.append(vd_traj[-1])
    if len(angle_traj) == num_steps:
        angle_traj.append(angle_traj[-1])

    speed_kmh = [s * 3.6 for s in speed]
    return time, speed_kmh, F_drive, F_aero, F_slope, F_damp, vd_traj, angle_traj


def generuj_nachylenia(Tsin=TSIN_DEFAULT, Tp=TP_DEFAULT, up_angle=12.0, down_angle=-5.0):
    N = int(Tsin / Tp)
    nachylenia = np.zeros(N)
    N1 = int(N * 0.35)
    N2 = int(N * 0.3)
    nachylenia[:N1] = 0.0
    nachylenia[N1:N1+N2] = up_angle
    nachylenia[N1+N2:] = down_angle
    return nachylenia


# --- UI: trzy zakładki (Prędkość, Regulator, Samochód) ---
app.layout = html.Div(
    [
        html.H1("Tempomat (PI)", style={"textAlign": "center"}),
        dcc.Tabs(
            [
                dcc.Tab(
                    label="Prędkość",
                    children=[
                        html.Div(
                            [
                                html.Label("Prędkość początkowa [km/h]"),
                                html.Div(dcc.Slider(0, 100, 1, value=0, id="v0-slider", marks={0: "0", 50: "50", 100: "100"}, tooltip={"always_visible": True}), style={"width": "40%", "margin": "0 auto", "padding": "6px 0"}),
                                html.Label("Prędkość zadana [km/h]"),
                                html.Div(dcc.Slider(10, 200, 5, value=60, id="vd-slider", marks={10: "10", 60: "60", 120: "120", 180: "180", 200: "200"}, tooltip={"always_visible": True}), style={"width": "40%", "margin": "0 auto", "padding": "6px 0"}),
                                html.Label("Druga prędkość zadana [km/h]"),
                                html.Div(dcc.Slider(10, 200, 5, value=80, id="vd2-slider", marks={10: "10", 80: "80", 150: "150", 200: "200"}, tooltip={"always_visible": True}), style={"width": "40%", "margin": "0 auto", "padding": "6px 0"}),
                                html.Label("Czas zmiany prędkości [s]"),
                                html.Div(dcc.Slider(10, 150, 5, value=60, id="tchange-slider", marks={10: "10", 60: "60", 120: "120", 150: "150"}, tooltip={"always_visible": True}), style={"width": "40%", "margin": "0 auto", "padding": "6px 0"}),
                                html.Div([html.Label("Włącz zmienną prędkość"), dcc.Checklist(options=[{"label": "Tak", "value": "on"}], id="varspeed-check", value=[])], style={"padding": "6px 0"}),
                            ],
                            style={"maxWidth": "520px", "margin": "0 auto"},
                        )
                    ],
                ),
                dcc.Tab(
                    label="Regulator",
                    children=[
                        html.Div(
                            [
                                html.Label("Wzmocnienie Kp (domyślnie 10)"),
                                html.Div(dcc.Slider(1, 60, 1, value=10, id="kp-slider", marks={1: "1", 15: "15", 30: "30", 45: "45", 60: "60"}, tooltip={"always_visible": True}), style={"width": "40%", "margin": "0 auto", "padding": "6px 0"}),
                                html.Label("Stała całkowania Ti [s]"),
                                html.Div(dcc.Slider(1, 20, 1, value=5, id="ti-slider", marks={1: "1", 5: "5", 10: "10", 20: "20"}, tooltip={"always_visible": True}), style={"width": "40%", "margin": "0 auto", "padding": "6px 0"}),
                            ],
                            style={"maxWidth": "520px", "margin": "0 auto"},
                        )
                    ],
                ),
                dcc.Tab(
                    label="Samochód",
                    children=[
                        html.Div([html.Label("Typ samochodu"), html.Div(dcc.Dropdown(options=[{"label": PRESETS[k]["label"], "value": k} for k in PRESETS], value="compact", id="car-dropdown"), style={"width": "40%", "margin": "0 auto", "padding": "6px 0"})]),
                        html.Div([html.Label("Wyboistość: amplituda siły [N] (opcjonalnie)"), html.Div(dcc.Slider(0, 500, 10, value=0, id="amp-slider", marks={0: "0", 100: "100", 300: "300", 500: "500"}, tooltip={"always_visible": True}), style={"width": "40%", "margin": "0 auto", "padding": "6px 0"})]),
                        html.Div([html.Label("Wyboistość: częstotliwość [Hz] (opcjonalnie)"), html.Div(dcc.Slider(0.01, 1.0, 0.01, value=0.1, id="freq-slider", marks={0.01: "0.01", 0.1: "0.1", 0.5: "0.5", 1.0: "1.0"}, tooltip={"always_visible": True}), style={"width": "40%", "margin": "0 auto", "padding": "6px 0"})]),
                    ],
                    style={"maxWidth": "520px", "margin": "0 auto"},
                ),
            ]
        ),
        html.Div(html.Button("Uruchom symulację", id="submit-btn", n_clicks=0), style={"textAlign": "center", "margin": "12px 0"}),
        html.Div(dcc.Graph(id="predkosc-graph"), style={"width": "90%", "margin": "0 auto"}),
        html.Div(dcc.Graph(id="sily-graph"), style={"width": "90%", "margin": "0 auto"}),
    ],
    style={"fontFamily": "Arial, sans-serif", "padding": "12px"},
)


@app.callback(
    Output("predkosc-graph", "figure"),
    Output("sily-graph", "figure"),
    Input("submit-btn", "n_clicks"),
    State("vd-slider", "value"),
    State("vd2-slider", "value"),
    State("tchange-slider", "value"),
    State("varspeed-check", "value"),
    State("v0-slider", "value"),
    State("kp-slider", "value"),
    State("ti-slider", "value"),
    State("car-dropdown", "value"),
    State("amp-slider", "value"),
    State("freq-slider", "value"),
    prevent_initial_call=True,
)
def update_plots(n_clicks, vd, vd2, tchange, vs_flag, v0, kp, ti, car_key, amp, freq):
    zmienna = ("on" in vs_flag)
    preset = PRESETS[car_key]

    # przygotuj nachylenia; domyślnie silniejszy podjazd (12°) aby efekt był widoczny
    nachylenia = generuj_nachylenia()
    if amp > 0:
        approx_angle_variation = (amp / (preset["m"] * G)) * (180.0 / np.pi)
        N = len(nachylenia)
        t = np.arange(N) * TP_DEFAULT
        nachylenia = nachylenia + approx_angle_variation * np.sin(2 * np.pi * freq * t)

    time, speed, F_drive, F_aero, F_slope, F_damp, vdtraj, angle_traj = symulacja(
        vd=vd,
        v0=v0,
        Kp=kp,
        Ti=ti,
        Tsin=TSIN_DEFAULT,
        Tp=TP_DEFAULT,
        Kt=preset["Kt"],
        m=preset["m"],
        A=preset["A"],
        cw=preset["cw"],
        nachylenia=nachylenia,
        zmienna_pred=zmienna,
        vd2=vd2,
        t_change=tchange,
    )

    x_range = [0, TSIN_DEFAULT]

    # --- wykres prędkości z autoskalowaniem górnej granicy ---
    fig_speed = go.Figure()
    fig_speed.add_trace(go.Scatter(x=time, y=speed, name="Prędkość pojazdu [km/h]", line=dict(color="blue")))
    fig_speed.add_trace(go.Scatter(x=time, y=vdtraj, name="Prędkość zadana [km/h]", line=dict(color="black", dash="dot")))
    max_speed_sim = max(speed) if len(speed) > 0 else 1.0
    speed_axis_fixed = max(v0, 1.2 * vd, 1.2 * vd2)
    speed_axis_max = max(max_speed_sim * 1.05, speed_axis_fixed)
    if speed_axis_max < 1.0:
        speed_axis_max = 1.0
    fig_speed.update_layout(xaxis_title="Czas [s]", yaxis_title="Prędkość [km/h]", title="Prędkość pojazdu", yaxis=dict(range=[0, speed_axis_max]), template="plotly_white")
    fig_speed.update_xaxes(range=x_range)

    # --- wykres sił: siły hamujące jako ujemne, napęd dodatni; kąt na drugiej osi y ---
    fig_forces = go.Figure()
    t_forces = time[:-1]
    fig_forces.add_trace(go.Scatter(x=t_forces, y=F_drive, name="Siła napędu [N]", line=dict(color="green")))
    fig_forces.add_trace(go.Scatter(x=t_forces, y=F_aero, name="Opór aerodynamiczny [N]", line=dict(color="red")))
    fig_forces.add_trace(go.Scatter(x=t_forces, y=F_damp, name="Tłumienie b·v [N]", line=dict(color="purple")))
    fig_forces.add_trace(go.Scatter(x=t_forces, y=F_slope, name="Siła zbocza [N]", line=dict(color="orange")))
    angle_aligned = angle_traj[:-1] if len(angle_traj) >= len(t_forces) else np.zeros(len(t_forces))
    fig_forces.add_trace(go.Scatter(x=t_forces, y=angle_aligned, name="Kąt nachylenia [°]", yaxis="y2", line=dict(color="brown", dash="dot")))
    fig_forces.update_layout(xaxis_title="Czas [s]", yaxis_title="Siła [N]", yaxis2=dict(title="Kąt [°]", overlaying="y", side="right", showgrid=False), title="Siły działające na pojazd i kąt nachylenia", template="plotly_white")
    fig_forces.update_xaxes(range=x_range)

    return fig_speed, fig_forces


if __name__ == "__main__":
    app.run()