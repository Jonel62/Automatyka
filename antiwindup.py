from dash import Dash, dcc, html, Input, Output, State
import plotly.graph_objects as go
import numpy as np

app = Dash(__name__)


def symulacja(vd=60, m=1500, b=11, v0=0, Kp=30, opory=0.012, Tsin=180, Tp=0.1,
              teren_amp=0.0, teren_freq=0.1, zmienna_pred=False, vd2=80, t_change=60):
    """
    Symulacja tempomatu z regulatorem PI, anti-windup,
    + wyboisty teren (oscylacja siły oporu)
    + zmienna prędkość zadana.
    """
    vd_ms = vd * (1000.0 / 3600.0)
    vd2_ms = vd2 * (1000.0 / 3600.0)
    v0_ms = v0 * (1000.0 / 3600.0)

    u_min, u_max = 0.0, 10.0
    vmax = 200.0 * (1000.0 / 3600.0)
    Kt = 400.0
    Ti = 5.0
    c_d = 0.404
    c_rr = opory

    ant_windup_param = 0.5
    Ki = Kp * (Tp / Ti) if Ti != 0 else 0.0
    integral_term = 0.0

    time = [0.0]
    speed = [v0_ms]
    control_signals = []
    vd_traj = []

    num_steps = int(Tsin / Tp)
    for step in range(num_steps):
        t = step * Tp

        # Zmienna prędkość zadana
        if zmienna_pred and t > t_change:
            vd_current = vd2_ms
        else:
            vd_current = vd_ms

        vd_traj.append(vd_current * 3.6)

        v = speed[-1]

        # Wyboisty teren – okresowe fluktuacje oporu toczenia
        F_teren = teren_amp * np.sin(2 * np.pi * teren_freq * t)

        # Zakłócenia
        F_dist = (v ** 2) * c_d + c_rr * m * 9.81 + F_teren

        e = vd_current - v
        u_unclamped = Kp * e + integral_term
        u_actual = max(u_min, min(u_max, u_unclamped))
        control_signals.append(u_actual)

        # Anti‑windup
        u_error = u_actual - u_unclamped
        integral_term += Ki * e + ant_windup_param * u_error

        acc = (Kt * u_actual - b * v - F_dist) / m
        v_next = v + Tp * acc
        v_next = max(0.0, min(vmax, v_next))

        time.append(t + Tp)
        speed.append(v_next)

    speed_kmh = [s * 3.6 for s in speed]
    return time, speed_kmh, control_signals, vd_traj


app.layout = html.Div([
    html.H1("Tempomat (PI)", style={"textAlign": "center"}),

    html.Div([
        dcc.Tabs([
            dcc.Tab(label='Wspolczynniki i parametry', children=[
                html.Div([
                    html.Label("Masa pojazdu [kg]"),
                    dcc.Slider(800, 3000, 100, value=1500, id="m-slider",
                               marks={800: "800", 1500: "1500", 3000: "3000"},
                               tooltip={"placement": "bottom", "always_visible": True}),
                ], style={"padding": "8px 0"}),
                html.Div([
                    html.Label("Wzmocnienie regulatora Kp"),
                    dcc.Slider(1, 150, 1, value=30, id="kp-slider",
                               marks={1: "1", 30: "30", 75: "75", 150: "150"},
                               tooltip={"placement": "bottom", "always_visible": True}),
                ], style={"padding": "8px 0"}),
                html.Div([
                    html.Label("Opór toczenia (c_rr)"),
                    dcc.Slider(0.001, 0.05, 0.001, value=0.012, id="opory-slider",
                               marks={0.001: "0.001", 0.01: "0.010", 0.02: "0.020", 0.03: "0.030", 0.05: "0.050"},
                               tooltip={"placement": "bottom", "always_visible": True}),
                ], style={"padding": "8px 0"}),
                html.Div([
                    html.Label("Współczynnik tłumienia b"),
                    dcc.Slider(0, 30, 1, value=11, id="b-slider",
                               marks={0: "0", 10: "10", 20: "20", 30: "30"},
                               tooltip={"placement": "bottom", "always_visible": True}),
                ], style={"padding": "8px 0"})
            ]),
            dcc.Tab(label='Zmienność terenu', children=[
                html.Div([
                    html.Label("Wyboistość – amplituda siły"),
                    dcc.Slider(0, 500, 10, value=0, id="amp-slider",
                               marks={0: "0", 100: "100", 300: "300", 500: "500"},
                               tooltip={"placement": "bottom", "always_visible": True}),
                ], style={"padding": "8px 0"}),

                html.Div([
                    html.Label("Wyboistość – częstotliwość [Hz]"),
                    dcc.Slider(0.01, 1.0, 0.01, value=0.1, id="freq-slider",
                               marks={0.01: "0.01", 0.1: "0.1", 0.5: "0.5", 1: "1.0"},
                               tooltip={"placement": "bottom", "always_visible": True}),
                ], style={"padding": "8px 0"})
            ]),
            dcc.Tab(label='Prędkość', children=[
                html.Div([
                    html.Label("Prędkość początkowa [km/h]"),
                    dcc.Slider(0, 100, 1, value=0, id="v0-slider",
                               marks={0: "0", 50: "50", 100: "100"},
                               tooltip={"placement": "bottom", "always_visible": True}),
                ], style={"padding": "8px 0"}),
                html.Div([
                    html.Label("Prędkość zadana [km/h]"),
                    dcc.Slider(10, 200, 5, value=60, id="vd-slider",
                               marks={10: "10", 60: "60", 120: "120", 180: "180"},
                               tooltip={"placement": "bottom", "always_visible": True}),
                ], style={"padding": "8px 0"}),
                html.Div([
                    html.Label("Druga prędkość zadana [km/h]"),
                    dcc.Slider(10, 200, 5, value=80, id="vd2-slider",
                               marks={10: "10", 80: "80", 150: "150", 200: "200"},
                               tooltip={"placement": "bottom", "always_visible": True}),
                ], style={"padding": "8px 0"}),

                html.Div([
                    html.Label("Czas zmiany prędkości [s]"),
                    dcc.Slider(10, 150, 5, value=60, id="tchange-slider",
                               marks={10: "10", 60: "60", 120: "120", 150: "150"},
                               tooltip={"placement": "bottom", "always_visible": True}),
                ], style={"padding": "8px 0"}),

                html.Div([
                    html.Label("Włącz zmienną prędkość"),
                    dcc.Checklist(options=[{"label": "Tak", "value": "on"}], id="varspeed-check", value=[])
                ], style={"padding": "8px 0"}),
            ], style={"maxWidth": "840px", "margin": "0 auto", "padding": "12px 6px"})
            ]),
        ]),
    html.Div(html.Button("Uruchom symulację", id="submit-btn", n_clicks=0),
             style={"textAlign": "center", "margin": "12px 0"}),

    dcc.Graph(id="output-graph"),
])


@app.callback(
    Output("output-graph", "figure"),
    Input("submit-btn", "n_clicks"),
    State("vd-slider", "value"),
    State("vd2-slider", "value"),
    State("tchange-slider", "value"),
    State("varspeed-check", "value"),
    State("m-slider", "value"),
    State("b-slider", "value"),
    State("v0-slider", "value"),
    State("kp-slider", "value"),
    State("opory-slider", "value"),
    State("amp-slider", "value"),
    State("freq-slider", "value"),
    prevent_initial_call=True,
)
def update_plot(n_clicks, vd, vd2, tchange, vs_flag, m, b, v0, kp, opory, amp, freq):
    zmienna = ("on" in vs_flag)

    time, speed, control, vdtraj = symulacja(
        vd=vd, vd2=vd2, t_change=tchange, zmienna_pred=zmienna,
        m=m, b=b, v0=v0, Kp=kp, opory=opory,
        teren_amp=amp, teren_freq=freq
    )

    fig = go.Figure()
    fig.add_trace(go.Scatter(x=time, y=speed, name="Prędkość [km/h]"))
    fig.add_trace(go.Scatter(x=time[:-1], y=control, name="Sterowanie [V]", yaxis="y2"))
    fig.add_trace(go.Scatter(x=time, y=vdtraj, name="Prędkość zadana", line=dict(dash="dot")))

    speed_axis_max = max(max(speed), max(vd, vd2)) * 1.2

    fig.update_layout(
        xaxis={"title": "Czas [s]"},
        yaxis={"title": "Prędkość [km/h]", "range": [0, speed_axis_max]},
        yaxis2={"title": "Sterowanie [V]", "overlaying": "y", "side": "right", "range": [0, 10]},
        hovermode="x unified",
        template="plotly_white",
    )
    return fig


if __name__ == "__main__":
    app.run(debug=True)
