from dash import Dash, dcc, html, Input, Output, State
import plotly.graph_objects as go

app = Dash(__name__)


def symulacja(vd=60, m=1500, b=11, v0=0, Kp=30, opory=0.012, Tsin=180, Tp=0.1):
    """
    Prosta symulacja tempomatu z regulatorem PI (anti-windup).
    Jednostki: wewnętrznie m/s, wykres prędkości w km/h.
    """
    # Konwersje jednostek
    vd_ms = vd * (1000.0 / 3600.0)  # km/h -> m/s
    v0_ms = v0 * (1000.0 / 3600.0)

    # Ograniczenia sygnału sterującego
    u_min, u_max = 0.0, 10.0
    vmax = 200.0 * (1000.0 / 3600.0)  # maksymalna prędkość w m/s (odpowiada 200 km/h)

    # Stałe modelu (twardo zakodowane)
    Kt = 80.0    # [N/V]
    Ti = 5.0     # stała całkowania [s]
    c_d = 0.404  # opór aerodynamiczny
    c_rr = opory # opór toczenia (teren) - slider

    # Anti-windup
    ant_windup_param = 0.5
    Ki = Kp * (Tp / Ti) if Ti != 0 else 0.0
    integral_term = 0.0

    time = [0.0]
    speed = [v0_ms]
    control_signals = []

    num_steps = int(Tsin / Tp)
    for _ in range(num_steps):
        t = time[-1] + Tp
        v = speed[-1]

        # Zakłócenia: opory aerodynamiczne + toczenia
        F_dist = (v ** 2) * c_d + c_rr * m * 9.81

        e = vd_ms - v

        u_unclamped = Kp * e + integral_term
        u_actual = max(u_min, min(u_max, u_unclamped))
        control_signals.append(u_actual)

        # Korekta anti-windup dla całki
        u_error = u_actual - u_unclamped
        integral_term += Ki * e + ant_windup_param * u_error

        # Równanie ruchu
        acc = (Kt * u_actual - b * v - F_dist) / m
        v_next = v + Tp * acc
        v_next = max(0.0, min(vmax, v_next))

        time.append(t)
        speed.append(v_next)

    # Do wykresu: prędkość w km/h
    speed_kmh = [s * 3.6 for s in speed]
    return time, speed_kmh, control_signals


# Minimalny, czytelny layout
app.layout = html.Div(
    [
        html.H1("Tempomat (PI)", style={"textAlign": "center"}),
        html.Div(
            [
                html.Div([
                    html.Label("Prędkość zadana [km/h]"),
                    dcc.Slider(10, 200, 5, value=60, id="vd-slider",
                               marks={10: "10", 60: "60", 120: "120", 180: "180"},
                               tooltip={"placement": "bottom", "always_visible": True}),
                ], style={"padding": "8px 0"}),

                html.Div([
                    html.Label("Masa pojazdu [kg]"),
                    dcc.Slider(800, 3000, 100, value=1500, id="m-slider",
                               marks={800: "800", 1500: "1500", 3000: "3000"},
                               tooltip={"placement": "bottom", "always_visible": True}),
                ], style={"padding": "8px 0"}),

                html.Div([
                    html.Label("Współczynnik tłumienia b"),
                    dcc.Slider(0, 30, 1, value=11, id="b-slider",
                               marks={0: "0", 10: "10", 20: "20", 30: "30"},
                               tooltip={"placement": "bottom", "always_visible": True}),
                ], style={"padding": "8px 0"}),

                html.Div([
                    html.Label("Prędkość początkowa [km/h]"),
                    dcc.Slider(0, 100, 1, value=0, id="v0-slider",
                               marks={0: "0", 50: "50", 100: "100"},
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
                               marks={0.001: "0.001", 0.01: "0.010", 0.02: "0.020", 0.03: "0.030"},
                               tooltip={"placement": "bottom", "always_visible": True}),
                ], style={"padding": "8px 0"}),
            ],
            style={"maxWidth": "840px", "margin": "0 auto", "padding": "12px 6px"},
        ),

        html.Div(html.Button("Zatwierdź i uruchom symulację", id="submit-btn", n_clicks=0),
                 style={"textAlign": "center", "margin": "12px 0"}),

        dcc.Graph(id="output-graph"),
    ]
)


@app.callback(
    Output("output-graph", "figure"),
    Input("submit-btn", "n_clicks"),
    State("vd-slider", "value"),
    State("m-slider", "value"),
    State("b-slider", "value"),
    State("v0-slider", "value"),
    State("kp-slider", "value"),
    State("opory-slider", "value"),
    prevent_initial_call=True,
)
def update_plot(n_clicks, vd, m, b, v0, kp, opory):
    time, speed, control = symulacja(vd=vd, m=m, b=b, v0=v0, Kp=kp, opory=opory)

    fig = go.Figure()

    # Prędkość (lewa oś)
    fig.add_trace(go.Scatter(x=time, y=speed, name="Prędkość [km/h]", line=dict(color="#1f77b4")))

    # Sygnał sterujący (prawa oś)
    fig.add_trace(go.Scatter(x=time[:-1], y=control, name="Sygnał sterujący [V]",
                             yaxis="y2", line=dict(color="#d62728", dash="dot")))

    # Ustalone przedziały osi:
    # górny próg prędkości = max(v0, 1.2 * vd) (wartości w km/h, bo slidery zwracają km/h)
    speed_axis_max = max(v0, 1.2 * vd)
    # upewnij się, że górny próg jest co najmniej 1 (unikamy np. 0)
    if speed_axis_max < 1.0:
        speed_axis_max = 1.0

    fig.update_layout(
        title=f"Symulacja tempomatu — vd={vd} km/h",
        xaxis=dict(title="Czas [s]"),
        yaxis=dict(title="Prędkość [km/h]", range=[0, speed_axis_max]),
        yaxis2=dict(title="Sygnał sterujący [V]", overlaying="y", side="right", range=[0, 10]),
        legend=dict(orientation="h", y=1.02, x=0.5),
        template="plotly_white",
        hovermode="x unified",
    )

    return fig


if __name__ == "__main__":
    app.run(debug=True)