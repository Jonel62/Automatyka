from dash import Dash, dcc, html, Input, Output
import plotly.express as px

app = Dash(__name__)

def symulacja(target_speed=10):
    target_speed = target_speed*(10/36) #konwersja na m/s
    u_min, u_max = 0.0, 10.0
    time, speed = [0.0], [0.0]
    Tp, Ti = 0.1, 5

    Kp=30
    Kt=80

    b=11
    car_mass=1500

    errors=[]
    control_signals=[]

    c_d=0.404
    c_rr=0.012

    ant_windup_param=0.5
    Ki=Kp*(Tp/Ti)
    integral_term=0.0
    for _ in range(int(1800 / Tp)):
        time.append(time[-1] + Tp)
        F_dist=speed[-1]**2*c_d+c_rr*car_mass*9.81
        errors.append(target_speed-speed[-1])
        u_unclamped = Kp*errors[-1] + integral_term# (Tp/Ti)*sum(errors))
        u_actual = max(u_min, min(u_max, u_unclamped))
        control_signals.append(u_actual)
        u_error=u_actual-u_unclamped
        integral_term+=Ki*errors[-1]+ant_windup_param*u_error
        v_actual=speed[-1]+(Tp/car_mass)*(-b*speed[-1]+Kt*control_signals[-1]-F_dist)
        speed.append(v_actual)
    speed=[i*3.6 for i in speed]
    return time, speed

app.layout = html.Div([
    html.H2("Sterowanie prędkością"),
    dcc.Slider(0, 100, 1, value=10, id='u-slider',
               marks={i: f"{i}" for i in range(0, 101, 10)}),
    dcc.Graph(id='plot')
])

@app.callback(
    Output('plot', 'figure'),
    Input('u-slider', 'value')
)
def update(target_speed):
    time, speed = symulacja(target_speed)
    fig = px.line(x=time, y=speed, labels={'x': 'Czas [s]', 'y': 'prędkość [km/h]'},
                  title=f'Symulacja (docelowa prędkośc={target_speed} km/h)')
    fig.update_layout(template='plotly_white')
    return fig

if __name__ == '__main__':
    app.run()
