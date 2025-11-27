from dash import Dash, dcc, html, Input, Output
import plotly.express as px

app = Dash(__name__)

def symulacja(target_speed=10):
    target_speed = target_speed*(10/36) #konwersja na m/s
    u_min, u_max = 0.0, 10.0
    b=11
    Tp, Ti = 0.1, 20
    car_mass=1500
    kp=5
    time, v = [0.0], [0.0]
    e=[]
    u_p=[]
    Kt=100
    c_d=0.43
    c_rr=0.012
    #F_dist=0
    for _ in range(int(1800 / Tp)):
        time.append(time[-1] + Tp)
        F_dist=v[-1]**2*c_d+c_rr*car_mass*9.81
        e.append(target_speed-v[-1])
        u_actual = kp*(e[-1]+(Tp/Ti)*sum(e))
        u_actual = max(u_min, min(u_max, u_actual))
        u_p.append(u_actual)
        v_actual =v[-1]+(Tp/car_mass)*(-b*v[-1]+Kt*u_p[-1]-F_dist)
        v.append(v_actual)
    v=[i*3.6 for i in v]
    return time, v

app.layout = html.Div([
    html.H2("Sterowanie prędkością"),
    dcc.Slider(0, 60, 1, value=10, id='u-slider',
               marks={i: f"{i}" for i in range(0, 61, 10)}),
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
