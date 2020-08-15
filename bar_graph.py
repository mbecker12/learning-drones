import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import plotly.express as px
import plotly.graph_objects as go

paramset = ["LL", "LM", "LH", "ML", "MM", "MH", "HL", "HM", "HH"]

param_p = ["low", "low", "low", "medium", "medium", "medium", "high", "high", "high"]

param_d = ["low", "medium", "high", "low", "medium", "high", "low", "medium", "high"]

paramset = ["LL", "LM", "LH", "ML", "MM", "MH", "HL", "HM", "HH"]

single = {
    "Rotation": ["Single" for _ in range(9)],
    # 'Paramset': ['00', '01', '02', '10', '11', '12', '20', '21', '22'],
    "Proportional Term": param_p,
    "Derivative Term": param_d,
    "Time / s": [2.03, 2.58, 3.9, 1.42, 1.69, 5.99, 1.22, 1.42, 5.99],
}

double = {
    "Rotation": ["Double" for _ in range(9)],
    # 'Paramset': ['00', '01', '02', '10', '11', '12', '20', '21', '22'],
    "Proportional Term": param_p,
    "Derivative Term": param_d,
    "Time / s": [5.14, 5.45, 5.51, 3.32, 3.58, 5.99, 2.37, 2.32, 5.99],
}

triple = {
    "Rotation": ["Triple" for _ in range(9)],
    # 'Paramset': ['00', '01', '02', '10', '11', '12', '20', '21', '22'],
    "Proportional Term": param_p,
    "Derivative Term": param_d,
    "Time / s": [5.99, 5.99, 5.99, 3.57, 4.15, 5.99, 2.32, 2.24, 5.99],
}


colors = {"single": "#ff0000", "double": "#00ff00", "triple": "#0000ff"}

df_single = pd.DataFrame(data=single)
df_double = pd.DataFrame(data=double)
df_triple = pd.DataFrame(data=triple)
df = pd.concat([df_single, df_double, df_triple])
m = df["Time / s"] > 5.9
df.where(m, 0.0)
# print(df)
# df.groupby(by='Rotation')
# # df['time'].plot(kind='bar', color=[colors[i] for i in df['Rotation']])
# ax = df.plot.bar('paramset', 'time', color=[colors[i] for i in df['Rotation']])
# # df[['paramset', 'time']].plot(kind='bar', stacked=True)
# plt.show()

fig = px.bar(
    df,
    x="Derivative Term",
    y="Time / s",
    color="Rotation",
    barmode="group",
    facet_col="Proportional Term",
)
fig.show()
# fig = go.Figure(data=[
#     go.Bar(name='SF Zoo', x=animals, y=[20, 14, 23]),
#     go.Bar(name='LA Zoo', x=animals, y=[12, 18, 29])
# ])
