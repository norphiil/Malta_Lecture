from matplotlib.axes import Axes
import matplotlib.pyplot as plt
import numpy as np


COLORS = {'black': '#000000', 'green': '#87C5BA', 'grey': '#C2BBAB'}


def graph(ax: Axes, data: dict, title: str):
    colors = list(data.keys())
    positions = np.arange(len(colors))
    labels = ['L', 'M', 'R']
    sensor_values = [[data[color][label] for color in colors] for label in labels]

    bar_width = 0.25
    # opacities = [0.6, 0.7, 0.8]
    opacities = [0.8, 0.8, 0.8]

    max_value = max([max(sub.values()) for sub in data.values()])  # Find the maximum value
    chart_min_value = 0
    chart_max_value = 0
    for i, label in enumerate(labels):
        opacity = opacities[i]
        label = labels[i]  # Color label is the same as 'L', 'M', 'R'
        for j, color in enumerate(colors):
            ax.bar(positions[j] + i * bar_width, sensor_values[i][j], bar_width,
                   color=COLORS[color.lower()], alpha=opacity, label=label, edgecolor='white', linewidth=1)
            value_text_position = sensor_values[i][j]
            if value_text_position > 0:
                value_text_position += max_value * 0.1
            else:
                value_text_position -= max_value * 0.1
            if value_text_position > chart_max_value:
                chart_max_value = value_text_position
            if value_text_position < chart_min_value:
                chart_min_value = value_text_position
            ax.text(positions[j] + i * bar_width, max_value*0.1,
                    label, ha='center', va='center', fontsize=10)
            if value_text_position > 0 and value_text_position < max_value*0.3:
                value_text_position = max_value*0.3
            ax.text(positions[j] + i * bar_width, value_text_position,
                    str(sensor_values[i][j]), ha='center', va='center', fontsize=10)

    ax.set_xlabel('Colors')
    ax.set_ylabel('Sensor Values')
    ax.set_title(title)
    ax.set_xticks(positions + bar_width)
    ax.set_xticklabels(colors)
    ax.set_ylim(chart_min_value*1.2, chart_max_value*1.2)

    plt.tight_layout()


fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))

data_color = {
    'Grey': {'L': 79, 'M': 44, 'R': 56},
    'Green': {'L': 148, 'M': 44, 'R': 60},
    'Black': {'L': 875, 'M': 820, 'R': 853}
}

data_background = {
    'Grey': {'L': 150, 'M': 50, 'R': 150},
    'Green': {'L': 272, 'M': 53, 'R': 165},
    'Black': {'L': 905, 'M': 811, 'R': 814}
}

graph(ax1, data_color, 'Color Sensor Values')
graph(ax2, data_background, 'Color With Background Sensor Values')

# Calculate the difference between the two graphs
data_diff = {}
for color in data_color:
    data_diff[color] = {}
    for label in data_color[color]:
        data_diff[color][label] = data_background[color][label] - data_color[color][label]

graph(ax3, data_diff, 'Difference Between Color and Background Sensor Values')


plt.show()
