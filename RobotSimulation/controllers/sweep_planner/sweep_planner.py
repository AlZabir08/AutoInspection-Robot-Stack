import numpy as np

def planning(ox, oy, resolution):
    min_x, max_x = min(ox), max(ox)
    min_y, max_y = min(oy), max(oy)

    sweep_path_x = []
    sweep_path_y = []

    y = min_y
    direction = 1

    while y <= max_y:
        if direction == 1:
            x_points = np.arange(min_x, max_x + resolution, resolution)
        else:
            x_points = np.arange(max_x, min_x - resolution, -resolution)

        for x in x_points:
            sweep_path_x.append(round(x, 3))
            sweep_path_y.append(round(y, 3))

        y += resolution
        direction *= -1

    return sweep_path_x, sweep_path_y
