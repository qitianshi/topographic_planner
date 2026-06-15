"""Visualizes a CloudCompare ASCII export as a 2.5D terrain map.

Options:
1. True color view: Uses raw RGB values via Delaunay triangulation (Mesh3d).
2. Z-Range view: Uses Z-range via a grid projection (Surface).
"""


import os
import sys
from enum import StrEnum

import numpy as np
import pandas as pd
import plotly.graph_objects as go
from scipy.interpolate import griddata


MAX_CELL_COST = 255
LETHAL_THRESHOLD = 0.3


class InputHeaders(StrEnum):
    """Headers for the input CSV"""
    X = "X"
    Y = "Y"
    Z = "Z"
    R = "R"
    G = "G"
    B = "B"
    SF_Z = "Z values (0.1)"
    SF_Z_STDDEV = "Z std. dev. (0.1)"
    SF_Z_RANGE = "Z range (0.1)"


def validate_paths() -> tuple[str, str]:
    """Validates the provided path arguments are valid."""

    if len(sys.argv) != 3:
        print("usage: python visualize_map.py <raw_in> <path_in>")
        sys.exit(1)

    if not os.path.exists(sys.argv[1]):
        print(f"{sys.argv[1]} does not exist")
        sys.exit(1)

    if not os.path.exists(sys.argv[2]):
        print(f"{sys.argv[2]} does not exist")
        sys.exit(1)

    return sys.argv[1], sys.argv[2]


def load_cloud(in_path: str) -> tuple[pd.DataFrame, np.ndarray, np.ndarray]:
    """Reads the cloud input CSV and formats the color and grid data."""

    df = pd.read_csv(
        in_path,
        usecols=[
            InputHeaders.X,
            InputHeaders.Y,
            InputHeaders.Z,
            InputHeaders.R,
            InputHeaders.G,
            InputHeaders.B,
            InputHeaders.SF_Z_RANGE
        ]
    )

    # Rename columns for convenience
    df = df.rename(columns={
        InputHeaders.X: 'x_val',
        InputHeaders.Y: 'y_val',
        InputHeaders.Z: 'z_val',
        InputHeaders.SF_Z_RANGE: 'z_range'
    })

    # Format RGB values into strings (for Plotly)
    df['color'] = (
        'rgb(' +
        df[InputHeaders.R].astype(str) + ',' +      # type: ignore
        df[InputHeaders.G].astype(str) + ',' +      # type: ignore
        df[InputHeaders.B].astype(str) + ')'
    )

    # Round to smooth out floating point imprecision for the grid
    df['x_val'] = df['x_val'].round(4)
    df['y_val'] = df['y_val'].round(4)

    sorted_x = np.sort(df['x_val'].unique())
    sorted_y = np.sort(df['y_val'].unique())

    return df, sorted_x, sorted_y


def create_interactive_view(
    df: pd.DataFrame,
    sorted_x: np.ndarray,
    sorted_y: np.ndarray
) -> go.Figure:
    """Creates a single 3D figure with both views and UI toggle buttons."""

    fig = go.Figure()

    # True colors (RGB). Uses Delaunay triangulation to reconstruct the mesh.
    fig.add_trace(go.Mesh3d(
        x=df['x_val'],
        y=df['y_val'],
        z=df['z_val'],
        vertexcolor=df['color'],
        delaunayaxis='z',
        flatshading=True,
        visible=True,
        name='True Color'
    ))

    # Z-Range / Cost trace
    z_grid = \
        df.pivot_table(index='y_val', columns='x_val', values='z_val').values
    z_range_grid = \
        df.pivot_table(index='y_val', columns='x_val', values='z_range').values

    # Apply the same linear mapping and clipping as the processing script
    cost_grid = np.clip((z_range_grid / LETHAL_THRESHOLD) * MAX_CELL_COST, 0, MAX_CELL_COST)

    fig.add_trace(go.Surface(
        x=sorted_x,
        y=sorted_y,
        z=z_grid,
        surfacecolor=cost_grid,                                   # type: ignore
        cmin=0,                  # Lock the color scale minimum
        cmax=MAX_CELL_COST,      # Lock the color scale maximum
        colorscale='RdBu_r',
        colorbar={"title": 'Cost (0-255)'},
        visible=False,
        name='Cost Map'
    ))

    # UI Toggle Buttons
    fig.update_layout(
        title="Terrain: True Color RGB",
        scene={"aspectmode": 'data'},
        updatemenus=[{
            "type": "buttons",
            "direction": "right",
            "x": 0.0,
            "y": 1.1,
            "showactive": True,
            "buttons": [
                {
                    "label": "True Color View",
                    "method": "update",
                    "args": [
                        {"visible": [True, False]},
                        {"title": "Terrain: True Color RGB"}
                    ]
                },
                {
                    "label": "Cost Map View",
                    "method": "update",
                    "args": [
                        {"visible": [False, True]},
                        {"title": "Terrain: Cost Palette"}
                    ]
                }
            ]
        }]
    )

    return fig


def add_path(
    fig: go.Figure,
    df: pd.DataFrame,
    waypoints_path: str
) -> go.Figure:
    """Reads 2D waypoints, densifies the segments, projects them onto the 3D
    terrain, and updates the figure to display the path.
    """

    # Load the 2D waypoints
    wp_df = pd.read_csv(waypoints_path)
    wp_x = wp_df['x'].values
    wp_y = wp_df['y'].values

    # Densifies the path by super-sampling
    step_size = 0.05
    dense_x: list[float] = []
    dense_y: list[float] = []

    for i in range(len(wp_x) - 1):

        # Start and end points
        x1, y1 = wp_x[i], wp_y[i]
        x2, y2 = wp_x[i+1], wp_y[i+1]

        # Calculate the number of micro-steps
        dist = np.hypot(x2 - x1, y2 - y1)
        num_steps = max(int(np.ceil(dist / step_size)), 2)

        # Evenly spaced samples
        seg_x = np.linspace(x1, x2, num_steps)
        seg_y = np.linspace(y1, y2, num_steps)

        # Remove the first point (already in the previous segment)
        if i > 0:
            seg_x = seg_x[1:]
            seg_y = seg_y[1:]

        dense_x.extend(seg_x)
        dense_y.extend(seg_y)

    dense_x_array = np.array(dense_x)
    dense_y_array = np.array(dense_y)

    # Project to 2.5D terrain
    dense_z = griddata(
        points=(df['x_val'], df['y_val']),
        values=df['z_val'],
        xi=(dense_x_array, dense_y_array),
        method='linear'
    )

    # Apply a small Z-offset to prevent Z-fighting, so the path sits on top of
    # the mesh rather than inside it
    z_offset = 0.05
    dense_z += z_offset

    # Add the 3D path trace to the figure
    path_trace = go.Scatter3d(
        x=dense_x_array,
        y=dense_y_array,
        z=dense_z,
        mode='lines',
        line={"color": '#FFD700', "width": 6},
        name='Projected Path'
    )
    fig.add_trace(path_trace)

    # Update UI Buttons so the path shows up in both views
    if fig.layout.updatemenus:
        menus = list(fig.layout.updatemenus)
        for button in menus[0].buttons:
            button.args[0]["visible"].append(True)
        fig.update_layout(updatemenus=menus)

    return fig


def main():
    in_path, waypoints_path = validate_paths()
    df, sorted_x, sorted_y = load_cloud(in_path)
    fig = create_interactive_view(df, sorted_x, sorted_y)
    fig = add_path(fig, df, waypoints_path)
    fig.show()


if __name__ == "__main__":
    main()
