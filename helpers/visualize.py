"""Visualizes a CloudCompare ASCII export as a 2.5D terrain map.

Options:
1. True color view: Uses raw RGB values via Delaunay triangulation (Mesh3d).
2. Raw cost view: Uses pre-inflation calculated costs via a grid projection (Surface).
3. Processed cost map view: Uses final processed costs via a grid projection (Surface).
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
    """Headers for the raw input CSV"""
    X = "X"
    Y = "Y"
    Z = "Z"
    R = "R"
    G = "G"
    B = "B"
    SF_Z = "Z values (0.1)"
    SF_Z_STDDEV = "Z std. dev. (0.1)"
    SF_Z_RANGE = "Z range (0.1)"


def validate_paths() -> tuple[str, str, str]:
    """Validates the provided path arguments are valid."""

    if len(sys.argv) != 4:
        print("usage: python visualize_map.py <raw_in> <processed_in> <path_in>")
        sys.exit(1)

    for i in range(1, 4):
        if not os.path.exists(sys.argv[i]):
            print(f"{sys.argv[i]} does not exist")
            sys.exit(1)

    return sys.argv[1], sys.argv[2], sys.argv[3]


def load_raw_cloud(in_path: str) -> tuple[pd.DataFrame, np.ndarray, np.ndarray]:
    """Reads the cloud input CSV and formats the color, range, and spatial data."""

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

    # Round to smooth out floating point imprecision for the grid boundaries
    df['x_val'] = df['x_val'].round(4)
    df['y_val'] = df['y_val'].round(4)

    sorted_x = np.sort(df['x_val'].unique())
    sorted_y = np.sort(df['y_val'].unique())

    return df, sorted_x, sorted_y


def load_processed_map(proc_path: str) -> pd.DataFrame:
    """Reads the processed map CSV containing the calculated costs."""
    return pd.read_csv(proc_path)


def create_interactive_view(
    raw_df: pd.DataFrame,
    proc_df: pd.DataFrame,
    sorted_x: np.ndarray,
    sorted_y: np.ndarray
) -> go.Figure:
    """Creates a single 3D figure with multiple views and UI toggle buttons."""

    fig = go.Figure()

    # Trace 0: True colors (RGB)
    fig.add_trace(go.Mesh3d(
        x=raw_df['x_val'],
        y=raw_df['y_val'],
        z=raw_df['z_val'],
        vertexcolor=raw_df['color'],
        delaunayaxis='z',
        flatshading=True,
        visible=True,
        name='True Color'
    ))

    # Base grid computations
    z_grid = \
        raw_df.pivot_table(index='y_val', columns='x_val', values='z_val').values

    # Recalculate raw cost grid manually
    z_range_grid = \
        raw_df.pivot_table(index='y_val', columns='x_val', values='z_range').values
    raw_cost_grid = \
        np.clip((z_range_grid / LETHAL_THRESHOLD) * MAX_CELL_COST, 0, MAX_CELL_COST)

    # Read processed grid from file
    processed_cost_grid = \
        proc_df.pivot_table(index='y', columns='x', values='cost').values

    # Trace 1: Raw Cost Map
    fig.add_trace(go.Surface(
        x=sorted_x,
        y=sorted_y,
        z=z_grid,
        surfacecolor=raw_cost_grid,                              # type: ignore
        cmin=0,
        cmax=MAX_CELL_COST,
        colorscale='RdBu_r',
        colorbar={"title": 'Cost (0-255)'},
        visible=False,
        name='Raw Cost'
    ))

    # Trace 2: Processed Cost Map
    fig.add_trace(go.Surface(
        x=sorted_x,
        y=sorted_y,
        z=z_grid,
        surfacecolor=processed_cost_grid,                        # type: ignore
        cmin=0,
        cmax=MAX_CELL_COST,
        colorscale='RdBu_r',
        colorbar={"title": 'Cost (0-255)'},
        visible=False,
        name='Processed Cost'
    ))

    # UI Toggle Buttons
    fig.update_layout(
        title="Terrain: True Color RGB",
        scene={"aspectmode": 'data'},
        updatemenus=[{
            "type": "buttons",
            "direction": "right",
            "x": 0.5,
            "xanchor": "center",
            "y": 1.05,
            "yanchor": "bottom",
            "showactive": True,
            "buttons": [
                {
                    "label": "True Color View",
                    "method": "update",
                    "args": [
                        {"visible": [True, False, False]},
                        {"title": "Terrain: True Color RGB"}
                    ]
                },
                {
                    "label": "Raw Cost View",
                    "method": "update",
                    "args": [
                        {"visible": [False, True, False]},
                        {"title": "Terrain: Raw Pre-inflation Cost"}
                    ]
                },
                {
                    "label": "Processed Cost Map View",
                    "method": "update",
                    "args": [
                        {"visible": [False, False, True]},
                        {"title": "Terrain: Processed Post-inflation Cost Map"}
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

    # Update UI Buttons so the path shows up in all 3 views
    if fig.layout.updatemenus:
        menus = list(fig.layout.updatemenus)
        for button in menus[0].buttons:
            button.args[0]["visible"].append(True)
        fig.update_layout(updatemenus=menus)

    return fig


def main():
    raw_path, proc_path, waypoints_path = validate_paths()

    raw_df, sorted_x, sorted_y = load_raw_cloud(raw_path)
    proc_df = load_processed_map(proc_path)

    fig = create_interactive_view(raw_df, proc_df, sorted_x, sorted_y)
    fig = add_path(fig, raw_df, waypoints_path)

    fig.show()


if __name__ == "__main__":
    main()
