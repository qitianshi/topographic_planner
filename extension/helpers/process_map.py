"""Processes a CloudCompare ASCII export to a pathfinder terrain map input.

Procedure to generate input file:
1. Obtain a raw scan of the terrain (e.g., an e57 file). Ensure the coordinate
   range is normalized. Open this file in CloudCompare.
2. Run the statistical outlier filter with neighbors = 6 std. dev. = 1.0.
3. Export the Z coordinate of the cloud to a scalar field (SF).
4. Use the Rasterize tool to calculate a 2.5D map. Settings:
    * Step: 0.10
    * Direction: Z
    * Metric: Average Z
    * Empty Cells: Interpolate
   Export the map and the standard deviation and range SFs as a Cloud.
5. Export the resultant cloud as a CSV.

This tool produces the `mapdata.csv` data file. Use the printed summary to make
the `map.config` configuration file. Pass both files to the pathfinder.
"""


from enum import StrEnum
import os
import sys

import numpy as np
import pandas as pd
from scipy import ndimage


MAX_CELL_COST = 255
INFLATION_RADIUS = 0.5
DECAY_RATE = 5.0  # Lambda factor for the exponential decay buffer
LETHAL_THRESHOLD = 0.3
SUMMARY_PRINT_WIDTH = 80


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
        print("usage: python process_map.py <raw_in> <proc_out>")
        sys.exit(1)

    if not os.path.exists(sys.argv[1]):
        print(f"{sys.argv[1]} does not exist")
        sys.exit(1)

    return sys.argv[1], sys.argv[2]


def load_cloud(in_path: str) -> tuple[pd.DataFrame, np.ndarray, np.ndarray]:
    """Reads the cloud input CSV and performs rounding and categorization."""

    df = pd.read_csv(
        in_path,
        usecols=[
            InputHeaders.X,
            InputHeaders.Y,
            InputHeaders.SF_Z,
            InputHeaders.SF_Z_RANGE
        ]
    )

    # Rename columns for convenience
    df = df.rename(columns={
        InputHeaders.X: 'x_val',
        InputHeaders.Y: 'y_val',
        InputHeaders.SF_Z: 'elev',
        InputHeaders.SF_Z_RANGE: 'z_range'
    })

    # Round to smooth out floating point imprecision
    df['x_val'] = df['x_val'].round(4)
    df['y_val'] = df['y_val'].round(4)

    sorted_x = np.sort(df['x_val'].unique())
    sorted_y = np.sort(df['y_val'].unique())

    # Converts sorted coordinates to zero-indexed integers
    df['x_idx'] = pd.Categorical(df['x_val'], categories=sorted_x).codes
    df['y_idx'] = pd.Categorical(df['y_val'], categories=sorted_y).codes

    return df, sorted_x, sorted_y


def apply_costmap_inflation(
    df: pd.DataFrame,
    npoints_x: int,
    npoints_y: int,
    resolution: float
) -> np.ndarray:
    """Applies mathematical morphology to dilate obstacles and decay costs.

    For each high-cost point, first expand this same cost so it covers the
    radius of the robot, then have it decay thereon.
    """

    # Project 1D dataframe costs into a 2D grid
    grid = np.zeros((npoints_y, npoints_x), dtype=float)
    grid[df['y_idx'], df['x_idx']] = df['cost']

    # Inscribed Radius Dilation (max filter)
    # Calculate footprint radius in grid cells
    radius_cells = int(np.ceil(INFLATION_RADIUS / resolution))

    # Circular structuring element (kernel)
    y, x = np.ogrid[-radius_cells:radius_cells+1, -radius_cells:radius_cells+1]
    kernel = x**2 + y**2 <= radius_cells**2

    dilated_grid = ndimage.maximum_filter(grid, footprint=kernel)

    # Inflation Radius Decay (Euclidean Distance Transform)
    # Isolate the lethal obstacles in the newly dilated grid
    lethal_mask = dilated_grid == MAX_CELL_COST

    if np.any(lethal_mask):
        distances = ndimage.distance_transform_edt(~lethal_mask) * resolution

        # Exponential decay
        decayed_costs = MAX_CELL_COST * np.exp(-DECAY_RATE * distances)
    else:
        # Edge case: No lethal obstacles exist on the map
        decayed_costs = np.zeros_like(dilated_grid)

    final_grid = np.maximum(dilated_grid, decayed_costs)

    # Convert back to 1D
    return final_grid[df['y_idx'], df['x_idx']]


def process_terrain(
    df: pd.DataFrame,
    sorted_x: np.ndarray,
    sorted_y: np.ndarray
) -> tuple[pd.DataFrame, dict[str, float]]:
    """Computes normalized cell costs, sorts data, and extracts runtime metrics.
    """

    total_count = len(df)
    nan_mask = df['z_range'].isna()
    nan_count = nan_mask.sum()

    # Only rows without NaN
    valid_df = df[~nan_mask]

    # Calculates stats
    if not valid_df.empty:
        max_range = valid_df['z_range'].max()
        min_range = valid_df['z_range'].min()
        max_elev = valid_df['elev'].max()
        min_elev = valid_df['elev'].min()
        avg_elev = valid_df['elev'].mean()
    else:
        # Edge case: all zeroes
        max_range = min_range = max_elev = min_elev = avg_elev = 0.0

    # Linearly map 0 -> LETHAL_THRESHOLD to 0 -> MAX_CELL_COST
    normalized = (df['z_range'] / LETHAL_THRESHOLD) * MAX_CELL_COST

    # Replace NaN with MAX_CELL_COST
    costs = np.where(nan_mask, MAX_CELL_COST, normalized)
    df['cost'] = np.round(costs).clip(0, MAX_CELL_COST).astype(int)

    # Calculate the spatial resolution of the map
    resolution = np.abs(sorted_x[-1] - sorted_x[0]) / (len(sorted_x) - 1)

    df['cost'] = apply_costmap_inflation(
        df=df,
        npoints_x=len(sorted_x),
        npoints_y=len(sorted_y),
        resolution=resolution
    )

    # Final clamp to integer values
    df['cost'] = np.round(df['cost']).clip(0, MAX_CELL_COST).astype(int)

    # Sort into row-major order
    df = df.sort_values(by=['y_idx', 'x_idx']).reset_index(drop=True)

    stats = {
        'total_count': total_count,
        'nan_count': nan_count,
        'npoints_x': len(sorted_x),
        'npoints_y': len(sorted_y),
        'max_range': max_range,
        'min_range': min_range,
        'max_elev': max_elev,
        'min_elev': min_elev,
        'avg_elev': avg_elev,
        'resolution': resolution
    }

    return df, stats


def write_output(df: pd.DataFrame, out_path: str) -> None:
    """Writes the processed dataframe to the output path."""

    # Rename to expected output columns
    output_df = df[['x_idx', 'y_idx', 'elev', 'cost']].rename(
        columns={'x_idx': 'x', 'y_idx': 'y', 'elev': 'elevation'}
    )

    output_df.to_csv(out_path, index=False, float_format='%.4f')


def print_summary(stats: dict[str, float]) -> None:
    """Prints a formatted summary of the data processing results."""

    nan_percent = (
        (stats['nan_count'] / stats['total_count'] * 100)
        if stats['total_count'] > 0
        else 0.0
    )
    expected_total = stats['npoints_x'] * stats['npoints_y']
    size_check_valid = expected_total == stats['total_count']

    print("\n" + "=" * SUMMARY_PRINT_WIDTH)
    print("DATA PROCESSING SUMMARY".center(SUMMARY_PRINT_WIDTH))
    print("=" * SUMMARY_PRINT_WIDTH)
    print(f"Total Points Parsed : {stats['total_count']}")
    print(f"Map Resolution      : {stats['resolution']:.4f} m/cell")
    print(f"NPOINTS_X           : {stats['npoints_x']}")
    print(f"NPOINTS_Y           : {stats['npoints_y']}")
    print(f"Map Size Check      : {expected_total} (Expected == Total Points: {size_check_valid})")
    print("-" * SUMMARY_PRINT_WIDTH)
    print(f"Out-of-bounds cells : {stats['nan_count']} cells ({nan_percent:.2f}%)")
    print("-" * SUMMARY_PRINT_WIDTH)
    print(f"Max Elevation       : {stats['max_elev']:.4f} m")
    print(f"Min Elevation       : {stats['min_elev']:.4f} m")
    print(f"Average Elevation   : {stats['avg_elev']:.4f} m")
    print("-" * SUMMARY_PRINT_WIDTH)
    print(f"Max Z-Range (Raw)   : {stats['max_range']:.4f} m")
    print(f"Min Z-Range (Raw)   : {stats['min_range']:.4f} m")
    print("=" * SUMMARY_PRINT_WIDTH + "\n")


def main():
    in_path, out_path = validate_paths()
    df, sorted_x, sorted_y = load_cloud(in_path)
    df, stats = process_terrain(df, sorted_x, sorted_y)
    write_output(df, out_path)
    print_summary(stats)


if __name__ == "__main__":
    main()
