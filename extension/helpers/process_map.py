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


import os
import sys
from enum import StrEnum
import numpy as np
import pandas as pd


MAX_CELL_COST = 255
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

    # Calculates costs
    if max_range == min_range:
        # Edge case: perfectly flat map
        costs = np.where(nan_mask, MAX_CELL_COST, 0)
    else:
        normalized = (df['z_range'] / max_range) * MAX_CELL_COST
        # Replace NaN with MAX_CELL_COST
        costs = np.where(nan_mask, MAX_CELL_COST, normalized)
        costs = np.round(costs).clip(0, 255).astype(int)

    df['cost'] = costs

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
        'avg_elev': avg_elev
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
    print(f"Max Z-Range (Raw)   : {stats['max_range']:.4f} m -> Mapped to Cost 255")
    print(f"Min Z-Range (Raw)   : {stats['min_range']:.4f} m -> Mapped to Cost 0")
    print("=" * SUMMARY_PRINT_WIDTH + "\n")


def main():
    in_path, out_path = validate_paths()
    df, sorted_x, sorted_y = load_cloud(in_path)
    df, stats = process_terrain(df, sorted_x, sorted_y)
    write_output(df, out_path)
    print_summary(stats)


if __name__ == "__main__":
    main()
