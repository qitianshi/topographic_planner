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

import csv
from dataclasses import dataclass, field
from enum import StrEnum
import os
import sys


MAX_CELL_COST = 255
SUMMARY_PRINT_WIDTH = 80

type CoordIdxMap = dict[float, int]


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


@dataclass
class InputCell:
    """A row of parsed raw input cell data."""
    x_val: float
    y_val: float
    elev: float
    z_range: float
    is_nan: bool


@dataclass
class OutputCell:
    """A row of output cell data."""
    x_idx: int
    y_idx: int
    elev: float
    cost: int


@dataclass
class InputStats:
    """Stats for the read raw input data."""
    max_range: float = -float('inf')
    min_range: float = float('inf')
    max_elev: float = -float('inf')
    min_elev: float = float('inf')
    sum_elev: float = 0.0
    nan_count: int = 0
    total_count: int = 0
    unique_x: set[float] = field(default_factory=set)
    unique_y: set[float] = field(default_factory=set)


def validate_paths() -> tuple[str, str]:
    """Validates the provided path arguments are valid."""

    if len(sys.argv) != 3:
        print("usage: python process_map.py <raw_in> <proc_out>")
        sys.exit(1)

    if not os.path.exists(sys.argv[1]):
        print(f"{sys.argv[1]} does not exist")
        sys.exit(1)

    return sys.argv[1], sys.argv[2]


def read_cloud(in_path: str) -> tuple[list[InputCell], InputStats]:
    """Reads and parses the raw cloud input file."""

    cells: list[InputCell] = []
    stats: InputStats = InputStats()

    with open(in_path, 'r', encoding="utf-8") as f:
        reader = csv.DictReader(f)

        for row in reader:
            stats.total_count += 1

            # Reads row
            # Rounds to smooth out floating point imprecision
            x_val = round(float(row[InputHeaders.X]), 4)
            y_val = round(float(row[InputHeaders.Y]), 4)
            elev_val = float(row[InputHeaders.SF_Z])
            range_str = row[InputHeaders.SF_Z_RANGE].strip().lower()

            stats.unique_x.add(x_val)
            stats.unique_y.add(y_val)

            # Works with both a numerical value and nan
            z_range = float(range_str)
            is_nan = range_str == "nan"

            if is_nan:
                stats.nan_count += 1
            else:
                stats.max_range = max(stats.max_range, z_range)
                stats.min_range = min(stats.min_range, z_range)
                stats.max_elev = max(stats.max_elev, elev_val)
                stats.min_elev = min(stats.min_elev, elev_val)
                stats.sum_elev += elev_val

            cells.append(InputCell(
                x_val=x_val,
                y_val=y_val,
                elev=elev_val,
                z_range=z_range,
                is_nan=is_nan
            ))

    return cells, stats


def process_cells(
    cells: list[InputCell],
    stats: InputStats,
    x_map: CoordIdxMap,
    y_map: CoordIdxMap
) -> list[OutputCell]:
    """Processes raw input cells and builds the output."""

    processed: list[OutputCell] = []

    for cell in cells:

        if cell.is_nan:
            # NaN indicates out-of-bounds cell, which is forbidden.
            cost = MAX_CELL_COST
        else:
            # Edge case for perfectly flat maps
            if stats.max_range == stats.min_range:
                cost = 0
            else:
                normalized = (cell.z_range / stats.max_range) * MAX_CELL_COST
                cost = max(0, min(255, round(normalized)))

        processed.append(OutputCell(
            x_idx=x_map[cell.x_val],
            y_idx=y_map[cell.y_val],
            elev=cell.elev,
            cost=cost
        ))

    return processed


def write_out(out_path: str, data: list[OutputCell]) -> None:
    """Writes the output to file."""

    with open(out_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(['x', 'y', 'elevation', 'cost'])

        for item in data:
            writer.writerow([
                item.x_idx,
                item.y_idx,
                f"{item.elev:.4f}",
                item.cost
            ])


def main():
    in_path, out_path = validate_paths()

    read_cells, stats = read_cloud(in_path)

    sorted_x = sorted(list(stats.unique_x))
    sorted_y = sorted(list(stats.unique_y))
    x_to_idx = {val: idx for idx, val in enumerate(sorted_x)}
    y_to_idx = {val: idx for idx, val in enumerate(sorted_y)}

    npoints_x = len(sorted_x)
    npoints_y = len(sorted_y)

    processed_data = process_cells(read_cells, stats, x_to_idx, y_to_idx)

    # Sorts into row-major order
    processed_data.sort(key=lambda item: (item.y_idx, item.x_idx))

    # Phase 5: Output Generation
    write_out(out_path, processed_data)

    # Phase 6: Print Summary
    valid_count = stats.total_count - stats.nan_count
    avg_elev = (stats.sum_elev / valid_count) if valid_count > 0 else 0.0
    nan_percent = (
        (stats.nan_count / stats.total_count) * 100
        if stats.total_count > 0
        else 0.0
    )

    print("\n" + "=" * SUMMARY_PRINT_WIDTH)
    print("DATA PROCESSING SUMMARY".center(SUMMARY_PRINT_WIDTH))
    print("=" * SUMMARY_PRINT_WIDTH)
    print(f"Total Points Parsed : {stats.total_count}")
    print(f"NPOINTS_X           : {npoints_x}")
    print(f"NPOINTS_Y           : {npoints_y}")
    print(f"Map Size Check      : {npoints_x * npoints_y}"
          f" (Expected == Total Points: {npoints_x * npoints_y == stats.total_count})")
    print("-" * SUMMARY_PRINT_WIDTH)
    print(f"Out-of-bounds cells : {stats.nan_count} cells ({nan_percent:.2f}%)")
    print("-" * SUMMARY_PRINT_WIDTH)
    print(f"Max Elevation       : {stats.max_elev:.4f} m")
    print(f"Min Elevation       : {stats.min_elev:.4f} m")
    print(f"Average Elevation   : {avg_elev:.4f} m")
    print("-" * SUMMARY_PRINT_WIDTH)
    print(f"Max Z-Range (Raw)   : {stats.max_range:.4f} m  -> Mapped to Cost 255")
    print(f"Min Z-Range (Raw)   : {stats.min_range:.4f} m  -> Mapped to Cost 0")
    print("=" * SUMMARY_PRINT_WIDTH + "\n")


if __name__ == "__main__":
    main()
