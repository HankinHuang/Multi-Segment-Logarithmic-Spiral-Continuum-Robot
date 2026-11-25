import argparse
import os
import pandas as pd
import numpy as np
from tabulate import tabulate

# Default data directory: <repo_root>/experiments/data
_DEFAULT_DATA_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "experiments", "data")
)

_SEGMENT_TITLES = {
    1: "The First segment",
    2: "The Second segment",
    3: "The Third segment",
    4: "The Fourth segment",
}


def evaluate_segment(segment: int, data_dir: str = _DEFAULT_DATA_DIR) -> None:
    """Compute 3D Cartesian positioning errors for a given robot segment CSV."""
    segment_name = _SEGMENT_TITLES[segment]
    input_path = os.path.join(data_dir, f"segment_{segment}_data.csv")
    output_path = os.path.join(data_dir, f"results_{segment_name}.csv")

    try:
        df = pd.read_csv(input_path)

        phi_theory_rad = np.deg2rad(df["phi_theory"])
        phi_real_rad = np.deg2rad(df["phi_real"])

        xt = df["r_theory"] * np.cos(phi_theory_rad)
        yt = df["r_theory"] * np.sin(phi_theory_rad)
        zt = df["z_theory"]

        xr = df["r_real"] * np.cos(phi_real_rad)
        yr = df["r_real"] * np.sin(phi_real_rad)
        zr = df["z_real"]

        df["error_mm"] = np.sqrt((xt - xr) ** 2 + (yt - yr) ** 2 + (zt - zr) ** 2)

        print("=" * 120)
        print(f"                          Appendix Table: Detailed Data on {segment_name} Positioning Error")
        print("=" * 120)

        appendix_table = df[["r_theory", "phi_theory", "z_theory", "r_real", "phi_real", "z_real", "error_mm"]].copy()
        appendix_table.columns = [
            "R_theory(mm)", "Phi_theory(deg)", "Z_theory(mm)",
            "R_real(mm)", "Phi_real(deg)", "Z_real(mm)", "Error(mm)",
        ]

        print(tabulate(appendix_table, headers="keys", tablefmt="grid", showindex=False, floatfmt=".2f"))

        print("\n\n" + "=" * 60)
        print(f"      Error Summary: {segment_name}")
        print("=" * 60)
        summary = {
            "Average Positioning Error (mm)": df["error_mm"].mean(),
            "Maximum Positioning Error (mm)": df["error_mm"].max(),
            "Minimum Positioning Error (mm)": df["error_mm"].min(),
            "Standard Deviation of Positioning Error (mm)": df["error_mm"].std(),
        }
        for key, value in summary.items():
            print(f"{key:<45}: {value:.2f}")
        print("=" * 60)

        print(f"\nSaving detailed results to: {output_path} ...")
        appendix_table.to_csv(output_path, index=False, encoding="utf-8-sig", float_format="%.2f")
        print("Saved successfully.")

    except FileNotFoundError:
        print(f"Error: file '{input_path}' not found.")
    except KeyError as e:
        print(f"Error: missing required column {e} in CSV.")
        print("Required columns: 'r_theory', 'phi_theory', 'z_theory', 'r_real', 'phi_real', 'z_real'")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Evaluate 3D positioning error from segment CSV data.")
    parser.add_argument("--segment", type=int, choices=[1, 2, 3, 4], default=4, help="Segment index (1-4)")
    parser.add_argument("--data-dir", type=str, default=_DEFAULT_DATA_DIR, help="Directory containing segment_*_data.csv")
    args = parser.parse_args()
    evaluate_segment(args.segment, args.data_dir)
