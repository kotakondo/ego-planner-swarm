#!/usr/bin/env python3
import os
import sys
import glob
import csv
import statistics
from typing import List, Tuple

def compute_stats(arr: List[float]):
    """
    Given a list of floats (or ints), return a dict with n, avg, min, max, std.
    """
    n = len(arr)
    if n == 0:
        return {"n": 0, "avg": 0.0, "min": 0.0, "max": 0.0, "std": 0.0}
    return {
        "n":   n,
        "avg": statistics.mean(arr),
        "min": min(arr),
        "max": max(arr),
        "std": statistics.stdev(arr) if n > 1 else 0.0,
    }

def try_parse_float(x: str):
    try:
        return float(x.strip())
    except Exception:
        return None

def read_raw_times(filepath: str) -> Tuple[List[float], List[float], List[float], List[float], List[float]]:
    """
    Reads a CSV where rows are:
      Success, Init Comp, Opt Comp, Refine Comp
    (Older files may have only 3 columns.)

    Returns five lists:
      success_vals, init_vals, opt_vals, refine_vals, total_vals
    - refine_vals only includes rows that actually have a refine value
    - total_vals = init + opt + (refine if present else 0)
    """
    success_vals, init_vals, opt_vals, refine_vals, total_vals = [], [], [], [], []

    with open(filepath, newline='') as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue
            s = try_parse_float(row[0]) if len(row) >= 1 else None
            i = try_parse_float(row[1]) if len(row) >= 2 else None
            o = try_parse_float(row[2]) if len(row) >= 3 else None
            r = try_parse_float(row[3]) if len(row) >= 4 else None

            # Need at least success, init, and opt to count this row
            if s is None or i is None or o is None:
                continue

            success_vals.append(s)
            init_vals.append(i)
            opt_vals.append(o)

            # Only record refine if present; treat missing as 0 for totals
            if r is not None:
                refine_vals.append(r)
                total_vals.append(i + o + r)
            else:
                total_vals.append(i + o)

    return success_vals, init_vals, opt_vals, refine_vals, total_vals

def fmt_stats_line(label: str, s: dict, is_success: bool = False) -> str:
    """
    Format a single stats line for the text report.
    For success we show 'rate' (avg) to 3 decimals; times get 6 decimals.
    """
    if is_success:
        return (f"  {label:<10}-> n={s['n']}, rate={s['avg']:.3f}, "
                f"min={s['min']:.3f}, max={s['max']:.3f}, std={s['std']:.3f}")
    else:
        return (f"  {label:<10}-> n={s['n']}, avg={s['avg']:.6f}, "
                f"min={s['min']:.6f}, max={s['max']:.6f}, std={s['std']:.6f}")

def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <folder_path>")
        sys.exit(1)

    folder = sys.argv[1]
    pattern = os.path.join(folder, "results_num_*.csv")
    files = sorted(glob.glob(pattern))
    if not files:
        print("No files matching pattern found in folder:", folder)
        sys.exit(1)

    all_succ: List[float] = []
    all_init: List[float] = []
    all_opt:  List[float] = []
    all_ref:  List[float] = []
    all_tot:  List[float] = []

    output_txt = os.path.join(folder, "ego_swarm_comp_times.txt")
    with open(output_txt, "w") as out:
        out.write("EGO Swarm Computation Statistics\n")
        out.write(f"Folder: {folder}\n")
        out.write(f"Files matched: {len(files)}\n")
        out.write("-" * 60 + "\n\n")

        # Per-file stats
        for fp in files:
            succ_vals, init_vals, opt_vals, ref_vals, tot_vals = read_raw_times(fp)
            succ_s = compute_stats(succ_vals)
            init_s = compute_stats(init_vals)
            opt_s  = compute_stats(opt_vals)
            ref_s  = compute_stats(ref_vals)
            tot_s  = compute_stats(tot_vals)

            # accumulate overall
            all_succ.extend(succ_vals)
            all_init.extend(init_vals)
            all_opt.extend(opt_vals)
            all_ref.extend(ref_vals)
            all_tot.extend(tot_vals)

            name = os.path.basename(fp)
            out.write(f"{name}:\n")
            out.write(fmt_stats_line("Success",   succ_s, is_success=True) + "\n")
            out.write(fmt_stats_line("Init Comp", init_s) + "\n")
            out.write(fmt_stats_line("Opt Comp",  opt_s)  + "\n")
            out.write(fmt_stats_line("Refine",    ref_s)  + "\n")
            out.write(fmt_stats_line("Total",     tot_s)  + "\n")
            out.write("\n")

        # Overall stats across all files
        overall_succ = compute_stats(all_succ)
        overall_init = compute_stats(all_init)
        overall_opt  = compute_stats(all_opt)
        overall_ref  = compute_stats(all_ref)
        overall_tot  = compute_stats(all_tot)

        out.write("-" * 60 + "\n")
        out.write("Overall across all files:\n")
        out.write(fmt_stats_line("Success",   overall_succ, is_success=True) + "\n")
        out.write(fmt_stats_line("Init Comp", overall_init) + "\n")
        out.write(fmt_stats_line("Opt Comp",  overall_opt)  + "\n")
        out.write(fmt_stats_line("Refine",    overall_ref)  + "\n")
        out.write(fmt_stats_line("Total",     overall_tot)  + "\n")

    print(f"Wrote summary to: {output_txt}")

if __name__ == "__main__":
    main()
