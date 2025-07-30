#!/usr/bin/env python3
import os
import sys
import glob
import csv
import statistics

def compute_stats(arr):
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
        "std": statistics.stdev(arr) if n > 1 else 0.0
    }

def read_raw_times(filepath):
    """
    Reads a CSV where:
      - line 1 is a header (ignored)
      - line 2 is a column header (ignored)
      - subsequent lines: Success, Init Comp, Opt Comp
    Returns three lists: (success_vals, init_vals, opt_vals)
    """
    success_vals = []
    init_vals    = []
    opt_vals     = []

    with open(filepath, newline='') as f:
        reader = csv.reader(f)
        next(reader, None)  # skip header
        next(reader, None)  # skip column names

        for row in reader:
            if len(row) < 3:
                continue
            try:
                succ = float(row[0])
                init = float(row[1])
                opt  = float(row[2])
            except ValueError:
                continue
            success_vals.append(succ)
            init_vals.append(init)
            opt_vals.append(opt)

    return success_vals, init_vals, opt_vals

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

    all_succ = []
    all_init = []
    all_opt  = []

    output_csv = os.path.join(folder, "init_opt_summary.csv")
    with open(output_csv, "w", newline="") as out_f:
        writer = csv.writer(out_f)
        # expanded header to include success stats
        writer.writerow([
            "file_name",
            "succ_n", "succ_avg", "succ_min", "succ_max", "succ_std",
            "init_n", "init_avg", "init_min", "init_max", "init_std",
            "opt_n",  "opt_avg",  "opt_min",  "opt_max",  "opt_std",
        ])

        for fp in files:
            succ_vals, init_vals, opt_vals = read_raw_times(fp)
            succ_s = compute_stats(succ_vals)
            init_s = compute_stats(init_vals)
            opt_s  = compute_stats(opt_vals)

            # accumulate overall
            all_succ.extend(succ_vals)
            all_init.extend(init_vals)
            all_opt.extend(opt_vals)

            name = os.path.basename(fp)
            print(f"{name}:")
            print(f"  Success   -> n={succ_s['n']}, "
                  f"rate={succ_s['avg']:.3f}, min={succ_s['min']}, "
                  f"max={succ_s['max']}, std={succ_s['std']:.3f}")
            print(f"  Init Comp -> n={init_s['n']}, "
                  f"avg={init_s['avg']:.6f}, min={init_s['min']:.6f}, "
                  f"max={init_s['max']:.6f}, std={init_s['std']:.6f}")
            print(f"  Opt Comp  -> n={opt_s['n']}, "
                  f"avg={opt_s['avg']:.6f}, min={opt_s['min']:.6f}, "
                  f"max={opt_s['max']:.6f}, std={opt_s['std']:.6f}")
            print()

            writer.writerow([
                name,
                succ_s["n"], f"{succ_s['avg']:.6f}", succ_s["min"], succ_s["max"], f"{succ_s['std']:.6f}",
                init_s["n"], f"{init_s['avg']:.6f}", f"{init_s['min']:.6f}",
                f"{init_s['max']:.6f}", f"{init_s['std']:.6f}",
                opt_s["n"],  f"{opt_s['avg']:.6f}",  f"{opt_s['min']:.6f}",
                f"{opt_s['max']:.6f}",  f"{opt_s['std']:.6f}",
            ])

    # overall stats
    overall_succ = compute_stats(all_succ)
    overall_init = compute_stats(all_init)
    overall_opt  = compute_stats(all_opt)

    print("Overall across all files:")
    print(f"  Success   -> n={overall_succ['n']}, "
          f"rate={overall_succ['avg']:.3f}, min={overall_succ['min']}, "
          f"max={overall_succ['max']}, std={overall_succ['std']:.3f}")
    print(f"  Init Comp -> n={overall_init['n']}, "
          f"avg={overall_init['avg']:.6f}, min={overall_init['min']:.6f}, "
          f"max={overall_init['max']:.6f}, std={overall_init['std']:.6f}")
    print(f"  Opt Comp  -> n={overall_opt['n']}, "
          f"avg={overall_opt['avg']:.6f}, min={overall_opt['min']:.6f}, "
          f"max={overall_opt['max']:.6f}, std={overall_opt['std']:.6f}")

    # append overall row
    with open(output_csv, "a", newline="") as out_f:
        writer = csv.writer(out_f)
        writer.writerow([
            "Overall",
            overall_succ["n"], f"{overall_succ['avg']:.6f}", overall_succ["min"], overall_succ["max"], f"{overall_succ['std']:.6f}",
            overall_init["n"], f"{overall_init['avg']:.6f}", f"{overall_init['min']:.6f}",
            f"{overall_init['max']:.6f}", f"{overall_init['std']:.6f}",
            overall_opt["n"],  f"{overall_opt['avg']:.6f}",  f"{overall_opt['min']:.6f}",
            f"{overall_opt['max']:.6f}",  f"{overall_opt['std']:.6f}",
        ])

    print(f"Summary (including overall) written to {output_csv}")

if __name__ == "__main__":
    main()
