import argparse
import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def load_data(path, x_key, y_key):
    with open(path, "r", encoding="utf-8") as f:
        rows = json.load(f)
    if not isinstance(rows, list) or not rows:
        raise ValueError("JSON must contain a non-empty list of objects")
    xs = np.array([float(r[x_key]) for r in rows], dtype=float)
    ys = np.array([float(r[y_key]) for r in rows], dtype=float)
    if xs.size < 3:
        raise ValueError("Need at least 3 data points")
    return xs, ys


def fit(xs, ys, degree):
    coeffs_desc = np.polyfit(xs, ys, degree)
    coeffs_asc = coeffs_desc[::-1]
    pred = np.polyval(coeffs_desc, xs)
    ss_res = float(np.sum((ys - pred) ** 2))
    ss_tot = float(np.sum((ys - np.mean(ys)) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot != 0 else float("nan")
    rmse = float(np.sqrt(np.mean((ys - pred) ** 2)))
    return coeffs_asc, r2, rmse


def loocv_rmse(xs, ys, degree):
    n = xs.size
    if degree >= n - 1:
        return float("nan")
    errors = []
    for i in range(n):
        mask = np.ones(n, dtype=bool)
        mask[i] = False
        coeffs_desc = np.polyfit(xs[mask], ys[mask], degree)
        pred_i = np.polyval(coeffs_desc, xs[i])
        errors.append(ys[i] - pred_i)
    return float(np.sqrt(np.mean(np.array(errors) ** 2)))


def java_snippet(coeffs, method_name="computeVelocity"):
    degree = len(coeffs) - 1
    prefix = method_name if method_name else "value"
    terms = " + ".join(f"{prefix}C{i} * d**{i}" if i > 1 else (f"{prefix}C1 * d" if i == 1 else f"{prefix}C0") for i in range(degree + 1))
    lines = []
    lines.append(f"private double {method_name}(double dist) {{")
    lines.append(f"        // tune regression: {method_name} = {terms.replace(' * ', '*').replace('d**', 'd^').replace('d^1', 'd')}")
    for i, c in enumerate(coeffs):
        lines.append(f"        // {prefix}C{i} = {format(c, '.12g')}")
    if degree == 0:
        horner = f"{prefix}C0"
    elif degree == 1:
        horner = f"{prefix}C0 + dist * {prefix}C1"
    else:
        inner = f"{prefix}C{degree}"
        for i in range(degree - 1, 0, -1):
            wrapped = inner if " " not in inner and "(" not in inner else f"({inner})"
            inner = f"{prefix}C{i} + dist * {wrapped}"
        wrapped = inner if " " not in inner and "(" not in inner else f"({inner})"
        horner = f"{prefix}C0 + dist * {wrapped}"
    lines.append(f"        return {horner};")
    lines.append("    }")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="Train polynomial regressions from a JSON dataset.")
    parser.add_argument("data", nargs="?", default="17_11_2026_tuning.json", help="Path to JSON file")
    parser.add_argument("--file", dest="file", help="Path to JSON file (overrides positional)")
    parser.add_argument("--x-key", default="dist", help="Key used as independent variable")
    parser.add_argument("--y-key", default="vel", help="Key used as dependent variable")
    parser.add_argument("--use", type=int, metavar="DEGREE", help="Print Java snippet for the given polynomial degree")
    parser.add_argument("--method-name", default="computeVelocity", help="Name of the generated Java method")
    parser.add_argument("--degrees", default="1,2,3,4,5", help="Comma-separated degrees to evaluate in default mode")
    parser.add_argument("--plot-output", default="regression_plot.png", help="Output path for the plot")
    args = parser.parse_args()

    path = args.file if args.file else args.data
    xs, ys = load_data(path, args.x_key, args.y_key)
    n = xs.size

    requested = [int(d.strip()) for d in args.degrees.split(",") if d.strip()]
    degrees = sorted({d for d in requested if 1 <= d <= n - 1})
    if not degrees:
        raise ValueError("No valid degrees to evaluate")

    if args.use is not None:
        degree = args.use
        if degree < 0 or degree > n - 1:
            raise ValueError(f"Degree must be between 0 and {n - 1}")
        coeffs, r2, rmse = fit(xs, ys, degree)
        print(java_snippet(coeffs, method_name=args.method_name))
        return

    results = []
    for d in degrees:
        coeffs, r2, rmse = fit(xs, ys, d)
        cv = loocv_rmse(xs, ys, d)
        results.append((d, coeffs, r2, rmse, cv))

    # suggest by lowest LOOCV RMSE, then lower degree
    candidates = [r for r in results if not np.isnan(r[4])]
    if candidates:
        best = min(candidates, key=lambda r: (r[4], r[0]))
    else:
        best = min(results, key=lambda r: (r[3], r[0]))

    print("Degree | R^2      | RMSE     | LOOCV RMSE")
    print("-" * 42)
    for d, coeffs, r2, rmse, cv in results:
        marker = "  <- suggested" if d == best[0] else ""
        cv_str = f"{cv:.4f}" if not np.isnan(cv) else "n/a"
        print(f"{d:6d} | {r2:8.4f} | {rmse:8.4f} | {cv_str}{marker}")

    print()
    print(f"Suggested regression: degree {best[0]} "
          f"(LOOCV RMSE={best[4]:.4f})" if not np.isnan(best[4]) else
          f"Suggested regression: degree {best[0]} (RMSE={best[3]:.4f})")
    print("Coefficients (c0 + c1*x + ...): " + ", ".join(f"{c:.12g}" for c in best[1]))

    # plot
    order = np.argsort(xs)
    xs_sorted = xs[order]
    ys_sorted = ys[order]
    x_line = np.linspace(xs.min(), xs.max(), 400)

    fig, ax = plt.subplots(figsize=(9, 6))
    ax.scatter(xs, ys, color="black", zorder=5, label="data")

    cmap = plt.get_cmap("tab10")
    for idx, (d, coeffs, r2, rmse, cv) in enumerate(results):
        y_line = np.polyval(coeffs[::-1], x_line)
        ax.plot(x_line, y_line, color=cmap(idx % 10), label=f"deg {d} (R^2={r2:.3f})")

    ax.set_xlabel(args.x_key)
    ax.set_ylabel(args.y_key)
    ax.set_title("Polynomial regressions")
    ax.legend(loc="best", fontsize=8)
    ax.grid(True, alpha=0.3)

    out_path = args.plot_output
    if not os.path.isabs(out_path):
        out_path = os.path.join(os.path.dirname(os.path.abspath(path)) or ".", out_path)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"Plot saved to: {out_path}")


if __name__ == "__main__":
    main()
