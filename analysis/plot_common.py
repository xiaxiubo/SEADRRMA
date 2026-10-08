from __future__ import annotations

import math
from pathlib import Path
from typing import Iterable

import pandas as pd


SummaryRow = dict[str, float | str]


def find_latest_csv(log_dir: Path, pattern: str) -> Path | None:
    csv_files = sorted(log_dir.glob(pattern), key=lambda path: path.stat().st_mtime, reverse=True)
    return csv_files[0] if csv_files else None


def require_columns(df: pd.DataFrame, columns: Iterable[str], csv_path: Path) -> None:
    missing = [column for column in columns if column not in df.columns]
    if missing:
        missing_text = ", ".join(missing)
        raise ValueError(
            f"{csv_path} is not a current-format CSV. Missing required column(s): {missing_text}"
        )


def warn_if_flat_signal(df: pd.DataFrame, column: str, label: str, threshold: float = 1e-6) -> None:
    std = float(df[column].std())
    min_val = float(df[column].min())
    max_val = float(df[column].max())
    if std <= threshold:
        print(
            f"Warning: {label} is nearly constant "
            f"(std={std:.3e}, min={min_val:.6f}, max={max_val:.6f})"
        )


def _quantile(series: pd.Series, q: float) -> float:
    values = pd.to_numeric(series, errors="coerce").dropna().astype(float).tolist()
    if not values:
        return float("nan")
    values.sort()
    idx = max(0, min(len(values) - 1, math.ceil(q * len(values)) - 1))
    return float(values[idx])


def summarize_timing(
    df: pd.DataFrame,
    timing_columns: Iterable[tuple[str, str]],
) -> list[SummaryRow]:
    rows: list[SummaryRow] = []
    for column, label in timing_columns:
        series = pd.to_numeric(df[column], errors="coerce").dropna().astype(float)
        if len(series) > 1:
            series = series.iloc[1:]
        rows.append(
            {
                "name": label,
                "mean_ms": float(series.mean() * 1000.0),
                "p50_ms": float(_quantile(series, 0.50) * 1000.0),
                "p95_ms": float(_quantile(series, 0.95) * 1000.0),
                "p99_ms": float(_quantile(series, 0.99) * 1000.0),
                "max_ms": float(series.max() * 1000.0),
            }
        )

    times = pd.to_numeric(df["time_s"], errors="coerce").dropna().astype(float).tolist()
    deltas = [b - a for a, b in zip(times, times[1:])]
    delta_series = pd.Series(deltas, dtype=float)
    rows.append(
        {
            "name": "Cycle Interval",
            "mean_ms": float(delta_series.mean() * 1000.0),
            "p50_ms": float(_quantile(delta_series, 0.50) * 1000.0),
            "p95_ms": float(_quantile(delta_series, 0.95) * 1000.0),
            "p99_ms": float(_quantile(delta_series, 0.99) * 1000.0),
            "max_ms": float(delta_series.max() * 1000.0),
        }
    )
    return rows


def summarize_schedule(df: pd.DataFrame) -> list[SummaryRow]:
    schedule_columns = [
        ("wake_lag_ms", "Wake Lag"),
        ("deadline_slip_ms", "Deadline Slip"),
        ("sleep_target_ms", "Sleep Target"),
        ("sleep_actual_ms", "Sleep Actual"),
        ("deadline_miss_delta", "Miss Delta"),
        ("deadline_miss_count", "Miss Count"),
    ]
    rows: list[SummaryRow] = []
    for column, label in schedule_columns:
        series = pd.to_numeric(df[column], errors="coerce").dropna().astype(float)
        if len(series) > 1 and column in {"wake_lag_ms", "deadline_slip_ms"}:
            series = series.iloc[1:]
        rows.append(
            {
                "name": label,
                "mean_ms": float(series.mean()),
                "p50_ms": float(series.quantile(0.50)),
                "p95_ms": float(series.quantile(0.95)),
                "p99_ms": float(series.quantile(0.99)),
                "max_ms": float(series.max()),
            }
        )
    return rows


def print_summary(title: str, rows: list[SummaryRow]) -> None:
    print(f"\n{title}", flush=True)
    header = f"{'name':<18} {'mean':>8} {'p50':>8} {'p95':>8} {'p99':>8} {'max':>8}"
    print(header, flush=True)
    print("-" * len(header), flush=True)
    for row in rows:
        print(
            f"{row['name']:<18} "
            f"{row['mean_ms']:>8.3f} "
            f"{row['p50_ms']:>8.3f} "
            f"{row['p95_ms']:>8.3f} "
            f"{row['p99_ms']:>8.3f} "
            f"{row['max_ms']:>8.3f}",
            flush=True,
        )
