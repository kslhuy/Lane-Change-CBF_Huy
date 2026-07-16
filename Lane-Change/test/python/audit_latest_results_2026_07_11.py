"""Audit the latest local/global/both simulation workbooks from 2026-07-11.

The reader uses only the Python standard library so that the MATLAB-generated
XLSX files can be checked without changing them or requiring Excel.
"""

from __future__ import annotations

import argparse
import json
import re
import statistics
import zipfile
from pathlib import Path
from typing import Any
from xml.etree import ElementTree as ET


NS = {"m": "http://schemas.openxmlformats.org/spreadsheetml/2006/main"}
REL_NS = {"r": "http://schemas.openxmlformats.org/package/2006/relationships"}
DOC_REL = "{http://schemas.openxmlformats.org/officeDocument/2006/relationships}id"


def column_index(cell_reference: str) -> int:
    letters = re.match(r"[A-Z]+", cell_reference).group(0)
    index = 0
    for letter in letters:
        index = index * 26 + ord(letter) - ord("A") + 1
    return index - 1


def coerce_number(value: str) -> int | float:
    number = float(value)
    return int(number) if number.is_integer() else number


def load_xlsx(path: Path) -> dict[str, list[list[Any]]]:
    with zipfile.ZipFile(path) as archive:
        shared_strings: list[str] = []
        if "xl/sharedStrings.xml" in archive.namelist():
            root = ET.fromstring(archive.read("xl/sharedStrings.xml"))
            for item in root.findall("m:si", NS):
                shared_strings.append("".join(node.text or "" for node in item.iterfind(".//m:t", NS)))

        relationships = ET.fromstring(archive.read("xl/_rels/workbook.xml.rels"))
        targets = {
            rel.attrib["Id"]: rel.attrib["Target"]
            for rel in relationships.findall("r:Relationship", REL_NS)
        }
        workbook = ET.fromstring(archive.read("xl/workbook.xml"))
        sheets: dict[str, list[list[Any]]] = {}
        for sheet in workbook.findall("m:sheets/m:sheet", NS):
            name = sheet.attrib["name"]
            target = targets[sheet.attrib[DOC_REL]].replace("\\", "/")
            member = target if target.startswith("xl/") else f"xl/{target}"
            worksheet = ET.fromstring(archive.read(member))
            sparse_rows: dict[int, dict[int, Any]] = {}
            max_column = -1
            for cell in worksheet.findall(".//m:sheetData/m:row/m:c", NS):
                reference = cell.attrib["r"]
                row_index = int(re.search(r"\d+", reference).group(0)) - 1
                col_index = column_index(reference)
                cell_type = cell.attrib.get("t", "n")
                value_node = cell.find("m:v", NS)
                if cell_type == "inlineStr":
                    value = "".join(node.text or "" for node in cell.iterfind(".//m:t", NS))
                elif value_node is None:
                    value = None
                elif cell_type == "s":
                    value = shared_strings[int(value_node.text)]
                elif cell_type == "b":
                    value = value_node.text == "1"
                elif cell_type in {"str", "e"}:
                    value = value_node.text or ""
                else:
                    value = coerce_number(value_node.text)
                sparse_rows.setdefault(row_index, {})[col_index] = value
                max_column = max(max_column, col_index)
            if not sparse_rows:
                sheets[name] = []
                continue
            max_row = max(sparse_rows)
            sheets[name] = [
                [sparse_rows.get(row, {}).get(col) for col in range(max_column + 1)]
                for row in range(max_row + 1)
            ]
        return sheets


def latest_workbooks(results_root: Path) -> dict[str, Path]:
    workbooks: dict[str, Path] = {}
    for mode in ("local", "global", "both"):
        group = results_root / mode / "attacker_V1" / "Mix_test"
        pointer = (group / "latest_run_folder.txt").read_text(encoding="utf-8-sig")
        match = re.search(r"^Latest run folder:\s*\n([^\r\n]+)", pointer, re.MULTILINE)
        if not match:
            raise ValueError(f"Cannot parse latest run pointer: {group}")
        run_folder = Path(match.group(1).strip())
        candidates = list(run_folder.glob("Results_*.xlsx"))
        if len(candidates) != 1:
            raise ValueError(f"Expected one workbook in {run_folder}, found {len(candidates)}")
        workbooks[mode] = candidates[0]
    return workbooks


def records(rows: list[list[Any]]) -> list[dict[str, Any]]:
    if not rows:
        return []
    header = [str(value) for value in rows[0]]
    return [dict(zip(header, row)) for row in rows[1:]]


REPORT_CASE = re.compile(
    r"^Case (?P<case>\d+) \((?P<description>[^)]+)\): "
    r"trust pre (?P<trust_pre>[0-9.]+), trust attack (?P<trust_attack>[0-9.]+), "
    r"drop (?P<trust_drop>[0-9.]+), global-source pre (?P<source_pre>[0-9.]+), "
    r"global-source attack (?P<source_attack>[0-9.]+), source drop (?P<source_drop>[0-9.]+), "
    r"source zero-rate (?P<source_zero_rate>[0-9.]+)%, detection (?P<detection_rate>[0-9.]+)%, "
    r"mean detection (?P<detection_time>[0-9.]+) s$",
    re.MULTILINE,
)


def parse_summary_report(path: Path) -> list[dict[str, Any]]:
    text = path.read_text(encoding="utf-8-sig")
    parsed: list[dict[str, Any]] = []
    for match in REPORT_CASE.finditer(text):
        item: dict[str, Any] = match.groupdict()
        item["case"] = int(item["case"])
        for key in (
            "trust_pre",
            "trust_attack",
            "trust_drop",
            "source_pre",
            "source_attack",
            "source_drop",
            "source_zero_rate",
            "detection_rate",
            "detection_time",
        ):
            item[key] = float(item[key])
        item["source_zero_rate"] /= 100.0
        item["detection_rate"] /= 100.0
        item["detection_delay"] = item["detection_time"] - 10.0
        parsed.append(item)
    if len(parsed) != 5:
        raise ValueError(f"Expected five case summaries in {path}, found {len(parsed)}")
    return parsed


COMMAND_CASE = re.compile(
    r"^Case (?P<case>\d+) \([^)]+\): .*?"
    r"benign false rejection (?P<false_rejection>[0-9.]+)%, "
    r"trusted neighbors (?P<trusted_neighbors>[0-9.]+),",
    re.MULTILINE,
)


def parse_command_diagnostic(path: Path, case_number: int) -> dict[str, float]:
    text = path.read_text(encoding="utf-8-sig")
    matches = [
        match.groupdict()
        for match in COMMAND_CASE.finditer(text)
        if int(match.group("case")) == case_number
    ]
    if not matches:
        raise ValueError(f"Cannot find Case {case_number} diagnostic in {path}")
    return {
        "false_rejection_rate": float(matches[-1]["false_rejection"]) / 100.0,
        "mean_trusted_neighbors": float(matches[-1]["trusted_neighbors"]),
    }


def mean(values: list[float]) -> float:
    return statistics.fmean(values)


def describe(values: list[float]) -> dict[str, float]:
    return {
        "mean": mean(values),
        "sample_std": statistics.stdev(values) if len(values) > 1 else 0.0,
        "min": min(values),
        "max": max(values),
    }


def analyze_latest(results_root: Path) -> dict[str, Any]:
    workbooks = latest_workbooks(results_root)
    all_cells: list[dict[str, Any]] = []
    all_cases: list[dict[str, Any]] = []
    channels: dict[str, Any] = {}
    checks: dict[str, Any] = {
        "raw_combined_formula_max_abs_error": 0.0,
        "trust_report_vs_workbook_max_abs_error": 0.0,
        "detection_report_vs_workbook_max_abs_error_s": 0.0,
        "source_column_mislabeled": True,
    }

    for mode, workbook_path in workbooks.items():
        sheets = load_xlsx(workbook_path)
        summary = records(sheets["Summary"])
        trust_rows = records(sheets["TrustWeightStats"])
        report_rows = parse_summary_report(workbook_path.with_name("summary_report.txt"))
        trust_by_case = {int(str(row["Case"]).split()[-1]): row for row in trust_rows}
        report_by_case = {row["case"]: row for row in report_rows}

        mode_cells: list[dict[str, Any]] = []
        for row in summary:
            case_number = int(str(row["Case"]).split()[-1])
            raw = float(row["AttackWindow_Raw_Combined"])
            component_sum = (
                float(row["AttackWindow_RMSE_Distance"])
                + float(row["AttackWindow_RMSE_Velocity"])
                + float(row["AttackWindow_RMSE_Acceleration"])
            )
            checks["raw_combined_formula_max_abs_error"] = max(
                checks["raw_combined_formula_max_abs_error"], abs(raw - component_sum)
            )
            item = {
                "mode": mode,
                "case": case_number,
                "vehicle": str(row["Vehicle"]),
                "raw_combined_rmse": raw,
                "distance_rmse": float(row["AttackWindow_RMSE_Distance"]),
                "orientation_rmse": float(row["AttackWindow_RMSE_Orientation"]),
                "velocity_rmse": float(row["AttackWindow_RMSE_Velocity"]),
                "acceleration_rmse": float(row["AttackWindow_RMSE_Acceleration"]),
            }
            mode_cells.append(item)
            all_cells.append(item)

        mode_cases: list[dict[str, Any]] = []
        for case_number in range(1, 6):
            source = report_by_case[case_number]
            workbook_trust = trust_by_case[case_number]
            case_cells = [row for row in mode_cells if row["case"] == case_number]
            for report_key, workbook_key in (
                ("trust_pre", "MeanTrust_PreAttack"),
                ("trust_attack", "MeanTrust_AttackWindow"),
                ("trust_drop", "TrustDrop_PreMinusAttack"),
            ):
                checks["trust_report_vs_workbook_max_abs_error"] = max(
                    checks["trust_report_vs_workbook_max_abs_error"],
                    abs(float(source[report_key]) - float(workbook_trust[workbook_key])),
                )
            checks["detection_report_vs_workbook_max_abs_error_s"] = max(
                checks["detection_report_vs_workbook_max_abs_error_s"],
                abs(float(source["detection_time"]) - float(workbook_trust["MeanDetectionTime"])),
            )
            # These workbook columns contain max(direct weight, source weight), not source weight.
            if abs(
                float(workbook_trust["MeanAttackerSourceInfluence_AttackWindow"])
                - float(source["source_attack"])
            ) < 1e-6:
                checks["source_column_mislabeled"] = False

            case_item = {
                "mode": mode,
                "case": case_number,
                "description": source["description"],
                "cell_count": len(case_cells),
                "raw_combined_rmse": describe([row["raw_combined_rmse"] for row in case_cells]),
                **{key: source[key] for key in source if key not in {"case", "description"}},
            }
            mode_cases.append(case_item)
            all_cases.append(case_item)

        raw_values = [row["raw_combined_rmse"] for row in mode_cells]
        source_pre = mean([row["source_pre"] for row in mode_cases])
        source_attack = mean([row["source_attack"] for row in mode_cases])
        channels[mode] = {
            "workbook": str(workbook_path.resolve()),
            "run_folder": str(workbook_path.parent.resolve()),
            "cell_count": len(mode_cells),
            "case_count": len(mode_cases),
            "raw_combined_rmse": describe(raw_values),
            "trust_pre": mean([row["trust_pre"] for row in mode_cases]),
            "trust_attack": mean([row["trust_attack"] for row in mode_cases]),
            "trust_drop": mean([row["trust_drop"] for row in mode_cases]),
            "detection_rate": mean([row["detection_rate"] for row in mode_cases]),
            "detection_delay_s": {
                **describe([row["detection_delay"] for row in mode_cases]),
                "median": statistics.median([row["detection_delay"] for row in mode_cases]),
            },
            "source_pre": source_pre,
            "source_attack": source_attack,
            "source_reduction_fraction": 1.0 - source_attack / source_pre,
            "source_zero_rate": mean([row["source_zero_rate"] for row in mode_cases]),
        }

    overall_source_pre = mean([row["source_pre"] for row in all_cases])
    overall_source_attack = mean([row["source_attack"] for row in all_cases])
    case_aggregates: list[dict[str, Any]] = []
    for case_number in range(1, 6):
        case_cells = [row for row in all_cells if row["case"] == case_number]
        case_rows = [row for row in all_cases if row["case"] == case_number]
        case_source_pre = mean([row["source_pre"] for row in case_rows])
        case_source_attack = mean([row["source_attack"] for row in case_rows])
        case_aggregates.append(
            {
                "case": case_number,
                "description": case_rows[0]["description"],
                "cell_count": len(case_cells),
                "raw_combined_rmse": describe([row["raw_combined_rmse"] for row in case_cells]),
                "trust_pre": mean([row["trust_pre"] for row in case_rows]),
                "trust_attack": mean([row["trust_attack"] for row in case_rows]),
                "trust_drop": mean([row["trust_drop"] for row in case_rows]),
                "detection_rate": mean([row["detection_rate"] for row in case_rows]),
                "detection_delay_s": mean([row["detection_delay"] for row in case_rows]),
                "source_pre": case_source_pre,
                "source_attack": case_source_attack,
                "source_reduction_fraction": 1.0 - case_source_attack / case_source_pre,
                "source_zero_rate": mean([row["source_zero_rate"] for row in case_rows]),
            }
        )
    case_aggregates.sort(key=lambda row: row["raw_combined_rmse"]["mean"], reverse=True)

    strongest_cell = max(all_cells, key=lambda row: row["raw_combined_rmse"])
    overall = {
        "run_count": 3,
        "attacker": "V1",
        "case_mode_count": len(all_cases),
        "vehicle_case_mode_cell_count": len(all_cells),
        "raw_combined_rmse": describe([row["raw_combined_rmse"] for row in all_cells]),
        "component_rmse_means": {
            "longitudinal_position_m": mean([row["distance_rmse"] for row in all_cells]),
            "orientation_rad_separate": mean([row["orientation_rmse"] for row in all_cells]),
            "velocity_m_per_s": mean([row["velocity_rmse"] for row in all_cells]),
            "acceleration_input_m_per_s2": mean(
                [row["acceleration_rmse"] for row in all_cells]
            ),
        },
        "strongest_cell": strongest_cell,
        "trust_pre": mean([row["trust_pre"] for row in all_cases]),
        "trust_attack": mean([row["trust_attack"] for row in all_cases]),
        "trust_drop": mean([row["trust_drop"] for row in all_cases]),
        "detection_rate": mean([row["detection_rate"] for row in all_cases]),
        "detection_delay_s": {
            **describe([row["detection_delay"] for row in all_cases]),
            "median": statistics.median([row["detection_delay"] for row in all_cases]),
        },
        "source_pre": overall_source_pre,
        "source_attack": overall_source_attack,
        "source_reduction_fraction": 1.0 - overall_source_attack / overall_source_pre,
        "source_zero_rate": mean([row["source_zero_rate"] for row in all_cases]),
    }
    both_case_2_collateral = parse_command_diagnostic(
        workbooks["both"].parent / "logs" / "command_window.log", 2
    )
    return {
        "scope": {
            "results_root": str(results_root.resolve()),
            "archive_date": "2026-07-11",
            "attack_window_s": [10.0, 15.0],
            "modes": ["local", "global", "both"],
            "attacker": "V1",
            "latest_pointer_used": True,
            "selected_run_ids": {
                mode: workbook.parent.name for mode, workbook in workbooks.items()
            },
            "detection_delay_definition": (
                "case-level mean of evaluator-wise first crossings below trust 0.70"
            ),
            "source_zero_rate_definition": (
                "fraction of source-only evaluator-time weights <= 1e-9; "
                "10020 samples/channel and 30060 samples overall"
            ),
        },
        "checks": checks,
        "diagnostics": {
            "both_case_2_attack_window_collateral": {
                **both_case_2_collateral,
                "not_a_no_attack_false_positive_rate": True,
            }
        },
        "overall": overall,
        "channels": channels,
        "cases_ranked_by_mean_raw_combined_rmse": case_aggregates,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-root", type=Path, default=Path("test/results/2026-07-11"))
    parser.add_argument("--mode", choices=("local", "global", "both"))
    parser.add_argument("--sheet")
    parser.add_argument("--max-rows", type=int, default=30)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--audit", action="store_true")
    args = parser.parse_args()

    if args.audit:
        print(json.dumps(analyze_latest(args.results_root), indent=2, ensure_ascii=False))
        return

    workbooks = latest_workbooks(args.results_root)
    modes = [args.mode] if args.mode else list(workbooks)
    result: dict[str, dict[str, list[list[Any]]]] = {}
    for mode in modes:
        sheets = load_xlsx(workbooks[mode])
        if args.sheet:
            sheets = {args.sheet: sheets[args.sheet]}
        result[mode] = {name: rows[: args.max_rows] for name, rows in sheets.items()}

    if args.json:
        print(json.dumps(result, indent=2, ensure_ascii=False))
        return
    for mode, sheets in result.items():
        print(f"[{mode}] {workbooks[mode]}")
        for name, rows in sheets.items():
            width = max((len(row) for row in rows), default=0)
            print(f"\n--- {name}: showing {len(rows)} row(s), {width} column(s) ---")
            for row_number, row in enumerate(rows, start=1):
                print(f"{row_number:>4}\t" + "\t".join("" if value is None else str(value) for value in row))


if __name__ == "__main__":
    main()
