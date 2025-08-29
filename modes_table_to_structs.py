#!/usr/bin/env python3
import argparse, csv, re, sys
from pathlib import Path
from typing import Dict, List, Tuple

# Default min_hmax/min_vmax per RAW depth (from your examples).
DEFAULTS = {
    "RAW10": {"min_hmax": 947,  "min_vmax": 1116},
    "RAW12": {"min_hmax": 1122, "min_vmax": 1111},
    "RAW14": {"min_hmax": 1730, "min_vmax": 1444},
}

def parse_args():
    ap = argparse.ArgumentParser(
        description="Generate imx294 supported_modes_*bit[] tables from a TSV of modes."
    )
    ap.add_argument("input", help="Tab-separated file with the readout modes table.")
    ap.add_argument("-o", "--output", help="Write generated C to this file (default: stdout).")
    # Optional overrides for min_hmax/min_vmax by bit depth
    ap.add_argument("--min-hmax-10", type=int, default=None)
    ap.add_argument("--min-vmax-10", type=int, default=None)
    ap.add_argument("--min-hmax-12", type=int, default=None)
    ap.add_argument("--min-vmax-12", type=int, default=None)
    ap.add_argument("--min-hmax-14", type=int, default=None)
    ap.add_argument("--min-vmax-14", type=int, default=None)
    return ap.parse_args()

def load_tsv(p: Path) -> Tuple[List[str], List[Dict[str, str]]]:
    with p.open("r", newline="") as f:
        r = csv.DictReader(f, delimiter="\t")
        rows = [ {k.strip(): (v.strip() if v is not None else "") for k,v in row.items()} for row in r ]
        return r.fieldnames or [], rows

def to_int(s: str) -> int:
    s = s.strip()
    if not s:
        return 0
    return int(s, 0)

def to_float(s: str) -> float:
    s = s.strip()
    if not s:
        return 0.0
    return float(s)

def round_div_inverse(x: float) -> int:
    # scale = round(1 / x); guard divide-by-zero
    if x == 0:
        return 0
    return int(round(1.0 / x))

def nice_aspect_from_mode(mode_token: str) -> str:
    # mode_2_17_9  ->  "17:9"
    parts = mode_token.split("_")
    if len(parts) >= 4 and parts[0] == "mode":
        return f"{parts[-2]}:{parts[-1]}"
    return ""

def mode_number_from_token(mode_token: str) -> str:
    # mode_2_17_9 -> "2", mode_1A_17_9 -> "1A", mode_0_4_3 -> "0"
    parts = mode_token.split("_")
    return parts[1] if len(parts) >= 2 else mode_token

def c_comment_for(mode_token: str, raw: str, description: str) -> str:
    ar = nice_aspect_from_mode(mode_token)
    num = mode_number_from_token(mode_token)
    # Prefer em dash if aspect known
    if ar:
        # Trim duplicate RAW mention if description already states it; keep your style
        return f"/* {ar} Mode {num} — {description.strip()} */"
    return f"/* {mode_token} — {description.strip()} ({raw}) */"

def group_by_bitdepth(rows: List[Dict[str,str]]) -> Dict[str, List[Dict[str,str]]]:
    groups = {"RAW10": [], "RAW12": [], "RAW14": []}
    for row in rows:
        raw = row.get("RAW","").upper()
        if raw in groups:
            groups[raw].append(row)
    return groups

def emit_group(rows: List[Dict[str,str]], raw: str, overrides: Dict[str,int]) -> str:
    if not rows:
        return f"static struct imx294_mode supported_modes_{raw[3:]}bit[] = {{}};\n"
    lines: List[str] = []
    lines.append(f"static struct imx294_mode supported_modes_{raw[3:]}bit[] = {{")
    for row in rows:
        mode_token = row["Readout mode No"]
        desc = row.get("Readout drive mode Mode description","").strip()

        total_w = to_int(row.get("total_width","0"))   # sensor readout width
        total_h = to_int(row.get("total_height","0"))  # sensor readout height
        top     = to_int(row.get("top","0"))
        left    = to_int(row.get("left","0"))
        cw      = to_int(row.get("width","0"))
        ch      = to_int(row.get("height","0"))

        hmax_per_h = to_float(row.get("HMAX number per H period","0"))
        scale = round_div_inverse(hmax_per_h)
        integration_offset = to_int(row.get("Offset","0"))
        min_shr = to_int(row.get("min_SHR","0"))

        # min_hmax/min_vmax from overrides/defaults
        min_hmax = to_int(row.get("min_hmax","0"))
        min_vmax = to_int(row.get("min_vmax","0"))

        reg_list_name = f"{mode_token}_regs"
        lines.append(f"   {c_comment_for(mode_token, raw, desc)}")
        lines.append("    {")

        # NEW: total_* hold the full readout size; width/height are zero at top-level
        lines.append(f"        .width  = {total_w},")
        lines.append(f"        .height = {total_h},")

        lines.append(f"        .min_hmax = {min_hmax},")
        lines.append(f"        .min_vmax = {min_vmax},")
        lines.append(f"        .scale = {scale},")
        lines.append(f"        .min_shr = {min_shr},")
        lines.append(f"        .integration_offset = {integration_offset},")

        lines.append("        .crop = {")
        lines.append(f"            .left = {left},")
        lines.append(f"            .top = {top},")
        lines.append(f"            .width = {cw},")
        lines.append(f"            .height = {ch},")
        lines.append("        },")

        lines.append("        .reg_list = {")
        lines.append(f"            .num_of_regs = ARRAY_SIZE({reg_list_name}),")
        lines.append(f"            .regs        = {reg_list_name},")
        lines.append("        },")
        lines.append("    },")
    lines.append("};\n")
    return "\n".join(lines)


def main():
    args = parse_args()
    headers, rows = load_tsv(Path(args.input))

    # Apply CLI overrides
    overrides_10 = {
        k: v for k, v in {
            "min_hmax": args.min_hmax_10,
            "min_vmax": args.min_vmax_10
        }.items() if v is not None
    }
    overrides_12 = {
        k: v for k, v in {
            "min_hmax": args.min_hmax_12,
            "min_vmax": args.min_vmax_12
        }.items() if v is not None
    }
    overrides_14 = {
        k: v for k, v in {
            "min_hmax": args.min_hmax_14,
            "min_vmax": args.min_vmax_14
        }.items() if v is not None
    }

    groups = group_by_bitdepth(rows)
    out = []
    out.append(emit_group(groups["RAW10"], "RAW10", overrides_10))
    out.append(emit_group(groups["RAW12"], "RAW12", overrides_12))
    out.append(emit_group(groups["RAW14"], "RAW14", overrides_14))
    text = "\n".join(out)

    if args.output:
        Path(args.output).write_text(text)
    else:
        sys.stdout.write(text)

if __name__ == "__main__":
    main()
