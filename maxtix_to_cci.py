#!/usr/bin/env python3
import argparse
import csv
import re
import sys
from pathlib import Path
from typing import Dict, List, Tuple

# Registers that should be printed as 16-bit (0xNNNN).
REGS_16BIT = {
    "HTRIMMING_END",
    "WRITE_VSIZE",
    "Y_OUT_SIZE",
    "MDSEL15",
    "MDSEL16",
    "MDSEL9",
    "MDSEL10",
    "SVR",
}

# If you need to force widths per-register beyond 8/16, extend this:
FORCED_WIDTH: Dict[str, int] = {  # name -> hex digits
    # e.g., "SOME_24BIT_REG": 6,
}

def sanitize_mode_token(tok: str) -> str:
    """Make the column header safe for inclusion in a C identifier tail."""
    # Keep letters/digits/underscore; replace others with underscore.
    t = re.sub(r"[^A-Za-z0-9_]", "_", tok.strip())
    # Avoid accidental double underscores.
    t = re.sub(r"__+", "_", t)
    return t

def hex_width_for(reg_name: str, value: int) -> int:
    """Choose hex digit width for a given register/value."""
    if reg_name in FORCED_WIDTH:
        return FORCED_WIDTH[reg_name]
    if reg_name in REGS_16BIT:
        return 4
    # Default to 2 for <=0xFF, 4 for <=0xFFFF, else minimal needed (rounded up to even).
    if value <= 0xFF:
        return 2
    if value <= 0xFFFF:
        return 4
    digits = len(f"{value:X}")
    if digits % 2 == 1:
        digits += 1
    return max(4, digits)

def parse_hex_or_int(cell: str) -> int:
    s = cell.strip()
    if not s:
        raise ValueError("Empty cell where a hex value was expected.")
    # int(..., 0) accepts 0x.., 0o.., 0b.., or decimal
    return int(s, 0)

def format_hex(reg_name: str, cell: str) -> str:
    v = parse_hex_or_int(cell)
    w = hex_width_for(reg_name, v)
    return f"0x{v:0{w}X}"

def load_matrix(tsv_path: Path) -> Tuple[List[str], Dict[str, Dict[str, str]]]:
    """
    Returns:
      columns: list of column headers (modes) in order
      rows: dict of reg_name -> dict{ column_header -> raw_cell_string }
    """
    with tsv_path.open("r", newline="") as f:
        reader = csv.reader(f, delimiter="\t")
        rows_list = [list(map(str.strip, r)) for r in reader if any(x.strip() for x in r)]

    if not rows_list:
        raise RuntimeError("Input file appears empty.")

    header = rows_list[0]
    if len(header) < 2:
        raise RuntimeError("First row must have at least one mode column.")

    # First cell is the row-label header (often blank); the rest are mode names
    columns = header[1:]

    rows: Dict[str, Dict[str, str]] = {}
    for r in rows_list[1:]:
        if len(r) < 2:
            continue
        reg_name = r[0].strip()
        if not reg_name:
            continue
        rows[reg_name] = {}
        for i, col in enumerate(columns):
            # Guard against jagged rows
            cell = r[i + 1] if i + 1 < len(r) else ""
            rows[reg_name][col] = cell.strip()

    return columns, rows

def make_reg_macro(reg_name: str, chip_prefix: str) -> str:
    return f"{chip_prefix}_REG_{reg_name}"

def emit_mode_array(
    chip_prefix: str,
    mode_token: str,
    prefix_replace: str,
    rows: Dict[str, Dict[str, str]],
) -> str:
    """
    Build a C array for one mode column.
    Array name: mode_{mode_token}_{prefix_replace}_regs
    """
    arr_name = f"mode_{sanitize_mode_token(mode_token)}_{prefix_replace}_regs"
    lines = [f"static const struct cci_reg_sequence {arr_name}[] = {{"]

    # Preserve the input row order as given by 'rows' dict insertion order (Py3.7+ keeps it)
    for reg_name, colmap in rows.items():
        if mode_token not in colmap:
            continue
        val_raw = colmap[mode_token]
        # Skip empty cells
        if val_raw == "":
            continue
        try:
            val_fmt = format_hex(reg_name, val_raw)
        except Exception as e:
            raise RuntimeError(f"Failed to parse value for {reg_name} in mode '{mode_token}': {val_raw!r}") from e

        macro = make_reg_macro(reg_name, chip_prefix)
        lines.append(f"    {{{macro},{val_fmt}}}, ")
    lines.append("};")
    return "\n".join(lines)

def main():
    ap = argparse.ArgumentParser(
        description="Convert a tab-separated register matrix to C cci_reg_sequence arrays."
    )
    ap.add_argument("input", help="Path to the tab-separated TXT file.")
    ap.add_argument(
        "--chip-prefix",
        default="IMX294",
        help="Chip macro prefix used for registers (default: IMX294). "
             "Results in macros like IMX294_REG_MDSEL1.",
    )
    ap.add_argument(
        "--mode-prefix",
        required=True,
        help="String to replace '17_9' in the example array name; used as the trailing chunk in the array name.",
    )
    ap.add_argument(
        "--only",
        nargs="+",
        help="Optional: one or more specific mode columns to emit (by header name). If omitted, all modes are emitted.",
    )
    ap.add_argument(
        "-o", "--output",
        help="Write output C code to this file. Defaults to stdout.",
    )

    args = ap.parse_args()
    in_path = Path(args.input)
    columns, rows = load_matrix(in_path)

    # Validate requested columns
    selected = columns if not args.only else args.only
    missing = [c for c in selected if c not in columns]
    if missing:
        available = ", ".join(columns)
        raise SystemExit(f"Unknown mode(s): {missing}. Available: {available}")

    chunks = []
    for col in selected:
        chunks.append(
            emit_mode_array(
                chip_prefix=args.chip_prefix,
                mode_token=col,
                prefix_replace=args.mode_prefix,
                rows=rows,
            )
        )

    out_text = "\n\n".join(chunks) + "\n"

    if args.output:
        Path(args.output).write_text(out_text)
    else:
        sys.stdout.write(out_text)

if __name__ == "__main__":
    main()
