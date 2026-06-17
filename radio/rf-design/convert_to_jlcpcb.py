#!/usr/bin/env python3
"""Convert KiCad BOM and CPL files to JLCPCB format."""

import argparse
import csv
from pathlib import Path


def convert_bom(input_path: str, output_path: str):
    """Convert KiCad BOM (semicolon-delimited) to JLCPCB BOM CSV."""
    rows = []
    with open(input_path, "r") as f:
        reader = csv.reader(f, delimiter=";")
        header = next(reader)
        for row in reader:
            if not row or not row[0].strip():
                continue
            # KiCad columns: Id, Designator, Footprint, Quantity, Designation, Supplier and ref
            designator = row[1].strip().strip('"')
            footprint = row[2].strip().strip('"')
            comment = row[4].strip().strip('"')  # "Designation" = value/comment
            rows.append([comment, designator, footprint, ""])

    with open(output_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Comment", "Designator", "Footprint", "JLCPCB Part #"])
        writer.writerows(rows)

    print(f"BOM: {len(rows)} entries -> {output_path}")


def convert_cpl(input_path: str, output_path: str):
    """Convert KiCad position CSV to JLCPCB CPL CSV."""
    rows = []
    with open(input_path, "r") as f:
        reader = csv.reader(f)
        header = next(reader)
        for row in reader:
            if not row or not row[0].strip():
                continue
            ref = row[0].strip().strip('"')
            pos_x = row[3].strip()
            pos_y = row[4].strip()
            rotation = row[5].strip()
            side = row[6].strip().strip('"')

            # Format coordinates with mm suffix
            mid_x = f"{float(pos_x)}mm"
            mid_y = f"{float(pos_y)}mm"

            # Normalize layer name
            layer = "Top" if side.lower() == "top" else "Bottom"

            # Normalize rotation to 0-360
            rot = float(rotation)
            rot = rot % 360

            rows.append([ref, mid_x, mid_y, layer, rot])

    with open(output_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
        writer.writerows(rows)

    print(f"CPL: {len(rows)} entries -> {output_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert KiCad BOM/CPL to JLCPCB format")
    parser.add_argument("bom", help="KiCad BOM CSV file")
    parser.add_argument("cpl", help="KiCad position (CPL) CSV file")
    args = parser.parse_args()

    bom_in = Path(args.bom)
    cpl_in = Path(args.cpl)
    bom_out = bom_in.with_stem(bom_in.stem + "_jlc")
    cpl_out = cpl_in.with_stem(cpl_in.stem + "_jlc")

    convert_bom(str(bom_in), str(bom_out))
    convert_cpl(str(cpl_in), str(cpl_out))
