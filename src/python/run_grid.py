#!/usr/bin/env python3
"""
Grid-search runner for PayloadFourDrones
• rebuilds (make) inside ./build
• executes every parameter combination
• collects distance → grid_results.csv
"""
import csv, itertools, os, subprocess, sys
from collections import OrderedDict as OD
import pandas as pd, pathlib, webbrowser, html

# ── 1. parameter “vectors” ───────────────────────────────────────
P = OD([
    # additional options
    ("printAllStates",     [0]),                # 0=false 1=true
    ("useSST",             [0,1]),                # 0=RRT 1=SST    
    
    # controls & torques
    ("maxTorquePitchRoll", [0.01, 0.1]),
    ("maxTorqueYaw",       [0.005]),
    ("minThrust",          [0]),
    ("maxThrust",          [30,45,60]),

    # drone limits
    ("maxDroneAngle",      [70]),
    ("maxDroneVel",        [20]),

    # payload limits
    ("maxAnglePayload",    [70]),
    ("maxPayloadVel",      [10]),
    ("maxPayloadAngVel",   [0.1, 1, 5]),

    # cable limits
    ("maxTheta",           [40,60]),
    ("maxThetaVel",        [20]),

    # sampling std-devs
    ("thrustStd",          [1,5]),
    ("torquePitchRollStd", [0.1]),
    ("torqueYawStd",       [0.0001]),

    # misc
    ("sameControls",       [0,1]),                # 1=true 0=false
    ("solveTime",          [10]),
])

# ── 2. build the project (if needed) ─────────────────────────────
root_dir  = os.path.abspath(os.path.dirname(__file__))
build_dir = os.path.join(root_dir, "../../build")
exe_path  = os.path.join(build_dir, "PayloadFourDrones")

print("📦  Building in", build_dir)
subprocess.run(["make", "-j", str(os.cpu_count())],
               cwd=build_dir, check=True)

# ── 3. prepare CSV output ────────────────────────────────────────
csv_file = os.path.join(root_dir, "grid_results.csv")
with open(csv_file, "w", newline="") as f:
    writer = csv.writer(f)
    header = list(P.keys()) + ["distance"]
    writer.writerow(header)

    # ── 4. iterate over Cartesian product ────────────────────────
    for values in itertools.product(*P.values()):
        param_set = dict(zip(P.keys(), values))
        cmd = [exe_path] + [f"--{k}={v}" for k, v in param_set.items()]

        print("▶", " ".join(cmd))
        completed = subprocess.run(cmd, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True)

        # look for the line we added in C++
        dist = "NaN"
        for line in completed.stdout.splitlines():
            if line.startswith("RESULT distance="):
                dist = line.split("=", 1)[1].strip()
                break

        writer.writerow(list(values) + [dist])

print("\n✅ finished – results in", csv_file)

# ── 5. display results in a browser tab ──────────────────────────
import pandas as pd, pathlib, webbrowser, html

df = pd.read_csv(csv_file)
html_path = pathlib.Path(csv_file).with_suffix(".html")

style = """
<style>
body   {font-family:sans-serif; margin:1rem;}
table  {border-collapse:collapse; width:100%; font-size:0.85rem;}
th,td  {border:1px solid #ccc; padding:2px 4px; text-align:right;}
th     {background:#eee; font-weight:600;}
tr:nth-child(even){background:#f9f9f9;}
</style>
"""

# build the table with “changed cells” in bold --------------------
header_cells = "".join(f"<th>{html.escape(c)}</th>" for c in df.columns)
rows_html    = []
prev = None
for _, r in df.iterrows():
    cells = []
    for c in df.columns:
        val = r[c]
        text = html.escape(str(val))
        # bold if this cell differs from the previous row
        if prev is not None and prev[c] != val:
            text = f"<b>{text}</b>"
        cells.append(f"<td>{text}</td>")
    rows_html.append("<tr>" + "".join(cells) + "</tr>")
    prev = r

table_html = f"<table><thead><tr>{header_cells}</tr></thead>" \
             f"<tbody>{''.join(rows_html)}</tbody></table>"

with html_path.open("w", encoding="utf8") as f:
    f.write(f"<html><head><title>PayloadFourDrones – grid</title>{style}</head>"
            f"<body><h2>PayloadFourDrones – grid-search results</h2>{table_html}</body></html>")

# open in Chrome (fallback to default browser)
url = html_path.as_uri()
try:
    webbrowser.get('google-chrome').open_new_tab(url)
except webbrowser.Error:
    webbrowser.open_new_tab(url)

print(f"🔎  Results opened: {url}")


