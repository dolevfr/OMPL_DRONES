#!/usr/bin/env python3
"""
Grid-search runner for PayloadFourDrones
• rebuilds (make) inside ./build
• executes every parameter combination
• collects distance → grid_results.csv
"""
import csv, itertools, os, subprocess, shutil, math
from collections import OrderedDict as OD
import pandas as pd, pathlib, webbrowser, html
from pathlib import Path

# Number of minutes to run the grid search
TARGET_MIN   = 1/6  # minutes

# ── 1. parameter “vectors” ───────────────────────────────────────
P = OD([
    # additional options
    ("printAllStates",     [0]),                # 0=false 1=true
    ("useSST",             [1]),                # 0=RRT 1=SST 
    
    # controls & torques
    ("maxTorquePitchRoll", [0.01]),
    ("maxTorqueYaw",       [0.005]),
    ("minThrust",          [0]),
    ("maxThrust",          [40]), 

    # drone limits
    ("maxDroneAngle",      [70]),
    ("maxDroneVel",        [20]),

    # payload limits
    ("maxAnglePayload",    [70]),
    ("maxPayloadVel",      [8]),
    ("maxPayloadAngVel",   [1]), 

    # cable limits
    ("maxTheta",           [60]), 
    ("maxThetaVel",        [20]),

    # sampling std-devs
    ("thrustStd",          [5]),
    ("torquePitchRollStd", [0.1]),
    ("torqueYawStd",       [0.0001]),

    # misc
    ("sameControls",       [1]),                # 1=true 0=false 
    ("solveTime",          [10]),
])

solve_time = P["solveTime"][0]                 # seconds (single value)

base_combo_count = math.prod(len(v) for k, v in P.items())
total_seconds    = TARGET_MIN * 60
repeats_needed   = max(1, math.ceil(total_seconds / (base_combo_count * solve_time)))

P["repeat"] = list(range(repeats_needed))
print(f"💡  Will perform {repeats_needed} identical repeats "
      f"of each base combination "
      f"→  total experiments = {base_combo_count * repeats_needed}")

# ── 2. build the project (if needed) ─────────────────────────────
script_dir = os.path.abspath(os.path.dirname(__file__))          # …/src/python
root_dir   = os.path.abspath(os.path.join(script_dir, "..", ".."))  # ← NEW
build_dir  = os.path.join(root_dir, "build")
exe_path   = os.path.join(build_dir, "PayloadFourDrones")
py_dir     = script_dir                                          # helper scripts live here

# ---------- clean up stale solution files ------------------------
for f in (
        os.path.join(build_dir, "solution_path.txt"),
        os.path.join(build_dir, "solution_path_se3.txt"),
        os.path.join(build_dir, "best_distance.txt"),
        os.path.join(root_dir,  "best_solution_path.txt"),
):
    try:
        os.remove(f)
    except FileNotFoundError:
        pass     # fine – nothing to delete


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
        cmd = [exe_path] + [f"--{k}={v}" for k, v in param_set.items() if k != "repeat"]

        print("▶", " ".join(cmd))
        completed = subprocess.run(cmd, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True, cwd=build_dir)

        # ------------- extract distance from program output -----------------
        dist = "NaN"
        for line in completed.stdout.splitlines():
            if line.startswith("RESULT distance="):
                dist = line.split("=", 1)[1].strip()
            # show any “Saved new BEST path…” message the C++ printed
            if line.startswith("🆕") or line.startswith("ℹ️"):
                print("   ", line)

        # convert to float (keep inf on parsing errors)
        try:
            dist_val = float(dist)
        except ValueError:
            dist_val = float('inf')

        # ------------- keep / report current best ---------------------------
        new_best = False
        if 'best_dist' not in globals():
            best_dist = float('inf')

        if dist_val < best_dist:
            new_best = True
            best_dist = dist_val
            shutil.copy2(os.path.join(build_dir, "solution_path.txt"),
                         os.path.join(root_dir,  "best_solution_path.txt"))

        # show concise status line
        tag = "(NEW BEST)" if new_best else ""
        print(f"   distance = {dist} {tag}")

        # ------------- write CSV immediately --------------------------------
        writer.writerow(list(values) + [dist])
        f.flush()



print("\n✅ finished – results in", csv_file)

# ── 5. display results in a browser tab ──────────────────────────
csv_file  = os.path.join(root_dir, "grid_results.csv")  # works unchanged
html_path = pathlib.Path(csv_file).with_suffix(".html") # works unchanged
df        = pd.read_csv(csv_file)


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

# ── 6.  post-processing: copy best path → build & launch GUI ──────
best_txt = os.path.join(root_dir,  "best_solution_path.txt")
if os.path.exists(best_txt):
    # a)  make it the “official” name inside build/
    dest = os.path.join(build_dir, "solution_path.txt")
    shutil.copy2(best_txt, dest)

    # b)  run extract_se3.py  (cwd = build so relative paths match)
    py_dir = os.path.join(root_dir, "src", "python")
    subprocess.run(
        ["python3",
         os.path.join(py_dir, "extract_se3.py"),
         "solution_path.txt",           # in build/
         "solution_path_se3.txt"],      # output in build/
        cwd=build_dir, check=True)

    # c)  launch the OMPL.app visualizer (still in build/)
    subprocess.run(
        ["python3",
         os.path.join(py_dir, "ompl_app_multiple.py")],
        cwd=build_dir, check=True)

    print("🎞️  Animation of BEST path launched.")
else:
    print("⚠️  No best_solution_path.txt found → nothing to visualise.")



