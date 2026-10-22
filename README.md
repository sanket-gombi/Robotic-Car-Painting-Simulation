# Robotic Car Painting Cell Simulation
## Project Title

**Automated Robotic Car-Body Painting Cell — RoboDK Simulation**

A complete RoboDK simulation of an industrial 6-axis robot performing multi-surface spray painting on a full-sized car body. The project covers station setup, cell layout design, reachability validation, trajectory programming, singularity avoidance, and collision checking.

---

## Software & Robot

| Item | Detail |
|------|--------|
| Simulation Software | RoboDK (installed locally) |
| Robot Model | ABB IRB 6700-150/3.20 LeanID |
| Robot Reach | 3,200 mm (nominal) |
| Robot Payload | 150 kg |
| Robot Axes | 6 rotary (J1–J6) |
| End-Effector | Custom `Paint_Gun` tool |
| TCP Offset | Z = +325.06 mm from robot flange |
| Programming Language | Python 3 (RoboDK API – `robodk.robolink`, `robodk.robomath`) |

---

## Vehicle Model

| Item | Detail |
|------|--------|
| Model Name | Car_Body (generic sedan body) |
| Format | STL mesh |
| Source | Custom-generated mesh placed at station origin |
| License | For academic/simulation use only |
| Overall Length (X) | 4,800 mm (±2,400 mm from origin) |
| Overall Width (Y) | 1,392 mm (±696 mm from origin) |
| Overall Height (Z) | 1,480 mm (0 – 1,480 mm) |
| Reference Frame | `Car_Reference_Frame` at world origin (0, 0, 0) |

---

## TCP Details

The `Paint_Gun` tool is attached to the robot's Tool0 flange:

| Parameter | Value |
|-----------|-------|
| TCP X | 0.0 mm |
| TCP Y | 0.0 mm |
| TCP Z | +325.06 mm |
| Orientation (Rx, Ry, Rz) | 0°, 0°, 0° |
| Spray direction convention | Tool +Z points **toward the surface** (into the target normal) |
| Working standoff distance | 200 mm (maintained throughout painting passes) |

---

## Robot Base Position

The robot is mounted on a 300 mm shop-floor pedestal riser beside the vehicle:

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Base X | 0 mm | Centered along 4,800 mm vehicle length |
| Base Y | −1,800 mm | 1,104 mm lateral clearance from car side panel |
| Base Z | +300 mm | Pedestal riser — eliminates elbow singularity on roof reach |
| Base Rz | +90° | Faces directly toward vehicle centerline |

---

## Opening the Station

1. **Open RoboDK**
2. **File → Open** → select `Simulation_Files/Car_Painting_Cell.rdk`
3. The station opens with robot, vehicle, paint gun, and all trajectory targets loaded.

### Running the Painting Program

**Via RoboDK GUI:**
- Double-click `Car_Painting_Demo` in the station tree → **Run**

**Via Python API:**
```bash
cd /home/sanket/Yuktii_Robotics_Assessment
python3 Scripts/create_painting_program.py
```

### Reproducing from Scratch (All Phases)

```bash
python3 Scripts/create_initial_station.py   # Phase 1-2: Station + Robot
python3 Scripts/import_car_model.py          # Phase 3: Vehicle model
python3 Scripts/add_painting_tool.py         # Phase 4: Paint gun TCP
python3 Scripts/validate_cell_layout.py      # Phase 5: Layout & reachability
python3 Scripts/create_painting_program.py   # Phase 6: Painting program
python3 Scripts/validate_collision_simulation.py  # Phase 7: Collision check
```

---

## Surfaces Painted

| Surface | Pass Type | Standoff | MoveL Passes |
|---------|-----------|----------|--------------|
| Roof (3 lanes) | Longitudinal raster | 200 mm | 3 × linear strokes |
| Front Hood | Transverse raster | 200 mm | 1 × linear stroke |
| Rear Trunk | Transverse raster | 200 mm | 1 × linear stroke |
| Left Side (cant rail, far side) | Longitudinal raster | 200 mm | 1 × linear stroke |
| Right Side – Front Fender (near side) | Diagonal fender-to-sill | 200 mm | 1 × linear stroke |
| Right Side – Rear Quarter (near side) | Diagonal quarter-to-sill | 200 mm | 1 × linear stroke |

---

## Validation Results

### Kinematic Feasibility
- **100%** of all 41 program instructions are kinematically feasible (verified via `prog.Update(check_collisions=0)`)
- Estimated cycle time: **~29 seconds**
- Total path travel distance: **~24,589 mm (24.6 m)**

### Singularity Avoidance
| Joint | Safe Margin | Threshold |
|-------|-------------|-----------|
| J5 (Wrist) | ≥ 60° at all painting poses | > 5° |
| J3 (Elbow) | ≥ 5.9° at all painting poses | > 3° |
- Paint_Home target: J5 = +80°, J3 = +20° (well clear of all singularities)

### Collision Map (Phase 7 Validation)
| Pair | Status | Reason |
|------|--------|--------|
| Robot links ↔ Car body | **ACTIVE** | Checked at every trajectory pose |
| Robot base ↔ Car body | **ACTIVE** | Static clearance verified |
| Paint gun ↔ Car body | **SUPPRESSED** | Known false-positive: gun mesh BBox clips surface; 200 mm standoff enforced by IK |
| Robot ↔ Paint gun | **SUPPRESSED** | Tool-on-flange self-contact by design |

### Pose-by-Pose Result
- All **35 trajectory target poses** checked via teleport + `rdk.Collisions()` call
- **0 robot-link collisions** detected at any target pose

---

## Known Limitations

1. **Front and rear bumper faces** are not fully reachable from a single fixed robot base position. The front bumper at X = +2,400 mm is at the edge of reach; the extreme corners (±2,400 mm, ±696 mm) exceed the 3,200 mm reach envelope. **Solution:** 7th-axis linear track or dual-robot layout (both validated kinematically in `validate_cell_layout.py`).

2. **Far (left) side full coverage** — only the upper cant rail at Y = +550 mm is reachable. Lower door panels on the far side (Y = +696 mm) are beyond the robot's reach from Y = −1,800 mm base. **Solution:** Symmetrical dual-robot layout with a mirrored robot at Y = +1,800 mm.

3. **MoveJ collision during path interpolation** — `prog.Update(check_collisions=1)` reports collisions during large MoveJ repositioning moves (joint-space paths sweep through unpredictable regions). Transit waypoints `Transit_Safe` and `Transit_Right_Mid` were added to mitigate this. Pose-by-pose validation at all target poses shows 0 robot-vs-car collisions.

4. **No actual spray simulation** — RoboDK free/standard edition does not simulate paint deposition or coverage. A CAM-based or specialized coating simulation tool would be needed for coverage analysis.

5. **Simulation only** — No physical robot, no actual vehicle, no real paint. All results are simulation-only.

---

## File Structure

```
Robotics_Assessment/
├── README.md                         ← This file
├── Technical_Report.pdf              ← 4-page professional report
├── Simulation_Files/
│   ├── Car_Painting_Cell.rdk         ← Main RoboDK station file
│   └── Car_Painting_Cell_backup.rdk  ← Backup station
├── Models/
│   └── Vehicle/
│       └── Car_Body.stl              ← Vehicle mesh model
├── Scripts/
│   ├── create_initial_station.py     ← Phase 1-2
│   ├── import_car_model.py           ← Phase 3
│   ├── add_painting_tool.py          ← Phase 4
│   ├── validate_cell_layout.py       ← Phase 5
│   ├── create_painting_program.py    ← Phase 6
│   ├── validate_collision_simulation.py ← Phase 7
│   └── verify_setup.py               ← Utility verification
├── Screenshots/
│   ├── Car_Painting_Cell_Layout_*.png   ← Cell layout views
│   ├── Car_Painting_Demo_*.png          ← Program demonstration
│   └── Validated_*.png                  ← Collision validation shots
└── Demo_Video/
    └── recording_checklist.md           ← Video recording guide
```

---

## Contact

**Sanket Gombi**  
KLE Technological University, Hubballi  
Assessment submission for Yuktii AI Labs – Robotic Simulation Engineer Internship
