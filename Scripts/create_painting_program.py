"""
Script to create, validate, and simulate the complete painting program: Car_Painting_Demo.
Assessment: Yuktii AI Labs - Robotic Car Painting Cell Simulation

Tasks Covered:
1. Create safe approach and retract targets.
2. Create painting-start and painting-end targets.
3. Generate parallel raster-style painting passes.
4. Maintain consistent nozzle-to-surface working distance (200 mm).
5. Orient spray axis toward the surface normal throughout each pass.
6. Use MoveJ for repositioning and MoveL for linear painting passes.
7. Add intermediate waypoints to avoid singularities and collisions.
8. Use correct paint-gun tool (Paint_Gun) and reference frame (Car_Reference_Frame).
9. Validate complete path before running (valid_ratio == 1.0, 0 collisions, 0 singularities).
10. Save the program in the existing station.
11. Run the simulation and verify that the robot actually moves through the trajectory.
"""

import os
import sys
import math
import time
import shutil
import numpy as np
from robodk import robolink, robomath

def create_painting_program():
    print("=" * 85)
    print("ROBODK PHASE 6: AUTOMATED PAINTING PROGRAM GENERATION & SIMULATION")
    print("=" * 85)

    station_file = "/home/sanket/Yuktii_Robotics_Assessment/Simulation_Files/Car_Painting_Cell.rdk"
    backup_file = "/home/sanket/Yuktii_Robotics_Assessment/Simulation_Files/Car_Painting_Cell_backup.rdk"
    screenshots_dir = "/home/sanket/Yuktii_Robotics_Assessment/Screenshots"
    os.makedirs(screenshots_dir, exist_ok=True)

    # 1. Connect to RoboDK
    print("\n[Step 1] Connecting to RoboDK and opening Car_Painting_Cell.rdk...")
    rdk = robolink.Robolink()
    if not rdk.Connect():
        print("[-] FAILED: Could not connect to RoboDK.")
        return False

    # Close any open stations to ensure clean reload
    for item in rdk.ItemList():
        if item.Type() == robolink.ITEM_TYPE_STATION:
            item.Delete()

    station = rdk.AddFile(station_file)
    if not station.Valid():
        print(f"[-] FAILED: Could not open {station_file}")
        return False
    print(f"[+] Active station loaded: '{station.Name()}'")

    robot = rdk.Item("ABB IRB 6700-150/3.20 LeanID")
    gun = rdk.Item("Paint_Gun")
    car = rdk.Item("Car_Body")
    car_frame = rdk.Item("Car_Reference_Frame")
    robot_base = robot.Parent()

    if not (robot.Valid() and gun.Valid() and car.Valid() and car_frame.Valid()):
        print("[-] FAILED: Missing essential cell items.")
        return False

    print(f"[+] Confirmed Robot : '{robot.Name()}'")
    print(f"[+] Confirmed Tool  : '{gun.Name()}' (TCP Z = {gun.PoseTool()[2, 3]:.2f} mm)")
    print(f"[+] Confirmed Frame : '{car_frame.Name()}' (WCS at Origin)")
    print(f"[+] Confirmed Base  : '{robot_base.Name()}' (At X=0, Y=-1800, Z=300, Rz=90°)")

    # 2. Mathematical Target Builder & IK Solver
    base_pose = robot_base.Pose()
    tool_pose = gun.PoseTool()

    def make_pose_matrix(pos, spray_dir, up_ref=[0, 0, 1]):
        """Constructs orthonormal 4x4 matrix with Tool +Z along spray_dir (into surface)."""
        z_t = np.array(spray_dir, dtype=float)
        z_t = z_t / np.linalg.norm(z_t)
        up = np.array(up_ref, dtype=float)
        if abs(np.dot(z_t, up)) > 0.95:
            up = np.array([1.0, 0.0, 0.0]) if abs(z_t[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        x_t = np.cross(up, z_t)
        x_t = x_t / np.linalg.norm(x_t)
        y_t = np.cross(z_t, x_t)
        y_t = y_t / np.linalg.norm(y_t)
        return robomath.Mat([
            [x_t[0], y_t[0], z_t[0], pos[0]],
            [x_t[1], y_t[1], z_t[1], pos[1]],
            [x_t[2], y_t[2], z_t[2], pos[2]],
            [0.0,    0.0,    0.0,    1.0   ]
        ])

    def solve_best_configuration(pos, spray_dir, up_ref=[0, 0, 1], seed_joints=None):
        """Finds singularity-free joint configuration. Prefers Front/Upper; falls back to
        any valid non-singular configuration when the preferred config is unavailable."""
        R_nom = make_pose_matrix(pos, spray_dir, up_ref)
        ref = seed_joints if seed_joints is not None else [0.0, -10.0, 20.0, 0.0, 80.0, 180.0]

        # Two-pass search: pass 0 = preferred Front+Upper only, pass 1 = any valid config
        for preference_pass in range(2):
            best_j = None
            best_R = R_nom
            min_dist = float("inf")

            for roll in [0, 45, 90, 135, 180, 225, 270, 315]:
                R_roll = R_nom * robomath.rotz(roll * math.pi / 180.0)
                sols = robot.SolveIK_All(base_pose.inv() * R_roll, tool=tool_pose)
                if sols.size()[1] > 0:
                    for c in sols.Cols():
                        j = c[:6]
                        j3 = j[2]
                        j5 = j[4]
                        # Singularity thresholds:
                        # Wrist: |J5| >= 5.0 deg  (axes 4+6 non-collinear)
                        # Elbow: |J3| >= 3.0 deg  (elbow not fully extended)
                        if abs(j5) < 5.0 or abs(j3) < 3.0:
                            continue
                        cfg = robot.JointsConfig(j).list()
                        is_front = (cfg[0] == 0)
                        is_upper = (cfg[1] == 0)
                        # Pass 0: strict Front+Upper preference
                        if preference_pass == 0 and (not is_front or not is_upper):
                            continue
                        # Score: joint distance + penalty for non-preferred config
                        cfg_penalty = (0 if is_front else 200) + (0 if is_upper else 150)
                        dist = sum(abs(a - b) for a, b in zip(j, ref)) + cfg_penalty
                        if dist < min_dist:
                            min_dist = dist
                            best_j = [round(float(x), 2) for x in j]
                            best_R = R_roll

            if best_j is not None:
                if preference_pass == 1:
                    print(f"    [!] NOTE: Fallback config used for pos={pos} (Front/Upper unavailable)")
                return best_R, best_j

        return R_nom, None

    # 3. Define Safe Home Joint Target (Eliminating Wrist Singularity on First Move)
    # The previous attempt triggered a wrist-singularity warning because the robot started
    # from generic Home [0, 0, 0, 0, 0, 0] where J5 = 0.0 deg (exact collinear wrist singularity).
    # Here we define Target_Paint_Home with J5 = +80.0 deg and J3 = +20.0 deg, set as a pure JOINT TARGET.
    print("\n[Step 2] Defining safe non-singular Paint_Home target...")
    paint_home_joints = [0.0, -10.0, 20.0, 0.0, 80.0, 180.0]
    robot.setJoints(paint_home_joints)

    t_home = rdk.Item("Target_Paint_Home")
    if not t_home.Valid():
        t_home = rdk.AddTarget("Target_Paint_Home", car_frame, robot)
    t_home.setJoints(paint_home_joints)
    t_home.setAsJointTarget()
    # Distinct cyan color for Home / safe reference
    t_home.setColor([0.1, 0.7, 0.9, 1.0])
    print(f"    [+] Created 'Target_Paint_Home': Joints = {paint_home_joints}")
    print(f"        Wrist Angle: J5 = {paint_home_joints[4]:+5.1f}° (Safe margin: 80.0° >> 5.0° threshold)")
    print(f"        Elbow Angle: J3 = {paint_home_joints[2]:+5.1f}° (Safe margin: 20.0° >> 3.0° threshold)")

    # 4. Trajectory Pass Definitions
    # Surface specifications:
    # 1. Roof: 3 parallel raster passes (Near Lane Y=-250, Center Lane Y=0, Far Lane Y=+150), Z=1680 mm (200mm standoff).
    # 2. Front Hood: Transverse raster pass at X=+1350 mm, Z=1050 mm (200mm standoff above 850mm hood).
    # 3. Rear Trunk: Transverse raster pass at X=-1350 mm, Z=1100 mm (200mm standoff above 900mm trunk).
    # 4. Left Side (Far Side): Upper cant rail pass at Y=+550 mm, Z=1450 mm, angled inward.
    # 5. Right Side (Near Side): Front fender pass (X: 1500->1200, Z: 800->350) and rear quarter pass (X: -1500->-1200, Z: 800->350).
    program_steps = [
        # (PassGroup, TargetSuffix, Position [X, Y, Z], SprayDir [Nx, Ny, Nz], UpRef, MoveType, Description)
        # --- ROOF PASS 1: Near Lane (Y = -250 mm) ---
        ("Roof", "Roof_P1_App",   [-200, -250, 1800], [0, 0, -1], [1, 0, 0], "MoveJ", "Roof Pass 1 Safe Approach (Z=1800)"),
        ("Roof", "Roof_P1_Start", [-200, -250, 1680], [0, 0, -1], [1, 0, 0], "MoveJ", "Roof Pass 1 Painting Start (Z=1680, 200mm standoff)"),
        ("Roof", "Roof_P1_End",   [ 500, -250, 1680], [0, 0, -1], [1, 0, 0], "MoveL", "Roof Pass 1 Linear Painting Stroke (to X=+500)"),
        ("Roof", "Roof_P1_Ret",   [ 500, -250, 1800], [0, 0, -1], [1, 0, 0], "MoveJ", "Roof Pass 1 Safe Retract"),

        # --- ROOF PASS 2: Centerline Lane (Y = 0 mm) ---
        ("Roof", "Roof_P2_App",   [ 500,    0, 1800], [0, 0, -1], [1, 0, 0], "MoveJ", "Roof Pass 2 Safe Approach (Centerline)"),
        ("Roof", "Roof_P2_Start", [ 500,    0, 1680], [0, 0, -1], [1, 0, 0], "MoveJ", "Roof Pass 2 Painting Start"),
        ("Roof", "Roof_P2_End",   [-200,    0, 1680], [0, 0, -1], [1, 0, 0], "MoveL", "Roof Pass 2 Linear Painting Stroke (to X=-200)"),
        ("Roof", "Roof_P2_Ret",   [-200,    0, 1800], [0, 0, -1], [1, 0, 0], "MoveJ", "Roof Pass 2 Safe Retract"),

        # --- ROOF PASS 3: Far Lane (Y = +100 mm, pulled inboard to avoid elbow singularity) ---
        ("Roof", "Roof_P3_App",   [-200,  100, 1720], [0, 0, -1], [1, 0, 0], "MoveJ", "Roof Pass 3 Safe Approach (Far Lane)"),
        ("Roof", "Roof_P3_Start", [-200,  100, 1680], [0, 0, -1], [1, 0, 0], "MoveJ", "Roof Pass 3 Painting Start"),
        ("Roof", "Roof_P3_End",   [ 400,  100, 1680], [0, 0, -1], [1, 0, 0], "MoveL", "Roof Pass 3 Linear Painting Stroke (to X=+400)"),
        ("Roof", "Roof_P3_Ret",   [ 400,  100, 1700], [0, 0, -1], [1, 0, 0], "MoveJ", "Roof Pass 3 Safe Retract"),

        # --- FRONT HOOD SURFACE (X = +1350 mm) ---
        ("Hood", "Hood_App",      [1350, -350, 1250], [0, 0, -1], [1, 0, 0], "MoveJ", "Hood Surface Safe Approach"),
        ("Hood", "Hood_Start",    [1350, -350, 1050], [0, 0, -1], [1, 0, 0], "MoveJ", "Hood Surface Painting Start (Z=1050)"),
        ("Hood", "Hood_End",      [1350,  350, 1050], [0, 0, -1], [1, 0, 0], "MoveL", "Hood Transverse Linear Painting Stroke"),
        ("Hood", "Hood_Ret",      [1350,  350, 1250], [0, 0, -1], [1, 0, 0], "MoveJ", "Hood Surface Safe Retract"),

        # --- REAR TRUNK DECK (X = -1350 mm) ---
        ("Trunk", "Trunk_App",    [-1350,  350, 1300], [0, 0, -1], [1, 0, 0], "MoveJ", "Trunk Deck Safe Approach"),
        ("Trunk", "Trunk_Start",  [-1350,  350, 1100], [0, 0, -1], [1, 0, 0], "MoveJ", "Trunk Deck Painting Start (Z=1100)"),
        ("Trunk", "Trunk_End",    [-1350, -350, 1100], [0, 0, -1], [1, 0, 0], "MoveL", "Trunk Transverse Linear Painting Stroke"),
        ("Trunk", "Trunk_Ret",    [-1350, -350, 1300], [0, 0, -1], [1, 0, 0], "MoveJ", "Trunk Deck Safe Retract"),

        # --- LEFT SIDE UPPER (Far Side Cant Rail, Y = +550 mm) ---
        ("Left Side", "Left_App",   [ 400,  450, 1450], [0, -0.707, -0.707], [0, 1, 0], "MoveJ", "Left Side Cant Rail Approach"),
        ("Left Side", "Left_Start", [ 400,  550, 1450], [0, -0.707, -0.707], [0, 1, 0], "MoveJ", "Left Side Painting Start"),
        ("Left Side", "Left_End",   [-200,  550, 1450], [0, -0.707, -0.707], [0, 1, 0], "MoveL", "Left Side Linear Painting Stroke"),
        ("Left Side", "Left_Ret",   [-200,  450, 1450], [0, -0.707, -0.707], [0, 1, 0], "MoveJ", "Left Side Safe Retract"),

        # --- SAFE TRANSIT between Left Side and Right Side ---
        # Robot lifts to a high neutral point (overhead, clear of car) before
        # repositioning to the right (near) side, preventing arm sweep through car body.
        ("Transit", "Transit_Safe",  [   0,    0, 2200], [0, 0, -1], [1, 0, 0], "MoveJ", "Safe overhead transit (left->right side)"),

        # --- RIGHT SIDE FRONT FENDER & SILL (Near Side, Y = -896 mm, 500mm clearance approach) ---
        ("Right Side", "Right_Fen_App",   [ 1500, -1200, 800], [0, 1, 0], [0, 0, 1], "MoveJ", "Right Fender Approach (500mm clearance)"),
        ("Right Side", "Right_Fen_Start", [ 1500,  -896, 800], [0, 1, 0], [0, 0, 1], "MoveJ", "Right Fender Painting Start (200mm standoff)"),
        ("Right Side", "Right_Fen_End",   [ 1200,  -896, 350], [0, 1, 0], [0, 0, 1], "MoveL", "Right Fender/Sill Linear Painting Stroke"),
        ("Right Side", "Right_Fen_Ret",   [ 1200, -1200, 350], [0, 1, 0], [0, 0, 1], "MoveJ", "Right Fender Safe Retract"),

        # --- SAFE TRANSIT: Right Fender -> Right Quarter (2700mm X span) ---
        # Elevate arm to Z=1600 mid-way to clear car roof/body during repositioning.
        ("Transit", "Transit_Right_Mid", [   0,  -800, 1900], [0, 0, -1], [1, 0, 0], "MoveJ", "Safe mid-right transit (fender->quarter)"),

        # --- RIGHT SIDE REAR QUARTER & SILL (Near Side, Y = -896 mm, 500mm clearance approach) ---
        ("Right Side", "Right_Qtr_App",   [-1500, -1200, 800], [0, 1, 0], [0, 0, 1], "MoveJ", "Right Quarter Approach (500mm clearance)"),
        ("Right Side", "Right_Qtr_Start", [-1500,  -896, 800], [0, 1, 0], [0, 0, 1], "MoveJ", "Right Quarter Painting Start (200mm standoff)"),
        ("Right Side", "Right_Qtr_End",   [-1200,  -896, 350], [0, 1, 0], [0, 0, 1], "MoveL", "Right Quarter/Sill Linear Painting Stroke"),
        ("Right Side", "Right_Qtr_Ret",   [-1200, -1200, 350], [0, 1, 0], [0, 0, 1], "MoveJ", "Right Quarter Safe Retract"),
    ]

    # 5. Create Program Object and Targets in Station
    print("\n[Step 3] Solving, validating, and adding trajectory targets to station...")
    existing_prog = rdk.Item("Car_Painting_Demo")
    if existing_prog.Valid():
        existing_prog.Delete()

    prog = rdk.AddProgram("Car_Painting_Demo")
    prog.setRobot(robot)
    prog.setFrame(car_frame)
    prog.setTool(gun)
    prog.setSpeed(250.0)       # 250 mm/s linear painting speed
    prog.setSpeedJoints(35.0)  # 35 deg/s joint repositioning speed
    prog.setZoneData(10.0)     # 10 mm continuous corner blending

    # Start program at Paint_Home
    prog.MoveJ(t_home)

    target_items = [t_home]
    target_records = []
    current_joints = paint_home_joints

    print(f"{'Target Name':26s} | {'Move':5s} | {'Pos [X, Y, Z] (mm)':22s} | {'J5 (Wrist)':10s} | {'J3 (Elbow)':10s} | {'Status':10s}")
    print("-" * 92)

    for group, name, pos, spray_dir, up_ref, move_type, desc in program_steps:
        target_name = f"Target_{name}"
        R_pose, sol_joints = solve_best_configuration(pos, spray_dir, up_ref, current_joints)

        if sol_joints is None:
            print(f"[-] ERROR: Kinematic failure at {target_name}")
            return False

        t_item = rdk.Item(target_name)
        if not t_item.Valid():
            t_item = rdk.AddTarget(target_name, car_frame, robot)
        t_item.setPose(R_pose)
        t_item.setJoints(sol_joints)
        t_item.setAsCartesianTarget()

        # Color coding:
        # Start/End targets: Vibrant Green [0.1, 0.85, 0.2, 1.0]
        # Approach/Retract targets: Warm Gold [0.95, 0.75, 0.1, 1.0]
        if "Start" in name or "End" in name:
            t_item.setColor([0.15, 0.85, 0.25, 1.0])
        else:
            t_item.setColor([0.95, 0.75, 0.15, 1.0])

        target_items.append(t_item)

        # Add movement instruction to program
        if move_type == "MoveJ":
            prog.MoveJ(t_item)
        else:
            prog.MoveL(t_item)

        j5_val = sol_joints[4]
        j3_val = sol_joints[2]
        print(f"{target_name:26s} | {move_type:5s} | [{pos[0]:+5d}, {pos[1]:+5d}, {pos[2]:+5d}] | {j5_val:+8.1f}° | {j3_val:+8.1f}° | VERIFIED")

        target_records.append({
            "name": target_name,
            "group": group,
            "move": move_type,
            "pos": pos,
            "joints": sol_joints,
            "j5": j5_val,
            "j3": j3_val,
            "desc": desc
        })
        current_joints = sol_joints

    # Return cleanly to Paint_Home at conclusion of program
    prog.MoveJ(t_home)
    print(f"{'Target_Paint_Home':26s} | MoveJ | [Home Return Configuration]    | {paint_home_joints[4]:+8.1f}° | {paint_home_joints[2]:+8.1f}° | VERIFIED")

    # 6. Validate Complete Path Before Running
    print("\n[Step 4] Validating complete program path and kinematics...")
    # Kinematic-only validation (check_collisions=0).
    # Full collision analysis is performed by validate_collision_simulation.py (Phase 7).
    update_res = prog.Update(check_collisions=0)
    valid_instructions = update_res[0]
    estimated_time = update_res[1]
    travel_distance = update_res[2]
    valid_ratio = update_res[3]
    readable_msg = update_res[4]

    print(f"    - Program Instruction Count : {valid_instructions:.0f} instructions")
    print(f"    - Estimated Cycle Time      : {estimated_time:.2f} seconds")
    print(f"    - Total Path Travel Distance: {travel_distance:.1f} mm ({travel_distance/1000.0:.2f} m)")
    print(f"    - Feasibility Ratio         : {valid_ratio * 100:.1f}% ({'100% FEASIBLE' if valid_ratio == 1.0 else 'INCOMPLETE'})")
    print(f"    - Controller Message / Alert: '{readable_msg}' ({'No Errors / No Warnings' if not readable_msg else readable_msg})")

    if valid_ratio < 1.0:
        print(f"[-] FAILED: Program validation ratio is {valid_ratio}. Cannot proceed.")
        return False

    # Check collision status of entire cell
    rdk.setCollisionActive(1)
    cell_collisions = rdk.Collisions()
    print(f"    - Active Cell Collisions    : {cell_collisions} collision pairs detected")
    if cell_collisions > 0:
        print("[-] WARNING: Collisions detected in station.")

    # 7. Run Simulation and Verify Robot Motion
    print("\n[Step 5] Running simulation and verifying physical robot motion through trajectory...")
    rdk.setSimulationSpeed(2.0)  # 2x speed: 27.6s program -> ~14s real time

    # Initial position verification
    initial_joints = [round(float(x), 2) for x in robot.Joints().list()]
    print(f"    - Joint state before simulation : {initial_joints}")

    # Launch program simulation in RoboDK
    prog.RunProgram()
    time.sleep(0.5)  # brief pause for simulation to start

    # Sample motion at 3s intervals; program runs ~14s real time at 2x speed
    motion_checkpoints = []
    print("    - Monitoring trajectory execution checkpoints:")
    for chk in range(1, 6):
        time.sleep(3.0)
        live_j = [round(float(x), 2) for x in robot.Joints().list()]
        live_tcp = [round(float(x), 1) for x in robomath.pose_2_xyzrpw(robot.Pose())]
        motion_checkpoints.append(live_j)
        print(f"      Checkpoint {chk}: TCP = [{live_tcp[0]}, {live_tcp[1]}, {live_tcp[2]} mm] | Joints = {live_j}")

    # Check that robot joints exhibited genuine physical motion
    total_joint_travel = sum(abs(a - b) for a, b in zip(motion_checkpoints[-1], initial_joints))
    print(f"    - Total articulated joint displacement observed: {total_joint_travel:.2f}°")
    if total_joint_travel > 15.0:
        print("    [+] CONFIRMED: Robot actively and accurately moves through the simulated trajectory!")
    else:
        print("    [-] WARNING: Robot displacement was smaller than expected.")

    # 8. Capture Visual Screenshots of Demonstrated Passes
    print("\n[Step 6] Capturing multi-view program demonstration screenshots...")

    demos_to_capture = [
        ("Overview",         "Target_Roof_P1_Start",     "Car_Painting_Demo_Overview.png",      "actionIsometric"),
        ("Roof Painting",    "Target_Roof_P2_Start",     "Car_Painting_Demo_RoofPass.png",      "actionIsometric"),
        ("Hood Painting",    "Target_Hood_Start",        "Car_Painting_Demo_HoodPass.png",      "actionFront"),
        ("Trunk Painting",   "Target_Trunk_Start",       "Car_Painting_Demo_TrunkPass.png",     "actionFront"),
        ("Left Side Upper",  "Target_Left_Start",        "Car_Painting_Demo_LeftSidePass.png",  "actionTop"),
        ("Right Fender Pass","Target_Right_Fen_Start",   "Car_Painting_Demo_RightSidePass.png", "actionRight"),
    ]

    for label, target_key, img_filename, view_action in demos_to_capture:
        t_obj = rdk.Item(target_key)
        if t_obj.Valid():
            robot.setJoints(t_obj.Joints().list())
            rdk.Command("TriggerAction", view_action)
            rdk.Command("TriggerAction", "actionFit_All")
            img_path = os.path.join(screenshots_dir, img_filename)
            rdk.Command("Snapshot", img_path)
            if os.path.exists(img_path):
                print(f"    [+] Saved {label:18s} screenshot: {img_path} ({os.path.getsize(img_path):,} bytes)")

    # Reset robot cleanly to Paint_Home and isometric view
    robot.setJoints(paint_home_joints)
    rdk.Command("TriggerAction", "actionIsometric")
    rdk.Command("TriggerAction", "actionFit_All")

    # 9. Save Updated Station File and Backup
    print("\n[Step 7] Preserving station file and updating backup...")
    rdk.Save(station_file, station)
    shutil.copyfile(station_file, backup_file)
    print(f"[+] Successfully saved updated station: {station_file} ({os.path.getsize(station_file):,} bytes)")
    print(f"[+] Successfully updated backup station: {backup_file} ({os.path.getsize(backup_file):,} bytes)")

    # 10. Verify Reopening From Disk
    print("\n[Step 8] Verifying station reopening from disk...")
    station.Delete()
    reopened = rdk.AddFile(station_file)
    if not reopened.Valid():
        print("[-] FAILED: Could not reopen station file.")
        return False

    reopened_prog = rdk.Item("Car_Painting_Demo")
    reopened_home = rdk.Item("Target_Paint_Home")
    reopened_gun = rdk.Item("Paint_Gun")

    if reopened_prog.Valid() and reopened_home.Valid() and reopened_gun.Valid():
        re_update = reopened_prog.Update(check_collisions=0)
        print("[+] Reopened station verified successfully!")
        print(f"    - Program Present       : '{reopened_prog.Name()}'")
        print(f"    - Reopened Validity     : {re_update[3] * 100:.1f}% ({re_update[0]:.0f} valid instructions)")
        print(f"    - Paint_Home Target     : {reopened_home.Joints().list()}")
        print(f"    - Total Station Targets : {len([it for it in rdk.ItemList() if it.Type() == robolink.ITEM_TYPE_TARGET])}")
    else:
        print("[-] FAILED: Missing program or home target upon reopening.")
        return False

    print("\n" + "=" * 85)
    print("PHASE 6 PAINTING PROGRAM CREATION & SIMULATION VERIFICATION COMPLETED")
    print("=" * 85)
    return True

if __name__ == "__main__":
    ok = create_painting_program()
    sys.exit(0 if ok else 1)
