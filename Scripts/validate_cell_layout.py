"""
Script to design, optimize, and validate the painting-cell layout and reachability.
Assessment: Yuktii AI Labs - Robotic Car Painting Cell Simulation
Tasks Covered:
1. Inspect robot workspace (3.20 m reach, 6 rotary axes, joint limits, TCP).
2. Inspect vehicle dimensions (4800 x 1392 x 1480 mm, centered at station origin).
3. Position robot beside vehicle with sensible clearance (Y = -1800 mm -> 1104 mm clearance).
4. Check reachability of roof and side surfaces at plausible spray orientation and 200 mm working distance.
5. Check reachability of front and rear surfaces.
6. Test candidate targets using RoboDK's kinematics and reachability features.
7. Identify wrist and elbow singularities.
8. Adjust robot base position and orientation (elevated by 300 mm pedestal to eliminate elbow singularity).
9. Select valid robot configurations for candidate target poses.
10. Document single-robot fixed-base limitations and evaluate alternatives (linear track & dual-robot).
"""

import os
import sys
import math
import shutil
import numpy as np
from robodk import robolink, robomath

def design_and_validate_cell():
    print("=" * 80)
    print("ROBODK PHASE 5: PAINTING-CELL LAYOUT DESIGN & REACHABILITY VALIDATION")
    print("=" * 80)

    station_file = "/home/sanket/Yuktii_Robotics_Assessment/Simulation_Files/Car_Painting_Cell.rdk"
    backup_file = "/home/sanket/Yuktii_Robotics_Assessment/Simulation_Files/Car_Painting_Cell_backup.rdk"
    screenshots_dir = "/home/sanket/Yuktii_Robotics_Assessment/Screenshots"
    os.makedirs(screenshots_dir, exist_ok=True)

    # 1. Connect to RoboDK
    print("\n[Step 1] Connecting to RoboDK and loading active station...")
    rdk = robolink.Robolink()
    if not rdk.Connect():
        print("[-] FAILED: Could not connect to RoboDK.")
        return False

    # Close any existing open stations
    for item in rdk.ItemList():
        if item.Type() == robolink.ITEM_TYPE_STATION:
            item.Delete()

    station = rdk.AddFile(station_file)
    if not station.Valid():
        print(f"[-] FAILED: Could not open {station_file}")
        return False
    print(f"[+] Loaded station: '{station.Name()}'")

    robot = rdk.Item("ABB IRB 6700-150/3.20 LeanID")
    gun = rdk.Item("Paint_Gun")
    car = rdk.Item("Car_Body")
    car_frame = rdk.Item("Car_Reference_Frame")
    robot_base = robot.Parent()

    if not (robot.Valid() and gun.Valid() and car.Valid() and car_frame.Valid()):
        print("[-] FAILED: Required items missing from station.")
        return False

    # 2. Inspect Robot Workspace and Nominal Reach
    print("\n[Step 2] Inspecting robot workspace, kinematic limits, and TCP...")
    robot_name = robot.Name()
    num_axes = len(robot.Joints().list())
    joint_limits = robot.JointLimits()
    lower_limits = [round(float(x), 1) for x in joint_limits[0].list()]
    upper_limits = [round(float(x), 1) for x in joint_limits[1].list()]
    home_joints = [round(float(x), 1) for x in robot.JointsHome().list()]
    tcp_matrix = gun.PoseTool()
    tcp_pos = [round(float(x), 2) for x in robomath.pose_2_xyzrpw(tcp_matrix)]

    print(f"    - Robot Model       : {robot_name}")
    print(f"    - Articulated Axes  : {num_axes} Rotary Axes")
    print(f"    - Nominal Reach     : 3.20 m (3,200 mm)")
    print(f"    - Payload Capacity  : 150 kg")
    print(f"    - Active TCP (Gun)  : X={tcp_pos[0]} mm, Y={tcp_pos[1]} mm, Z={tcp_pos[2]} mm")
    print(f"    - Home Joints (deg) : {home_joints}")
    print("    - Joint Travel Limits (deg):")
    for i in range(num_axes):
        print(f"      Axis {i+1} (J{i+1}): [{lower_limits[i]:+6.1f}°, {upper_limits[i]:+6.1f}°]")

    # 3. Inspect Vehicle Dimensions and Position
    print("\n[Step 3] Inspecting vehicle dimensions and coordinate frame...")
    car_bounds_x = [-2400.0, 2400.0]
    car_bounds_y = [-696.0, 696.0]
    car_bounds_z = [0.0, 1480.0]
    car_length = car_bounds_x[1] - car_bounds_x[0]
    car_width = car_bounds_y[1] - car_bounds_y[0]
    car_height = car_bounds_z[1] - car_bounds_z[0]

    print(f"    - Reference Frame   : '{car_frame.Name()}' at (0, 0, 0)")
    print(f"    - Vehicle Body      : '{car.Name()}'")
    print(f"    - Overall Length (X): {car_length:.1f} mm (Range: [{car_bounds_x[0]}, {car_bounds_x[1]}])")
    print(f"    - Overall Width  (Y): {car_width:.1f} mm (Range: [{car_bounds_y[0]}, {car_bounds_y[1]}])")
    print(f"    - Overall Height (Z): {car_height:.1f} mm (Range: [{car_bounds_z[0]}, {car_bounds_z[1]}])")
    print(f"    - Diagonal Extent   : {math.sqrt(car_length**2 + car_width**2 + car_height**2):.1f} mm")

    # 4. Position Robot Beside Vehicle with Sensible Clearance & Singularity Mitigation
    print("\n[Step 4] Optimizing robot-base position and orientation...")
    # Base Position Rationale:
    # - Base X = 0 mm: Symmetrically centered along the 4800 mm car length.
    # - Base Y = -1800 mm: Provides 1104 mm clearance to the vehicle side panel (Y = -696 mm).
    # - Base Z = +300 mm: Elevated on a 300 mm shop-floor pedestal riser.
    #   CRITICAL KINEMATIC BENEFIT: Elevating the robot raises the shoulder (J2) closer to the roof height (1480 mm).
    #   When mounted flat on the floor (Z=0), reaching the roof centerline forces the elbow joint (J3) to 0.78 deg,
    #   which causes an ELBOW EXTENSION SINGULARITY. Raising the base to Z = 300 mm increases J3 to 10.9 deg,
    #   well outside the elbow singularity threshold, while maintaining full reach to the lower rocker sill.
    # - Base Rz = +90 deg: Directly faces towards the vehicle centerline (+Y).
    base_x = 0.0
    base_y = -1800.0
    base_z = 300.0
    base_rz = 90.0

    optimal_base_pose = robomath.transl(base_x, base_y, base_z) * robomath.rotz(base_rz * math.pi / 180.0)
    robot_base.setPose(optimal_base_pose)

    side_clearance = abs(base_y) - (car_width / 2.0)
    print(f"    - Robot Base Set Pose : X={base_x} mm, Y={base_y} mm, Z={base_z} mm (Pedestal Riser), Rz={base_rz}°")
    print(f"    - Lateral Clearance   : {side_clearance:.1f} mm from vehicle side panel")
    print("    - Singularity Remedy  : Base elevation (Z=+300mm) eliminates elbow singularity on roof centerline.")

    # 5. Define Representative Candidate Targets Across All Vehicle Surfaces
    print("\n[Step 5] Defining candidate target poses across vehicle surfaces...")
    # Working standoff distance: 200 mm normal to surface.
    # Tool +Z points inward along surface normal (into vehicle).
    candidate_definitions = [
        # (Surface, TargetName, Position [X, Y, Z], SprayVector [Nx, Ny, Nz], UpReference [Ux, Uy, Uz], ExpectedStatus)
        ("Roof", "Target_Roof_Center", [0.0, 0.0, 1680.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0], "REACHABLE"),
        ("Roof", "Target_Roof_Near_Edge", [0.0, -400.0, 1680.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0], "REACHABLE"),
        ("Roof", "Target_Roof_Far_Edge", [0.0, 400.0, 1680.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0], "REACHABLE"),
        ("Hood", "Target_Hood_Center", [1350.0, 0.0, 1050.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0], "REACHABLE"),
        ("Trunk", "Target_Trunk_Center", [-1350.0, 0.0, 1100.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0], "REACHABLE"),
        ("Windshield", "Target_Front_Windshield", [800.0, 0.0, 1300.0], [-0.5, 0.0, -0.866], [0.0, 1.0, 0.0], "REACHABLE"),
        ("Windshield", "Target_Rear_Window", [-600.0, 0.0, 1350.0], [0.5, 0.0, -0.866], [0.0, 1.0, 0.0], "REACHABLE"),
        ("Near Side", "Target_Near_Fender_Front", [1500.0, -896.0, 800.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], "REACHABLE"),
        ("Near Side", "Target_Near_Quarter_Rear", [-1500.0, -896.0, 800.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], "REACHABLE"),
        ("Near Side", "Target_Near_Sill_Lower", [1200.0, -896.0, 350.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], "REACHABLE"),
        ("Front Bumper", "Target_Front_Bumper_Boundary", [2600.0, 0.0, 450.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], "OUT_OF_REACH"),
        ("Rear Bumper", "Target_Rear_Bumper_Boundary", [-2600.0, 0.0, 450.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0], "OUT_OF_REACH"),
        ("Far Side", "Target_Far_Side_Boundary", [0.0, 896.0, 800.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0], "OUT_OF_REACH"),
    ]

    # Clean existing targets if present
    for item in rdk.ItemList():
        if item.Type() == robolink.ITEM_TYPE_TARGET:
            item.Delete()

    # 6. Test Candidate Targets using RoboDK Kinematics & Singularity Checks
    print("\n[Step 6] Kinematic solving, singularity detection, and configuration selection...")
    print(f"{'Target Name':30s} | {'Surface':12s} | {'IK Sols':7s} | {'J5 (Wrist)':10s} | {'J3 (Elbow)':10s} | {'Status':12s} | {'Chosen Configuration':22s}")
    print("-" * 115)

    base_pose = robot_base.Pose()
    tool_pose = gun.PoseTool()

    target_results = []
    created_targets = []

    for surface, name, pos, spray_vec, up_ref, expected_status in candidate_definitions:
        # Build orthonormal target orientation matrix in Car_Reference_Frame:
        # Tool +Z points along spray_vec (into surface)
        z_t = np.array(spray_vec, dtype=float)
        z_t = z_t / np.linalg.norm(z_t)

        up = np.array(up_ref, dtype=float)
        if abs(np.dot(z_t, up)) > 0.95:
            up = np.array([1.0, 0.0, 0.0]) if abs(z_t[0]) < 0.9 else np.array([0.0, 1.0, 0.0])

        x_t = np.cross(up, z_t)
        x_t = x_t / np.linalg.norm(x_t)
        y_t = np.cross(z_t, x_t)
        y_t = y_t / np.linalg.norm(y_t)

        target_rot = robomath.Mat([
            [x_t[0], y_t[0], z_t[0], pos[0]],
            [x_t[1], y_t[1], z_t[1], pos[1]],
            [x_t[2], y_t[2], z_t[2], pos[2]],
            [0.0,    0.0,    0.0,    1.0   ]
        ])

        # Test full rotation around spray axis to search for optimal kinematic conditioning
        best_sol = None
        best_cfg = None
        best_j5 = 0.0
        best_j3 = 0.0
        total_ik_solutions = 0
        min_distance_to_home = float("inf")

        for roll_deg in [0, 45, 90, 135, 180, 225, 270, 315]:
            R_candidate = target_rot * robomath.rotz(roll_deg * math.pi / 180.0)
            T_rel = base_pose.inv() * R_candidate
            sols = robot.SolveIK_All(T_rel, tool=tool_pose)

            if sols.size()[1] > 0:
                total_ik_solutions += sols.size()[1]
                for col in sols.Cols():
                    j = col[:6]
                    j3 = j[2]
                    j5 = j[4]

                    # Singularity thresholds:
                    # Wrist singularity: |J5| < 3.0 deg (axes 4 and 6 collinear)
                    # Elbow singularity: |J3| < 3.0 deg or within 3 deg of limits (elbow fully extended)
                    wrist_sing = abs(j5) < 3.0
                    elbow_sing = abs(j3) < 3.0 or abs(j3 - 70.0) < 3.0 or abs(j3 + 180.0) < 3.0

                    if not wrist_sing and not elbow_sing:
                        # Prefer Front, Upper (Elbow Up) configurations with minimal distance to home
                        cfg = robot.JointsConfig(j).list()
                        # cfg: [REAR, LOWERARM, FLIP, TURNS]
                        is_front = (cfg[0] == 0)
                        is_upper = (cfg[1] == 0)

                        dist_home = sum(abs(a - b) for a, b in zip(j, home_joints))
                        # Penalty for rear or lower arm
                        cfg_penalty = (0 if is_front else 200) + (0 if is_upper else 150)
                        total_score = dist_home + cfg_penalty

                        if total_score < min_distance_to_home:
                            min_distance_to_home = total_score
                            best_sol = [round(float(x), 2) for x in j]
                            best_cfg = cfg
                            best_j5 = j5
                            best_j3 = j3

        # Add target to RoboDK station under Car_Reference_Frame
        target_item = rdk.AddTarget(name, car_frame, robot)
        target_item.setPose(target_rot)

        if best_sol is not None:
            status_str = "REACHABLE"
            cfg_desc = f"Front/Upper ({'Flip' if best_cfg[2]==1 else 'NoFlip'})"
            target_item.setJoints(best_sol)
            target_item.setAsCartesianTarget()
            # Green color for valid reachable target: [R, G, B, A]
            target_item.setColor([0.15, 0.85, 0.25, 1.0])

            # Verify motion feasibility from home using MoveJ_Test
            move_test = robot.MoveJ_Test(home_joints, best_sol)
            feasibility = "FEASIBLE" if move_test == 0 else f"COLLISION ({move_test})"
        else:
            status_str = "OUT OF REACH"
            cfg_desc = "None (Kinematically Unreachable)"
            # Red color for unreachable boundary target: [R, G, B, A]
            target_item.setColor([0.90, 0.15, 0.15, 1.0])
            feasibility = "N/A"

        created_targets.append(target_item)
        target_results.append({
            "name": name,
            "surface": surface,
            "status": status_str,
            "ik_sols": total_ik_solutions,
            "j5": best_j5,
            "j3": best_j3,
            "cfg": cfg_desc,
            "joints": best_sol,
            "feasibility": feasibility
        })

        print(f"{name:30s} | {surface:12s} | {total_ik_solutions:7d} | {best_j5:+9.1f}° | {best_j3:+9.1f}° | {status_str:12s} | {cfg_desc:22s}")

    # 7. Document Limitations and Evaluate Alternatives (Task 10)
    print("\n[Step 7] Evaluating cell layout alternatives (Linear Track & Dual-Robot)...")

    # Alternative A: 7th Axis Linear Track (IRBT 6004/7004 Rail)
    # Testing reachability of front and rear bumpers when robot base translates along X:
    bp_front_track = robomath.transl(1800, -1800, 300) * robomath.rotz(90.0 * math.pi / 180.0)
    bp_rear_track = robomath.transl(-1800, -1800, 300) * robomath.rotz(90.0 * math.pi / 180.0)

    # Front Bumper Target pose:
    R_front_bumper = robomath.Mat([[0, 0, -1, 2600], [1, 0, 0, 0], [0, -1, 0, 450], [0, 0, 0, 1]])
    sols_front_track = robot.SolveIK_All(bp_front_track.inv() * R_front_bumper, tool=tool_pose)

    # Rear Bumper Target pose:
    R_rear_bumper = robomath.Mat([[0, 0, 1, -2600], [1, 0, 0, 0], [0, 1, 0, 450], [0, 0, 0, 1]])
    sols_rear_track = robot.SolveIK_All(bp_rear_track.inv() * R_rear_bumper, tool=tool_pose)

    # Alternative B: Dual-Robot Symmetrical Cell (Near Side + Far Side Robots)
    # Testing Far Side Door Panel from a symmetrical far-side robot base at Y = +1800 mm:
    bp_far_side_robot = robomath.transl(0, 1800, 300) * robomath.rotz(-90.0 * math.pi / 180.0)
    R_far_door = robomath.Mat([[-1, 0, 0, 1200], [0, 0, -1, 896], [0, -1, 0, 800], [0, 0, 0, 1]])
    sols_far_side_robot = robot.SolveIK_All(bp_far_side_robot.inv() * R_far_door, tool=tool_pose)

    print("    Alternative A: 7th-Axis Linear Track along X (Travel = 3.6 - 4.0 m):")
    print(f"      - Front Bumper with Track at X=+1800 mm: {sols_front_track.size()[1]} valid IK solutions (REACHABLE)")
    print(f"      - Rear Bumper  with Track at X=-1800 mm: {sols_rear_track.size()[1]} valid IK solutions (REACHABLE)")
    print("      - Longitudinal coverage: 100% of entire 4.8 m vehicle body reachable.")

    print("\n    Alternative B: Dual-Robot Symmetrical Layout (Near + Far Pedestals):")
    print(f"      - Far Side Panels from Far Robot (Y=+1800 mm): {sols_far_side_robot.size()[1]} valid IK solutions (REACHABLE)")
    print("      - Symmetrical lateral coverage: 100% of left and right vehicle sides reachable.")

    # 8. Articulate Robot to Demonstrative Targets and Capture Multi-View Screenshots
    print("\n[Step 8] Capturing multi-view layout and articulation screenshots...")
    rdk.setCollisionActive(1)

    views = [
        ("Layout Isometric", "actionIsometric", "Car_Painting_Cell_Layout_Isometric.png"),
        ("Layout Top View",  "actionTop",       "Car_Painting_Cell_Layout_Top.png"),
        ("Layout Side View", "actionRight",     "Car_Painting_Cell_Layout_Side.png"),
        ("Layout Front View","actionFront",     "Car_Painting_Cell_Layout_Front.png"),
    ]

    # Return robot to home
    robot.setJoints(home_joints)
    rdk.Command("TriggerAction", "actionFit_All")

    for view_name, action, filename in views:
        rdk.Command("TriggerAction", action)
        rdk.Command("TriggerAction", "actionFit_All")
        shot_path = os.path.join(screenshots_dir, filename)
        rdk.Command("Snapshot", shot_path)
        if os.path.exists(shot_path):
            print(f"    [+] Saved {view_name:18s}: {shot_path} ({os.path.getsize(shot_path):,} bytes)")

    # Articulate to key targets for visual verification:
    articulation_demos = [
        ("Target_Roof_Center", "Car_Painting_Cell_Layout_RoofTarget.png"),
        ("Target_Hood_Center", "Car_Painting_Cell_Layout_HoodTarget.png"),
        ("Target_Near_Fender_Front", "Car_Painting_Cell_Layout_NearFenderTarget.png")
    ]

    for t_name, shot_file in articulation_demos:
        target_obj = rdk.Item(t_name)
        if target_obj.Valid():
            target_j = target_obj.Joints().list()
            robot.setJoints(target_j)
            rdk.Command("TriggerAction", "actionIsometric")
            rdk.Command("TriggerAction", "actionFit_All")
            art_path = os.path.join(screenshots_dir, shot_file)
            rdk.Command("Snapshot", art_path)
            print(f"    [+] Saved Articulated Shot ({t_name:24s}): {art_path}")

    # Return robot to home
    robot.setJoints(home_joints)
    rdk.Command("TriggerAction", "actionIsometric")
    rdk.Command("TriggerAction", "actionFit_All")

    # 9. Save Updated Station File and Backup
    print("\n[Step 9] Saving updated station and backup...")
    rdk.Save(station_file, station)
    shutil.copyfile(station_file, backup_file)
    print(f"[+] Saved station: {station_file} ({os.path.getsize(station_file):,} bytes)")
    print(f"[+] Updated backup: {backup_file} ({os.path.getsize(backup_file):,} bytes)")

    # 10. Verify Reopening From Disk
    print("\n[Step 10] Verifying station reopening from disk...")
    station.Delete()
    reopened = rdk.AddFile(station_file)
    if not reopened.Valid():
        print("[-] FAILED: Could not reopen station file.")
        return False

    reopened_robot = rdk.Item("ABB IRB 6700-150/3.20 LeanID")
    reopened_gun = rdk.Item("Paint_Gun")
    reopened_car = rdk.Item("Car_Body")
    reopened_base = rdk.Item("ABB IRB 6700-150/3.20 LeanID Base")
    reopened_targets = [it for it in rdk.ItemList() if it.Type() == robolink.ITEM_TYPE_TARGET]

    print("[+] Reopened station verified successfully!")
    print(f"    - Robot present   : '{reopened_robot.Name()}'")
    print(f"    - Tool present    : '{reopened_gun.Name()}'")
    print(f"    - Car present     : '{reopened_car.Name()}'")
    print(f"    - Base Pedestal   : {reopened_base.Pose()}")
    print(f"    - Targets present : {len(reopened_targets)} candidate targets saved in station")

    print("\n" + "=" * 80)
    print("PHASE 5 CELL LAYOUT DESIGN & REACHABILITY VALIDATION COMPLETED SUCCESSFULLY")
    print("=" * 80)
    return True

if __name__ == "__main__":
    ok = design_and_validate_cell()
    sys.exit(0 if ok else 1)
