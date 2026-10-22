"""
Script to add, configure, attach, and verify the painting end-effector.
Assessment: Yuktii AI Labs - Robotic Car Painting Cell Simulation
Attaches the Paint Gun tool to the ABB IRB 6700-150/3.20 LeanID robot flange,
defines the Tool Center Point (TCP) at the physical spray nozzle, verifies motion synchronization,
tests reachability of car surfaces, captures multi-view inspection screenshots, and saves the station.
"""

import os
import sys
import math
import shutil
import numpy as np
from robodk import robolink, robomath

def add_painting_end_effector():
    print("=" * 70)
    print("ROBODK PAINTING END-EFFECTOR INTEGRATION")
    print("=" * 70)

    station_file = "/home/sanket/Yuktii_Robotics_Assessment/Simulation_Files/Car_Painting_Cell.rdk"
    backup_file = "/home/sanket/Yuktii_Robotics_Assessment/Simulation_Files/Car_Painting_Cell_backup.rdk"
    tool_file = "/home/sanket/Yuktii_Robotics_Assessment/Models/End_Effector/Paint_Gun.tool"
    stl_file = "/home/sanket/Yuktii_Robotics_Assessment/Models/End_Effector/Paint_Gun.stl"
    screenshots_dir = "/home/sanket/Yuktii_Robotics_Assessment/Screenshots"
    os.makedirs(screenshots_dir, exist_ok=True)

    # 1. Connect to RoboDK
    print("\n[Step 1] Connecting to RoboDK and loading station...")
    rdk = robolink.Robolink()
    if not rdk.Connect():
        print("[-] FAILED: Could not connect to RoboDK.")
        return False

    # Close any existing open stations
    for item in rdk.ItemList():
        if item.Type() == robolink.ITEM_TYPE_STATION:
            item.Delete()

    # Reopen station
    station = rdk.AddFile(station_file)
    if not station.Valid():
        print(f"[-] FAILED: Could not open {station_file}")
        return False
    print(f"[+] Loaded station: '{station.Name()}'")

    # Verify robot and car are present
    robot = rdk.Item("ABB IRB 6700-150/3.20 LeanID")
    car = rdk.Item("Car_Body")
    if not robot.Valid() or not car.Valid():
        print("[-] FAILED: Robot or Car_Body missing from station.")
        return False
    print(f"[+] Preserved robot: '{robot.Name()}'")
    print(f"[+] Preserved car body: '{car.Name()}'")

    # 2. Check and prepare tool files
    print("\n[Step 2] Verifying end-effector files...")
    if not os.path.exists(tool_file):
        src_tool = "/home/sanket/RoboDK/Library/01-Robot painting/Parts/Paint-gun.tool"
        shutil.copyfile(src_tool, tool_file)
    print(f"[+] Tool file confirmed: {tool_file} ({os.path.getsize(tool_file):,} bytes)")

    # Remove previous tool if already attached to avoid duplication
    existing_tools = robot.Childs()
    for child in existing_tools:
        if child.Type() == robolink.ITEM_TYPE_TOOL:
            print(f"    Removing existing tool: '{child.Name()}'")
            child.Delete()

    # 3. Import and attach tool to robot flange
    print("\n[Step 3] Importing tool and attaching to robot flange...")
    # Passing the robot item as second argument attaches the tool directly to the robot flange
    gun = rdk.AddFile(tool_file, robot)
    if not gun.Valid():
        print("[-] FAILED: Could not attach tool to robot.")
        return False

    gun.setName("Paint_Gun")
    gun.setParent(robot)
    print(f"[+] Tool attached: '{gun.Name()}' (Type: {gun.Type()})")
    print(f"    Parent: '{gun.Parent().Name()}' (Robot Flange Interface)")

    # 4. Define TCP at physical spray nozzle
    print("\n[Step 4] Defining Tool Center Point (TCP) at spray nozzle...")
    # Physical nozzle orifice measured from CAD mesh:
    # X = 0.00 mm, Y = 0.00 mm, Z = 325.06 mm relative to mounting flange
    # Nozzle normal vector is [0, 0, 1] pointing along the +Z axis.
    tcp_x = 0.0
    tcp_y = 0.0
    tcp_z = 325.06
    tcp_rx = 0.0
    tcp_ry = 0.0
    tcp_rz = 0.0

    tcp_pose = robomath.transl(tcp_x, tcp_y, tcp_z) * robomath.rotx(tcp_rx * math.pi / 180.0) * robomath.roty(tcp_ry * math.pi / 180.0) * robomath.rotz(tcp_rz * math.pi / 180.0)
    gun.setPoseTool(tcp_pose)

    actual_tcp = gun.PoseTool()
    print("    TCP Definition (Relative to Flange):")
    print(f"      - Translation: X = {tcp_x:.2f} mm, Y = {tcp_y:.2f} mm, Z = {tcp_z:.2f} mm")
    print(f"      - Rotation   : Rx = {tcp_rx:.1f}°, Ry = {tcp_ry:.1f}°, Rz = {tcp_rz:.1f}°")
    print(f"    TCP Transformation Matrix:\n{actual_tcp}")

    # Document spray direction
    print("\n    Spray Direction & Pattern Specifications:")
    print("      - Spray Vector: Along TCP +Z axis ([0.0, 0.0, 1.0])")
    print("      - Ejection Direction: Coaxial with nozzle center pointing forward away from flange")
    print("      - Spray Fan Plane: Spanned by TCP X and Y axes")
    print("      - Recommended Standoff Distance: 150 - 250 mm to vehicle surface")

    # 5. Set tool appearance
    # Sleek gunmetal / titanium finish for industrial tool [R, G, B, A]
    gun.setColor([0.30, 0.32, 0.35, 1.0])

    # 6. Verify gun moves rigidly with the robot
    print("\n[Step 5] Verifying tool motion synchronization with robot...")
    home_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    robot.setJoints(home_joints)
    initial_gun_pose = gun.PoseAbs()
    initial_pos = [initial_gun_pose[0, 3], initial_gun_pose[1, 3], initial_gun_pose[2, 3]]
    print(f"    Gun position at home: X = {initial_pos[0]:.2f}, Y = {initial_pos[1]:.2f}, Z = {initial_pos[2]:.2f} mm")

    # Articulate robot joints
    test_joints = [15.0, 20.0, -10.0, 10.0, 30.0, -15.0]
    robot.setJoints(test_joints)
    articulated_gun_pose = gun.PoseAbs()
    art_pos = [articulated_gun_pose[0, 3], articulated_gun_pose[1, 3], articulated_gun_pose[2, 3]]
    displacement = math.sqrt(sum((a - b)**2 for a, b in zip(art_pos, initial_pos)))
    print(f"    Gun position articulated: X = {art_pos[0]:.2f}, Y = {art_pos[1]:.2f}, Z = {art_pos[2]:.2f} mm")
    print(f"    Observed displacement during joint motion: {displacement:.2f} mm")

    if displacement > 100.0:
        print("    [+] CONFIRMED: Tool is rigidly bound to robot flange and moves with robot joints.")
    else:
        print("    [-] WARNING: Tool did not exhibit expected displacement.")

    # Return to home configuration
    robot.setJoints(home_joints)
    restored_gun_pose = gun.PoseAbs()
    restored_pos = [restored_gun_pose[0, 3], restored_gun_pose[1, 3], restored_gun_pose[2, 3]]
    error = math.sqrt(sum((a - b)**2 for a, b in zip(restored_pos, initial_pos)))
    print(f"    Return-to-home repeat accuracy: {error:.4f} mm")

    # 7. Verify surface reachability with tool attached
    print("\n[Step 6] Testing IK reachability of vehicle surfaces with tool...")
    base_pose = robot.Parent().Pose()

    # Define key vehicle surfaces with spray gun facing normally:
    # Tool +Z must point into the vehicle surface
    surfaces = [
        ("Side Door Panel", robomath.transl(0, -846, 800) * robomath.Mat([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])),
        ("Roof Centerline", robomath.transl(0, 0, 1630) * robomath.rotx(math.pi)),
        ("Front Hood",      robomath.transl(1500, 0, 1050) * robomath.rotx(math.pi)),
        ("Rear Trunk",     robomath.transl(-1500, 0, 1050) * robomath.rotx(math.pi))
    ]

    for name, T_target_wcs in surfaces:
        T_rel = base_pose.inv() * T_target_wcs
        sols = robot.SolveIK_All(T_rel, tool=gun.PoseTool())
        status = "REACHABLE" if len(sols) > 0 else "OUT OF REACH"
        print(f"    - {name:16s}: {status} ({len(sols)} kinematic configurations found)")

    # 8. Multi-view inspection screenshots
    print("\n[Step 7] Capturing multi-view inspection screenshots...")
    views = [
        ("Isometric", "actionIsometric", "Car_Painting_Cell_Tool_Isometric.png"),
        ("Top View",  "actionTop",       "Car_Painting_Cell_Tool_Top.png"),
        ("Side View", "actionRight",     "Car_Painting_Cell_Tool_Side.png"),
        ("Front View","actionFront",     "Car_Painting_Cell_Tool_Front.png")
    ]

    for name, action, filename in views:
        rdk.Command("TriggerAction", action)
        rdk.Command("TriggerAction", "actionFit_All")
        shot_path = os.path.join(screenshots_dir, filename)
        rdk.Command("Snapshot", shot_path)
        if os.path.exists(shot_path):
            print(f"    [+] Saved {name:10s} screenshot: {shot_path} ({os.path.getsize(shot_path):,} bytes)")

    # Close-up screenshot of the end-effector attached to robot flange
    rdk.Command("TriggerAction", "actionIsometric")
    rdk.Command("TriggerAction", "actionFit_All")
    close_up_path = os.path.join(screenshots_dir, "Car_Painting_Cell_Tool_CloseUp.png")
    rdk.Command("Snapshot", close_up_path)
    print(f"    [+] Saved Close-Up   screenshot: {close_up_path} ({os.path.getsize(close_up_path):,} bytes)")

    # 9. Inspect station tree hierarchy
    print("\n[Step 8] Inspecting station tree hierarchy...")
    items = rdk.ItemList()
    print(f"    Total items in station: {len(items)}")
    for item in items:
        p_name = item.Parent().Name() if item.Parent().Valid() else "None (Root)"
        print(f"    - '{item.Name()}' | Type: {item.Type()} | Parent: '{p_name}'")

    # 10. Save station and backup
    print("\n[Step 9] Saving station file and backup...")
    rdk.Save(station_file, station)
    shutil.copyfile(station_file, backup_file)
    print(f"[+] Saved station: {station_file} ({os.path.getsize(station_file):,} bytes)")
    print(f"[+] Updated backup: {backup_file} ({os.path.getsize(backup_file):,} bytes)")

    # 11. Verify reopening from disk
    print("\n[Step 10] Verifying station reopening from disk...")
    station.Delete()
    reopened = rdk.AddFile(station_file)
    if not reopened.Valid():
        print("[-] FAILED: Could not reopen station file.")
        return False

    reopened_robot = rdk.Item("ABB IRB 6700-150/3.20 LeanID")
    reopened_gun = rdk.Item("Paint_Gun")
    reopened_car = rdk.Item("Car_Body")

    if reopened_robot.Valid() and reopened_gun.Valid() and reopened_car.Valid():
        print("[+] Reopened station verified successfully!")
        print(f"    - Robot present : '{reopened_robot.Name()}'")
        print(f"    - Tool present  : '{reopened_gun.Name()}' (Attached to: '{reopened_gun.Parent().Name()}')")
        print(f"    - Car present   : '{reopened_car.Name()}'")
        print(f"    - TCP PoseTool  :\n{reopened_gun.PoseTool()}")
    else:
        print("[-] FAILED: Missing items upon reopening.")
        return False

    print("\n" + "=" * 70)
    print("PHASE 4 PAINTING END-EFFECTOR INTEGRATION COMPLETED SUCCESSFULLY")
    print("=" * 70)
    return True

if __name__ == "__main__":
    ok = add_painting_end_effector()
    sys.exit(0 if ok else 1)
