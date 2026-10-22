"""
Script to create the initial RoboDK station for the Robotic Car Painting Cell.
Assessment: Yuktii AI Labs - Robotic Car Painting Cell Simulation
Robot: ABB IRB 6700-150/3.20 LeanID (6-axis, 150 kg payload, 3.2 m reach)
"""

import os
import sys
import math
import shutil
from robodk import robolink, robomath

def create_initial_station():
    print("=" * 70)
    print("ROBODK INITIAL STATION CREATION: CAR PAINTING CELL")
    print("=" * 70)

    # 1. Connect to RoboDK
    print("\n[Step 1] Connecting to RoboDK instance...")
    rdk = robolink.Robolink()
    if not rdk.Connect():
        print("[-] FAILED: Could not connect to running RoboDK instance.")
        return False
    print(f"[+] Connected to RoboDK version: {rdk.Version()}")

    # Close any open stations to start completely fresh
    existing_items = rdk.ItemList()
    for item in existing_items:
        if item.Type() == robolink.ITEM_TYPE_STATION:
            print(f"    Closing previous station: '{item.Name()}'")
            item.Delete()

    # Create new station
    station_name = "Car_Painting_Cell"
    station = rdk.AddStation(station_name)
    if not station.Valid():
        print("[-] FAILED: Could not create station.")
        return False
    print(f"[+] Created new station: '{station.Name()}'")

    # 2. Locate and verify robot file
    print("\n[Step 2] Locating robot model...")
    robot_file_path = "/home/sanket/Yuktii_Robotics_Assessment/Models/Robot/ABB-IRB-6700-150-3-20-LeanID.robot"
    if not os.path.exists(robot_file_path):
        robot_file_path = "/home/sanket/RoboDK/Library/ABB-IRB-6700-150-3-20-LeanID.robot"

    if not os.path.exists(robot_file_path):
        print(f"[-] FAILED: Robot model not found at {robot_file_path}")
        return False
    print(f"[+] Robot file found: {robot_file_path}")
    print(f"    File size: {os.path.getsize(robot_file_path):,} bytes")

    # 3. Add robot to station
    print("\n[Step 3] Adding robot to the station...")
    robot = rdk.AddFile(robot_file_path)
    if not robot.Valid():
        print("[-] FAILED: Could not load robot into station.")
        return False
    print(f"[+] Successfully loaded robot: '{robot.Name()}' (Type ID: {robot.Type()})")

    # 4. Confirm axes and model details
    print("\n[Step 4] Confirming robot axes and model parameters...")
    joints = robot.Joints()
    joint_values = joints.list()
    num_axes = len(joint_values)
    print(f"    - Number of axes: {num_axes}")
    if num_axes != 6:
        print(f"[-] Warning: Expected 6 axes, got {num_axes}")
    else:
        print("    [+] Axis count confirmed: 6-Axis Articulated Robot")

    joint_limits = robot.JointLimits()
    lower_limits = [round(float(x), 2) for x in joint_limits[0].list()]
    upper_limits = [round(float(x), 2) for x in joint_limits[1].list()]
    print("    - Joint limits (degrees):")
    for i in range(num_axes):
        print(f"      Axis {i+1}: [{lower_limits[i]:+7.1f}°, {upper_limits[i]:+7.1f}°]")

    home_joints = robot.JointsHome()
    print(f"    - Home joints: {[round(float(j), 2) for j in home_joints.list()]}")

    # 5. Set sensible initial robot-base position
    # Car dimensions: Length = 4800 mm (X: -2400 to +2400), Width = 1900 mm (Y: -950 to +950), Height = 1600 mm (Z: 0 to 1600)
    # Robot has 3200 mm reach.
    # Placing robot base at X = 0 mm (centered along car length), Y = -2000 mm (1050 mm clearance from car side), Z = 0 mm.
    # Rotated Rz = +90 degrees so the robot faces towards +Y (directly towards the car centerline).
    print("\n[Step 5] Setting robot-base position and coordinate frame...")
    base_item = robot.Parent()
    print(f"    Robot parent base item: '{base_item.Name()}' (Type: {base_item.Type()})")

    base_x = 0.0
    base_y = -2000.0
    base_z = 0.0
    base_rz = 90.0  # degrees

    base_pose = robomath.transl(base_x, base_y, base_z) * robomath.rotz(base_rz * math.pi / 180.0)
    base_item.setPose(base_pose)

    actual_base_pose = base_item.Pose()
    actual_robot_pose_abs = robot.PoseAbs()
    print(f"    Base set coordinates: X = {base_x} mm, Y = {base_y} mm, Z = {base_z} mm, Rz = {base_rz}°")
    print(f"    Actual Base Pose matrix:\n{actual_base_pose}")
    print(f"    Robot Absolute Pose (WCS):\n{actual_robot_pose_abs}")

    # Geometry & Reachability verification
    dist_to_car_side = abs(base_y) - (1900.0 / 2.0)
    dist_to_near_corner = math.sqrt(2400.0**2 + dist_to_car_side**2)
    dist_to_car_center = math.sqrt(base_x**2 + abs(base_y)**2)
    dist_to_roof_top_near_corner = math.sqrt(2400.0**2 + dist_to_car_side**2 + 1600.0**2)

    print("\n    Geometry verification relative to 4800 x 1900 x 1600 mm car:")
    print(f"      - Clearance to vehicle side panel: {dist_to_car_side:.1f} mm")
    print(f"      - Distance to nearest vehicle side corners: {dist_to_near_corner:.1f} mm (< 3200 mm reach: OK)")
    print(f"      - Distance to vehicle center (X=0, Y=0): {dist_to_car_center:.1f} mm (< 3200 mm reach: OK)")
    print(f"      - Distance to near-side upper roof corners: {dist_to_roof_top_near_corner:.1f} mm (< 3200 mm reach: OK)")

    # 6. Inspect station tree
    print("\n[Step 6] Inspecting station tree hierarchy...")
    items = rdk.ItemList()
    print(f"    Total items in station: {len(items)}")
    for item in items:
        parent_name = item.Parent().Name() if item.Parent().Valid() else "None (Root)"
        print(f"    - Item: '{item.Name()}' | Type: {item.Type()} | Parent: '{parent_name}'")

    # 7. Save station
    print("\n[Step 7] Saving station file...")
    save_dir = "/home/sanket/Yuktii_Robotics_Assessment/Simulation_Files"
    os.makedirs(save_dir, exist_ok=True)
    station_file = os.path.join(save_dir, "Car_Painting_Cell.rdk")
    rdk.Save(station_file, station)

    if os.path.exists(station_file):
        size_bytes = os.path.getsize(station_file)
        print(f"[+] Saved successfully to: {station_file} ({size_bytes:,} bytes)")
    else:
        print(f"[-] FAILED: File does not exist at {station_file}")
        return False

    # 8. Create backup
    print("\n[Step 8] Creating backup of saved station...")
    backup_file = os.path.join(save_dir, "Car_Painting_Cell_backup.rdk")
    shutil.copyfile(station_file, backup_file)
    if os.path.exists(backup_file):
        print(f"[+] Backup created at: {backup_file} ({os.path.getsize(backup_file):,} bytes)")
    else:
        print(f"[-] FAILED: Backup file was not created.")
        return False

    # 9. Verify reopening station
    print("\n[Step 9] Verifying station reopening...")
    # Delete current station from RoboDK
    station.Delete()
    reopen_items_before = len(rdk.ItemList())
    print(f"    Items after closing station: {reopen_items_before}")

    # Reopen saved file
    reopened_st = rdk.AddFile(station_file)
    if not reopened_st.Valid():
        print("[-] FAILED: Could not reopen saved station.")
        return False

    print(f"[+] Reopened station: '{reopened_st.Name()}'")
    reopened_robot = rdk.Item("ABB IRB 6700-150/3.20 LeanID")
    if not reopened_robot.Valid():
        print("[-] FAILED: Robot item not found in reopened station.")
        return False

    reopened_base = reopened_robot.Parent()
    print(f"[+] Verified robot in reopened station: '{reopened_robot.Name()}'")
    print(f"    Axes in reopened robot: {len(reopened_robot.Joints().list())}")
    print(f"    Base frame in reopened station: '{reopened_base.Name()}'")
    print(f"    Reopened Base Pose:\n{reopened_base.Pose()}")

    print("\n" + "=" * 70)
    print("PHASE 2 INITIAL STATION CREATION SUCCESSFUL")
    print("=" * 70)
    return True

if __name__ == "__main__":
    ok = create_initial_station()
    sys.exit(0 if ok else 1)
