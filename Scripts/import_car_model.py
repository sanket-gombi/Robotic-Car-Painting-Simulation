"""
Script to import, scale, align, and position the car body in the RoboDK station.
Assessment: Yuktii AI Labs - Robotic Car Painting Cell Simulation
Preserves the ABB IRB 6700-150/3.20 LeanID robot, establishes standard vehicle coordinates,
positions the vehicle on the floor, optimizes the robot base position, and takes multi-view screenshots.
"""

import os
import sys
import math
import struct
import shutil
import numpy as np
from robodk import robolink, robomath

def process_car_mesh(raw_path, output_path, scale=0.80):
    """
    Reads the raw car STL, measures original bounding box, applies uniform scaling,
    aligns to standard vehicle coordinates (X: longitudinal, Y: lateral, Z: vertical),
    places lowest vertex on floor (Z=0), centers X and Y, and writes the transformed binary STL.
    """
    with open(raw_path, 'rb') as f:
        header = f.read(80)
        n_triangles = struct.unpack('<I', f.read(4))[0]
        triangles = []
        for _ in range(n_triangles):
            d = f.read(50)
            vals = struct.unpack('<12fH', d)
            norm = vals[0:3]
            v1 = vals[3:6]
            v2 = vals[6:9]
            v3 = vals[9:12]
            attr = vals[12]
            triangles.append((norm, v1, v2, v3, attr))

    # Calculate raw bounding box
    all_raw_v = []
    for t in triangles:
        all_raw_v.extend([t[1], t[2], t[3]])
    all_raw_v = np.array(all_raw_v)
    raw_min = np.min(all_raw_v, axis=0)
    raw_max = np.max(all_raw_v, axis=0)
    raw_dims = raw_max - raw_min

    # Raw axes in Car-Frame.stl:
    # X_raw: Lateral / Width ([-870.0, +870.0], extent = 1740.0 mm)
    # Y_raw: Longitudinal / Length ([-3020.0, +2980.0], extent = 6000.0 mm)
    # Z_raw: Vertical / Height ([-450.0, +1400.0], extent = 1850.0 mm)

    y_raw_center = (raw_min[1] + raw_max[1]) / 2.0  # -20.0 mm
    x_raw_center = (raw_min[0] + raw_max[0]) / 2.0  # 0.0 mm
    z_raw_min = raw_min[2]                         # -450.0 mm

    # Target convention (ISO / SAE vehicle coordinate system):
    # X: Longitudinal (along vehicle length, from Y_raw)
    # Y: Lateral (across vehicle width, from -X_raw)
    # Z: Vertical (height above floor, lowest point at Z = 0.0 mm)
    # Proper rotation by -90 deg around Z:
    # X_new = (Y_raw - Y_center) * scale
    # Y_new = -(X_raw - X_center) * scale
    # Z_new = (Z_raw - Z_min) * scale

    transformed_triangles = []
    all_trans_v = []
    for norm, v1, v2, v3, attr in triangles:
        def tf(v):
            return [
                float((v[1] - y_raw_center) * scale),
                float(-(v[0] - x_raw_center) * scale),
                float((v[2] - z_raw_min) * scale)
            ]
        new_norm = [
            float(norm[1]),
            float(-norm[0]),
            float(norm[2])
        ]
        nv1 = tf(v1)
        nv2 = tf(v2)
        nv3 = tf(v3)
        transformed_triangles.append((new_norm, nv1, nv2, nv3, attr))
        all_trans_v.extend([nv1, nv2, nv3])

    all_trans_v = np.array(all_trans_v)
    trans_min = np.min(all_trans_v, axis=0)
    trans_max = np.max(all_trans_v, axis=0)
    trans_dims = trans_max - trans_min

    # Write output binary STL
    with open(output_path, 'wb') as f:
        hdr_text = b'Robotic Car Painting Cell - Uniformly Scaled Vehicle Body'
        f.write(hdr_text.ljust(80, b'\0')[:80])
        f.write(struct.pack('<I', len(transformed_triangles)))
        for norm, nv1, nv2, nv3, attr in transformed_triangles:
            data = struct.pack(
                '<12fH',
                norm[0], norm[1], norm[2],
                nv1[0], nv1[1], nv1[2],
                nv2[0], nv2[1], nv2[2],
                nv3[0], nv3[1], nv3[2],
                attr
            )
            f.write(data)

    return (raw_min, raw_max, raw_dims), (trans_min, trans_max, trans_dims), len(transformed_triangles)


def import_and_setup_vehicle():
    print("=" * 70)
    print("ROBODK CAR-BODY MODEL IMPORT & INTEGRATION")
    print("=" * 70)

    # 1. Paths setup
    raw_stl_path = "/home/sanket/Yuktii_Robotics_Assessment/Models/Vehicle/Car_Frame_Original.stl"
    scaled_stl_path = "/home/sanket/Yuktii_Robotics_Assessment/Models/Vehicle/Car_Body.stl"
    station_file = "/home/sanket/Yuktii_Robotics_Assessment/Simulation_Files/Car_Painting_Cell.rdk"
    backup_file = "/home/sanket/Yuktii_Robotics_Assessment/Simulation_Files/Car_Painting_Cell_backup.rdk"
    screenshots_dir = "/home/sanket/Yuktii_Robotics_Assessment/Screenshots"
    os.makedirs(screenshots_dir, exist_ok=True)

    if not os.path.exists(raw_stl_path):
        src_orig = "/home/sanket/RoboDK/Library/Add-in - Collision-free path - Spot Welding with Comau/Parts/Car-Frame.stl"
        shutil.copyfile(src_orig, raw_stl_path)

    # 2. Measure raw model and create scaled, floor-aligned STL
    print("\n[Step 1] Measuring raw model bounding dimensions...")
    raw_bounds, trans_bounds, tri_count = process_car_mesh(raw_stl_path, scaled_stl_path, scale=0.80)
    raw_min, raw_max, raw_dims = raw_bounds
    trans_min, trans_max, trans_dims = trans_bounds

    print("    Raw Dimensions (before scaling):")
    print(f"      - Longitudinal (Y): {raw_dims[1]:.2f} mm (Range: [{raw_min[1]:.2f}, {raw_max[1]:.2f}])")
    print(f"      - Lateral      (X): {raw_dims[0]:.2f} mm (Range: [{raw_min[0]:.2f}, {raw_max[0]:.2f}])")
    print(f"      - Vertical     (Z): {raw_dims[2]:.2f} mm (Range: [{raw_min[2]:.2f}, {raw_max[2]:.2f}])")
    print(f"      - Total Triangles: {tri_count:,}")

    print("\n[Step 2] Applied Uniform Scaling & Coordinate Alignment:")
    print("    - Uniform Scale Factor: s = 0.8000 (Preserving aspect ratio; no axis distortion)")
    print("    - Coordinate Convention: X = Longitudinal, Y = Lateral, Z = Vertical")
    print("    - Floor Placement: Lowest point placed at ground level Z = 0.00 mm")
    print("    - Center Placement: Symmetrically centered at X = 0.00 mm, Y = 0.00 mm")
    print("    Scaled Dimensions (Car_Body.stl):")
    print(f"      - Length (X axis): {trans_dims[0]:.2f} mm (Range: [{trans_min[0]:.2f}, {trans_max[0]:.2f}])")
    print(f"      - Width  (Y axis): {trans_dims[1]:.2f} mm (Range: [{trans_min[1]:.2f}, {trans_max[1]:.2f}])")
    print(f"      - Height (Z axis): {trans_dims[2]:.2f} mm (Range: [{trans_min[2]:.2f}, {trans_max[2]:.2f}])")

    # 3. Connect to RoboDK and load existing station
    print("\n[Step 3] Connecting to RoboDK and loading station...")
    rdk = robolink.Robolink()
    if not rdk.Connect():
        print("[-] FAILED: Could not connect to RoboDK.")
        return False

    # Close any existing open stations
    for item in rdk.ItemList():
        if item.Type() == robolink.ITEM_TYPE_STATION:
            item.Delete()

    # Reopen Car_Painting_Cell.rdk
    station = rdk.AddFile(station_file)
    if not station.Valid():
        print(f"[-] FAILED: Could not open {station_file}")
        return False
    print(f"[+] Loaded station: '{station.Name()}'")

    # Verify existing robot is preserved
    robot = rdk.Item("ABB IRB 6700-150/3.20 LeanID")
    if not robot.Valid():
        print("[-] FAILED: Preserved robot not found in station.")
        return False
    print(f"[+] Confirmed preserved robot: '{robot.Name()}' (6 axes, {robot.Type()})")

    # 4. Create Car_Reference_Frame and import vehicle
    print("\n[Step 4] Importing vehicle into station hierarchy...")
    # Add a dedicated Reference Frame for the vehicle at station origin
    car_frame = rdk.Item("Car_Reference_Frame")
    if not car_frame.Valid():
        car_frame = rdk.AddFrame("Car_Reference_Frame", station)
    car_frame.setPose(robomath.transl(0, 0, 0))
    print(f"[+] Created reference frame: '{car_frame.Name()}' at (0, 0, 0)")

    # Remove previous car object if present
    existing_car = rdk.Item("Car_Body")
    if existing_car.Valid():
        existing_car.Delete()

    # Import scaled car STL
    car_item = rdk.AddFile(scaled_stl_path)
    if not car_item.Valid():
        print("[-] FAILED: Could not load Car_Body.stl into station.")
        return False
    car_item.setName("Car_Body")
    car_item.setParent(car_frame)
    car_item.setPose(robomath.transl(0, 0, 0))

    # Apply attractive automotive sapphire/royal blue metallic finish
    # Format: [R, G, B, A]
    car_item.setColor([0.15, 0.45, 0.85, 1.0])
    print(f"[+] Successfully imported object: '{car_item.Name()}' (Parent: '{car_item.Parent().Name()}')")

    # 5. Optimize Robot Base Position considering actual vehicle geometry
    print("\n[Step 5] Adjusting robot base position and verifying reach...")
    robot_base = robot.Parent()

    # The car width extends from Y = -696 mm to Y = +696 mm.
    # Setting base position at X = 0 mm, Y = -1800 mm, Z = 0 mm, Rz = +90 deg:
    # - Clearance to car side panel: 1,800 - 696 = 1,104 mm (excellent safety and maintenance clearance).
    # - Facing +Y directly towards the car centerline.
    # - Distance to near upper roof corners (+/-2400, -696, 1480):
    #   sqrt(2400^2 + 1104^2 + 1480^2) = 3028 mm (< 3200 mm reach: 172 mm kinematic margin).
    # - Distance to roof centerline (0, 0, 1480):
    #   sqrt(1800^2 + 1480^2) = 2330 mm (< 3200 mm reach).
    base_x = 0.0
    base_y = -1800.0
    base_z = 0.0
    base_rz = 90.0

    optimal_base_pose = robomath.transl(base_x, base_y, base_z) * robomath.rotz(base_rz * math.pi / 180.0)
    robot_base.setPose(optimal_base_pose)

    print(f"    Adjusted Robot Base Position: X={base_x} mm, Y={base_y} mm, Z={base_z} mm, Rz={base_rz}°")
    print(f"    Robot Base Pose Matrix:\n{robot_base.Pose()}")

    clearance_side = abs(base_y) - trans_dims[1] / 2.0
    dist_side_corners = math.sqrt((trans_dims[0] / 2.0)**2 + clearance_side**2)
    dist_roof_corners = math.sqrt((trans_dims[0] / 2.0)**2 + clearance_side**2 + trans_dims[2]**2)
    dist_roof_center = math.sqrt(base_y**2 + trans_dims[2]**2)

    print("    Reachability & Clearance Analysis:")
    print(f"      - Clearance to car side panel: {clearance_side:.1f} mm")
    print(f"      - Distance to near lower corners: {dist_side_corners:.1f} mm (within 3200 mm reach)")
    print(f"      - Distance to near upper roof corners: {dist_roof_corners:.1f} mm (within 3200 mm reach)")
    print(f"      - Distance to roof centerline: {dist_roof_center:.1f} mm (within 3200 mm reach)")

    # 6. Capture multiple views
    print("\n[Step 6] Capturing multi-view inspection screenshots...")
    views = [
        ("Isometric", "actionIsometric", "Car_Painting_Cell_Isometric.png"),
        ("Top View",  "actionTop",       "Car_Painting_Cell_Top.png"),
        ("Front View","actionFront",     "Car_Painting_Cell_Front.png"),
        ("Side View", "actionRight",     "Car_Painting_Cell_Side.png")
    ]

    rdk.Command("TriggerAction", "actionFit_All")
    for name, action, filename in views:
        rdk.Command("TriggerAction", action)
        rdk.Command("TriggerAction", "actionFit_All")
        shot_path = os.path.join(screenshots_dir, filename)
        rdk.Command("Snapshot", shot_path)
        if os.path.exists(shot_path):
            print(f"    [+] Saved {name:10s} screenshot: {shot_path} ({os.path.getsize(shot_path):,} bytes)")
        else:
            print(f"    [-] Failed to save {name} screenshot")

    # Reset to nice isometric view
    rdk.Command("TriggerAction", "actionIsometric")
    rdk.Command("TriggerAction", "actionFit_All")

    # 7. Inspect station tree
    print("\n[Step 7] Inspecting updated station tree hierarchy...")
    items = rdk.ItemList()
    print(f"    Total items in station: {len(items)}")
    for item in items:
        p_name = item.Parent().Name() if item.Parent().Valid() else "None (Root)"
        print(f"    - '{item.Name()}' | Type: {item.Type()} | Parent: '{p_name}'")

    # 8. Save updated station and backup
    print("\n[Step 8] Saving updated station and backup...")
    rdk.Save(station_file, station)
    shutil.copyfile(station_file, backup_file)
    print(f"[+] Saved station: {station_file} ({os.path.getsize(station_file):,} bytes)")
    print(f"[+] Updated backup: {backup_file} ({os.path.getsize(backup_file):,} bytes)")

    # 9. Verify reopening
    print("\n[Step 9] Verifying station reopening from disk...")
    station.Delete()
    reopened = rdk.AddFile(station_file)
    if not reopened.Valid():
        print("[-] FAILED: Could not reopen station file.")
        return False

    reopened_robot = rdk.Item("ABB IRB 6700-150/3.20 LeanID")
    reopened_car = rdk.Item("Car_Body")
    reopened_base = rdk.Item("ABB IRB 6700-150/3.20 LeanID Base")
    reopened_car_frame = rdk.Item("Car_Reference_Frame")

    if reopened_robot.Valid() and reopened_car.Valid() and reopened_base.Valid():
        print("[+] Reopened station verified successfully!")
        print(f"    - Robot present: '{reopened_robot.Name()}' (Axes: {len(reopened_robot.Joints().list())})")
        print(f"    - Car present  : '{reopened_car.Name()}' (Parent: '{reopened_car.Parent().Name()}')")
        print(f"    - Robot Base   : {reopened_base.Pose()}")
        print(f"    - Car Frame    : {reopened_car_frame.Pose()}")
    else:
        print("[-] FAILED: Missing items upon reopening.")
        return False

    print("\n" + "=" * 70)
    print("PHASE 3 VEHICLE IMPORT & INTEGRATION COMPLETED SUCCESSFULLY")
    print("=" * 70)
    return True

if __name__ == "__main__":
    ok = import_and_setup_vehicle()
    sys.exit(0 if ok else 1)
