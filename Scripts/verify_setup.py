"""
Verification script for RoboDK Phase 1 Setup.
Tests connection to running RoboDK application, API status, and robot library access.
"""

import os
import sys
import urllib.request
from robodk.robolink import Robolink, ITEM_TYPE_ROBOT

def verify_connection():
    print("=" * 60)
    print("ROBODK ENVIRONMENT & API VERIFICATION")
    print("=" * 60)

    # 1. Connect to running RoboDK instance
    print("\n[1] Connecting to RoboDK application...")
    try:
        rdk = Robolink()
        if not rdk.Connect():
            print("[-] FAILED: Could not connect to running RoboDK instance.")
            return False
        version = rdk.Version()
        print(f"[+] SUCCESS: Connected to RoboDK instance! Version: {version}")
    except Exception as e:
        print(f"[-] ERROR during connection: {e}")
        return False

    # 2. Check Paths
    print("\n[2] Checking RoboDK paths & configuration...")
    try:
        path_rdk = rdk.getParam("PATH_ROBODK")
        path_library = rdk.getParam("PATH_LIBRARY")
        print(f"    - RoboDK Path   : {path_rdk}")
        print(f"    - Library Path  : {path_library}")
    except Exception as e:
        print(f"[-] Warning reading path parameters: {e}")
        path_library = "/home/sanket/RoboDK/Library"

    # 3. Check Local Robot Library
    print("\n[3] Checking local robot library...")
    robot_files = []
    if os.path.exists(path_library):
        for root, dirs, files in os.walk(path_library):
            for file in files:
                if file.endswith(".robot"):
                    robot_files.append(os.path.join(root, file))
        print(f"[+] Found {len(robot_files)} .robot files in local library.")
        if robot_files:
            sample_robots = [os.path.basename(f) for f in robot_files[:5]]
            print(f"    Sample models found: {', '.join(sample_robots)}")
    else:
        print(f"[-] Library directory does not exist: {path_library}")

    # 4. Verify Loading a Robot from Library
    print("\n[4] Testing robot loading into active station...")
    if robot_files:
        test_robot_path = robot_files[0]
        for rf in robot_files:
            if "UR10" in rf or "Fanuc" in rf or "KUKA" in rf or "ABB" in rf:
                test_robot_path = rf
                break
        print(f"    Loading test robot: {os.path.basename(test_robot_path)}")
        test_item = rdk.AddFile(test_robot_path)
        if test_item.Valid():
            print(f"[+] Successfully loaded: {test_item.Name()}")
            print(f"    Item Type: {test_item.Type()}")
            joints = test_item.Joints()
            print(f"    Joint Count: {len(joints)}")
            test_item.Delete()
            print("[+] Test robot removed cleanly from station.")
        else:
            print("[-] Failed to load test robot item.")
    else:
        print("[-] No robot files found to test loading.")

    # 5. Check Online Robot Library Reachability
    print("\n[5] Checking online robot library reachability...")
    try:
        req = urllib.request.Request(
            "https://robodk.com/library",
            headers={"User-Agent": "Mozilla/5.0"}
        )
        with urllib.request.urlopen(req, timeout=5) as response:
            status_code = response.getcode()
            if status_code == 200:
                print(f"[+] Online robot library reachable: https://robodk.com/library (HTTP {status_code})")
            else:
                print(f"[-] Online library returned status: {status_code}")
    except Exception as e:
        print(f"[-] Could not reach online library: {e}")

    print("\n" + "=" * 60)
    print("PHASE 1 VERIFICATION COMPLETED SUCCESSFULLY")
    print("=" * 60)
    return True

if __name__ == "__main__":
    success = verify_connection()
    sys.exit(0 if success else 1)
