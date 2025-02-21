# Full-Scale Launch Operation Procedure

> **Disclaimer**  
> Any test performed, including mishandling of electronics or causing software issues, is done at the operator’s own risk. The operator bears full responsibility for any failures or damages.

---

## Table of Contents
1. [Prerequisites](#prerequisites)
2. [General Steps to Connect](#general-steps-to-connect)
3. [Case 1: Run IMU Data with `MainDriver.py`](#case-1-run-imu-data-with-maindriverpy)
4. [Case 2: Test Post-Landing](#case-2-test-post-landing)
5. [Important](#important)

---

## Prerequisites
1. **Visual Studio Code (VS Code)** installed.
2. A **Web Browser** (to access your router settings).
3. **Internet/Wi-Fi** connectivity, ensuring both your Raspberry Pi and computer are on the same network.

---

## General Steps to Connect

1. **Open your web browser** and navigate to `http://192.168.8.1/`
   - **Router Password**: `VADL123456`
2. **Find the Raspberry Pi’s IP Address**  
   - Within the router interface, locate **Devices** or **Connected Devices** to see a list of connected clients.  
   - Look for the device named **Raspberry Pi** and note its IP address (e.g., `192.168.8.xxx`).
3. **Open VS Code**  
   - Confirm the **Remote - SSH** extension (or similar) is installed.
4. **Connect to Raspberry Pi via SSH**  
   - In the **bottom-left corner** of VS Code, click the “><” icon (or open **Remote Explorer**).  
   - Select **“Connect to Host…”**.  
   - Enter: `RPI4@[IP Address]` (substitute `[IP Address]` with the actual IP, e.g., `192.168.8.xxx`).  
   - When prompted for a password, use: `123`.
5. **Open the IMU Folder**  
   - In VS Code’s File Explorer, open the directory:  
     ```
     RocketTeam/IMU
     ```
   - Ensure that your **terminal** in VS Code is also in the `RocketTeam/IMU` folder.

---

## Operation Procedure

1. **Calibrate Altitude**  
   - In the `RocketTeam/IMU` directory, run:
     ```bash
     sudo python MainDriver.py
     ```
   - Observe the **altitude readings** in the terminal output.

2. **Stop the Program**  
   - Press **Ctrl + C** to terminate the script.

3. **Adjust Landing Parameters** (If Needed)  
   - Within the program, you may see parameters like:
     ```python
     landingAccMagThreshold = 25  # m/s^2
     groundLevel = 137.70         # Example ground level
     initialAltitudeThreshold = groundLevel - 10
     landingAltitudeThreshold = groundLevel + 5
     ```
   - **Modify `groundLevel`** (and associated thresholds) to suit your test scenario.

4. **Re-run the Program in the Background**  
   - When ready for continuous operation:
     ```bash
     sudo nohup python3 MainDriver.py >/dev/null 2>err.out & echo $! > script.pid
     ```
   - This command allows you to simulate or monitor post-landing conditions.

5. **Check If Running & Stop**
   ```bash
   pgrep -f MainDriver.py
   ```  
   ```bash
   kill -SIGTERM $(cat script.pid)
   ```

6. **Download IMU Data**  
   - Using VS Code’s File Explorer, **download** the relevant data files to your local machine.

---

## Important

- Handle electronics with extreme care.  
- If any hardware modifications or software edits lead to failures, the operator assumes **full responsibility** for damage or malfunction.

---

**End of Procedure**  
_These instructions have been provided in good faith to assist with IMU testing and data collection. The user/operator remains fully responsible for the safe handling and operation of all software and hardware components._
```
