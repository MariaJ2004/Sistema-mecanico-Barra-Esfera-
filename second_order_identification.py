import serial

import time

import csv

import sys

import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from scipy.signal import savgol_filter

SERIAL_PORT = 'COM13'
BAUD_RATE = 115200
OUTPUT_FILENAME = 'arduino_data.csv'

def setup_serial_connection(port, baud):
    print(f"Trying to connect to {port} at {baud} baud...")
    try:
        ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)
        print("Serial connection established successfully!")
        return ser

    except serial.SerialException as e:

        print(f"\n--- CONNECTION ERROR ---\nFailed to open serial port {port}.")
        print("Possible causes:")
        print("1. The port does not exist or is misspelled (check SERIAL_PORT).")
        print("2. The Arduino is not connected.")
        print("3. Another program (such as the Arduino IDE Serial Monitor) is using the port.")
        print(f"Error details: {e}")
        sys.exit(1)

def log_data(ser, filename):
    try:
        file = open(filename, 'w', newline='')

    except IOError as e:

        print(f"Error opening file {filename}: {e}")

        return

    print(f"\nOutput file: {os.path.abspath(filename)}")

    print("Starting data logging. Press CTRL+C to stop.")

    try:

        header = ser.readline().decode('utf-8').strip()

        if not header:

             print("\nWarning: No header received. Make sure the Arduino is sending data.")
             header = "time_ms,angle_mpu,distance_cm"

        file.write(header + '\n')
        print(f"Header written: {header}")




    except Exception as e:

        print(f"Error reading/writing the header: {e}")

        file.close()

        ser.close()

        sys.exit(1)

    try:

        while True:

            if ser.in_waiting > 0:

                line = ser.readline().decode('utf-8').strip()

                if line and not line.startswith('time_ms'):

                    file.write(line + '\n')

                    file.flush()

                    print(f"Line read and saved: {line}")



    except KeyboardInterrupt:

        print("\nData logging stopped by the user (CTRL+C).")

    except Exception as e:

        print(f"\nAn unexpected error occurred: {e}")

    finally:

        print("Closing serial connection and file...")

        file.close()

        ser.close()



if __name__ == "__main__":
    print("Make sure the Arduino is loaded and ready.")
    serial_connection = setup_serial_connection(SERIAL_PORT, BAUD_RATE)

   

    if serial_connection:

        log_data(serial_connection, OUTPUT_FILENAME)
        print("Process finished.")

# ==========================================================
# 1. DEFINE THE MODEL FUNCTION (Second-Order Step Response)
# ==========================================================
#    This is the mathematical model we will fit to the data.
#    t = time, K = Gain, wn = Natural Frequency, zeta = Damping Ratio
#
#    Formula for the step response of a second-order underdamped system:
#    y(t) = K * [1 - (1 / sqrt(1 - zeta^2)) * exp(-zeta*wn*t) * sin(wd*t + phi)]
#    where wd = wn * sqrt(1 - zeta^2), phi = arccos(zeta)

def second_order_model(t, K, wn, zeta):
    if zeta < 1:  # Underdamped case
        wd = wn * np.sqrt(1 - zeta**2)
        phi = np.arccos(zeta)
        return K * (1 - (1 / np.sqrt(1 - zeta**2)) *
                    np.exp(-zeta * wn * t) *
                    np.sin(wd * t + phi))
    else:  # Overdamped approximation
        return K * (1 - np.exp(-wn * t))
data = pd.read_csv("arduino_data.csv", sep=',')

# Extract time and response
t_data = data["time_ms"].values / 1000   # Convert ms to seconds
y_noisy = data["distance_cm"].values  # Output variable

# ==========================================================
# 4. SMOOTH THE RAW DATA (OPTIONAL)
# ==========================================================
#    To make the signal more readable, we apply a Savitzky-Golay filter.
#    This preserves the shape while reducing noise.

y_smooth = savgol_filter(y_noisy, window_length=31, polyorder=3)

# ==========================================================
# 5. SELECT THE STEP RESPONSE REGION
# ==========================================================
#    We assume that the step input occurs at 3 seconds.
#    Only data after 3 seconds will be used for the identification.

idx = t_data >= 3.0
t_fit = t_data[idx] - 3.0   
y_fit = y_smooth[idx]

# ==========================================================
# 6. CURVE FITTING (MODEL IDENTIFICATION)
# ==========================================================
#    Use scipy's curve_fit to find K, wn, and zeta that best match the data.

initial_guess = [np.max(y_fit), 1.0, 0.5]
popt, pcov = curve_fit(second_order_model, t_fit, y_fit, p0=initial_guess, maxfev=10000)
K_fit, wn_fit, zeta_fit = popt

# Generate model output with identified parameters
y_model = second_order_model(t_fit, K_fit, wn_fit, zeta_fit)

# ==========================================================
# 7. CALCULATE ERROR METRICS (MSE, RMSE, R²)
# ==========================================================
#    These metrics evaluate how well the model matches the measured data.
#    - MSE: Mean Squared Error → average of squared differences.
#    - RMSE: Root Mean Squared Error → same units as data.
#    - R²: Coefficient of Determination → 1 means perfect fit.
residuals = y_fit - y_model
MSE = np.mean(residuals**2)
RMSE = np.sqrt(MSE)
SS_res = np.sum(residuals**2)
SS_tot = np.sum((y_fit - np.mean(y_fit))**2)
R2 = 1 - (SS_res / SS_tot)

# ==========================================================
# 8. PRINT AND PLOT RESULTS (EXTENDED MODEL TO t=0)
# ==========================================================
print("\n--- System Identification Results (Second-Order) ---")
print(f"Identified Parameters: K={K_fit:.3f}, wn={wn_fit:.3f}, zeta={zeta_fit:.3f}")
print("----------------------------------------------------")

plt.figure(figsize=(10, 6))

# Plot raw measured data (light blue points)
plt.plot(t_data - 3, y_noisy, 'o', color='g', alpha=0.5, markersize=4, label='Raw Measured Data')

# Plot smoothed experimental response
plt.plot(t_fit, y_fit, 'b-', linewidth=2, label='Smoothed Data')

# --- Extend the fitted model to cover entire time range ---
t_extended = np.linspace(0, np.max(t_fit), 400)
y_extended = second_order_model(t_extended, K_fit, wn_fit, zeta_fit)

# Plot the extended model curve
plt.plot(t_extended, y_extended, 'r--', linewidth=2.5,
         label=f'Fitted 2nd-Order Model\n(K={K_fit:.2f}, wn={wn_fit:.2f}, ζ={zeta_fit:.2f})\n'
               f'Model Error Metrics\n'
               f'(MSE={MSE:.4f}, RMSE={RMSE:.4f}, R2={R2:.4f})')
# Titles and labels
plt.title('Second-Order System Identification from Experimental Data', fontsize=14)
plt.xlabel('Time (s)', fontsize=12)
plt.ylabel('Response (Distance [cm])', fontsize=12)

# Grid and legend
plt.grid(True, linestyle='--', alpha=0.7)
plt.legend(fontsize=10, loc='best')
plt.xlim(0, np.max(t_data - 3))
plt.ylim(np.min(y_noisy)*0.9, np.max(y_noisy)*1.05)
plt.tight_layout()
plt.show()


