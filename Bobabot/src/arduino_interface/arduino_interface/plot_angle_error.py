import serial
import matplotlib.pyplot as plt
import time
import re
import datetime
import threading
import csv

# --- Configuration ---
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
NUM_SERVOS = 5

# Regex to match serial output
pattern = re.compile(r"A:([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+),"
                     r"([01]{8}),([01]{8}),([01]{8}),([01]{8}),([01]{8})")

# Global data cache
angles = [[] for _ in range(NUM_SERVOS)]
timestamps = []
target_angles = []

# Control flags
start_event = threading.Event()
stop_event = threading.Event()

def is_all_stopped(status_bits):
    return all(int(s, 2) & 0b00000001 == 1 for s in status_bits)

def is_any_started(status_bits):
    return any(int(s, 2) & 0b00000001 == 0 for s in status_bits)

def get_user_angles():
    while True:
        user_input = input("Enter 5 angles separated by commas (e.g., 10,20,30,40,50): ").strip()
        parts = user_input.split(',')
        if len(parts) != NUM_SERVOS:
            print("Invalid input format. Please try again.")
            continue
        try:
            angles_input = [float(a.strip()) for a in parts]
            return angles_input
        except ValueError:
            print("Angles must be numeric values.")

def read_serial(ser):
    print("Serial listener thread started.")
    t0 = None
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        match = pattern.match(line)
        if not match:
            continue

        vals = match.groups()
        status_bits = vals[NUM_SERVOS:]

        if not start_event.is_set() and is_any_started(status_bits):
            t0 = time.time()
            start_event.set()
            print("Servo movement started.")

        if start_event.is_set() and not stop_event.is_set():
            t = time.time() - t0
            timestamps.append(t)
            for i in range(NUM_SERVOS):
                angles[i].append(float(vals[i]))

            if is_all_stopped(status_bits):
                print("All servos stopped. Continuing to record for 1 second.")
                end_time = time.time() + 1.0
                while time.time() < end_time:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    match = pattern.match(line)
                    if match:
                        vals = match.groups()
                        t = time.time() - t0
                        timestamps.append(t)
                        for i in range(NUM_SERVOS):
                            angles[i].append(float(vals[i]))
                stop_event.set()
                print("Recording complete.")

        if stop_event.is_set():
            break

def save_csv(filename_csv):
    with open(filename_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time'] + [f'servo_{i+1}' for i in range(NUM_SERVOS)])
        for j in range(len(timestamps)):
            row = [timestamps[j]] + [angles[i][j] for i in range(NUM_SERVOS)]
            writer.writerow(row)
    print(f"Data saved to: {filename_csv}")

def plot_results(timestamp_str, timestamps, angles, target_angles, NUM_SERVOS):
    plt.style.use('seaborn-darkgrid')
    plt.rcParams.update({'font.size': 12})

    # Servo angle vs. time plot
    plt.figure(figsize=(10, 6))
    for i in range(NUM_SERVOS):
        plt.plot(timestamps, angles[i], label=f"Servo {i+1}")
        plt.hlines(y=target_angles[i], xmin=0, xmax=timestamps[-1],
                   linestyle=':', color=plt.gca().lines[i].get_color(),
                   alpha=0.5, label=f"Target {i+1}")

    plt.axvline(x=timestamps[0], color='gray', linestyle='--', label='Start')
    plt.axvline(x=timestamps[-1], color='black', linestyle='--', label='Stop')

    plt.xlabel("Time (s)")
    plt.ylabel("Angle (deg)")
    plt.title("Servo Angle vs Time")
    plt.legend(loc='upper right')
    plt.grid(True)
    filename_pos = f"servo_plot_{timestamp_str}.png"
    plt.savefig(filename_pos, dpi=300, bbox_inches='tight')
    print(f"Angle plot saved to: {filename_pos}")
    plt.show()

    # Error plot
    plt.figure(figsize=(10, 6))
    for i in range(NUM_SERVOS):
        error = [target_angles[i] - a for a in angles[i]]
        plt.plot(timestamps, error, label=f"Error Servo {i+1}")
        final_err = target_angles[i] - angles[i][-1]
        plt.plot(timestamps[-1], final_err, 'o',
                label=f"Final Error {i+1}: {final_err:.2f}°")

    plt.axhline(y=0, color='gray', linestyle='--')
    plt.xlabel("Time (s)")
    plt.ylabel("Angle Error (deg)")
    plt.title("Servo Tracking Error")
    plt.legend(loc='upper right')
    plt.grid(True)
    filename_err = f"servo_error_plot_{timestamp_str}.png"
    plt.savefig(filename_err, dpi=300, bbox_inches='tight')
    print(f"Error plot saved to: {filename_err}")
    plt.show()

    # Boxplot of error distribution
    plt.figure(figsize=(8, 5))
    error_data = [[target_angles[i] - a for a in angles[i]] for i in range(NUM_SERVOS)]
    plt.boxplot(error_data, labels=[f"Servo {i+1}" for i in range(NUM_SERVOS)])
    plt.ylabel("Error (deg)")
    plt.title("Angle Error Distribution (Boxplot)")
    plt.grid(True, axis='y')
    filename_box = f"servo_error_boxplot_{timestamp_str}.png"
    plt.savefig(filename_box, dpi=300, bbox_inches='tight')
    print(f"Boxplot saved to: {filename_box}")
    plt.show()

def main():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print("Serial port opened successfully.")

    # Start listener thread
    thread = threading.Thread(target=read_serial, args=(ser,))
    thread.start()

    global target_angles
    target_angles = get_user_angles()
    angle_str = ','.join([str(a) for a in target_angles])
    ser.write((angle_str + '\n').encode())
    ser.flush()
    print(f"Target angles sent: {angle_str}")

    print("Waiting for servo movement to start...")
    start_event.wait()
    stop_event.wait()
    ser.close()

    print("Generating plots and saving data...")

    timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    plot_results(timestamp_str, timestamps, angles, target_angles, NUM_SERVOS)
    save_csv(f"servo_data_{timestamp_str}.csv")

if __name__ == "__main__":
    main()
