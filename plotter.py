import serial
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
import time 

PORT = 'COM5'
BAUD_RATE = 115200

MAX_POINTS = 50

frame_id = 0
frameIDOffset = 0

tnow = 0
tnowOffset = 0

# Extra filtering (average of last 5) [NOT USING]
LAST_N = 5
filtered_values = []
alpha = 0.7

ser = serial.Serial(PORT, BAUD_RATE, timeout=1)

# address -> data
radars = {}

plt.ion()
fig, ax = plt.subplots()

ax.set_ylim(5, 30)
ax.yaxis.set_major_locator(MultipleLocator(1))
ax.xaxis.set_major_locator(MultipleLocator(5))
ax.set_xlabel("Sample")
ax.set_ylabel("Median + EMA Distance (inches)")
ax.set_title("XM125 Live Median(3) + EMA Distance")

text_display = ax.text(
    0.02, 0.95, "",
    transform=ax.transAxes,
    fontsize=10,
    verticalalignment="top"
)

print("Listening to serial...")

def get_radar(addr):
    if addr not in radars:
        line_filtered, = ax.plot([], [], 'g-', label=f"Radar {addr} Filtered")
        line_raw, = ax.plot([], [], 'r--', label=f"Radar {addr} Raw")
        # extra_line, = ax.plot([],[],'r--', label=f"Radar {addr} Hybrid")
        radars[addr] = {
            "frames": [],
            "distances_filtered": [],
            "distances_raw": [],
            # "extra_distances": [],
            "line_filtered": line_filtered,
            "line_raw": line_raw,
            # "extra_line": extra_line,
            "latest": None
        }
        ax.legend()

    return radars[addr]

log_file = open("data_logs/mobile_ground_dataset/mobile_16.txt", "a", buffering=1)  # line-buffered

while True:
    try:
        line_str = ser.readline().decode(errors="ignore").strip()
            

        if not line_str or not line_str[0].isdigit():
            continue

        parts = line_str.split(",")

        # CSV:
        # frame,address,loop_start,retCode,distances,raw_mm,filtered_mm,strength,...
        # trailing comma creates an empty last item, so accept 14 or 13
        if len(parts) < 16:
            continue

        frame_id = int(parts[0])+frameIDOffset
        
        addr = parts[1]
        
        retcode = int(parts[3])
        num_distances = int(parts[4])
        raw_mm = int(parts[5])
        filtered_mm = float(parts[6])
        filtered_in = float(parts[7])
        p0_str = int(parts[8])
        
        tnow = int(parts[14])+tnowOffset
        height_level = int(parts[15])

        # # Extra filtering before logging
        # if raw_mm < 0 or len(filtered_values) == 0:
        #     extra_filtered_mm = filtered_mm
        #     filtered_values.append(filtered_mm)
        # else:
        #     avg_n_filtered = sum(filtered_values[-LAST_N:]) / len(filtered_values[-LAST_N:])
        #     extra_filtered_mm = int(alpha * avg_n_filtered + (1 - alpha) * filtered_mm)
        #     filtered_values.append(filtered_mm)

        radar = get_radar(addr)

        radar["frames"].append(frame_id)
        radar["latest"] = filtered_in
        radar["distances_filtered"].append(filtered_in)
        radar["distances_raw"].append(raw_mm/25.4)
        # radar["extra_distances"].append(extra_filtered_mm)

        if len(radar["frames"]) > MAX_POINTS:
            radar["frames"] = radar["frames"][-MAX_POINTS:]
            radar["distances_filtered"] = radar["distances_filtered"][-MAX_POINTS:]
            radar["distances_raw"] = radar["distances_raw"][-MAX_POINTS:]
            # radar["extra_distances"] = radar["extra_distances"][-MAX_POINTS:]

        x = range(len(radar["distances_filtered"]))

        radar["line_filtered"].set_xdata(x)
        radar["line_raw"].set_xdata(x)
        radar["line_filtered"].set_ydata(radar["distances_filtered"])
        radar["line_raw"].set_ydata(radar["distances_raw"])

        # radar["extra_line"].set_xdata(x)
        # radar["extra_line"].set_ydata(radar["extra_distances"])

        display = ""

        if raw_mm <= 0 or not (150 <= raw_mm <= 1600):
            text_display.set_text(f"Radar {addr}: No Peak Detected")
        else:
            for a, r in radars.items():
                if r["latest"] is not None:
                    display += f"Radar {a}: {r['latest']:.2f} in\n"
            
            text_display.set_text(display.strip())


        ax.relim()
        ax.autoscale_view(scaley=False)

        plt.pause(0.01)
        if log_file.tell() == 0:
            log_file.write("frame id,address,retcode,numDistances,raw_mm,medianEMA_mm,medianEMA_in,p0 str,time now,LED status\n")
        log_file.write(f"{frame_id},{addr},{retcode},{num_distances},{raw_mm},{filtered_mm},{filtered_in},{p0_str},{tnow},{height_level}\n")

    except serial.SerialException:
        frameIDOffset=frame_id+1
        tnowOffset=tnow+1
        try:
            ser.close()
        except Exception:
            pass

        while True:
            try:
                ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
                ser.reset_input_buffer()
                print("Reconnected!")
                break
            except serial.SerialException:
                time.sleep(1)
    except KeyboardInterrupt:
        print("Stopped")
        break