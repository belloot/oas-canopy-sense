import serial
import time
from pathlib import Path

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets


PORT = "COM5"
BAUD_RATE = 115200

VISIBLE_POINTS = 50
LOG_PATH = Path("logfile.txt")

frame_id = 0
frameIDOffset = 0

tnow = 0
tnowOffset = 0

log_file = LOG_PATH.open("a", buffering=1)

FILTERED_PEN = pg.mkPen((180, 255, 255), width=2)  # whitish cyan
RAW_PEN = pg.mkPen((180, 255, 120), width=2)       # whitish green

if LOG_PATH.stat().st_size == 0:
    log_file.write(
        "frame id,address,retcode,numDistances,raw_mm,"
        "medianEMA_mm,medianEMA_in,p0 str,time now,LED status\n"
    )
    log_file.flush()


class SerialReader(QtCore.QThread):
    data_received = QtCore.pyqtSignal(object)
    status_received = QtCore.pyqtSignal(str)

    def run(self):
        global frame_id, frameIDOffset, tnow, tnowOffset

        ser = None

        while True:
            try:
                if ser is None or not ser.is_open:
                    ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
                    ser.reset_input_buffer()
                    self.status_received.emit("Connected!")

                line_str = ser.readline().decode(errors="ignore").strip()

                if not line_str or not line_str[0].isdigit():
                    continue

                parts = line_str.split(",")

                if len(parts) < 16:
                    continue

                frame_id = int(parts[0]) + frameIDOffset
                addr = parts[1]

                retcode = int(parts[3])
                distances = int(parts[4])
                raw_mm = int(parts[5])
                filtered_mm = float(parts[6])
                filtered_in = float(parts[7])
                p0_str = int(parts[8])

                tnow = int(parts[14]) + tnowOffset
                height_level = int(parts[15])

                log_file.write(
                    f"{frame_id},{addr},{retcode},{distances},"
                    f"{raw_mm},{filtered_mm},{filtered_in},{p0_str},"
                    f"{tnow},{height_level}\n"
                )
                log_file.flush()

                self.data_received.emit({
                    "frame_id": frame_id,
                    "addr": addr,
                    "raw_mm": raw_mm,
                    "filtered_mm": filtered_mm,
                })

            except (serial.SerialException, OSError, PermissionError) as e:
                frameIDOffset = frame_id + 1
                tnowOffset = tnow + 1

                self.status_received.emit(f"Serial disconnected: {e}")

                try:
                    if ser is not None:
                        ser.close()
                except Exception:
                    pass

                ser = None
                time.sleep(1)

            except ValueError:
                continue


app = QtWidgets.QApplication([])

radars = {}
latest_text = {}
status_message = "Disconnected"

win = pg.GraphicsLayoutWidget(show=True, title="XM125 Live Plot")
win.resize(1000, 600)

plot = win.addPlot(title="XM125 Live Plot")

plot.setLabel("bottom", "Sample")
plot.setLabel("left", "Median + EMA Distance", units="mm")
plot.getAxis("left").enableAutoSIPrefix(False)

plot.showGrid(x=True, y=True)

plot.enableAutoRange(axis="x", enable=False)
plot.enableAutoRange(axis="y", enable=True)
# plot.setAutoVisible(y=True)

label = pg.LabelItem(justify="left")
win.nextRow()
win.addItem(label)


def update_label():
    detection_lines = "<br>".join(latest_text.values())

    if not detection_lines:
        detection_lines = "Radar 0x51: No Peak Detected"

    label.setText(f"""
<font color="#B4FFFF">──── Radar 0x51 Filtered</font><br>
<font color="#B4FF78">──── Radar 0x51 Raw</font><br><br>
{status_message}<br>
{detection_lines}
""")


update_label()


def get_radar(addr):
    if addr not in radars:
        filtered_curve = plot.plot(
            [],
            [],
            pen=FILTERED_PEN,
        )

        raw_curve = plot.plot(
            [],
            [],
            pen=RAW_PEN,
        )

        radars[addr] = {
            "frames": [],
            "distances": [],
            "raw_distances": [],
            "curve": filtered_curve,
            "raw_curve": raw_curve,
            "latest": None,
        }

    return radars[addr]


def handle_data(d):
    addr = d["addr"]
    raw_mm = d["raw_mm"]
    filtered_mm = d["filtered_mm"]
    sample = d["frame_id"]

    radar = get_radar(addr)

    radar["frames"].append(sample)
    radar["distances"].append(filtered_mm)
    radar["raw_distances"].append(raw_mm)
    radar["latest"] = filtered_mm

    radar["curve"].setData(radar["frames"], radar["distances"])
    radar["raw_curve"].setData(radar["frames"], radar["raw_distances"])

    latest_frame = max(
        r["frames"][-1]
        for r in radars.values()
        if r["frames"]
    )

    left = max(0, latest_frame - VISIBLE_POINTS + 1)
    right = latest_frame + 1

    plot.setXRange(left, right, padding=0)

    if raw_mm <= 0 or not (150 <= raw_mm <= 1600):
        latest_text[addr] = f"Radar {addr}: No Peak Detected"
    else:
        latest_text[addr] = f"Radar {addr}: {filtered_mm:.1f} mm"

    update_label()


def handle_status(message):
    global status_message

    print(message)
    status_message = message
    update_label()


def shutdown():
    try:
        log_file.flush()
        log_file.close()
    except Exception:
        pass


app.aboutToQuit.connect(shutdown)

reader = SerialReader()
reader.data_received.connect(handle_data)
reader.status_received.connect(handle_status)
reader.start()

app.exec()