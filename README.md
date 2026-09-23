# Tracevision Workplace Monitor

Distributed vision system for detecting production events across three stations: incoming table, outgoing table, and environmental station. The environmental station reads the DataMatrix, receives events from the tables, correlates them, and publishes them through MQTT.

See [context.md](context.md) for the complete technical description in Italian.

## Roles

| `node_id` | Station | Camera | Function |
| --- | --- | --- | --- |
| `A` | Incoming table | RealSense D455 or ZED X Mini | Sends `ENTER_DETECT` |
| `B` | Outgoing table | RealSense D455 or ZED X Mini | Sends `EXIT_DETECT` and, when enabled, `BUTTON_PRESSED` |
| `C` | Environmental | ZED2, ZED X One 4K, or RealSense | Reads the DataMatrix, synchronizes the tables, and publishes MQTT events |

## Prerequisites

- Python 3.
- Camera drivers and SDK installed on the relevant station.
- A `config.py` copy configured for each machine.
- IP connectivity between nodes on `SERVER_PORT`.

Install common dependencies:

```bash
python3 -m pip install -r requirements.txt
```

Install the dependencies required by the configured hardware as well:

| Condition | Dependency |
| --- | --- |
| RealSense camera | `pyrealsense2` |
| ZED2, ZED X Mini, or ZED X One 4K camera | ZED SDK and `pyzed.sl` |
| DataMatrix decoding on node C | `pylibdmtx` and system `libdmtx` |
| LED or button | `gpiod` |
| HTTP receiver | `Flask` |
| HTTP sender | `requests` |
| MQTT | `paho-mqtt` |

The program validates required dependencies at startup and exits if the configured hardware and installed libraries are inconsistent.

## Configuration

`config.py` is specific to the station where the program is deployed. Configure at least:

```python
# Station camera
IS_ZED = False               # True on C with ZED2/ZED X One, or on A/B with ZED X Mini
ZED_ENV_HAS_DEPTH = True     # C only: False for a monocular ZED X One 4K
ARUCO_MODE = False           # Must be True on C when ZED_ENV_HAS_DEPTH is False

# Local peripherals
THERE_IS_LED = True          # Tables with an LED
THERE_IS_BUTTON = True       # Only table B when its button is installed

# Tables -> environmental station
SERVER_URL = "<environmental_station_ip>"
SERVER_PORT = 9000

# Node C only: IP addresses of nodes A and B
TABLE_NODE_IPS = ("<table_A_ip>", "<table_B_ip>")
WORK_STATE_SYNC_INTERVAL = 1

# Node C only, when MQTT is enabled
ONLINE_SENDER_ENV = True
BROKER_IP = "<broker>"
MQTT_PORT = 443

# Daily camera recovery on every node (local system time)
NIGHT_RECOVERY_ENABLED = True
NIGHT_RECOVERY_HOUR = 0
NIGHT_RECOVERY_MINUTE = 0
NIGHT_RECOVERY_DURATION_SECONDS = 15 * 60
```

Configure ROI values, depth thresholds, shape thresholds, and debounce values according to camera height, lighting, and the physical table. `TABLE_NODE_IPS` is mandatory on node `C`; the environmental node exits at startup if it is empty.

### Tables with ZED X Mini

For A or B with a ZED X Mini, set `IS_ZED=True`. The node uses the ZED SDK point cloud in metres and the same plane, height-map, shape and debounce algorithm used by the RealSense backend. It opens the camera at the native resolution selected by ZED SDK; it does not force the ZED2 resolution used by C.

Set the existing ROI of the physical table in the configuration deployed to that station before starting it:

```python
ROI_A = (<x0>, <y0>, <x1>, <y1>)  # Node A
ROI_B = (<x0>, <y0>, <x1>, <y1>)  # Node B
```

Coordinates must fall within the acquired native resolution. Because `config.py` is deployed per station, `ROI_A` and `ROI_B` contain the coordinates calibrated for the camera installed on that specific node, whether it is RealSense or ZED X Mini. Verify `PLANE_THRESHOLD`, height thresholds, `MIN_AREA_PIXELS` and the expected-shape set with the installed camera. ZED X Mini requires a compatible Jetson host, ZED Link capture hardware and ZED SDK 4.0 or later.

### Environmental ZED X One 4K

Node C also supports a monocular ZED X One 4K without creating a separate camera backend. Configure its station-specific `config.py` as follows:

```python
IS_ZED = True
ZED_ENV_HAS_DEPTH = False
ARUCO_MODE = True
```

The camera uses the ZED SDK monocular `CameraOne` API; it has no depth mode, plane or point cloud. With no DataMatrix, a visible ArUco placeholder means normal visibility and permits `QR_REMOVED`; if both the DataMatrix and ArUco are absent, the reader reports occlusion and leaves the QR FSM unchanged. This mode is rejected at startup unless OpenCV ArUco support and a ZED SDK exposing `CameraOne` are installed. `ZED_ENV_HAS_DEPTH=False` is valid only on node C.

## Running

Run one instance per station.

Incoming table:

```bash
python3 main.py --node_id A
```

Outgoing table:

```bash
python3 main.py --node_id B
```

Environmental station:

```bash
python3 main.py \
  --node_id C \
  --workspace PIPE_CUT \
  --mqtt_psw '<password>' \
  --topic 'workplace40/Tracevision'
```

For playback from a camera-backend-supported file, add `--file_bag <file_path>`.

## Behaviour

1. Node `C` reads the DataMatrix and generates `QR_APPEND`.
2. Node `C` sends `work_state=true` to the tables.
3. With `work_state=true`, tables send item events and keep the LED steadily on.
4. With `work_state=false`, detected objects and button presses do not generate events and the LED blinks.
5. When DataMatrix removal is confirmed, `C` sends `work_state=false` and generates `QR_REMOVED`.

The environmental station periodically retransmits the state to bring tables back in sync after a reboot or network interruption.

## Daily Camera Recovery

At the configured local time (00:00 by default), each node performs one local recovery cycle. It first logs an anomaly if its `work_state` is `true`, forces it to `false`, and logs and discards any pending items in its own persistent HTTP queue. HTTP loops remain active throughout; on node C, the existing periodic state synchronization therefore continues to send `false` to the tables.

On depth cameras, the node saves the current table plane to `.runtime/table_plane_<node_id>.json`, closes the camera for `NIGHT_RECOVERY_DURATION_SECONDS` (15 minutes by default), then opens it again. Whenever a saved plane exists, it is reused without RANSAC; if the file is absent, the normal initial calibration is used instead. A ZED X One 4K uses no plane or point cloud. Detection and DataMatrix FSM state is reset locally without emitting a `QR_REMOVED` event. A process that starts or restarts during the 00:00--00:15 recovery window does not run a recovery retroactively.

The schedule uses the station's local system clock: keep it synchronized and configured for the intended local time zone.

## Event Transport

- Tables to environmental station: `POST /event`.
- Environmental station to tables: `POST /work_state`.
- HTTP queues are persisted under `.runtime/`, retried automatically, and deduplicated through `Event-ID`.
- The environmental station builds the `EventXLayer` payload and publishes it through MQTT.

`engine/event.py` and `engine/schema.json` define the MQTT contract. Do not modify them without verifying server-side validation.

## Debugging And Shutdown

With `DEBUGGING=True`, individual RGB images are saved as JPEG files, detection sequences are saved as `.npy` files, and file/log names use the `YYYYMMDD_HHMMSS` format.

Stop the process with `Ctrl+C` or `SIGTERM`; the node stops workers and closes its HTTP receiver to release the port.
