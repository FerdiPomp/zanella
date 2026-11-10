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
- A YAML configuration file configured for each machine.
- IP connectivity from A/B to C on `SERVER_PORT` and from C to A/B on their `WORK_STATE_PORT` values.

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

`config.yaml` is specific to the station where the program is deployed. It is loaded by default; a different file can be selected with `--config <file_path>`. Start each station configuration by copying the complete default file. The original configuration keys are mandatory; `LED_SOCKET_PATH`, `WORK_STATE_PORT`, `TABLE_NODE_ENDPOINTS`, `RUNTIME_ROOT`, and `CAMERA_SERIAL_NUMBER` are optional for compatibility with older files. Unknown keys are rejected to prevent configuration typos.

Configure at least:

```yaml
# Station camera
IS_ZED: false                # True on C with ZED2/ZED X One, or on A/B with ZED X Mini
ZED_ENV_HAS_DEPTH: true      # C only: False for a monocular ZED X One 4K
ARUCO_MODE: false            # Must be True on C when ZED_ENV_HAS_DEPTH is false
CAMERA_SERIAL_NUMBER: ""      # A/B live camera serial; quote it, even when numeric

# Local peripherals
THERE_IS_LED: true           # Tables with an LED
LED_SOCKET_PATH: ""          # Empty: local LED; shared box: same absolute socket path on A and B
THERE_IS_BUTTON: true        # Only table B when its button is installed

# Tables -> environmental station
SERVER_URL: "<environmental_station_ip>"
SERVER_PORT: 9000
WORK_STATE_PORT: 9000        # A/B local /work_state receiver; distinct ports on one box

# Node C only: IP addresses of nodes A and B
TABLE_NODE_IPS: ["<table_A_ip>", "<table_B_ip>"]
TABLE_NODE_ENDPOINTS: {}     # C: use A/B host+port mappings instead when sharing one IP
WORK_STATE_SYNC_INTERVAL: 1

# Node C only, when MQTT is enabled
ONLINE_SENDER_ENV: true
BROKER_IP: "<broker>"
MQTT_PORT: 443

# Daily camera recovery on every node (local system time)
NIGHT_RECOVERY_ENABLED: true
NIGHT_RECOVERY_HOUR: 0
NIGHT_RECOVERY_MINUTE: 0
NIGHT_RECOVERY_DURATION_SECONDS: 900

# All writable files; use distinct absolute roots for A and B on one box
RUNTIME_ROOT: "."
```

Configure ROI values, depth thresholds, shape thresholds, and debounce values according to camera height, lighting, and the physical table. YAML ROI values are lists and are converted to the tuples used internally by the application. On C, provide either `TABLE_NODE_IPS` for the legacy common port or `TABLE_NODE_ENDPOINTS` with explicit ports; the environmental node exits at startup if both are empty.

### Tables with ZED X Mini

For A or B with a ZED X Mini, set `IS_ZED=True`. The node uses the ZED SDK point cloud in metres and the same plane, height-map, shape and debounce algorithm used by the RealSense backend. It opens the camera at the native resolution selected by ZED SDK; it does not force the ZED2 resolution used by C.

Set the existing ROI of the physical table in the configuration deployed to that station before starting it:

```yaml
ROI_A: [<x0>, <y0>, <x1>, <y1>] # Node A
ROI_B: [<x0>, <y0>, <x1>, <y1>] # Node B
```

Coordinates must fall within the acquired native resolution. Because `config.yaml` is deployed per station, `ROI_A` and `ROI_B` contain the coordinates calibrated for the camera installed on that specific node, whether it is RealSense or ZED X Mini. Verify `PLANE_THRESHOLD`, height thresholds, `MIN_AREA_PIXELS` and the expected-shape set with the installed camera. ZED X Mini requires a compatible Jetson host, ZED Link capture hardware and ZED SDK 4.0 or later.

### Environmental ZED X One 4K

Node C also supports a monocular ZED X One 4K without creating a separate camera backend. Configure its station-specific `config.yaml` as follows:

```yaml
IS_ZED: true
ZED_ENV_HAS_DEPTH: false
ARUCO_MODE: true
```

The camera uses the ZED SDK monocular `CameraOne` API; it has no depth mode, plane or point cloud. With no DataMatrix, a visible ArUco placeholder means normal visibility and permits `QR_REMOVED`; if both the DataMatrix and ArUco are absent, the reader reports occlusion and leaves the QR FSM unchanged. This mode is rejected at startup unless OpenCV ArUco support and a ZED SDK exposing `CameraOne` are installed. `ZED_ENV_HAS_DEPTH=False` is valid only on node C.

## Running

Run one instance per station, plus the LED controller when A and B share a GPIO pin.

Incoming table:

```bash
python3 main.py --node_id A
```

With a station-specific file:

```bash
python3 main.py --node_id A --config configs/tavolo-ingresso.yaml
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

For playback from a camera-backend-supported file, add `--file_bag <file_path>`. Playback selects the file and ignores `CAMERA_SERIAL_NUMBER`.

### A and B on the same ZED Box

Run two separate processes from the same repository directory with two separate configuration files. Both send events to C at its `SERVER_URL:SERVER_PORT`. Each table listens for C on its own `WORK_STATE_PORT`, uses a distinct `RUNTIME_ROOT`, and selects its physical camera with `CAMERA_SERIAL_NUMBER`. A RealSense and a ZED X Mini can be mixed; set `IS_ZED` independently in each file. Obtain the actual serials with `rs-enumerate-devices -S` or `ZED_Explorer -a`; do not reuse a serial between configurations. The serial remains selected after camera reopen and nightly recovery.

For example, in A's configuration:

```yaml
IS_ZED: true                 # Or false for a RealSense
CAMERA_SERIAL_NUMBER: "<A_camera_serial>"
WORK_STATE_PORT: 9101
RUNTIME_ROOT: "/var/lib/tracevision/A"
NIGHT_RECOVERY_HOUR: 0
NIGHT_RECOVERY_MINUTE: 0
```

In B's configuration:

```yaml
IS_ZED: false                # Or true for a ZED X Mini
CAMERA_SERIAL_NUMBER: "<B_camera_serial>"
WORK_STATE_PORT: 9102
RUNTIME_ROOT: "/var/lib/tracevision/B"
NIGHT_RECOVERY_HOUR: 0
NIGHT_RECOVERY_MINUTE: 15
```

In C's configuration, address both receivers explicitly; replace `<box_ip>` with the ZED Box address:

```yaml
TABLE_NODE_IPS: []
TABLE_NODE_ENDPOINTS:
  A: {host: "<box_ip>", port: 9101}
  B: {host: "<box_ip>", port: 9102}
```

The port and recovery times above are examples; configure unused ports and the intended local schedule. Keep the process working directory at the repository root so `expected_shape/` and `certificate.crt` remain readable. `RUNTIME_ROOT` changes where the program writes `.runtime/` queues, saved planes, deduplication markers, and debug recordings. Give A and B separate writable roots. A second process using the same root fails at startup. Before switching an existing station, inspect and move any pending `.runtime/http_out_<node>` events and saved `table_plane_<node>.json` into its new root; changing the path does not migrate them automatically. An occupied receiver port now fails at startup.

### One LED shared by A and B

When A and B run on the same ZED Box and share one GPIO LED, set `THERE_IS_LED: true` and the same absolute `LED_SOCKET_PATH` (for example `/run/tracevision/led.sock`) in both table configurations. Set `THERE_IS_BUTTON: true` only on B if its button is installed. Start one controller on the box, using a configuration with the same socket path, `THERE_IS_LED: true`, and the physical `LED_PIN`:

```bash
python3 main.py --node_id LED --config configs/tavolo-ingresso.yaml
```

The controller is the only process that opens the LED GPIO. A and B send their object and button LED states over the local socket; neither table process opens the LED pin. Run the controller as its own supervised service and start it before the table processes. It opens no camera and no HTTP port. The socket directory must be writable by the controller, and the table processes must be able to send to the socket. If the controller is unavailable, the tables keep detecting events and retry their LED state every second. A source that stops reporting is ignored after three seconds; the controller turns the LED off on shutdown.

The physical output follows `BLINK > ON > OFF`:

| A request / B request | OFF | ON | BLINK |
| --- | --- | --- | --- |
| OFF | OFF | ON | BLINK |
| ON | ON | ON | BLINK |
| BLINK | BLINK | BLINK | BLINK |

Node B's object and button are tracked separately. A blink lasts nine seconds, as before; afterwards the output returns to `ON` if another source still requests it. The single LED cannot identify which table caused a blink.

With `LED_SOCKET_PATH` set, A and B must also configure a separate `RUNTIME_ROOT` and a camera serial for live capture. Start the single LED controller with either table configuration before starting A and B.

## Behaviour

1. Node `C` reads the DataMatrix and generates `QR_APPEND`.
2. Node `C` sends `work_state=true` to the tables.
3. With `work_state=true`, tables send item events and keep the LED steadily on.
4. With `work_state=false`, detected objects and button presses do not generate events and the LED blinks.
5. When DataMatrix removal is confirmed, `C` sends `work_state=false` and generates `QR_REMOVED`.

The environmental station periodically retransmits the state to bring tables back in sync after a reboot or network interruption.

## Daily Camera Recovery

At the configured local time (00:00 by default), each node performs one local recovery cycle. It first logs an anomaly if its `work_state` is `true`, forces it to `false`, and logs and discards any pending items in its own persistent HTTP queue. HTTP loops remain active throughout; on node C, the existing periodic state synchronization therefore continues to send `false` to the tables.

On depth cameras, the node saves the current table plane to `<RUNTIME_ROOT>/.runtime/table_plane_<node_id>.json`, closes the camera for `NIGHT_RECOVERY_DURATION_SECONDS` (15 minutes by default), then opens it again. Whenever a saved plane exists, it is reused without RANSAC; if the file is absent, the normal initial calibration is used instead. A ZED X One 4K uses no plane or point cloud. Detection and DataMatrix FSM state is reset locally without emitting a `QR_REMOVED` event. A process that starts or restarts during its configured recovery window does not run a recovery retroactively.

The schedule uses the station's local system clock: keep it synchronized and configured for the intended local time zone.

## Event Transport

- Tables to environmental station: `POST /event`.
- Environmental station to tables: `POST /work_state` on each configured `WORK_STATE_PORT`.
- HTTP queues are persisted under `<RUNTIME_ROOT>/.runtime/`, retried automatically, and deduplicated through `Event-ID`.
- The environmental station builds the `EventXLayer` payload and publishes it through MQTT.

`engine/event.py` and `engine/schema.json` define the MQTT contract. Do not modify them without verifying server-side validation.

## Debugging And Shutdown

With `DEBUGGING=True`, individual RGB images are saved as JPEG files, detection sequences are saved as `.npy` files, and file/log names use the `YYYYMMDD_HHMMSS` format.

Stop the process with `Ctrl+C` or `SIGTERM`; the node stops workers and closes its HTTP receiver to release the port.
