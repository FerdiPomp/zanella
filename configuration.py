from pathlib import Path
import math

try:
    import yaml
except ModuleNotFoundError:
    yaml = None


CONFIG_KEYS = {
    "IS_ZED",
    "ZED_ENV_HAS_DEPTH",
    "ARUCO_MODE",
    "DEBUGGING",
    "ROI",
    "ROI_A",
    "ROI_B",
    "ROI_QR",
    "ROI_QR_ZED",
    "PLANE_THRESHOLD",
    "THRESH_STD",
    "SHAPE_THRESHOLD",
    "MIN_HEIGHT_THRESHOLD",
    "MAX_HEIGHT_THRESHOLD",
    "MIN_AREA_PIXELS",
    "MAX_OCCUPATION",
    "MAX_LEN",
    "OCCLUSION_THRESHOLD",
    "ZED_GRAB_RETRY_COUNT",
    "ZED_GRAB_RETRY_DELAY",
    "BUTTON_PIN",
    "LED_PIN",
    "THERE_IS_LED",
    "THERE_IS_BUTTON",
    "IS_JETSON",
    "NO_OBJECT",
    "OBJECT_PRESENT",
    "NO_PRESS",
    "PRESSED",
    "NO_QR",
    "QR",
    "OBJ_MIN_OFF_TIME",
    "BUT_MIN_ON_TIME",
    "BUT_MIN_OFF_TIME",
    "QR_MIN_ON_TIME",
    "QR_MIN_OFF_TIME",
    "QR_MIN_CHANGE_TIME",
    "ONLINE_SENDER",
    "ONLINE_RECIEVER",
    "ONLINE_SENDER_ENV",
    "NIGHT_RECOVERY_ENABLED",
    "NIGHT_RECOVERY_HOUR",
    "NIGHT_RECOVERY_MINUTE",
    "NIGHT_RECOVERY_DURATION_SECONDS",
    "SERVER_URL",
    "SERVER_PORT",
    "TABLE_NODE_IPS",
    "WORK_STATE_SYNC_INTERVAL",
    "BROKER_IP",
    "MQTT_PORT",
}

OPTIONAL_KEYS = {
    "LED_SOCKET_PATH",
    "WORK_STATE_PORT",
    "TABLE_NODE_ENDPOINTS",
    "RUNTIME_ROOT",
    "CAMERA_SERIAL_NUMBER",
}

BOOLEAN_KEYS = {
    "IS_ZED",
    "ZED_ENV_HAS_DEPTH",
    "ARUCO_MODE",
    "DEBUGGING",
    "THERE_IS_LED",
    "THERE_IS_BUTTON",
    "IS_JETSON",
    "ONLINE_SENDER",
    "ONLINE_RECIEVER",
    "ONLINE_SENDER_ENV",
    "NIGHT_RECOVERY_ENABLED",
}

INTEGER_KEYS = {
    "MIN_AREA_PIXELS",
    "MAX_OCCUPATION",
    "MAX_LEN",
    "OCCLUSION_THRESHOLD",
    "ZED_GRAB_RETRY_COUNT",
    "BUTTON_PIN",
    "LED_PIN",
    "NO_OBJECT",
    "OBJECT_PRESENT",
    "NO_PRESS",
    "PRESSED",
    "NO_QR",
    "QR",
    "NIGHT_RECOVERY_HOUR",
    "NIGHT_RECOVERY_MINUTE",
    "SERVER_PORT",
    "MQTT_PORT",
}

NUMBER_KEYS = {
    "PLANE_THRESHOLD",
    "THRESH_STD",
    "SHAPE_THRESHOLD",
    "MIN_HEIGHT_THRESHOLD",
    "MAX_HEIGHT_THRESHOLD",
    "ZED_GRAB_RETRY_DELAY",
    "OBJ_MIN_OFF_TIME",
    "BUT_MIN_ON_TIME",
    "BUT_MIN_OFF_TIME",
    "QR_MIN_ON_TIME",
    "QR_MIN_OFF_TIME",
    "QR_MIN_CHANGE_TIME",
    "NIGHT_RECOVERY_DURATION_SECONDS",
    "WORK_STATE_SYNC_INTERVAL",
}

ROI_KEYS = {"ROI", "ROI_A", "ROI_B", "ROI_QR", "ROI_QR_ZED"}
STRING_KEYS = {"SERVER_URL", "BROKER_IP"}

CONFIG_PATH = None
_loaded = False


def _is_integer(value):
    return isinstance(value, int) and not isinstance(value, bool)


def _is_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _validate_configuration(values, path: Path):
    if not isinstance(values, dict):
        raise ValueError(f"Configuration file {path} must contain a YAML mapping")

    missing = CONFIG_KEYS - values.keys()
    unknown = values.keys() - CONFIG_KEYS - OPTIONAL_KEYS
    if missing or unknown:
        details = []
        if missing:
            details.append(f"missing keys: {', '.join(sorted(missing))}")
        if unknown:
            details.append(f"unknown keys: {', '.join(sorted(unknown))}")
        raise ValueError(f"Invalid configuration in {path}: {'; '.join(details)}")

    for key in BOOLEAN_KEYS:
        if not isinstance(values[key], bool):
            raise ValueError(f"Invalid configuration in {path}: {key} must be boolean")

    for key in INTEGER_KEYS:
        if not _is_integer(values[key]):
            raise ValueError(f"Invalid configuration in {path}: {key} must be an integer")

    for key in NUMBER_KEYS:
        if not _is_number(values[key]):
            raise ValueError(f"Invalid configuration in {path}: {key} must be a finite number")

    for key in STRING_KEYS:
        if not isinstance(values[key], str) or not values[key]:
            raise ValueError(f"Invalid configuration in {path}: {key} must be a non-empty string")

    led_socket_path = values.get("LED_SOCKET_PATH", "")
    if not isinstance(led_socket_path, str):
        raise ValueError(f"Invalid configuration in {path}: LED_SOCKET_PATH must be a string")
    if led_socket_path and not Path(led_socket_path).is_absolute():
        raise ValueError(f"Invalid configuration in {path}: LED_SOCKET_PATH must be absolute")

    runtime_root = values.get("RUNTIME_ROOT", ".")
    if not isinstance(runtime_root, str) or not runtime_root.strip() or runtime_root != runtime_root.strip():
        raise ValueError(f"Invalid configuration in {path}: RUNTIME_ROOT must be a non-empty path")

    camera_serial = values.get("CAMERA_SERIAL_NUMBER", "")
    if not isinstance(camera_serial, str):
        raise ValueError(f"Invalid configuration in {path}: CAMERA_SERIAL_NUMBER must be a quoted string")
    if camera_serial != camera_serial.strip():
        raise ValueError(f"Invalid configuration in {path}: CAMERA_SERIAL_NUMBER must not contain surrounding spaces")
    if camera_serial and values["IS_ZED"] and (not camera_serial.isascii() or not camera_serial.isdigit() or int(camera_serial) <= 0):
        raise ValueError(f"Invalid configuration in {path}: ZED CAMERA_SERIAL_NUMBER must be a positive integer string")

    for key in ROI_KEYS:
        roi = values[key]
        if not isinstance(roi, list) or len(roi) != 4 or not all(_is_integer(value) for value in roi):
            raise ValueError(f"Invalid configuration in {path}: {key} must contain four integer coordinates")
        if roi[0] >= roi[2] or roi[1] >= roi[3]:
            raise ValueError(f"Invalid configuration in {path}: {key} coordinates are invalid")

    table_node_ips = values["TABLE_NODE_IPS"]
    if not isinstance(table_node_ips, list) or not all(isinstance(value, str) and value for value in table_node_ips):
        raise ValueError(f"Invalid configuration in {path}: TABLE_NODE_IPS must be a list of non-empty strings")
    if len(set(table_node_ips)) != len(table_node_ips):
        raise ValueError(f"Invalid configuration in {path}: duplicate table IPs require TABLE_NODE_ENDPOINTS with separate ports")

    table_node_endpoints = values.get("TABLE_NODE_ENDPOINTS", {})
    if not isinstance(table_node_endpoints, dict):
        raise ValueError(f"Invalid configuration in {path}: TABLE_NODE_ENDPOINTS must be a mapping")
    if table_node_endpoints:
        if table_node_ips:
            raise ValueError(f"Invalid configuration in {path}: use TABLE_NODE_ENDPOINTS or TABLE_NODE_IPS, not both")
        if set(table_node_endpoints) != {"A", "B"}:
            raise ValueError(f"Invalid configuration in {path}: TABLE_NODE_ENDPOINTS must contain A and B")
        destinations = []
        for node_id, endpoint in table_node_endpoints.items():
            if not isinstance(endpoint, dict) or set(endpoint) != {"host", "port"}:
                raise ValueError(f"Invalid configuration in {path}: endpoint {node_id} needs host and port")
            host, port = endpoint["host"], endpoint["port"]
            if not isinstance(host, str) or not host or host != host.strip() or not _is_integer(port) or not 1 <= port <= 65535:
                raise ValueError(f"Invalid configuration in {path}: endpoint {node_id} has invalid host or port")
            destinations.append((host, port))
        if len(set(destinations)) != len(destinations):
            raise ValueError(f"Invalid configuration in {path}: TABLE_NODE_ENDPOINTS contains duplicate destinations")

    if not 0 <= values["NIGHT_RECOVERY_HOUR"] <= 23:
        raise ValueError(f"Invalid configuration in {path}: NIGHT_RECOVERY_HOUR must be between 0 and 23")
    if not 0 <= values["NIGHT_RECOVERY_MINUTE"] <= 59:
        raise ValueError(f"Invalid configuration in {path}: NIGHT_RECOVERY_MINUTE must be between 0 and 59")
    for key in {"SERVER_PORT", "MQTT_PORT", "WORK_STATE_PORT"}:
        value = values.get(key, values["SERVER_PORT"])
        if not _is_integer(value) or not 1 <= value <= 65535:
            raise ValueError(f"Invalid configuration in {path}: {key} must be between 1 and 65535")
    for key in {"BUTTON_PIN", "LED_PIN", "MIN_AREA_PIXELS", "MAX_OCCUPATION", "OCCLUSION_THRESHOLD", "ZED_GRAB_RETRY_COUNT"}:
        if values[key] < 0:
            raise ValueError(f"Invalid configuration in {path}: {key} must not be negative")
    if values["THERE_IS_LED"] and values["THERE_IS_BUTTON"] and values["LED_PIN"] == values["BUTTON_PIN"]:
        raise ValueError(f"Invalid configuration in {path}: LED_PIN and BUTTON_PIN must be different")
    for key in {"MAX_LEN", "NIGHT_RECOVERY_DURATION_SECONDS", "WORK_STATE_SYNC_INTERVAL"}:
        if values[key] <= 0:
            raise ValueError(f"Invalid configuration in {path}: {key} must be greater than zero")


def load(path):
    global CONFIG_PATH, _loaded

    if _loaded:
        raise RuntimeError("Configuration has already been loaded")
    if yaml is None:
        raise RuntimeError("PyYAML is required to load the configuration file")

    config_path = Path(path)
    try:
        with config_path.open("r", encoding="utf-8") as handle:
            values = yaml.safe_load(handle)
    except OSError as error:
        raise RuntimeError(f"Unable to read configuration file {config_path}: {error}") from error
    except yaml.YAMLError as error:
        raise RuntimeError(f"Unable to parse configuration file {config_path}: {error}") from error

    _validate_configuration(values, config_path)
    normalized_values = {
        "LED_SOCKET_PATH": "",
        "WORK_STATE_PORT": values["SERVER_PORT"],
        "TABLE_NODE_ENDPOINTS": {},
        "RUNTIME_ROOT": ".",
        "CAMERA_SERIAL_NUMBER": "",
        **values,
    }
    for key in ROI_KEYS:
        normalized_values[key] = tuple(values[key])
    normalized_values["TABLE_NODE_IPS"] = tuple(values["TABLE_NODE_IPS"])

    globals().update(normalized_values)
    CONFIG_PATH = config_path.resolve()
    _loaded = True


def __getattr__(name):
    if not _loaded:
        raise RuntimeError("Configuration not loaded. Start the application through main.py")
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
