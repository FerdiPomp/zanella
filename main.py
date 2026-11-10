import argparse
import importlib
import signal
from pathlib import Path

import configuration as CONFIG


DEFAULT_CONFIG_PATH = Path(__file__).with_name("config.yaml")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Esempio con argparse")
    parser.add_argument("--node_id", choices=("A", "B", "C", "LED"), required=True, help="Station role")
    parser.add_argument("--workspace", type=str, default=None, help="Workspace name as str")
    parser.add_argument("--mqtt_psw", default=None, help="password")
    parser.add_argument("--topic", default="workplace40/Tracevision", help="topic workplace_id/Tracevision")
    parser.add_argument("--file_bag", type=str, default=None, help="Bag file path for realsense data")
    parser.add_argument("--config", type=str, default=str(DEFAULT_CONFIG_PATH), help="YAML configuration file path")
    return parser


def _require_module(module_name: str):
    return importlib.import_module(module_name)


def validate_runtime_dependencies(node_id: str, file_bag: str = None) -> None:
    if node_id == "LED":
        if not CONFIG.THERE_IS_LED or not CONFIG.LED_SOCKET_PATH:
            raise RuntimeError("LED controller requires THERE_IS_LED=True and LED_SOCKET_PATH")
        _require_module("gpiod")
        return

    if node_id in {"A", "B"} and CONFIG.IS_ZED and not CONFIG.ZED_ENV_HAS_DEPTH:
        raise RuntimeError("ZED_ENV_HAS_DEPTH=False is supported only on node C")

    if node_id in {"A", "B"} and CONFIG.LED_SOCKET_PATH:
        if Path(CONFIG.RUNTIME_ROOT).resolve() == Path.cwd().resolve():
            raise RuntimeError("Shared-box table nodes require a separate RUNTIME_ROOT")
        if not file_bag and not CONFIG.CAMERA_SERIAL_NUMBER:
            raise RuntimeError("Shared-box table nodes require CAMERA_SERIAL_NUMBER for live cameras")

    if node_id == "C" and CONFIG.IS_ZED and not CONFIG.ZED_ENV_HAS_DEPTH and CONFIG.CAMERA_SERIAL_NUMBER:
        raise RuntimeError("CAMERA_SERIAL_NUMBER is not supported for ZED X One")

    if node_id == "C" and CONFIG.IS_ZED and not CONFIG.ZED_ENV_HAS_DEPTH:
        if not CONFIG.ARUCO_MODE:
            raise RuntimeError("ZED_ENV_HAS_DEPTH=False requires ARUCO_MODE=True")
        cv2 = _require_module("cv2")
        if not hasattr(cv2, "aruco"):
            raise RuntimeError("ZED_ENV_HAS_DEPTH=False requires OpenCV ArUco support")
        sl = _require_module("pyzed.sl")
        if not hasattr(sl, "CameraOne") or not hasattr(sl, "InitParametersOne"):
            raise RuntimeError("ZED X One requires a ZED SDK with CameraOne support")

    if node_id in {"A", "B"}:
        if CONFIG.IS_ZED:
            _require_module("pyzed.sl")
        else:
            _require_module("pyrealsense2")

    if node_id == "C":
        if CONFIG.IS_ZED:
            _require_module("pyzed.sl")
        else:
            _require_module("pyrealsense2")
        _require_module("pylibdmtx.pylibdmtx")

    if (CONFIG.THERE_IS_LED and not CONFIG.LED_SOCKET_PATH) or CONFIG.THERE_IS_BUTTON:
        _require_module("gpiod")

    if CONFIG.ONLINE_SENDER or node_id == "C":
        _require_module("requests")

    if node_id in {"A", "B"} or (node_id == "C" and CONFIG.ONLINE_RECIEVER):
        _require_module("flask")

    if node_id == "C" and CONFIG.ONLINE_SENDER_ENV:
        _require_module("paho.mqtt.client")


def build_node(args):
    if args.node_id == "LED":
        from engine.led import LedController

        return LedController(CONFIG.LED_SOCKET_PATH)

    from engine.node import EnterNode, EnvNode, ExitNode

    if args.node_id == "A":
        return EnterNode(node_id=args.node_id)
    if args.node_id == "B":
        return ExitNode(node_id=args.node_id)
    if args.node_id == "C":
        return EnvNode(
            node_id=args.node_id,
            workspace=args.workspace,
            broker_psw=args.mqtt_psw,
            topic=args.topic,
        )
    return None


def wait_for_stop_signal(node=None) -> None:
    from engine.runtime_utils import print_log

    print_log("SIGINT to exit")
    signals = {signal.SIGINT, signal.SIGTERM}
    if node is None:
        signal.sigwait(signals)
    else:
        while node.thread.is_alive():
            if signal.sigtimedwait(signals, 1) is not None:
                return
        raise RuntimeError("LED controller stopped unexpectedly")


def main() -> None:
    args = build_parser().parse_args()
    CONFIG.load(args.config)
    validate_runtime_dependencies(args.node_id, args.file_bag)
    from engine.runtime_utils import RuntimeLease

    lease = RuntimeLease() if args.node_id in {"A", "B", "C"} else None
    node = None
    try:
        node = build_node(args)
        signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGINT, signal.SIGTERM})
        if node is not None:
            node.start(file_bag=args.file_bag)
        wait_for_stop_signal(node if args.node_id == "LED" else None)
    finally:
        try:
            if node is not None:
                node.stop()
        finally:
            if lease is not None:
                lease.close()


if __name__ == "__main__":
    main()
