import argparse
import importlib
import signal

import config as CONFIG
from engine.node import EnterNode, EnvNode, ExitNode
from engine.runtime_utils import print_log


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Esempio con argparse")
    parser.add_argument("--node_id", type=str, help="Jetson ID, as the jetson node role")
    parser.add_argument("--workspace", type=str, default=None, help="Workspace name as str")
    parser.add_argument("--mqtt_psw", default=None, help="password")
    parser.add_argument("--topic", default="workplace40/Tracevision", help="topic workplace_id/Tracevision")
    parser.add_argument("--file_bag", type=str, default=None, help="Bag file path for realsense data")
    return parser


def _require_module(module_name: str) -> None:
    importlib.import_module(module_name)


def validate_runtime_dependencies(node_id: str) -> None:
    if node_id in {"A", "B"}:
        if CONFIG.IS_ZED:
            raise RuntimeError("I nodi A e B usano ObjCamera RealSense: con IS_ZED=True questa configurazione e' incoerente")
        _require_module("pyrealsense2")

    if node_id == "C":
        if CONFIG.IS_ZED:
            _require_module("pyzed.sl")
        else:
            _require_module("pyrealsense2")
        _require_module("pylibdmtx.pylibdmtx")

    if CONFIG.THERE_IS_LED or CONFIG.THERE_IS_BUTTON:
        _require_module("gpiod")

    if CONFIG.ONLINE_SENDER or node_id == "C":
        _require_module("requests")

    if node_id in {"A", "B"} or (node_id == "C" and CONFIG.ONLINE_RECIEVER):
        _require_module("flask")

    if node_id == "C" and CONFIG.ONLINE_SENDER_ENV:
        _require_module("paho.mqtt.client")


def build_node(args):
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


def wait_for_stop_signal() -> None:
    print_log("SIGINT to exit")
    signals = {signal.SIGINT, signal.SIGTERM}
    signal.pthread_sigmask(signal.SIG_BLOCK, signals)
    signal.sigwait(signals)


def main() -> None:
    args = build_parser().parse_args()
    validate_runtime_dependencies(args.node_id)
    node = build_node(args)

    if node is not None:
        node.start(file_bag=args.file_bag)

    wait_for_stop_signal()
    node.stop()


if __name__ == "__main__":
    main()
