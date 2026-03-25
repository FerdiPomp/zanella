import argparse
import time
from engine.node import EnterNode, ExitNode, EnvNode
import threading

import sys
import signal


parser = argparse.ArgumentParser(description="Esempio con argparse")

parser.add_argument("--node_id", type=str, help="Jetson ID, as the jetson node role")
parser.add_argument("--workspace", type=str, default = None, help="Workspace name as str")
parser.add_argument("--broker_ip", default = None, help="broker_ip")
parser.add_argument("--topic", default = None, help="topic")
parser.add_argument("--file_bag", type=str, default = None, help="Bag file path for realsense data")

args = parser.parse_args()



if args.node_id=='A':
    node = EnterNode(node_id=args.node_id)
    node.start(file_bag=args.file_bag)

elif args.node_id=='B':
    node = ExitNode(node_id=args.node_id)
    node.start(file_bag=args.file_bag)

elif args.node_id=='C':
    node = EnvNode(node_id=args.node_id, workspace=args.workspace, broker_ip=args.broker_ip, topic=args.topic)
    node.start(file_bag=args.file_bag)


print("SIGINT to exit")

signals = {signal.SIGINT, signal.SIGTERM}
signal.pthread_sigmask(signal.SIG_BLOCK, signals)
signum = signal.sigwait(signals)

node.stop()




