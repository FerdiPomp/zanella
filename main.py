import argparse
import time
from engine.node import EnterNode, ExitNode, EnvNode
import threading

import sys


parser = argparse.ArgumentParser(description="Esempio con argparse")

parser.add_argument("--node_id", type=str, help="Jetson ID, as the jetson node role")
parser.add_argument("--workspace", type=str, default = None, help="Workspace name as str")
parser.add_argument("--broker_ip", default = None, help="broker_ip")
parser.add_argument("--topic", default = None, help="topic")
parser.add_argument("--file_bag", type=str, default = None, help="Bag file path for realsense data")
parser.add_argument("--led_pin", type=int, default = None, help="Jetson Pin number for led connection")
parser.add_argument("--button_pin", type=int, default = None, help="Jetson Pin number for button connection")

args = parser.parse_args()



if args.node_id=='A':
    node = EnterNode(node_id=args.node_id)
    node.start(file_bag=args.file_bag, led_pin=args.led_pin)

elif args.node_id=='B':
    node = ExitNodeNode(node_id=args.node_id)
    node.start(file_bag=args.file_bag, led_pin=args.led_pin, button_pin=args.button_pin)

elif args.node_id=='C':
    node = EnvNode(node_id=args.node_id, workspace=args.workspace, broker_ip=args.broker_ip, topic=args.topic)
    node.start(file_bag=args.file_bag)

#TODO: implement more correct way to quit with the thread (stop_event = threading.Event())
print("Enter q to exit")
for line in sys.stdin:
    if line.strip().lower() == 'q':
        print("Program exit")
        break

node.stop()




