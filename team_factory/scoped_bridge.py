"""Stdio MCP transport only; no commands, files, grants or model work."""
import argparse
import os
import selectors
import socket
import sys

def main():
    p=argparse.ArgumentParser();p.add_argument('--socket',required=True);a=p.parse_args()
    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as sock:
        sock.connect(a.socket)
        selector=selectors.DefaultSelector()
        selector.register(sys.stdin.buffer,selectors.EVENT_READ,'stdin')
        selector.register(sock,selectors.EVENT_READ,'socket')
        while selector.get_map():
            for key,_ in selector.select():
                if key.data=='stdin':
                    data=os.read(sys.stdin.fileno(),65536)
                    if not data:
                        sock.shutdown(socket.SHUT_WR);selector.unregister(key.fileobj)
                    else:sock.sendall(data)
                else:
                    data=sock.recv(65536)
                    if not data:return
                    sys.stdout.buffer.write(data);sys.stdout.buffer.flush()

if __name__=='__main__':main()
