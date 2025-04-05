import argparse
import numpy as np
import pandas
import socket
import struct
import threading
import time

# TODO: change name of repo and make PoC for arduino devices
# TODO: add argparse to allow overrides
# CRC32 of "transmit-v1"
MAGIC_NUM = 0x61462cdf
BYTES_IN_HEADER = 20
BYTES_PER_FRAME = 4+2
PORT = 55732
BACKLOG = 15
SERVER_ACCEPT_TIMEOUT = 5
CLIENT_RECV_TIMEOUT = 5
PREFIX = ""


def handle_client(conn:socket.socket, addr:str, local_eptime:float):
    print(f"New connection from {addr}")
    try:
        # Set timeout, read and assert magic number
        conn.settimeout(CLIENT_RECV_TIMEOUT)
        buf = conn.recv(4)
        if len(buf) < 4:
            raise ValueError("Failed to get magic number")
        recv_magic_num, = struct.unpack("!I", buf)
        if recv_magic_num != MAGIC_NUM:
            raise ValueError(f"Magic number mismatch! Got {recv_magic_num:08x}, expected {MAGIC_NUM:08x}!")
        
        # Read rest of the header
        buf = conn.recv(BYTES_IN_HEADER-4)
        if len(buf) < BYTES_IN_HEADER-4:
            raise ValueError("Incomplete header")
        raw_quad,remote_mstime,num_frames = struct.unpack("!QII", buf)
        extra_flags = raw_quad >> 48
        chipid = raw_quad & (2**48-1)
        print(f"Recvd flags: {extra_flags:016b}, cid: {chipid:012x}, remote_mstime: {remote_mstime}, num_frames: {num_frames}")
        #TODO VERIFY AND USE THESE NUMS

        # Enter the frames into 2 numpy arrays
        times = np.zeros(num_frames,dtype=float)
        rssis = np.zeros(num_frames,dtype=np.int16)
        i = 0
        buf = b""
        while i < num_frames:
            if len(buf) < BYTES_PER_FRAME:
                # TODO: test transmission of data (all makes it across / this formula is right)
                buf += conn.recv(BYTES_PER_FRAME*(num_frames-i)-len(buf))
            a,b = struct.unpack("!Ih",buf[:BYTES_PER_FRAME])
            times[i] = local_eptime - (remote_mstime - a)/1000
            rssis[i] = b
            buf = buf[BYTES_PER_FRAME:]
            i += 1

        # Create a pandas DataFrame and export to csv
        data = pandas.DataFrame({"time":times,"rssi":rssis})
        data.to_csv(f"data/{chipid:08X}_{local_eptime:.0f}.csv")
    except Exception as err:
        print(f"Connection ended with error: {err}")
    finally:
        print("Closing connection")
        conn.close()


def main():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.settimeout(SERVER_ACCEPT_TIMEOUT)
    server.bind(("0.0.0.0", PORT))
    server.listen(BACKLOG)
    print("Server ready")
    
    try:
        while True:
            try:
                conn,addr = server.accept()
                local_eptime = time.time()
                threading.Thread(target=handle_client,args=(conn,addr,local_eptime),daemon=True).start()
            except TimeoutError:
                pass
                # print("server.accept() timeout")
    except KeyboardInterrupt:
        print("Server interrupted")
    except Exception as err:
        print(f"Server ended with error: {err}")
    finally:
        print("Closing server")
        server.close()


if __name__ == "__main__":
    main()
