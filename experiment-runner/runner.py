import argparse
import asyncio
import datetime
import os.path
import struct

import numpy as np
import pandas as pd

# CRC32 of "experiment-runner-v1"
MAGIC_NUM = 0xba41dba8
BYTES_IN_HEADER = 4+8
BYTES_PER_MEASUREMENT = 8+2
PORT = 55732

START_MESSAGE = "^".encode("ascii")
STOP_MESSAGE = "$".encode("ascii")
CLOSE_MESSAGE = "#".encode("ascii")
ACK_MESSAGE = "A".encode("ascii")
# TODO SERVER CONNECTION RESPONSES

global_data: dict[int,tuple[list[float],list[int]]] = {}
global_data_lock = asyncio.Lock()
writers: list[asyncio.StreamWriter] = []
start_barrier: asyncio.Barrier = None


async def handle_client(reader:asyncio.StreamReader, writer:asyncio.StreamWriter):
    addr = writer.get_extra_info('peername')
    header_data = await reader.readexactly(BYTES_IN_HEADER)
    recv_magic_num,raw_quad = struct.unpack("!IQ",header_data)
    if recv_magic_num != MAGIC_NUM:
        raise ValueError(f"Magic number mismatch! Got {recv_magic_num:08x}, expected {MAGIC_NUM:08x}!")
    extra_flags = raw_quad >> 48
    chipid = raw_quad & (2**48-1)
    measurement_unpack_fmtstr = ("<" if extra_flags>>15 else "!")+"dh"
    assert BYTES_PER_MEASUREMENT == struct.calcsize(measurement_unpack_fmtstr)
    print(f"[+] Client connected: {chipid:x} at {addr[0]}:{addr[1]}")
    times = []
    rssis = []
    writer.write(ACK_MESSAGE)
    await writer.drain()
    async with global_data_lock:
        global_data[chipid] = (times,rssis)
        writers.append(writer)
    await start_barrier.wait()
    
    try:
        while True:
            measurement_data = await reader.readexactly(BYTES_PER_MEASUREMENT)
            ts,rssi = struct.unpack(measurement_unpack_fmtstr,measurement_data)
            if ts == 0 and rssi == 0:
                print(f"[!] End of stream from {chipid:x}")
                writer.write(CLOSE_MESSAGE)
                break
            times.append(ts)
            rssis.append(rssi)
            #print(f"[{datetime.datetime.now(datetime.UTC).isoformat()}] recvd {chipid:x}'s measurement recd at {datetime.datetime.fromtimestamp(ts,datetime.UTC).isoformat()}")
    except Exception as e:
        print(f"[!] Error with client {chipid:x}: {e}")
    finally:
        writer.close()
        await writer.wait_closed()


async def main(n:int, duration:float, port:int=PORT):
    global start_barrier
    start_barrier = asyncio.Barrier(n+1)
    server = await asyncio.start_server(handle_client, host='0.0.0.0', port=port)
    server_addr = server.sockets[0].getsockname()
    print(f"[*] Server listening on {server_addr[0]}:{server_addr[1]}")

    # Wait until all clients are connected, then close
    await start_barrier.wait()
    server.close()
    print("[*] Server shut down.")

    # Server is closed an no longer receiving new connections
    # Send START to everyone
    print("[*] Sending START to all clients.")
    async with global_data_lock:
        for w in writers:
            w.write(START_MESSAGE)
        await asyncio.gather(*(w.drain() for w in writers))

    # Wait for experiment to complete
    await asyncio.sleep(duration)

    # Send STOP to everyone
    print("[*] Sending STOP to all clients.")
    async with global_data_lock:
        for w in writers:
            w.write(STOP_MESSAGE)
        await asyncio.gather(*(w.drain() for w in writers))

    await server.wait_closed()
    print("[*] All clients disconnected")
    
    N = sum(len(times) for (times,_) in global_data.values())
    chipids = np.zeros(N,dtype=int)
    times = np.zeros(N,dtype=float)
    rssis = np.zeros(N,dtype=int)
    i = 0
    for cid,(t,r) in global_data.items():
        assert len(t) == len(r)
        j = i+len(t)
        chipids[i:j] = cid
        times[i:j] = t
        rssis[i:j] = r
        i = j
    assert i == N
    data = pd.DataFrame({"chipid":chipids,"time":times,"rssi":rssis})
    data.sort_values(by="time",inplace=True)
    data.to_csv(os.path.join(__file__,"..",f"data/{datetime.datetime.now().timestamp():.0f}.csv"))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-n","--num",type=int,required=True,help="Number of receivers in experiment")
    parser.add_argument("-t","--duration",type=float,help="How long to run the experiment (secs)")
    parser.add_argument("-p","--port",type=int,default=PORT,help="Which port to run on")
    opt = parser.parse_args()

    try:
        asyncio.run(main(opt.num,opt.duration,opt.port))
    except KeyboardInterrupt:
        print("\n[!] Server shut down by user.")