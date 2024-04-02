import asyncio
from bleak import discover
async def scan():
    devices = await discover()
    for d in devices:
        print(d)
loop = asyncio.get_event_loop()
loop.run_until_complete(scan())