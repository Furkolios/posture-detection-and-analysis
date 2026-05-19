import asyncio
from bleak import BleakClient

ADDRESS = "50:51:A9:7E:F6:75"
CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"

def handle_notify(sender, data):
    print(list(data))

async def main():
    async with BleakClient(ADDRESS) as client:
        await client.start_notify(CHAR_UUID, handle_notify)
        print("Connected. Listening...")
        await asyncio.sleep(60)  # listen for 60 seconds

asyncio.run(main())