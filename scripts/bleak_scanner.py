import asyncio
from bleak import BleakScanner, BleakClient

# Replace this with your board’s name or MAC address
TARGET_NAME = "microbit"

# Full 128‑bit UUID strings:
SERVICE_UUID    = "0000180A-0000-1000-8000-00805F9B34FB"
ACCEL_UUID      = "00002C1E-0000-1000-8000-00805F9B34FB"
MAG_UUID        = "00002AA1-0000-1000-8000-00805F9B34FB"
TEMP_UUID       = "00002A6E-0000-1000-8000-00805F9B34FB"

def handle_accel(_, data: bytearray):
    # Unpack three little‑endian signed 16‑bit ints
    x, y, z = (int.from_bytes(data[i:i+2], 'little', signed=True)
               for i in (0, 2, 4))
    print(f"Accel: {x} / {y} / {z} mm/s²")

async def main():
    # 1) Find the device
    devices = await BleakScanner.discover()
    # Print all discovered devices
    for d in devices:
        print(f"Discovered device: {d.name or 'Unknown'} [{d.address}]")
    # Find the target device
    target = next((d for d in devices if d.name == TARGET_NAME), None)
    if not target:
        print("Board not found!")
        return

    # 2) Connect & start notifications
    async with BleakClient(target.address) as client:
        await client.start_notify(ACCEL_UUID, handle_accel)
        print("Subscribed to accel. Waiting for data...")
        await asyncio.sleep(10)   # keep alive for 10 s

if __name__ == "__main__":
    asyncio.run(main())
