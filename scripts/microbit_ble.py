#!/usr/bin/env python3
import asyncio
import sys
from dbus_next.aio import MessageBus
from dbus_next.constants import BusType, MessageType
from dbus_next import Message

LED_UUID = '12345678-1234-5678-1234-9abcdef0f0f1'.lower()

async def get_managed_objects():
    bus = await MessageBus(bus_type=BusType.SYSTEM).connect()
    msg = Message(destination='org.bluez',
                  path='/',
                  interface='org.freedesktop.DBus.ObjectManager',
                  member='GetManagedObjects')
    reply = await bus.call(msg)
    return reply.body[0] if reply.message_type == MessageType.METHOD_RETURN else {}

async def discover_devices_with_led():
    objs = await get_managed_objects()
    results = []

    for path, interfaces in objs.items():
        if 'org.bluez.GattCharacteristic1' in interfaces:
            uuid = interfaces['org.bluez.GattCharacteristic1'].get('UUID').value.lower()
            if uuid != LED_UUID:
                continue

            # Traverse upward until we find org.bluez.Device1
            device_path = None
            parts = path.split('/')
            for i in range(len(parts), 0, -1):
                sub_path = '/'.join(parts[:i])
                dev_iface = objs.get(sub_path, {}).get('org.bluez.Device1')
                if dev_iface:
                    device_path = sub_path
                    break

            if not device_path:
                continue

            dev_info = objs.get(device_path, {}).get('org.bluez.Device1')
            if dev_info:
                connected = dev_info.get('Connected').value
                if not connected:
                    continue  # skip if not actually connected

                name = dev_info.get('Name').value if dev_info.get('Name') else "Unknown"
                addr = dev_info.get('Address').value
                results.append({
                    'address': addr,
                    'name': name,
                    'char_path': path
                })


    return results

async def handle_discover():
    devices = await discover_devices_with_led()
    if not devices:
        print("⚠️  No connected devices exposing the LED characteristic.")
        return

    print("Discovered micro:bit devices with LED control:")
    for d in devices:
        print(f"  {d['address']} ({d['name']})")

async def write_value(address, value):
    devices = await discover_devices_with_led()
    match = next((d for d in devices if d['address'].lower() == address.lower()), None)

    if not match:
        print(f"No connected device found with address: {address}")
        return

    bus = await MessageBus(bus_type=BusType.SYSTEM).connect()
    msg = Message(
        destination='org.bluez',
        path=match['char_path'],
        interface='org.bluez.GattCharacteristic1',
        member='WriteValue',
        signature='aya{sv}',
        body=[value.encode(), {}]
    )
    reply = await bus.call(msg)
    if reply.message_type == MessageType.ERROR:
        print(f"Failed to write: {reply.error_name}")
    else:
        print(f"Wrote '{value}' to {match['address']} ({match['name']})")

async def main():
    if len(sys.argv) == 2 and sys.argv[1] == "--discover":
        await handle_discover()
    elif len(sys.argv) == 3:
        await write_value(sys.argv[1], sys.argv[2])
    else:
        print("Usage:")
        print("  microbit_ble.py --discover")
        print("  microbit_ble.py <ADDRESS> <0|1>")

if __name__ == "__main__":
    asyncio.run(main())
