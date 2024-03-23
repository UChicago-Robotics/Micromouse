"""
Establish data transmission with BLE
"""
import logging
import asyncio
import time
import json

from bleak import BleakClient
from bleak import BleakScanner


SERVICE_UUID = "edff0b05-dd61-4aef-b941-3583bf773dfe"
ACC_CHAR_UUID = "5b86af8f-925f-4029-9c95-458599341f96"
GYRO_CHAR_UUID = "373ceb13-35ad-41b5-b924-0604e56d8a4d"
NAME = 'UCMicromouse'
logger = logging.getLogger(__name__)


# Function to handle accelerometer data received as a notification
def handle_accel_notification(sender, data):
    """
    Handle accel notification
    """
    # Split the data into separate x, y, z values
    x, y, z = data.decode().split(',')
    acX = float(x)
    acY = float(y)
    acZ = float(z)

    # Get the current time
    timestamp = time.time()

    # Create a dictionary with the accelerometer data
    accel_data = {"acc_x": acX, "acc_y": acY, "acc_z": acZ, "Timestamp": timestamp}

    print(f"accel_data: {accel_data}")
    # Write the accelerometer data to a JSON file
    with open("data.json", "a", encoding="utf-8") as json_file:
        json.dump(accel_data, json_file)
        json_file.write(',\n')

# Function to handle gyroscope data received as a notification
def handle_gyro_notification(sender, data):
    """
    Handle gyro notification
    """
    # Split the data into separate x, y, z values
    gx, gy, gz = data.decode().split(',')
    gX = float(gx)
    gY = float(gy)
    gZ = float(gz)

    # Get the current time
    timestamp = time.time()

    # Create a dictionary with the gyroscope data
    gyro_data = {"gy_x": gX, "gy_y": gY, "gy_z": gZ, "Timestamp": timestamp}

    print(f"gyro_data: {gyro_data}")
    # Write the gyroscope data to a JSON file
    with open("data.json", "a", encoding="utf-8") as json_file:
        json.dump(gyro_data, json_file)
        json_file.write(',\n')

async def run():
    """
    Main running loop to read data from the robot.
    
    To be run until complete, but we will never 'complete' per se.
    """
    print('UCMicromouse Central Device...')
    print('Looking for UCMicromouse Peripheral Device...')

    device = await BleakScanner.find_device_by_name(NAME)
    if device is None:
        logger.error("could not find device with name '%s'", NAME)
        return

    logger.info("connecting to device...")

    async with BleakClient(device) as client:
        await client.start_notify(ACC_CHAR_UUID, handle_accel_notification)
        await client.start_notify(GYRO_CHAR_UUID, handle_gyro_notification)
        while True:
            await asyncio.sleep(0.001)

if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)-15s %(name)-8s %(levelname)s: %(message)s",
    )
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(run())
    except KeyboardInterrupt:
        print('\nReceived Keyboard Interrupt')
    finally:
        print('Program finished')
