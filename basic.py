import asyncio
import numpy as np
from mavsdk import System

async def run():
    drone = System()
    await drone.connect(system.address="serial:///dev/ttyAMA10:57600")
    print("waiting for connection)")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("connected")
            break
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("good enough???")
            break

    print("arming")
    await drone.action.arm()
    await asyncio.sleep(5)
    print("ending")
    await drone.action.disarm()

if __name__ == "__main__":
    asyncio.run(run())
