#!/usr/bin/env python3

"""
MAVSDK Flight Controller Diagnostic Tool

This script provides comprehensive diagnostics for understanding why sensors
appear as unavailable in MAVSDK despite being calibrated in QGroundControl.

The script monitors multiple layers of the system:
1. Connection health and MAVLink message flow
2. Raw sensor data streams
3. EKF (Extended Kalman Filter) status and health
4. Flight controller's internal health assessments
5. Arming readiness checks

Think of this as a medical examination that checks different systems
to understand why the patient (drone) won't cooperate.
"""

import asyncio
import time
from mavsdk import System
from mavsdk.telemetry import LandedState, FlightMode, HealthAllOk
import logging

# Configure logging to see detailed information flow
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class FlightControllerDiagnostic:
    def __init__(self, connection_string="udp://:14540"):
        """
        Initialize the diagnostic tool.
        
        The connection string tells MAVSDK how to talk to your flight controller.
        UDP connection on port 14540 is the default for SITL and many setups,
        but you might need to change this based on your configuration.
        """
        self.drone = System()
        self.connection_string = connection_string
        self.diagnostic_data = {}
        
    async def connect_and_diagnose(self):
        """
        Main diagnostic routine that systematically checks each component.
        
        This function orchestrates the entire diagnostic process, similar to
        how a doctor follows a checklist during a physical examination.
        """
        print("🔍 Starting Flight Controller Diagnostic Analysis")
        print("=" * 60)
        
        # Step 1: Establish connection - like checking if the patient is responsive
        connection_success = await self._test_connection()
        if not connection_success:
            print("❌ Cannot proceed without stable connection")
            return
            
        # Step 2: Wait for system to stabilize - giving the flight controller time to "wake up"
        print("\n⏳ Allowing system to stabilize (30 seconds)...")
        print("   (Flight controllers need time to initialize all systems)")
        await asyncio.sleep(30)
        
        # Step 3: Run comprehensive diagnostics
        await self._run_comprehensive_diagnostics()
        
        # Step 4: Provide analysis and recommendations
        self._analyze_and_recommend()
        
    async def _test_connection(self):
        """
        Test the fundamental connection between MAVSDK and the flight controller.
        
        This is like checking if you can hear the patient's heartbeat - without
        a good connection, all other diagnostics are meaningless.
        """
        print(f"🔌 Connecting to flight controller at {self.connection_string}")
        
        try:
            await self.drone.connect(system_address=self.connection_string)
            print("✅ Connection established")
            
            # Wait for the drone to be discovered on the network
            print("🔍 Waiting for flight controller to be discovered...")
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    print(f"✅ Flight controller discovered and connected")
                    break
                    
            return True
            
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            print("💡 Check your connection string and ensure the flight controller is powered")
            return False
    
    async def _run_comprehensive_diagnostics(self):
        """
        Execute all diagnostic tests in parallel.
        
        Like running multiple medical tests simultaneously to get a complete
        picture of the system's health.
        """
        print("\n🧪 Running Comprehensive Diagnostics")
        print("-" * 40)
        
        # Run all diagnostic tasks concurrently for efficiency
        diagnostic_tasks = [
            self._monitor_basic_telemetry(),
            self._monitor_sensor_health(),
            self._monitor_ekf_status(),
            self._monitor_position_systems(),
            self._monitor_imu_data(),
            self._check_arming_status()
        ]
        
        # Execute all diagnostics simultaneously
        await asyncio.gather(*diagnostic_tasks, return_exceptions=True)
    
    async def _monitor_basic_telemetry(self):
        """
        Monitor basic telemetry to understand overall system communication.
        
        This is like checking basic vital signs - heart rate, temperature, etc.
        If basic telemetry fails, it indicates a communication problem.
        """
        print("📡 Monitoring Basic Telemetry...")
        
        try:
            # Check if we're receiving basic flight information
            flight_mode_count = 0
            battery_count = 0
            
            # Monitor for 10 seconds to see if we get regular updates
            timeout = time.time() + 10
            
            async for flight_mode in self.drone.telemetry.flight_mode():
                flight_mode_count += 1
                if time.time() > timeout:
                    break
                    
            async for battery in self.drone.telemetry.battery():
                battery_count += 1
                if time.time() > timeout:
                    break
            
            self.diagnostic_data['telemetry_health'] = {
                'flight_mode_updates': flight_mode_count,
                'battery_updates': battery_count,
                'communication_healthy': flight_mode_count > 0 and battery_count > 0
            }
            
            if flight_mode_count > 0:
                print(f"✅ Receiving flight mode updates ({flight_mode_count} in 10s)")
            else:
                print("❌ No flight mode updates received")
                
            if battery_count > 0:
                print(f"✅ Receiving battery updates ({battery_count} in 10s)")
            else:
                print("❌ No battery updates received")
                
        except Exception as e:
            print(f"❌ Basic telemetry error: {e}")
            self.diagnostic_data['telemetry_health'] = {'error': str(e)}
    
    async def _monitor_sensor_health(self):
        """
        Monitor the health reporting system that MAVSDK uses for arming checks.
        
        This examines the same health flags that prevent your drone from arming.
        Think of it as checking the specific indicators that are showing "red"
        in your original problem.
        """
        print("🏥 Monitoring Sensor Health Reports...")
        
        try:
            health_samples = []
            
            # Collect health data over 15 seconds
            timeout = time.time() + 15
            async for health in self.drone.telemetry.health_all_ok():
                health_samples.append({
                    'timestamp': time.time(),
                    'is_global_position_ok': health.is_global_position_ok,
                    'is_home_position_ok': health.is_home_position_ok,
                    'is_local_position_ok': health.is_local_position_ok,
                    'is_accelerometer_calibration_ok': health.is_accelerometer_calibration_ok,
                    'is_gyroscope_calibration_ok': health.is_gyroscope_calibration_ok,
                    'is_magnetometer_calibration_ok': health.is_magnetometer_calibration_ok
                })
                
                if time.time() > timeout:
                    break
            
            self.diagnostic_data['health_samples'] = health_samples
            
            if health_samples:
                latest = health_samples[-1]
                print(f"📊 Health Status Summary ({len(health_samples)} samples):")
                print(f"   Global Position: {'✅' if latest['is_global_position_ok'] else '❌'}")
                print(f"   Local Position:  {'✅' if latest['is_local_position_ok'] else '❌'}")
                print(f"   Home Position:   {'✅' if latest['is_home_position_ok'] else '❌'}")
                print(f"   Accelerometer:   {'✅' if latest['is_accelerometer_calibration_ok'] else '❌'}")
                print(f"   Gyroscope:       {'✅' if latest['is_gyroscope_calibration_ok'] else '❌'}")
                print(f"   Magnetometer:    {'✅' if latest['is_magnetometer_calibration_ok'] else '❌'}")
            else:
                print("❌ No health data received")
                
        except Exception as e:
            print(f"❌ Health monitoring error: {e}")
            self.diagnostic_data['health_samples'] = []
    
    async def _monitor_ekf_status(self):
        """
        Monitor Extended Kalman Filter status - the "brain" that fuses sensor data.
        
        The EKF is like the flight controller's internal reasoning system that
        decides whether sensor data makes sense. Even calibrated sensors can be
        rejected by the EKF if they don't align with its expectations.
        """
        print("🧠 Monitoring EKF (Extended Kalman Filter) Status...")
        
        try:
            # Try to get EKF status information
            # Note: This requires specific MAVLink messages that may not be available on all systems
            ekf_samples = []
            
            # Monitor for a shorter time since EKF data is less frequent
            timeout = time.time() + 5
            
            # Unfortunately, MAVSDK doesn't expose detailed EKF status directly
            # But we can infer EKF health from position and attitude data consistency
            position_samples = []
            attitude_samples = []
            
            async for position in self.drone.telemetry.position():
                position_samples.append({
                    'lat': position.latitude_deg,
                    'lon': position.longitude_deg,
                    'alt': position.absolute_altitude_m,
                    'timestamp': time.time()
                })
                if time.time() > timeout:
                    break
            
            timeout = time.time() + 5
            async for attitude in self.drone.telemetry.attitude_euler():
                attitude_samples.append({
                    'roll': attitude.roll_deg,
                    'pitch': attitude.pitch_deg,
                    'yaw': attitude.yaw_deg,
                    'timestamp': time.time()
                })
                if time.time() > timeout:
                    break
            
            self.diagnostic_data['ekf_inference'] = {
                'position_samples': len(position_samples),
                'attitude_samples': len(attitude_samples),
                'position_data_available': len(position_samples) > 0,
                'attitude_data_available': len(attitude_samples) > 0
            }
            
            print(f"📊 EKF Health Inference:")
            print(f"   Position samples: {len(position_samples)}")
            print(f"   Attitude samples: {len(attitude_samples)}")
            
            if len(position_samples) > 0:
                print(f"   Latest position: {position_samples[-1]['lat']:.6f}, {position_samples[-1]['lon']:.6f}")
            else:
                print("   ❌ No position data - EKF may not have GPS lock")
                
        except Exception as e:
            print(f"❌ EKF monitoring error: {e}")
            self.diagnostic_data['ekf_inference'] = {'error': str(e)}
    
    async def _monitor_position_systems(self):
        """
        Specifically examine GPS and position estimation systems.
        
        Position systems are often the culprit in "sensor unavailable" issues
        because they require external conditions (GPS satellites) that might
        not be available in your testing environment.
        """
        print("🛰️ Monitoring Position Systems...")
        
        try:
            # Check GPS status
            gps_samples = []
            timeout = time.time() + 10
            
            async for gps_info in self.drone.telemetry.gps_info():
                gps_samples.append({
                    'num_satellites': gps_info.num_satellites,
                    'fix_type': gps_info.fix_type,
                    'timestamp': time.time()
                })
                if time.time() > timeout:
                    break
            
            # Check home position
            try:
                home_position = await self.drone.telemetry.home().__anext__()
                home_set = True
            except:
                home_set = False
            
            self.diagnostic_data['position_systems'] = {
                'gps_samples': gps_samples,
                'home_position_set': home_set
            }
            
            if gps_samples:
                latest_gps = gps_samples[-1]
                print(f"📡 GPS Status:")
                print(f"   Satellites: {latest_gps['num_satellites']}")
                print(f"   Fix Type: {latest_gps['fix_type']}")
                print(f"   Home Position: {'✅ Set' if home_set else '❌ Not Set'}")
            else:
                print("❌ No GPS data received")
                
        except Exception as e:
            print(f"❌ Position systems error: {e}")
            self.diagnostic_data['position_systems'] = {'error': str(e)}
    
    async def _monitor_imu_data(self):
        """
        Monitor Inertial Measurement Unit (accelerometer, gyroscope) data streams.
        
        This checks if the IMU sensors are actually producing data that the
        flight controller can use, beyond just being calibrated.
        """
        print("📏 Monitoring IMU Data Streams...")
        
        try:
            imu_samples = []
            timeout = time.time() + 5
            
            async for imu in self.drone.telemetry.imu():
                imu_samples.append({
                    'acceleration_frd': {
                        'forward': imu.acceleration_frd.forward_m_s2,
                        'right': imu.acceleration_frd.right_m_s2,
                        'down': imu.acceleration_frd.down_m_s2
                    },
                    'angular_velocity_frd': {
                        'forward': imu.angular_velocity_frd.forward_rad_s,
                        'right': imu.angular_velocity_frd.right_rad_s,
                        'down': imu.angular_velocity_frd.down_rad_s
                    },
                    'timestamp': time.time()
                })
                if time.time() > timeout:
                    break
            
            self.diagnostic_data['imu_data'] = {
                'samples': len(imu_samples),
                'data_available': len(imu_samples) > 0
            }
            
            if imu_samples:
                latest = imu_samples[-1]
                print(f"📊 IMU Data Status:")
                print(f"   Samples received: {len(imu_samples)}")
                print(f"   Latest acceleration (down): {latest['acceleration_frd']['down']:.2f} m/s²")
                print(f"   (Should be ~9.8 m/s² when stationary)")
            else:
                print("❌ No IMU data received")
                
        except Exception as e:
            print(f"❌ IMU monitoring error: {e}")
            self.diagnostic_data['imu_data'] = {'error': str(e)}
    
    async def _check_arming_status(self):
        """
        Check the specific reasons why arming might be failing.
        
        This is the most direct check - it asks the flight controller
        "why won't you arm?" and reports the specific blockers.
        """
        print("⚔️ Checking Arming Status...")
        
        try:
            # Check if already armed
            async for armed in self.drone.telemetry.armed():
                is_armed = armed
                break
            
            print(f"   Current armed state: {'✅ Armed' if is_armed else '❌ Disarmed'}")
            
            if not is_armed:
                print("   Attempting to identify arming blockers...")
                
                # Try to arm and capture the failure reason
                try:
                    await self.drone.action.arm()
                    print("✅ Arming successful!")
                except Exception as arm_error:
                    print(f"❌ Arming failed: {arm_error}")
                    self.diagnostic_data['arming_error'] = str(arm_error)
            
        except Exception as e:
            print(f"❌ Arming check error: {e}")
            self.diagnostic_data['arming_check'] = {'error': str(e)}
    
    def _analyze_and_recommend(self):
        """
        Analyze all collected data and provide specific recommendations.
        
        This is like a doctor reviewing all test results and providing
        a diagnosis with specific treatment recommendations.
        """
        print("\n" + "=" * 60)
        print("🔬 DIAGNOSTIC ANALYSIS & RECOMMENDATIONS")
        print("=" * 60)
        
        # Analyze communication health
        if 'telemetry_health' in self.diagnostic_data:
            telemetry = self.diagnostic_data['telemetry_health']
            if telemetry.get('communication_healthy', False):
                print("✅ Communication: HEALTHY")
                print("   MAVSDK is successfully communicating with your flight controller")
            else:
                print("❌ Communication: POOR")
                print("   🔧 RECOMMENDATION: Check connection settings and cables")
                print("   💡 Try different baud rates or connection methods")
                return  # No point continuing if communication is broken
        
        # Analyze sensor health patterns
        if 'health_samples' in self.diagnostic_data and self.diagnostic_data['health_samples']:
            print("\n📊 Sensor Health Analysis:")
            samples = self.diagnostic_data['health_samples']
            
            # Check for patterns in sensor failures
            global_pos_failures = sum(1 for s in samples if not s['is_global_position_ok'])
            local_pos_failures = sum(1 for s in samples if not s['is_local_position_ok'])
            accel_failures = sum(1 for s in samples if not s['is_accelerometer_calibration_ok'])
            
            total_samples = len(samples)
            
            if global_pos_failures == total_samples:
                print("❌ Global Position: CONSISTENTLY FAILING")
                print("   🔧 LIKELY CAUSE: No GPS lock or insufficient satellites")
                print("   💡 SOLUTION: Test outdoors with clear sky view")
                
            if local_pos_failures == total_samples:
                print("❌ Local Position: CONSISTENTLY FAILING")
                print("   🔧 LIKELY CAUSE: Depends on global position or external positioning")
                print("   💡 SOLUTION: Fix global position first, or set up local positioning")
                
            if accel_failures == total_samples:
                print("❌ Accelerometer: CONSISTENTLY FAILING")
                print("   🔧 LIKELY CAUSE: EKF rejecting accelerometer data")
                print("   💡 SOLUTION: Check for vibrations, recalibrate, or check mounting")
        
        # Analyze GPS situation
        if 'position_systems' in self.diagnostic_data:
            pos_data = self.diagnostic_data['position_systems']
            if 'gps_samples' in pos_data and pos_data['gps_samples']:
                latest_gps = pos_data['gps_samples'][-1]
                if latest_gps['num_satellites'] < 6:
                    print(f"\n🛰️ GPS Analysis: INSUFFICIENT SATELLITES ({latest_gps['num_satellites']})")
                    print("   🔧 ROOT CAUSE: Indoor testing or poor GPS conditions")
                    print("   💡 SOLUTION: Move outdoors or use SITL for indoor testing")
        
        # Provide specific action plan
        print("\n" + "🎯 RECOMMENDED ACTION PLAN")
        print("-" * 30)
        print("1. Test outdoors with clear sky view for GPS lock")
        print("2. Wait 2-3 minutes after power-on for full system initialization")
        print("3. Ensure only one ground control station is connected")
        print("4. Check for magnetic interference sources")
        print("5. Verify firmware compatibility between flight controller and MAVSDK")
        
        if 'arming_error' in self.diagnostic_data:
            print(f"\n⚠️  Specific arming error to investigate: {self.diagnostic_data['arming_error']}")

async def main():
    """
    Main function to run the diagnostic tool.
    
    Modify the connection string below to match your setup:
    - "udp://:14540" for SITL or standard UDP
    - "serial:///dev/ttyUSB0:57600" for serial connection
    - "tcp://192.168.1.100:5760" for TCP connection
    """
    
    # Modify this connection string to match your setup
    connection_string = "udp://:14540"
    
    diagnostic_tool = FlightControllerDiagnostic(connection_string)
    await diagnostic_tool.connect_and_diagnose()

if __name__ == "__main__":
    # Run the diagnostic tool
    print("MAVSDK Flight Controller Diagnostic Tool")
    print("This tool will help identify why sensors appear unavailable in MAVSDK")
    print("while showing as calibrated in QGroundControl.\n")
    
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n🛑 Diagnostic interrupted by user")
    except Exception as e:
        print(f"\n💥 Diagnostic failed with error: {e}")
        print("Check your connection settings and try again.")
