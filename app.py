from flask import Flask, render_template, request, jsonify, session, Response
from flask_socketio import SocketIO, emit
import paho.mqtt.client as mqtt
import json
import threading
import time
import random
from datetime import datetime
import uuid
import cv2
import os # Import os for directory creation
from traffic_detector import TrafficDetector # Ensure traffic_detector.py is in the same directory

app = Flask(__name__)
app.config['SECRET_KEY'] = 'your-secret-key-here' # Change this to a strong, random key in production
socketio = SocketIO(app, cors_allowed_origins="*")

# Initialize traffic detector (from traffic_detector.py)
# This handles YOLO, OCR, and AI traffic light logic, publishing to MQTT.
traffic_detector = TrafficDetector()

# Initialize camera
camera = cv2.VideoCapture(1)  # Use camera 1 (external camera)
if not camera.isOpened():
    print("Failed to open camera 1, trying camera 0...")
    camera = cv2.VideoCapture(0)  # Try built-in webcam
    if not camera.isOpened():
        print("Failed to open any camera. Video feed will not be available.")

# Set camera properties
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
camera.set(cv2.CAP_PROP_FPS, 30)

# Global variables for system state (to be updated by MQTT messages)
# These reflect the *actual* state of the physical system as reported by ESP32/Nano
traffic_lights = {
    'lane1': {'state': 'red', 'pedestrianState': 'red', 'countdown': 0}, # Corresponds to Zone A
    'lane2': {'state': 'red', 'pedestrianState': 'red', 'countdown': 0}, # Corresponds to Zone B
    'lane3': {'state': 'red', 'pedestrianState': 'red', 'countdown': 0}, # Corresponds to Zone C
    'lane4': {'state': 'red', 'pedestrianState': 'red', 'countdown': 0}  # Corresponds to Zone D
}

traffic_data = {
    'vehicleCounts': {'lane1': 0, 'lane2': 0, 'lane3': 0, 'lane4': 0}, # Updated by traffic/lanes/status
    'waitTimes': {'lane1': 0, 'lane2': 0, 'lane3': 0, 'lane4': 0}, # Can be derived or simulated
    'pedestrianRequests': {'lane1': False, 'lane2': False, 'lane3': False, 'lane4': False}, # Updated by button presses
    'emergencyVehicle': {'detected': False, 'direction': None}, # Updated by controlsalma EMERGENCY messages
    'autoMode': True # Can be toggled from UI, sent to Nano via AI
}

violations = [] # Stores detected violations

mqtt_connected = False
mqtt_client = None

# MQTT Configuration (consistent with ESP32 and traffic_detector.py)
MQTT_BROKER = "broker.hivemq.com"
MQTT_PORT = 1883
MQTT_TOPICS = [
    "traffic/buttonssalma",         # From ESP32 (button presses)
    "traffic/soundsalma",           # From ESP32 (sound sensor)
    "traffic/lcd_messagesalma",     # From AI (app.py) to ESP32 LCD
    "traffic/countdownsalma",       # From AI (app.py) to ESP32 7-segment
    "traffic/controlsalma",         # From AI (app.py) to ESP32/Nano (commands like CROSS, EMERGENCY, BUZZER)
    "traffic/violations/newsalma",  # From AI (app.py) (full violation data)
    "traffic/lanes/status",         # From AI (app.py) (vehicle counts per lane)
    "traffic/lights/current_statussalma" # NEW: From ESP32/Nano (actual LED states)
]

def on_connect(client, userdata, flags, rc):
    """Callback function when the MQTT client connects to the broker."""
    global mqtt_connected
    if rc == 0:
        mqtt_connected = True
        print("Connected to MQTT broker")
        for topic in MQTT_TOPICS:
            client.subscribe(topic)
            print(f"Subscribed to {topic}") # Debugging
        socketio.emit('mqtt_status', {'connected': True}) # Notify UI of MQTT connection status
    else:
        mqtt_connected = False
        print(f"Failed to connect to MQTT broker: {rc}")
        socketio.emit('mqtt_status', {'connected': False}) # Notify UI of MQTT connection status

def on_message(client, userdata, msg):
    """Callback function when an MQTT message is received."""
    global traffic_lights, traffic_data, violations

    topic = msg.topic
    payload_str = msg.payload.decode()
    
    print(f"Received MQTT message on {topic}: {payload_str}") # Always log raw payload for debugging

    try:
        # Handle JSON payloads
        if topic in [
            "traffic/buttonssalma",
            "traffic/soundsalma",
            "traffic/violations/newsalma",
            "traffic/lanes/status"
        ]:
            payload = json.loads(payload_str)
            if topic == "traffic/buttonssalma":
                handle_button_press(payload)
            elif topic == "traffic/soundsalma":
                # Handle sound sensor data if needed in Flask UI (e.g., display a sound indicator)
                print(f"Sound detected: {payload.get('sound_detected')}")
                socketio.emit('sound_detection_update', {'detected': payload.get('sound_detected')})
            elif topic == "traffic/violations/newsalma":
                handle_violation(payload)
            elif topic == "traffic/lanes/status":
                handle_lane_status(payload)
            
            # Emit the parsed JSON payload to the UI for general debugging/display
            socketio.emit('mqtt_message', {'topic': topic, 'payload': payload})

        # Handle string payloads (commands from AI to ESP32/Nano, or LED status from Nano/ESP32)
        elif topic == "traffic/controlsalma":
            handle_control_command(payload_str)
            # Emit raw string for control commands to UI
            socketio.emit('mqtt_message', {'topic': topic, 'payload': payload_str})
        
        elif topic == "traffic/lcd_messagesalma":
            # LCD message is primarily for the physical device, but can be displayed in UI
            print(f"LCD Message: {payload_str}")
            socketio.emit('lcd_message_update', {'message': payload_str}) # Emit to UI if needed
            socketio.emit('mqtt_message', {'topic': topic, 'payload': payload_str})

        elif topic == "traffic/countdownsalma":
            # Update countdown for relevant zones. This requires knowing which zone is active.
            # For now, we'll just emit the value. The UI would need logic to map this to the active green zone.
            countdown_value = int(payload_str)
            print(f"7-Segment Countdown: {countdown_value}")
            socketio.emit('countdown_update', {'value': countdown_value}) # Emit to UI for 7-segment display
            socketio.emit('mqtt_message', {'topic': topic, 'payload': payload_str})

        elif topic == "traffic/lights/current_statussalma":
            handle_led_status(payload_str)
            # Emit raw string for LED status to UI
            socketio.emit('mqtt_message', {'topic': topic, 'payload': payload_str})

    except json.JSONDecodeError:
        print(f"ERROR: Could not decode JSON from topic {topic}: {payload_str}")
    except ValueError as ve:
        print(f"ERROR: ValueError processing message on {topic}: {ve} - Payload: {payload_str}")
    except Exception as e:
        print(f"ERROR: Error processing MQTT message on {topic}: {e} - Payload: {payload_str}")

def handle_button_press(payload):
    """Handles button press messages from ESP32."""
    global traffic_data
    button_num = payload.get('button')
    state = payload.get('state')
    
    # Map button_num to lane (e.g., button 1 for lane1, etc.)
    # Assuming buttons are 1-indexed for lanes
    if button_num is not None:
        lane_name = f'lane{button_num}' 
        if lane_name in traffic_data['pedestrianRequests']:
            traffic_data['pedestrianRequests'][lane_name] = (state == "pressed")
            socketio.emit('pedestrian_request_update', {'lane': lane_name, 'state': (state == "pressed")})
            print(f"Pedestrian request for {lane_name}: {state}")

def handle_control_command(command_string):
    """Handles control commands sent by the AI (traffic_detector.py) to ESP32/Nano."""
    global traffic_lights, traffic_data
    print(f"Handling AI control command: {command_string}")

    if command_string.startswith("CROSS:"):
        phase_index = int(command_string.split(':')[1])
        # Update traffic_lights based on the phase logic from traffic_detector.py
        # This keeps the Flask UI's internal model consistent with AI's intent.
        if phase_index == 0: # A/C Green, B/D Red
            traffic_lights['lane1']['state'] = 'green'
            traffic_lights['lane3']['state'] = 'green'
            traffic_lights['lane2']['state'] = 'red'
            traffic_lights['lane4']['state'] = 'red'
            traffic_lights['lane1']['pedestrianState'] = 'red'
            traffic_lights['lane3']['pedestrianState'] = 'red'
            traffic_lights['lane2']['pedestrianState'] = 'green'
            traffic_lights['lane4']['pedestrianState'] = 'green'
        elif phase_index == 1: # A/C Orange, B/D Red
            traffic_lights['lane1']['state'] = 'orange'
            traffic_lights['lane3']['state'] = 'orange'
            traffic_lights['lane2']['state'] = 'red'
            traffic_lights['lane4']['state'] = 'red'
            traffic_lights['lane1']['pedestrianState'] = 'green' # Pedestrian green during orange
            traffic_lights['lane3']['pedestrianState'] = 'green'
            traffic_lights['lane2']['pedestrianState'] = 'red'
            traffic_lights['lane4']['pedestrianState'] = 'red'
        elif phase_index == 2: # B/D Green, A/C Red
            traffic_lights['lane1']['state'] = 'red'
            traffic_lights['lane3']['state'] = 'red'
            traffic_lights['lane2']['state'] = 'green'
            traffic_lights['lane4']['state'] = 'green'
            traffic_lights['lane1']['pedestrianState'] = 'green'
            traffic_lights['lane3']['pedestrianState'] = 'green'
            traffic_lights['lane2']['pedestrianState'] = 'red'
            traffic_lights['lane4']['pedestrianState'] = 'red'
        elif phase_index == 3: # B/D Orange, A/C Red
            traffic_lights['lane1']['state'] = 'red'
            traffic_lights['lane3']['state'] = 'red'
            traffic_lights['lane2']['state'] = 'orange'
            traffic_lights['lane4']['state'] = 'orange'
            traffic_lights['lane1']['pedestrianState'] = 'green'
            traffic_lights['lane3']['pedestrianState'] = 'green'
            traffic_lights['lane2']['pedestrianState'] = 'red'
            traffic_lights['lane4']['pedestrianState'] = 'red'
        
        # This update will be reinforced by handle_led_status for actual physical state
        socketio.emit('traffic_lights_update', traffic_lights) 

    elif command_string.startswith("AMBULANCE_EMERGENCY_ON"):
        traffic_data['emergencyVehicle']['detected'] = True
        try:
            zone_idx = int(command_string.split(':')[1])
            traffic_data['emergencyVehicle']['direction'] = f'lane{zone_idx + 1}'
        except IndexError:
            traffic_data['emergencyVehicle']['direction'] = 'all' # Default if no specific lane
        socketio.emit('emergency_update', traffic_data['emergencyVehicle'])
        
        # During emergency, all lights typically go red except the emergency lane
        for lane in traffic_lights:
            traffic_lights[lane]['state'] = 'red'
            traffic_lights[lane]['pedestrianState'] = 'green' # Pedestrians always green for safety
        if traffic_data['emergencyVehicle']['direction'] != 'all':
            traffic_lights[traffic_data['emergencyVehicle']['direction']]['state'] = 'green'
            traffic_lights[traffic_data['emergencyVehicle']['direction']]['pedestrianState'] = 'red'
        socketio.emit('traffic_lights_update', traffic_lights) # Update UI to show emergency lights

    elif command_string == "AMBULANCE_EMERGENCY_OFF":
        traffic_data['emergencyVehicle']['detected'] = False
        traffic_data['emergencyVehicle']['direction'] = None
        socketio.emit('emergency_update', traffic_data['emergencyVehicle'])
        # Lights will revert to normal cycle via subsequent CROSS commands from AI

    elif command_string == "BUZZER_PED_A_WARN_ON":
        # This indicates a pedestrian warning is active for Zone A (lane1)
        print("Pedestrian warning ON for Zone A (lane1)")
        socketio.emit('pedestrian_warning_status', {'lane': 'lane1', 'active': True})

    elif command_string == "BUZZER_PED_A_WARN_OFF":
        print("Pedestrian warning OFF for Zone A (lane1)")
        socketio.emit('pedestrian_warning_status', {'lane': 'lane1', 'active': False})

    elif command_string.startswith("VIOLATION:"):
        # Violation detected by AI, but the full violation data comes on traffic/violations/newsalma
        # This is a redundant message, but can be used for quick logging or a simple alert.
        print(f"Simple violation alert for lane: {command_string.split(':')[1]}")

    elif command_string == "auto":
        traffic_data['autoMode'] = True
        socketio.emit('mode_update', {'autoMode': True})
        print("AI set to Automatic Mode.")
    
    elif command_string == "manual":
        traffic_data['autoMode'] = False
        socketio.emit('mode_update', {'autoMode': False})
        print("AI set to Manual Mode.")

    else:
        print(f"Unhandled AI control command: {command_string}")

def handle_violation(payload):
    """Handles new violation data from traffic_detector.py."""
    global violations
    violation = {
        'id': payload.get('id', str(uuid.uuid4())),
        'timestamp': payload.get('timestamp', datetime.now().isoformat()),
        'type': payload.get('type', 'Unknown'),
        'lane': payload.get('lane', 'Unknown'), 
        'plate_number': payload.get('plate_number', 'Unknown'), 
        'image_path': payload.get('image_path', 'N/A'),
        'status': 'pending'
    }
    violations.append(violation)
    socketio.emit('violation_update', violation) # Emit new violation to UI
    print(f"New violation recorded: {violation}")

def handle_lane_status(payload):
    """Handles lane status updates (vehicle counts, emptiness) from traffic_detector.py."""
    global traffic_data
    if 'lanes' in payload:
        for lane_name, status in payload['lanes'].items():
            if lane_name in traffic_data['vehicleCounts']:
                traffic_data['vehicleCounts'][lane_name] = status['cars']
                # You might want to calculate wait times here based on cars and light state
                # For now, just update vehicle counts
                print(f"Lane {lane_name} - Cars: {status['cars']}, Ambulances: {status['ambulances']}, Empty: {status['empty']}")
        socketio.emit('traffic_data_update', traffic_data) # Emit general traffic data update

def handle_led_status(payload_str):
    """Handles actual LED status messages from ESP32/Nano, updating Flask's internal traffic_lights."""
    global traffic_lights
    # Expected format: "LED_STATUS:Z0_R,Z1_O,Z2_G,Z3_R;P0_G,P1_R,P2_R,P3_R"
    try:
        status_parts = payload_str[len("LED_STATUS:"):].split(';')
        
        # Process vehicle lights
        if len(status_parts) > 0:
            car_lights_str = status_parts[0]
            car_light_entries = car_lights_str.split(',')
            for entry in car_light_entries:
                if '_' in entry:
                    zone_info, color_char = entry.split('_')
                    if zone_info.startswith('Z'):
                        zone_idx = int(zone_info[1])
                        # Map zone_idx (0-3) to lane name (lane1-lane4)
                        lane_name = f'lane{zone_idx + 1}'
                        if lane_name in traffic_lights:
                            traffic_lights[lane_name]['state'] = color_char.lower()
                            # print(f"Updated {lane_name} car light to {color_char.lower()}") # Debugging
        
        # Process pedestrian lights
        if len(status_parts) > 1:
            ped_lights_str = status_parts[1]
            ped_light_entries = ped_lights_str.split(',')
            for entry in ped_light_entries:
                if '_' in entry:
                    zone_info, color_char = entry.split('_')
                    if zone_info.startswith('P'):
                        zone_idx = int(zone_info[1])
                        lane_name = f'lane{zone_idx + 1}'
                        if lane_name in traffic_lights:
                            traffic_lights[lane_name]['pedestrianState'] = color_char.lower()
                            # print(f"Updated {lane_name} ped light to {color_char.lower()}") # Debugging
        
        # Emit the updated traffic_lights state to all connected clients
        # This is the single source of truth for UI light states
        socketio.emit('traffic_lights_update', traffic_lights)

    except Exception as e:
        print(f"ERROR: Error parsing LED status message '{payload_str}': {e}")


def init_mqtt():
    """Initializes the MQTT client and connects to the broker."""
    global mqtt_client
    try:
        mqtt_client = mqtt.Client()
        mqtt_client.on_connect = on_connect
        mqtt_client.on_message = on_message # Set the general on_message callback
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        mqtt_client.loop_start() # Start the MQTT loop in a non-blocking way
        return True
    except Exception as e:
        print(f"Failed to initialize MQTT: {e}")
        return False

def publish_mqtt(topic, payload):
    """Publishes a message to an MQTT topic."""
    global mqtt_client, mqtt_connected
    if mqtt_client and mqtt_connected:
        try:
            # Check if payload is already a string (e.g., for direct commands)
            if isinstance(payload, str):
                mqtt_client.publish(topic, payload)
            else:
                mqtt_client.publish(topic, json.dumps(payload))
            print(f"Published MQTT message: '{payload}' to topic '{topic}'")
            return True
        except Exception as e:
            print(f"Failed to publish MQTT message: {e}")
            return False
    print(f"MQTT not connected, failed to publish to {topic}.")
    return False

def simulate_traffic_data():
    """Simulates traffic data when MQTT connection is not available."""
    # This simulation should ONLY run if MQTT fails to connect at startup.
    if not mqtt_connected:
        print("Running in simulation mode: Simulating traffic data...")
        while True:
            # Simulate traffic light changes
            zones = ['lane1', 'lane2', 'lane3', 'lane4'] # Use lane names for consistency
            states = ['red', 'orange', 'green'] 

            for zone in zones:
                if random.random() < 0.3:  # 30% chance to change state
                    traffic_lights[zone]['state'] = random.choice(states)
                    traffic_lights[zone]['countdown'] = random.randint(5, 60)
                    if traffic_lights[zone]['state'] == 'green':
                        traffic_lights[zone]['pedestrianState'] = 'red'
                    else: # red or orange
                        traffic_lights[zone]['pedestrianState'] = 'green'

            # Simulate vehicle detection (just counts for now, actual detections come from app.py)
            for zone in zones:
                traffic_data['vehicleCounts'][zone] = random.randint(0, 50)
                traffic_data['waitTimes'][zone] = random.randint(0, 120) # Simulate wait times

            # Emit updates to clients
            socketio.emit('traffic_lights_update', traffic_lights)
            socketio.emit('traffic_data_update', traffic_data) # Emit general traffic data update

            # Randomly simulate emergency
            if random.random() < 0.02:  # 2% chance
                emergency_data = {
                    'detected': True,
                    'direction': random.choice(zones)
                }
                socketio.emit('emergency_update', emergency_data)
            else:
                # If no emergency, ensure it's off
                if traffic_data['emergencyVehicle']['detected']:
                    traffic_data['emergencyVehicle']['detected'] = False
                    traffic_data['emergencyVehicle']['direction'] = None
                    socketio.emit('emergency_update', traffic_data['emergencyVehicle'])
            
            time.sleep(5)  # Update every 5 seconds
    else:
        print("MQTT connected successfully. Simulation thread will not start.")

def gen_frames():
    """Generator function for video streaming"""
    while True:
        success, frame = camera.read()
        if not success:
            print("Failed to read frame from camera")
            break
        else:
            try:
                # Process frame with TrafficDetector
                processed_frame = traffic_detector.process_frame(frame)
                
                # Add debug information
                print("Frame processed successfully")
                
                # Encode the frame
                ret, buffer = cv2.imencode('.jpg', processed_frame)
                if not ret:
                    print("Failed to encode frame")
                    continue
                    
                frame = buffer.tobytes()
                
                # Yield the frame in the format required by Flask
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                time.sleep(0.1)  # Add a small delay to control frame rate
            except Exception as e:
                print(f"Error processing frame: {e}")
                continue

@app.route('/')
def index():
    """Renders the main HTML page."""
    return render_template('index.html')

@app.route('/api/traffic-lights')
def get_traffic_lights():
    """API endpoint to get current traffic light states."""
    return jsonify(traffic_lights)

@app.route('/api/traffic-lights', methods=['POST'])
def update_traffic_lights_from_ui():
    """API endpoint to update traffic light states from the UI (manual override)."""
    data = request.json
    zone = data.get('zone') # e.g., 'zoneA'
    state = data.get('state') # e.g., 'red', 'orange', 'green'
    
    # Map UI 'zoneA' to internal 'lane1' for consistency with traffic_detector.py
    lane_map = {'zoneA': 'lane1', 'zoneB': 'lane2', 'zoneC': 'lane3', 'zoneD': 'lane4'}
    lane_name = lane_map.get(zone)

    if lane_name and state in ['red', 'orange', 'green']:
        # Publish to MQTT as a direct command for Nano/ESP32
        # Example command: "A:red", "B:green"
        # Note: This bypasses the AI's control_traffic_lights logic.
        # Only use for manual override from UI.
        zone_char = chr(ord('A') + int(lane_name[-1]) - 1) # Convert lane1 -> A, lane2 -> B etc.
        command = f"{zone_char}:{state}"
        publish_mqtt(traffic_detector.MQTT_CONTROL_TOPIC, command) # Use traffic_detector's topic

        # Update Flask's internal state immediately for responsiveness (will be overwritten by LED_STATUS later)
        traffic_lights[lane_name]['state'] = state
        if state == 'green':
            traffic_lights[lane_name]['pedestrianState'] = 'red'
        else: # red or orange
            traffic_lights[lane_name]['pedestrianState'] = 'green'
        
        socketio.emit('traffic_lights_update', traffic_lights) # Update UI immediately
        
        return jsonify({'success': True})
    
    return jsonify({'success': False, 'error': 'Invalid zone or state'}), 400

@app.route('/api/violations')
def get_violations():
    """API endpoint to get recorded violations."""
    return jsonify(violations)

@app.route('/api/violations/<violation_id>', methods=['PUT'])
def update_violation(violation_id):
    """API endpoint to update the status of a specific violation."""
    data = request.json
    status = data.get('status')
    
    for violation in violations:
        if violation['id'] == violation_id:
            violation['status'] = status
            socketio.emit('violation_status_update', {'id': violation_id, 'status': status})
            return jsonify({'success': True})
    
    return jsonify({'success': False, 'error': 'Violation not found'}), 404

@app.route('/api/violations/<violation_id>', methods=['DELETE'])
def delete_violation(violation_id):
    """API endpoint to delete a specific violation."""
    global violations
    violations = [v for v in violations if v['id'] != violation_id]
    socketio.emit('violation_deleted', {'id': violation_id})
    return jsonify({'success': True})

@app.route('/api/emergency', methods=['POST'])
def trigger_emergency():
    """API endpoint to trigger/deactivate emergency mode."""
    data = request.json
    active = data.get('active', False)
    location = data.get('location', 'lane1') # Use lane names for consistency with internal data

    if active:
        # Send emergency ON command to AI via MQTT
        # AI will then send specific commands to ESP32/Nano
        publish_mqtt(traffic_detector.MQTT_CONTROL_TOPIC, f"AMBULANCE_EMERGENCY_ON:{traffic_detector.lane_to_zone_index(location)}")
    else:
        # Send emergency OFF command
        publish_mqtt(traffic_detector.MQTT_CONTROL_TOPIC, "AMBULANCE_EMERGENCY_OFF")
    
    # Update Flask's internal state (will be reinforced by LED_STATUS later)
    traffic_data['emergencyVehicle']['detected'] = active
    traffic_data['emergencyVehicle']['direction'] = location if active else None
    socketio.emit('emergency_update', traffic_data['emergencyVehicle']) # Update UI immediately
    
    return jsonify({'success': True})

@app.route('/api/pedestrian-request', methods=['POST'])
def pedestrian_request():
    """API endpoint to send a pedestrian request."""
    data = request.json
    zone = data.get('zone') # This is 'zoneA', 'zoneB', etc. from UI

    # Map UI 'zoneA' to internal 'lane1' for consistency with traffic_detector.py
    lane_map = {'zoneA': 'lane1', 'zoneB': 'lane2', 'zoneC': 'lane3', 'zoneD': 'lane4'}
    lane_name = lane_map.get(zone)
    
    if lane_name in traffic_data['pedestrianRequests']:
        # Send pedestrian request to AI via MQTT (AI will then manage buzzer/lights)
        # The AI will send "BUZZER_PED_A_WARN_ON" etc.
        publish_mqtt(traffic_detector.MQTT_CONTROL_TOPIC, f"PED_BUTTON:{traffic_detector.lane_to_zone_index(lane_name)}")
        
        # Update Flask's internal state
        traffic_data['pedestrianRequests'][lane_name] = True
        socketio.emit('pedestrian_request', {'zone': lane_name}) # Emit with lane name
        
        return jsonify({'success': True})
    
    return jsonify({'success': False, 'error': 'Invalid zone'}), 400

@app.route('/api/mode', methods=['POST'])
def toggle_mode():
    """API endpoint to toggle between auto and manual mode."""
    data = request.json
    auto_mode = data.get('auto', True)
    
    # Send mode command to AI via MQTT
    command = "auto" if auto_mode else "manual"
    publish_mqtt(traffic_detector.MQTT_CONTROL_TOPIC, command)

    # Update Flask's internal state
    traffic_data['autoMode'] = auto_mode
    socketio.emit('mode_update', {'autoMode': auto_mode}) # Update UI
    
    return jsonify({'success': True, 'autoMode': auto_mode})

@app.route('/api/statistics')
def get_statistics():
    """API endpoint to get traffic statistics."""
    # This should ideally pull real-time data from traffic_data
    # For now, it uses the current counts from traffic_data
    stats = {
        'vehicleCount': [
            # You can add more time points here if you store historical data
            {'time': datetime.now().strftime("%H:%M:%S"), 
             'lane1': traffic_data['vehicleCounts'].get('lane1', 0), 
             'lane2': traffic_data['vehicleCounts'].get('lane2', 0), 
             'lane3': traffic_data['vehicleCounts'].get('lane3', 0), 
             'lane4': traffic_data['vehicleCounts'].get('lane4', 0)},
        ],
        'waitTimes': [
            {'lane': 'lane1', 'average': traffic_data['waitTimes'].get('lane1', 0), 'max': 0}, # Max not tracked
            {'lane': 'lane2', 'average': traffic_data['waitTimes'].get('lane2', 0), 'max': 0},
            {'lane': 'lane3', 'average': traffic_data['waitTimes'].get('lane3', 0), 'max': 0},
            {'lane': 'lane4', 'average': traffic_data['waitTimes'].get('lane4', 0), 'max': 0}
        ],
        'distribution': [
            {'name': 'lane1', 'value': traffic_data['vehicleCounts'].get('lane1', 0)},
            {'name': 'lane2', 'value': traffic_data['vehicleCounts'].get('lane2', 0)},
            {'name': 'lane3', 'value': traffic_data['vehicleCounts'].get('lane3', 0)},
            {'name': 'lane4', 'value': traffic_data['vehicleCounts'].get('lane4', 0)}
        ]
    }
    return jsonify(stats)


@app.route('/video_feed')
def video_feed():
    """Streams the processed video feed to the web UI."""
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/lane-status')
def get_lane_status():
    """API endpoint to get real-time lane status (vehicle counts) from traffic_detector."""
    return jsonify(traffic_detector.lane_counts)

# SocketIO Events
@socketio.on('connect')
def handle_connect():
    """Handles new client connections to SocketIO."""
    print('Client connected')
    emit('mqtt_status', {'connected': mqtt_connected}) # Send MQTT connection status
    emit('traffic_lights_update', traffic_lights) # Send current light states
    emit('emergency_update', traffic_data['emergencyVehicle']) # Send emergency status
    emit('traffic_data_update', traffic_data) # Send initial traffic data (vehicle counts, etc.)

@socketio.on('disconnect')
def handle_disconnect():
    """Handles client disconnections from SocketIO."""
    print('Client disconnected')

# Add cleanup when the app is shutting down
@app.teardown_appcontext
def cleanup(exception=None):
    """Clean up resources when the app is shutting down"""
    global camera
    if camera is not None:
        camera.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    # Initialize MQTT client
    mqtt_success = init_mqtt()
    if not mqtt_success:
        print("MQTT initialization failed, running in simulation mode")
    
    # Start simulation thread ONLY if MQTT connection failed at startup
    if not mqtt_success:
        simulation_thread = threading.Thread(target=simulate_traffic_data, daemon=True)
        simulation_thread.start()
    else:
        print("MQTT connected successfully. Simulation thread will not start.")
    
    # Run the Flask app with SocketIO
    socketio.run(app, debug=True, host='0.0.0.0', port=5000)
