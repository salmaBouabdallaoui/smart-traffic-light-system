import cv2
import numpy as np
from ultralytics import YOLO
import paho.mqtt.client as mqtt
import json
import time
from datetime import datetime
import threading
import logging
import easyocr 
import os

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class TrafficDetector:
    def __init__(self):
        # Initialize YOLO model
        try:
            from ultralytics import YOLO
            # Download model if not present
            self.model = YOLO('yolov8n.pt')
            print("YOLO model loaded successfully")
            
            # Test the model with a small image
            test_img = np.zeros((100, 100, 3), dtype=np.uint8)
            test_results = self.model(test_img)
            print("YOLO model test successful")
        except Exception as e:
            print(f"Error loading YOLO model: {e}")
            raise
        
        # Initialize EasyOCR for license plate recognition (English language)
        # You might need to download models on first run, requires internet.
        self.reader = easyocr.Reader(['en']) 
        logging.info("EasyOCR reader initialized.")

        # Ensure 'violations' directory exists
        if not os.path.exists("violations"):
            os.makedirs("violations")
            logging.info("Created 'violations' directory.")
        
        # Initialize MQTT client
        self.mqtt_client = None
        self.mqtt_connected = False
        
        # Store real-time MQTT data received from ESP32/Nano
        # This will store the *actual* state of the lights and sensor data
        self.mqtt_data = {
            'lane1': {'cars': 0, 'ambulances': 0, 'empty': True, 'light_state_car': 'red', 'light_state_ped': 'red'}, # Zone A
            'lane2': {'cars': 0, 'ambulances': 0, 'empty': True, 'light_state_car': 'red', 'light_state_ped': 'red'}, # Zone B
            'lane3': {'cars': 0, 'ambulances': 0, 'empty': True, 'light_state_car': 'red', 'light_state_ped': 'red'}, # Zone C
            'lane4': {'cars': 0, 'ambulances': 0, 'empty': True, 'light_state_car': 'red', 'light_state_ped': 'red'}  # Zone D
        }
        
        # Initialize lane counts for current frame detection
        self.lane_counts = {
            'lane1': {'cars': 0, 'ambulances': 0, 'empty': True}, # Zone A
            'lane2': {'cars': 0, 'ambulances': 0, 'empty': True}, # Zone B
            'lane3': {'cars': 0, 'ambulances': 0, 'empty': True}, # Zone C
            'lane4': {'cars': 0, 'ambulances': 0, 'empty': True}  # Zone D
        }
        
        # Define lane regions as quadrilaterals (adjust these based on your camera position and perspective)
        # Format: [(top-left-x, top-left-y), (top-right-x, top-right-y), (bottom-right-x, bottom-right-y), (bottom-left-x, bottom-left-y)]
        # These are example coordinates for a 1280x720 frame. YOU MUST ADJUST THESE FOR YOUR CAMERA FEED.
        self.lane_regions = {
            'lane1': np.array([ # Top approach (Zone A)
                [500, 0], [780, 0], [700, 300], [580, 300]
            ], np.int32),
            'lane2': np.array([ # Right approach (Zone B)
                [1280, 250], [1280, 470], [900, 400], [900, 320]
            ], np.int32),
            'lane3': np.array([ # Bottom approach (Zone C)
                [780, 720], [500, 720], [580, 420], [700, 420]
            ], np.int32),
            'lane4': np.array([ # Left approach (Zone D)
                [0, 470], [0, 250], [380, 320], [380, 400]
            ], np.int32)
        }

        # Traffic light states and control variables (these are the AI's *intended* states)
        self.traffic_lights = {
            'lane1': 'red', 'lane2': 'red', 'lane3': 'red', 'lane4': 'red'
        }
        self.current_phase = 0 # 0: A/C Green, 1: A/C Orange, 2: B/D Green, 3: B/D Orange
        self.phase_timers = {
            0: 15, # A/C Green duration (seconds)
            1: 3,  # A/C Orange duration (seconds)
            2: 15, # B/D Green duration (seconds)
            3: 3   # B/D Orange duration (seconds)
        }
        self.last_phase_change_time = time.time()
        self.emergency_active = False
        self.emergency_lane = None
        self.pedestrian_warning_active = False # Flag for pedestrian buzzer warning

        # MQTT topics (consistent with ESP32 code - updated to ...salma)
        self.MQTT_CONTROL_TOPIC = "traffic/controlsalma"
        self.MQTT_LANE_STATUS_TOPIC = "traffic/lanes/status" # This is still AI's outbound lane status (no salma)
        self.MQTT_VIOLATIONS_TOPIC = "traffic/violations/newsalma" # Added salma for consistency
        self.MQTT_BUTTON_PRESS_TOPIC = "traffic/buttonssalma" # ESP32 publishes button presses here
        self.MQTT_SOUND_SENSOR_TOPIC = "traffic/soundsalma"   # ESP32 publishes sound detection here
        self.MQTT_7SEG_DISPLAY_TOPIC = "traffic/countdownsalma" # Topic for 7-segment display countdown
        self.MQTT_LED_STATUS_TOPIC = "traffic/lights/current_statussalma" # NEW: ESP32/Nano publishes actual LED states here

        self.setup_mqtt() # Call setup_mqtt after all topics are defined

        # Start the traffic light control thread
        self.control_thread = threading.Thread(target=self.control_traffic_lights)
        self.control_thread.daemon = True
        self.control_thread.start()

    def setup_mqtt(self):
        """Setup MQTT client with error handling"""
        try:
            self.mqtt_client = mqtt.Client(client_id="TrafficAI_PythonClient") # Unique Client ID for Python app
            self.mqtt_client.on_connect = self.on_connect
            self.mqtt_client.on_disconnect = self.on_disconnect
            
            # Assign specific callbacks for different topics for clarity and better handling
            self.mqtt_client.message_callback_add(self.MQTT_BUTTON_PRESS_TOPIC, self.on_button_message)
            self.mqtt_client.message_callback_add(self.MQTT_SOUND_SENSOR_TOPIC, self.on_sound_message)
            self.mqtt_client.message_callback_add(self.MQTT_LED_STATUS_TOPIC, self.on_led_status_message)
            # Default callback for any other unhandled messages
            self.mqtt_client.on_message = self.on_default_message 
            
            # Try to connect to MQTT broker
            try:
                self.mqtt_client.connect("broker.hivemq.com", 1883, 60) # Using broker.hivemq.com as in ESP32 code
                self.mqtt_client.loop_start() # Start the non-blocking loop
                self.mqtt_connected = True
                logging.info("Successfully connected to MQTT broker")
            except Exception as e:
                logging.error(f"Failed to connect to MQTT broker: {e}")
                logging.warning("Continuing without MQTT functionality")
                self.mqtt_connected = False
        except Exception as e:
            logging.error(f"Error setting up MQTT client: {e}")
            self.mqtt_connected = False

    def on_connect(self, client, userdata, flags, rc):
        """Callback when connected to MQTT broker"""
        if rc == 0:
            self.mqtt_connected = True
            logging.info("Connected to MQTT broker")
            # Subscribe to topics where ESP32 publishes data
            client.subscribe(self.MQTT_BUTTON_PRESS_TOPIC)
            client.subscribe(self.MQTT_SOUND_SENSOR_TOPIC)
            client.subscribe(self.MQTT_LED_STATUS_TOPIC) # Subscribe to new LED status topic
            logging.info(f"Subscribed to {self.MQTT_BUTTON_PRESS_TOPIC}, {self.MQTT_SOUND_SENSOR_TOPIC}, and {self.MQTT_LED_STATUS_TOPIC}")
        else:
            self.mqtt_connected = False
            logging.error(f"Failed to connect to MQTT broker with code: {rc}")

    def on_disconnect(self, client, userdata, rc):
        """Callback when disconnected from MQTT broker"""
        self.mqtt_connected = False
        logging.warning(f"Disconnected from MQTT broker with code: {rc}")

    def on_default_message(self, client, userdata, msg):
        """Default callback for unhandled MQTT messages"""
        logging.info(f"Unhandled MQTT message received on topic '{msg.topic}': {msg.payload.decode()}")

    def on_button_message(self, client, userdata, msg):
        """Callback for incoming MQTT messages from button presses (from ESP32)"""
        try:
            data = json.loads(msg.payload.decode())
            button_num = data.get('button')
            state = data.get('state')
            logging.info(f"Button {button_num} {state} detected by ESP32.")
            # Implement logic here for pedestrian requests etc.
            # Example: if button_num == 1 and state == "pressed": self.trigger_pedestrian_request()
        except json.JSONDecodeError:
            logging.error(f"Failed to decode JSON from button press message: {msg.payload.decode()}")
        except Exception as e:
            logging.error(f"Error processing button message: {e}")

    def on_sound_message(self, client, userdata, msg):
        """Callback for incoming MQTT messages from sound sensor (from ESP32)"""
        try:
            data = json.loads(msg.payload.decode())
            sound_detected = data.get('sound_detected')
            if sound_detected:
                logging.warning("Loud sound detected by ESP32! Consider emergency action.")
                # This could trigger an emergency sequence if not already active
                # Example: self.activate_emergency_if_needed()
        except json.JSONDecodeError:
            logging.error(f"Failed to decode JSON from sound sensor message: {msg.payload.decode()}")
        except Exception as e:
            logging.error(f"Error processing sound message: {e}")

    def on_led_status_message(self, client, userdata, msg):
        """Callback for incoming MQTT messages with LED status (from ESP32/Nano)"""
        # Expected format: "LED_STATUS:Z0_R,Z1_O,Z2_G,Z3_R;P0_G,P1_R,P2_R,P3_R"
        payload_str = msg.payload.decode()
        logging.info(f"Received LED status: {payload_str}")
        
        if payload_str.startswith("LED_STATUS:"):
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
                                lane_name = f'lane{zone_idx + 1}'
                                if lane_name in self.mqtt_data:
                                    self.mqtt_data[lane_name]['light_state_car'] = color_char.lower()
                                    # logging.debug(f"Updated {lane_name} car light to {color_char.lower()}") # Debugging
                
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
                                if lane_name in self.mqtt_data:
                                    self.mqtt_data[lane_name]['light_state_ped'] = color_char.lower()
                                    # logging.debug(f"Updated {lane_name} ped light to {color_char.lower()}") # Debugging
            except Exception as e:
                logging.error(f"Error parsing LED status message '{payload_str}': {e}")
        else:
            logging.warning(f"Unexpected LED status format: {payload_str}")


    def publish_mqtt_message(self, topic, payload):
        """Helper to publish messages to MQTT broker"""
        if self.mqtt_connected and self.mqtt_client:
            try:
                self.mqtt_client.publish(topic, payload)
                # logging.info(f"Published MQTT message: '{payload}' to topic '{topic}'") # Too frequent, uncomment for specific debugging
            except Exception as e:
                logging.error(f"Error publishing MQTT message '{payload}' to topic '{topic}': {e}")
                # self.mqtt_connected = False # Reconnection handled by loop_start

    def is_in_lane(self, box_center, lane_polygon):
        """Check if a point (center of detection box) is within a quadrilateral lane region"""
        x, y = box_center
        # cv2.pointPolygonTest returns positive if inside, negative if outside, 0 if on edge
        return cv2.pointPolygonTest(lane_polygon, (int(x), int(y)), False) >= 0

    def process_frame(self, frame):
        try:
            # Get frame dimensions
            height, width = frame.shape[:2]
            print(f"Processing frame of size: {width}x{height}")
            
            # Calculate center coordinates
            center_x = width // 2
            center_y = height // 2
            
            # Define red zone size (100x100 pixels)
            zone_size = 250
            half_size = zone_size // 2
            
            # Create red zone centered in frame
            self.red_line_zone = np.array([
                [center_x - half_size, center_y - half_size],  # Top-left
                [center_x + half_size, center_y - half_size],  # Top-right
                [center_x + half_size, center_y + half_size],  # Bottom-right
                [center_x - half_size, center_y + half_size],  # Bottom-left
            ], np.int32)

            # Run YOLO detection
            print("Running YOLO detection...")
            results = self.model(frame, verbose=False)
            print(f"YOLO detection completed. Found {len(results[0].boxes)} objects")
            
            # Reset lane counts
            for lane in self.lane_counts:
                self.lane_counts[lane]['cars'] = 0
                self.lane_counts[lane]['ambulances'] = 0
                self.lane_counts[lane]['empty'] = True

            # Draw red zone
            cv2.polylines(frame, [self.red_line_zone], isClosed=True, color=(0, 0, 255), thickness=2)

            # Process detections
            for result in results:
                for box, cls in zip(result.boxes.xyxy, result.boxes.cls):
                    label = self.model.names[int(cls)]
                    print(f"Detected object: {label}")

                    # Check for vehicles
                    if label in ["car", "truck", "bus"]:
                        x1, y1, x2, y2 = map(int, box)
                        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

                        # Check if center is in red zone
                        if cv2.pointPolygonTest(self.red_line_zone, (cx, cy), False) >= 0:
                            print(f"Violation detected: {label} in red zone")
                            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                            cv2.putText(frame, "INFRACTION", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                        else:
                            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            return frame
        except Exception as e:
            print(f"Error in process_frame: {e}")
            return frame

    def publish_lane_status(self):
        """Publish lane status (from AI's detection) to MQTT if connected"""
        if not self.mqtt_connected or self.mqtt_client is None:
            logging.warning("MQTT not connected, skipping lane status publish.")
            return
            
        try:
            status_data = {
                'timestamp': datetime.now().isoformat(),
                'lanes': {
                    lane: {'cars': self.lane_counts[lane]['cars'], 
                           'ambulances': self.lane_counts[lane]['ambulances'], 
                           'empty': self.lane_counts[lane]['empty']} 
                    for lane in self.lane_counts
                }
            }
            self.mqtt_client.publish(self.MQTT_LANE_STATUS_TOPIC, json.dumps(status_data))
            # logging.info("Published lane status.") # Too frequent, uncomment for specific debugging
        except Exception as e:
            logging.error(f"Error publishing lane status: {e}")

    def control_traffic_lights(self):
        """Control traffic lights based on lane status and phase logic"""
        while True:
            if not self.mqtt_connected or self.mqtt_client is None:
                time.sleep(1)
                continue
            
            current_time = time.time()

            # --- Emergency Override Logic ---
            # Check for ambulances in any lane based on AI's detection (self.lane_counts)
            ambulance_detected = False
            for lane, status in self.lane_counts.items(): # Use self.lane_counts for AI detection
                if status['ambulances'] > 0:
                    ambulance_detected = True
                    if not self.emergency_active or self.emergency_lane != lane:
                        logging.warning(f"Ambulance detected in {lane}! Activating emergency mode.")
                        self.emergency_active = True
                        self.emergency_lane = lane
                        # Send command to ESP32 to activate emergency lights
                        self.publish_mqtt_message(self.MQTT_CONTROL_TOPIC, f"AMBULANCE_EMERGENCY_ON:{self.lane_to_zone_index(lane)}")
                        # Update AI's internal intended light states
                        for l in self.traffic_lights:
                            self.traffic_lights[l] = 'red'
                        self.traffic_lights[lane] = 'green'
                        self.last_phase_change_time = current_time # Reset timer for emergency duration
                    break # Only prioritize one ambulance at a time
            
            if self.emergency_active and not ambulance_detected:
                # If emergency was active but no ambulance is detected anymore (e.g., it passed)
                # Keep emergency green for at least 10s even if ambulance disappears immediately
                if current_time - self.last_phase_change_time > 10: 
                    logging.info("Ambulance passed. Deactivating emergency mode.")
                    self.emergency_active = False
                    self.emergency_lane = None
                    self.publish_mqtt_message(self.MQTT_CONTROL_TOPIC, "AMBULANCE_EMERGENCY_OFF")
                    # Reset to a default phase (e.g., all red, then start normal cycle)
                    self.set_lights_for_phase(0) # Start normal cycle from Phase 0
                    self.last_phase_change_time = current_time
                    self.pedestrian_warning_active = False # Ensure buzzer warning is off
            
            # During emergency, send 0 to 7-segment display
            if self.emergency_active:
                self.publish_mqtt_message(self.MQTT_7SEG_DISPLAY_TOPIC, "0")


            # --- Normal Traffic Light Sequencing (if no emergency) ---
            if not self.emergency_active:
                time_in_current_phase = current_time - self.last_phase_change_time
                current_phase_duration = self.phase_timers[self.current_phase]

                # Adjust duration based on lane emptiness (simple example)
                # Use self.lane_counts for AI's decision on emptiness
                if self.current_phase == 0: # A/C Green
                    if self.lane_counts['lane1']['empty'] and self.lane_counts['lane3']['empty']:
                        current_phase_duration = 5 # Shorten if both A and C are empty
                elif self.current_phase == 2: # B/D Green
                    if self.lane_counts['lane2']['empty'] and self.lane_counts['lane4']['empty']:
                        current_phase_duration = 5 # Shorten if both B and D are empty
                
                # Pedestrian Buzzer Warning Logic (for Zone A, which is part of Phase 0/1)
                # This logic is based on the AI's current phase, not the actual light state.
                # Assumes pedestrian green for Zone A is active when Lane A/C is RED/ORANGE
                # and pedestrian red for Zone A is active when Lane A/C is GREEN
                if self.current_phase == 2 or self.current_phase == 3: # When A/C are RED/ORANGE (pedestrian A is GREEN)
                    # Trigger warning a few seconds before A/C turns green (and pedestrian A turns RED)
                    if time_in_current_phase >= (current_phase_duration - 3) and not self.pedestrian_warning_active:
                        self.publish_mqtt_message(self.MQTT_CONTROL_TOPIC, "BUZZER_PED_A_WARN_ON")
                        self.pedestrian_warning_active = True
                        logging.info("Sending pedestrian warning ON for Zone A.")
                else: # When A/C are GREEN (pedestrian A is RED)
                    if self.pedestrian_warning_active:
                        self.publish_mqtt_message(self.MQTT_CONTROL_TOPIC, "BUZZER_PED_A_WARN_OFF")
                        self.pedestrian_warning_active = False
                        logging.info("Sending pedestrian warning OFF for Zone A.")

                # Calculate and send countdown to 7-segment display
                remaining_time = max(0, int(current_phase_duration - time_in_current_phase))
                self.publish_mqtt_message(self.MQTT_7SEG_DISPLAY_TOPIC, str(remaining_time))

                if time_in_current_phase >= current_phase_duration:
                    self.current_phase = (self.current_phase + 1) % len(self.phase_timers)
                    self.set_lights_for_phase(self.current_phase)
                    self.last_phase_change_time = current_time
            
            time.sleep(0.5) # Check traffic light logic every 0.5 seconds

    def set_lights_for_phase(self, phase):
        """Sets traffic light states (AI's internal model) and sends commands to ESP32 based on the current phase"""
        if phase == 0: # A/C Green, B/D Red
            self.traffic_lights['lane1'] = 'green'
            self.traffic_lights['lane3'] = 'green'
            self.traffic_lights['lane2'] = 'red'
            self.traffic_lights['lane4'] = 'red'
            self.publish_mqtt_message(self.MQTT_CONTROL_TOPIC, "CROSS:0") # Command for ESP32 to set A/C green
            self.publish_mqtt_message("traffic/lcd_messagesalma", "A/C Vert - B/D Rouge") # LCD message
        elif phase == 1: # A/C Orange, B/D Red
            self.traffic_lights['lane1'] = 'orange'
            self.traffic_lights['lane3'] = 'orange'
            self.traffic_lights['lane2'] = 'red'
            self.traffic_lights['lane4'] = 'red'
            self.publish_mqtt_message(self.MQTT_CONTROL_TOPIC, "CROSS:1") # Command for ESP32 to set A/C orange
            self.publish_mqtt_message("traffic/lcd_messagesalma", "A/C Orange - B/D Rouge") # LCD message
        elif phase == 2: # B/D Green, A/C Red
            self.traffic_lights['lane1'] = 'red'
            self.traffic_lights['lane3'] = 'red'
            self.traffic_lights['lane2'] = 'green'
            self.traffic_lights['lane4'] = 'green'
            self.publish_mqtt_message(self.MQTT_CONTROL_TOPIC, "CROSS:2") # Command for ESP32 to set B/D green
            self.publish_mqtt_message("traffic/lcd_messagesalma", "B/D Vert - A/C Rouge") # LCD message
        elif phase == 3: # B/D Orange, A/C Red
            self.traffic_lights['lane1'] = 'red'
            self.traffic_lights['lane3'] = 'red'
            self.traffic_lights['lane2'] = 'orange'
            self.traffic_lights['lane4'] = 'orange'
            self.publish_mqtt_message(self.MQTT_CONTROL_TOPIC, "CROSS:3") # Command for ESP32 to set B/D orange
            self.publish_mqtt_message("traffic/lcd_messagesalma", "B/D Orange - A/C Rouge") # LCD message
        
        logging.info(f"Traffic lights set for Phase {phase}. Current AI states: {self.traffic_lights}")

    def lane_to_zone_index(self, lane_name):
        """Converts lane name (e.g., 'lane1') to a zone index (0-3)"""
        if lane_name == 'lane1': return 0
        if lane_name == 'lane2': return 1
        if lane_name == 'lane3': return 2
        if lane_name == 'lane4': return 3
        return -1 # Should not happen

def main():
    # Ensure 'violations' directory exists
    import os
    if not os.path.exists('violations'):
        os.makedirs('violations')

    # Initialize camera with proper settings
    cap = cv2.VideoCapture(1)  # Use default camera (usually built-in webcam)
    
    # Set camera resolution and properties
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)  # Set to 30 FPS
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)  # Enable autofocus if available
    
    # Initialize detector
    detector = TrafficDetector()
    
    logging.info("Starting video processing loop...")
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                logging.error("Failed to read frame from camera. Retrying...")
                time.sleep(1)  # Wait a bit before retrying
                continue
                
            # Process frame
            processed_frame = detector.process_frame(frame)
            
            # Display frame
            cv2.imshow('Traffic Detection', processed_frame)
            
            # Break loop on 'q' press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                logging.info("'q' pressed. Exiting.")
                break
    except KeyboardInterrupt:
        logging.info("Keyboard interrupt received. Exiting.")
    except Exception as e:
        logging.error(f"Error in main loop: {e}")
    finally:
        # Clean up
        cap.release()
        cv2.destroyAllWindows()
        logging.info("Application terminated.")

if __name__ == "__main__":
    main()

