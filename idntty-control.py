#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time
import paho.mqtt.client as mqtt
import subprocess
from idntty_locate import get_gps_location  # Import the GPS function

# MQTT setup
base_client_id = "idntty_control_client_id"
timestamp = int(time.time())
mqtt_client_id = f"{base_client_id}_{timestamp}"
mqtt_broker = "100.117.132.98"
mqtt_port = 1883
mqtt_command_topic = "asset/security/command"
mqtt_state_topic = "asset/security/state"
mqtt_location_topic = "asset/location"  # New topic for location updates
current_state = None
print("MQTT Client ID:", mqtt_client_id)

# GPIO setup for relay
arming_relay_pin = 23
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(arming_relay_pin, GPIO.OUT)

# MQTT Callback Functions
def on_log(client, userdata, level, buf):
    print("log: ", buf)

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe(mqtt_command_topic, 1)
    initial_status = "Ready"
    client.publish(mqtt_state_topic, payload=initial_status, qos=1, retain=True)
    global current_state
    current_state = initial_status
    print(f"Published initial status: {initial_status}")

def on_disconnect(client, userdata, rc):
    print(f"Disconnected with result code {rc}")
    if rc != 0:
        print("Unexpected disconnection. Reconnecting...")
        client.reconnect()

def on_message(client, userdata, msg):
    global current_state

    if msg.topic == mqtt_command_topic:
        command = msg.payload.decode()

        if command == "GET_LOCATION":
            latitude, longitude = get_gps_location()
            if latitude is not None and longitude is not None:
                location_payload = f"Latitude: {latitude}, Longitude: {longitude}"
                client.publish(mqtt_location_topic, payload=location_payload, qos=1, retain=True)
                print(f"Published location to {mqtt_location_topic}: {location_payload}")
            else:
                print("Failed to get GPS location")

        elif command != current_state:
            if command == "ARMED":
                GPIO.output(arming_relay_pin, GPIO.LOW)
                print("System Armed - Relay is OFF")
            elif command == "DISARMED":
                GPIO.output(arming_relay_pin, GPIO.HIGH)
                print("System Disarmed - Relay is ON")
            client.publish(mqtt_state_topic, payload=command, qos=1, retain=True)
            current_state = command

# Internet Connectivity Check
def check_internet():
    try:
        subprocess.check_call(["ping", "-c", "1", "8.8.8.8"])
        return True
    except subprocess.CalledProcessError:
        return False

while not check_internet():
    print("Waiting for internet connectivity...")
    time.sleep(10)

# MQTT Client Setup
mqtt_client = mqtt.Client(mqtt_client_id)
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.on_disconnect = on_disconnect

mqtt_client.connect(mqtt_broker, mqtt_port, 60)
mqtt_client.loop_start()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Script interrupted by the user. Cleaning up...")
    GPIO.cleanup()
    mqtt_client.loop_stop()
    mqtt_client.disconnect()
