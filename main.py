import network
import time
import urequests
import json
from machine import Pin, I2C
from time import sleep
import machine
import bme280
import dht
from imu import MPU6050
import uasyncio as asyncio
import gc

ssid = "pophome_2.4G"
password = "0818813830"

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(ssid, password)

sensor = dht.DHT22(Pin(22))
#sensor = dht.DHT11(Pin(22))

#MPU6050
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
imu = MPU6050(i2c)

def restart_pico():
    print("Restarting the Pico...")
    machine.reset()

def reset_wifi():
    print("Resetting Wi-Fi connection...")
    wlan.active(False)  # Disable the interface
    time.sleep(2)       # Wait for 2 seconds
    wlan.active(True)   # Re-enable the interface
    # Reconnect using your credentials
    wlan.connect(ssid, password)
    # Wait until connected or timeout
    max_wait = 10
    while max_wait > 0:
        status = wlan.status()
        if status == 3:
            print("Reconnected to Wi-Fi")
            break
        max_wait -= 1
        print("Waiting for Wi-Fi connection...")
        time.sleep(1)
    if max_wait == 0:
        print("Failed to reconnect to Wi-Fi")
        
def check_memory_and_reboot():
    free_memory = gc.mem_free()
    print("Free memory:", free_memory)
    blink_on(2, 1)
    if free_memory < 20000:  # Example threshold in bytes
        print("Low memory! Restarting...")
        restart_pico()

async def allSensor():
    
    #DHT22
    sensor.measure()
    dht_temp = sensor.temperature()
    dht_hum = sensor.humidity()
    dht_temp_f = dht_temp * (9/5) + 32.0
    
    
    #MPU6050
    mpu_ax=round(imu.accel.x,2)
    mpu_ay=round(imu.accel.y,2)
    mpu_az=round(imu.accel.z,2)
    mpu_gx=round(imu.gyro.x)
    mpu_gy=round(imu.gyro.y)
    mpu_gz=round(imu.gyro.z)
    mpu_temp=round(imu.temperature,2)
    
    
    sensor_data = {
        "DHT22":
        {
            "temperature": dht_temp,
            "humidity": dht_hum,
            "temperature_f": dht_temp_f
        },
        "MPU6050":
        {
            "temperature": mpu_temp,
            "accel":
                {
                    "x": mpu_ax,
                    "y": mpu_ay,
                    "z": mpu_az
                },
            "gyro":
                {
                    "x": mpu_gx,
                    "y": mpu_gy,
                    "z": mpu_gz
                }
        }
        }
    return sensor_data   
    
led = Pin("LED", Pin.OUT)   
def blink_on(time, interval):
    for _ in range(time):
        led.on()
        sleep(interval)
        led.off()
        sleep(interval)
    
    
async def connect_to_wifi():
    max_wait = 10
    while max_wait > 0:
        status = wlan.status()
        if status < 0 or status >= 3:
            break
        max_wait -= 1
        print('Waiting for connection...')
        await asyncio.sleep(1)  # Non-blocking sleep

    if status == 3:
        print('Connected')
        print('IP:', wlan.ifconfig()[0])
        blink_on(5, 0.5)
        return True
    else:
        blink_on(10, 0.5)
        print('Failed to connect, status:', status)
        return False

async def main():
    if await connect_to_wifi():
        while True:
            try:
                sensor_data = await allSensor()
                json_data = json.dumps(sensor_data)

                url = "https://script.google.com/macros/s/AKfycbwbvH19lbGvQn4uVDGdecbQ-AYItid9RYGwmCc0TdfNw219aEqhwbgkvn-vH_ShdnPE/exec"
                
                try:
                    response = urequests.post(url, data=json_data, timeout=20)
                    print("Server response status:", response.status_code)
                    print("Server response:", response.text)
                    response.close()
                    check_memory_and_reboot()
                except Exception as e:
                    print("Error sending data:", e)
                    blink_on(10, 0.5)
                    reset_wifi()
                
            except Exception as e:
                blink_on(10, 0.5)
                print("Unexpected error in main loop:", e)
            
            await asyncio.sleep(10)

asyncio.run(main())
