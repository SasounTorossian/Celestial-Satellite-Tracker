import re
import time
import gps
import board
import busio
import adafruit_lsm303
import math
import geomag
import os
import RPi.GPIO as GPIO
import pigpio
import subprocess
from skyfield.api import load
from skyfield.api import Topos

def start_pigpiod():
    out = subprocess.Popen(['sudo', 'pigpiod'], 
						universal_newlines = True,
						stdout=subprocess.PIPE, 
						stderr=subprocess.STDOUT)
    time.sleep(2)
                        
def start_gpiod():
    out = subprocess.Popen(['sudo', 'gpsd', '/dev/ttyAMA0', '-F', '/var/run/gpsd.socket'], 
						universal_newlines = True,
						stdout=subprocess.PIPE, 
						stderr=subprocess.STDOUT)
    time.sleep(2)
                        
def setup_gpio():
    gpio = pigpio.pi()
    return gpio

def setup_stepper(gpio):
    print("setup_stepper")
    global DELAY
    DELAY = 0.01 #Delay between steps
    
    global DIR
    DIR = 20   #Direction of GPIO Pin
    
    global STEP
    STEP = 21  # Step GPIO Pin
    
    global SLP
    SLP = 16   # Sleep Pin
    
    # Microstep Resolution GPIO Pins
    MODE_PIN1 = 6
    MODE_PIN2 = 13
    MODE_PIN3 = 19

    gpio.set_mode(DIR, pigpio.OUTPUT)
    gpio.set_mode(STEP, pigpio.OUTPUT)
    gpio.set_mode(SLP, pigpio.OUTPUT)
    
    gpio.set_mode(MODE_PIN1, pigpio.OUTPUT)
    gpio.set_mode(MODE_PIN2, pigpio.OUTPUT)
    gpio.set_mode(MODE_PIN3, pigpio.OUTPUT)
    
    #RESOLUTION
    #'Full': (0, 0, 0),
    #'Half': (1, 0, 0),
    #'1/4': (0, 1, 0),
    #'1/8': (1, 1, 0), SELECTED
    #'1/16': (0, 0, 1),
    #'1/32': (1, 0, 1)}
    
    gpio.write(MODE_PIN1, 1)
    gpio.write(MODE_PIN2, 1)
    gpio.write(MODE_PIN3, 0)
    

def run_stepper(angle, gpio):
    print("run_stepper")
    if(angle < 0):
        angle += 360
    print("current angle: {}".format(run_stepper.current_angle))
    print("new angle: {}".format(angle))
    print("change in angle: {}".format(angle - run_stepper.current_angle))
    
    if((angle - run_stepper.current_angle) < -180):
        angle_change = 360 + (angle - run_stepper.current_angle)
        
    elif((angle - run_stepper.current_angle) > 180):
        angle_change = (angle - run_stepper.current_angle) - 360
        
    else:
        angle_change = angle - run_stepper.current_angle
    
    CW = 1 if angle_change >= 0 else 0
    step_count = int(abs(angle_change)/0.6)
    if(step_count <= 2):
        print("step count: {}".format(step_count))
        print("CW: {}".format(CW))
        print("increment too small, no movement >:(")
    else:
        print("step count: {}".format(step_count))
        print("CW: {}".format(CW))
        run_stepper.current_angle = angle
        print("updated current angle: {}".format(run_stepper.current_angle))
    
    gpio.write(DIR, CW)
    
    print('Exiting Sleep Mode')
    gpio.write(SLP, 1)
    
    for x in range (step_count*8):
        gpio.write(STEP, 1)
        time.sleep(DELAY)
        gpio.write(STEP, 0)
        time.sleep(DELAY)
    print('Entering Sleep Mode')
    gpio.write(SLP, 0)
run_stepper.current_angle = 0

def align_stepper(angle, gpio):
    print("align_stepper")
    if (angle <= 180):
        CW = 0
        step_count = int(angle/1.8)
    else:
        CW = 1
        step_count = int((360-angle)/0.6)
    
    print("step count: {}".format(step_count*8))
    print("CW: {}".format(CW))
    gpio.write(DIR, CW)
    
    print('Exiting Sleep Mode')
    gpio.write(SLP, 1)
    
    for x in range (step_count*8):
        gpio.write(STEP, 1)
        time.sleep(DELAY)
        gpio.write(STEP, 0)
        time.sleep(DELAY)
    print('Entering Sleep Mode')
    gpio.write(SLP, 0)

def setup_servo(gpio):
    print("setup_servo")
    gpio.set_PWM_frequency(18,50)

def run_servo(angle, gpio):
    print("run_servo")
    dc = (100/9)*(-angle) + 1500
    dc = round(dc, 0)
    print(dc)
    gpio.set_servo_pulsewidth(18, dc)
    time.sleep(0.25)

def setup_mag():
    print("setup_mag")
    #initialise i2c comm on the RPi3
    i2c = busio.I2C(board.SCL, board.SDA)
    #initialise the lsm303 accelorometer via i2c comm
    MAGsensor = adafruit_lsm303.LSM303(i2c)
    return MAGsensor

def setup_gps():
    print("setup_gps")
    #listen on port 2947 (gpsd) of local
    GPSsensor = gps.gps('localhost', '2947')
    #start streaming gps data
    GPSsensor.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)
    return GPSsensor

def get_heading(sensor):
    print("get_heading")
    '''
    #retreive accelerometer data
    raw_acc_x, raw_acc_y, raw_acc_z = sensor.raw_acceleration
    raw_mag_x, raw_mag_y, raw_mag_z = sensor.raw_magnetic

    #normalise the raw data from the X and Y axis of the accelerometer
    accXnorm = raw_acc_x/math.sqrt(raw_acc_x*raw_acc_x + raw_acc_y*raw_acc_y + raw_acc_z*raw_acc_z)
    accYnorm = raw_acc_y/math.sqrt(raw_acc_x*raw_acc_x + raw_acc_y*raw_acc_y + raw_acc_z*raw_acc_z)

    #calculate the pith and roll 
    pitch = math.asin(accXnorm)
    roll = -math.asin(accYnorm/math.cos(pitch))

    #caculate new pitch compensated X and Y magenetic values based of accelerometer data
    magXcomp = raw_mag_x*math.cos(pitch) + raw_mag_z*math.sin(pitch)
    magYcomp = raw_mag_x*math.sin(roll)*math.sin(pitch) + raw_mag_y*math.cos(roll) - raw_mag_z*math.sin(roll)*math.cos(pitch)

    #print heading and compensated heading
    #heading = (math.atan2(raw_mag_y,raw_mag_x)*180)/3.1415
    heading = (math.atan2(magYcomp, magXcomp)*180)/3.1415
    '''
    acc_x, acc_y, acc_z = sensor.acceleration
    mag_x, mag_y, mag_z = sensor.magnetic
    heading = (math.atan2(mag_y,mag_x)*180)/3.14
    
    if(heading < 0):
        heading += 360
    
    return heading

def get_lat_lon(session):
    print("get_lat_lon")
    #lat/lon of Manchester, UK. Used in event that GPS cannot conect
    default_lat = 53.48
    default_lon = -2.24
    gps_retry = 20
    while(1):
        if(gps_retry < 10):
            report = session.next()
            #print(report) #prints raw incoming gps data
            if report['class'] == 'TPV':
                if (hasattr(report, 'lat') and hasattr(report, 'lon')):
                    print('Latitude: ', report.lat)
                    print('Longitude: ', report.lon)                
                    return report.lat, report.lon
                else:
                    print('gps attributes not ready')
            else:
                print('Attempting to establish GPS connection')
                time.sleep(1)
            gps_retry += 1
        else:
            print('Cannot establish GPS connection, using default values')
            return default_lat, default_lon
            
def get_true_north(lat, lon, heading):
    print("get_true_north")
    mag_dec = geomag.declination(lat, lon)
    print('heading at this location is: {}'.format(heading))
    print('magnetic declination at this location is: {}'.format(mag_dec))
    
    if ((heading + mag_dec) < 0):
        true_north = 360 + (heading + mag_dec)
        
    elif ((heading + mag_dec) > 360):
        true_north = 360 - (heading + mag_dec)
        
    elif(heading > 180):
        true_north = (heading - mag_dec)
    
    else:
        true_north = (heading + mag_dec)
        
    print('True north is: {0:10.3f} from current position'.format(true_north))
    return true_north

def load_planet(planet_select):
    print("load_planet")
    planets = load('de421.bsp')
    return planets[planet_select]

def load_satellites(satellite_select):
    print("load_satellites")
    stations_url = 'http://celestrak.com/NORAD/elements/stations.txt'
    satellites = load.tle(stations_url)
    satellite = satellites[satellite_select]
    
    #get time difference between now and satellite epoch
    ts = load.timescale()
    t = ts.now()
    days = t - satellite.epoch
    
    #if difference is greater than 2 weeks, update tle with latest table
    if (abs(days) > 14):
        satellites = load.tle(stations_url, reload=True)
        satellite = satellites[satellite_select]
        
    return satellite

def load_current_location(earth, lat, lon):
    print("load_current_location")
    print(lat)
    print(lon)
    location_astrometric = earth + Topos(lat, lon)
    location_topocentric = Topos(lat, lon)
    return location_astrometric, location_topocentric

def get_planet_location(location_astrometric, planet):
    print("get_planet_location")
    ts = load.timescale()
    t = ts.now()
    astrometric = location_astrometric.at(t).observe(planet)
    alt, az, dis = astrometric.apparent().altaz()
    return alt.degrees, az.degrees, dis.km

def get_satellite_location(location_topocentric, satellite):
    print("get_satellite_location")
    ts = load.timescale()
    t = ts.now()
    difference = satellite - location_topocentric
    topocentric = difference.at(t)
    alt, az, dis = topocentric.altaz()
    return alt.degrees, az.degrees, dis.km
    
def main():
    #clear console screen
    os.system('clear')
    start_pigpiod()
    start_gpiod()
    
    #setup compass and gps
    MAGsensor = setup_mag()
    GPSsensor = setup_gps()
    
    #set up stepper and servo motors
    gpio = setup_gpio()
    setup_stepper(gpio)
    setup_servo(gpio)

    #get heading, latitude, and longitude
    heading = get_heading(MAGsensor)
    lat, lon = get_lat_lon(GPSsensor)
    
    #obtain true north, then allign stepper with true north
    true_north = get_true_north(lat, lon, heading)
    align_stepper(true_north, gpio)

    print('loading planets')
    earth =  load_planet('earth')
    moon = load_planet('moon')
    mars = load_planet('mars')
    
    print('loading satellites')
    iss = load_satellites('ISS (ZARYA)')
    
    print('loading current location')
    loc_ast, loc_top = load_current_location(earth, lat, lon)

    
    while(1):
        alt, az, dis  = get_satellite_location(loc_top, iss)
        #alt, az, dis  = get_planet_location(loc_ast, mars)
        #alt, az, dis = get_planet_location(loc_ast, moon)
        print('Altitude: {}'.format(alt))
        print('Azimuth: {}'.format(az))
        print('Distance: {}'.format(dis))
        print(' ')
        run_servo(alt, gpio)
        run_stepper(az, gpio)
        print('-----------------------------------------')
        time.sleep(20)
    
    time.sleep(1)

if __name__ == '__main__':
    try:
        main()
    except KeyboardError:
        pass
    except KeyboardInterrupt:
        gpio.stop()
        quit()
    except StopIteration:
        session = None
    finally:
        gpio.stop()
