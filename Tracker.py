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
from skyfield.api import load
from skyfield.api import Topos

def setup_stepper():
    global DIR
    DIR = 20   #Direction of GPIO Pin
    global STEP
    STEP = 21  # Step GPIO Pin
    global SLP
    SLP = 16   # Sleep Pin
    MODE = (6, 13, 19) # Microstep Resolution GPIO Pins
    global DELAY
    DELAY = 0.01 #Delay between steps

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(DIR, GPIO.OUT)
    GPIO.setup(STEP, GPIO.OUT)
    GPIO.setup(SLP, GPIO.OUT)
    GPIO.setup(MODE, GPIO.OUT)
    RESOLUTION = {'Full': (0, 0, 0),
                  'Half': (1, 0, 0),
                  '1/4': (0, 1, 0),
                  '1/8': (1, 1, 0),
                  '1/16': (0, 0, 1),
                  '1/32': (1, 0, 1)}
    GPIO.output(MODE, RESOLUTION['1/8'])

def setup_servo():
    servoPIN = 18
    GPIO.setup(servoPIN, GPIO.OUT)
    pwm = GPIO.PWM(servoPIN, 50) # GPIO 18 for PWM with 50Hz
    pwm.start(7.5)
    return pwm

def run_stepper(step_count, CW):
    GPIO.output(DIR, CW)
    #print('Exiting Sleep Mode')
    GPIO.output(SLP, GPIO.HIGH)
    for x in range (step_count*8):
        GPIO.output(STEP, GPIO.HIGH)
        time.sleep(DELAY)
        GPIO.output(STEP, GPIO.LOW)
        time.sleep(DELAY)
    #print('Entering Sleep Mode')
    GPIO.output(SLP, GPIO.LOW)

def run_servo(angle, pwm):
    dc = (23/360)*angle + 7.5 #equation to convert angles to duty cycles
    pwm.ChangeDutyCycle(dc)
    time.sleep(0.1)

def setup_mag():
    #initialise i2c comm on the RPi3
    i2c = busio.I2C(board.SCL, board.SDA)
    #initialise the lsm303 accelorometer via i2c comm
    MAGsensor = adafruit_lsm303.LSM303(i2c)
    return MAGsensor

def setup_gps():
    #listen on port 2947 (gpsd) of local
    GPSsensor = gps.gps('localhost', '2947')
    #start streaming gps data
    GPSsensor.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)
    return GPSsensor

def get_heading(sensor):
    #retreive accelorometer data
    raw_acc_x, raw_acc_y, raw_acc_z = sensor.raw_acceleration
    raw_mag_x, raw_mag_y, raw_mag_z = sensor.raw_magnetic

    #normalise the raw data from the X and Y axis of the accelorometer
    accXnorm = raw_acc_x/math.sqrt(raw_acc_x*raw_acc_x + raw_acc_y*raw_acc_y + raw_acc_z*raw_acc_z)
    accYnorm = raw_acc_y/math.sqrt(raw_acc_x*raw_acc_x + raw_acc_y*raw_acc_y + raw_acc_z*raw_acc_z)

    #calculate the pith and roll 
    pitch = math.asin(accXnorm)
    roll = -math.asin(accYnorm/math.cos(pitch))

    #caculate new pitch compensated X and Y magenetic values based of accelorometer data
    magXcomp = raw_mag_x*math.cos(pitch) + raw_mag_z*math.sin(pitch)
    magYcomp = raw_mag_x*math.sin(roll)*math.sin(pitch) + raw_mag_y*math.cos(roll) - raw_mag_z*math.sin(roll)*math.cos(pitch)

    #print heading and compensated heading
    #heading = (math.atan2(raw_mag_y,raw_mag_x)*180)/3.1415
    heading_comp = (math.atan2(magYcomp, magXcomp)*180)/3.1415

    #if (heading_comp < 0):
    #    heading_comp += 360

    return heading_comp

def align_stepper_north(trueNorth):
    if(trueNorth > 0):
        CW = 0
    else:
        CW = 1
    alignStep = int(abs(trueNorth)/1.8)
    run_stepper(alignStep, CW)

def get_gps(session, heading):
    tryConnect = 1
    while(tryConnect == 1):
        report = session.next()
        #print(report)

        if report['class'] == 'TPV':
            tryConnect = 0
            if hasattr(report, 'lat'):
                print('Latitude: ', report.lat)
            if hasattr(report, 'lon'):
                print('Longitude: ', report.lon)
            if hasattr(report, 'alt'):
                print('Altitude: ', report.alt)
            getDec = 1
            if (hasattr(report, 'lat') and hasattr(report, 'lon') and getDec == 1):
                mag_dec = geomag.declination(report.lat, report.lon)
                #print('Magnetic decination at local area is: ', mag_dec)
                #print(' ')
                get_dec = 0

            trueNorth = heading +  geomag.declination(report.lat, report.lon)
            print('True north is: {0:10.3f}'.format(trueNorth))
            return trueNorth, report.lat, report.lon
            #print('Magnetic north is: {0:10.3f}'.format(heading))
            #print('Pitch compensated magnetic north is: {0:10.3f}'.format(heading_comp))
            #print('Raw accelorometer data is: ({0:10.3f}, {0:10.3f}, {0:10.3f})'.format(raw_acc_x, raw_acc_y, raw_acc_z))
            print(' ')

        else:
            print('Attempting to establish GPS connection')
            time.sleep(1)

def load_planet():
    print('loading file')
    planets = load('de421.bsp')
    '''
    switcher = {
        'sun': planets['sun'],
        'mercury': planets['mercury'],
        'venus': planets['venus'],
        'earth': planets['earth'],
        'moon': planets['moon'],
        'mars': planets['mars'],
        'jupiter': planets['jupiter'],
        'saturn': planets['saturn'],
        'uranus': planets['uranus'],
        'neptune': planets['neptune'],
        'pluto': planets['pluto'],
    }
    '''
    earth = planets['earth']
    mars = planets['mars']
    moon = planets['moon']
    print('returning')
    return earth, mars, moon
    #return switcher.get(planet, 'pass')

def load_satellites():
    stations_url = 'http://celestrak.com/NORAD/elements/stations.txt'
    satellites = load.tle(stations_url)
    satellites = satellites['ISS (ZARYA)']
    return satellites

def load_current_location(earth, lat, lon):
    location_astrometric = earth + Topos(lat, lon)
    location_topocentric = Topos(lat, lon)
    return location_astrometric, location_topocentric

def get_planet_location(location_astrometric, planet):
    ts = load.timescale()
    t = ts.now()
    astrometric = location_astrometric.at(t).observe(planet)
    alt, az, dis = astrometric.apparent().altaz()
    #altDec, azDec = degrees_to_decimal(alt, az)
    print('Altitude: {}'.format(alt))
    print('Azimuth: {}'.format(az))
    #print('Altitude Decimal: {}'.format(altDec))
    #print('Azimuth Decimal: {}'.format(azDec))
    print('Distance: {}'.format(dis))

def get_satellite_location(location_topocentric, satellite):
    ts = load.timescale()
    t = ts.now()
    difference = satellite - location_topocentric
    topocentric = difference.at(t)
    alt, az, dis = topocentric.altaz()
    print('ISS Altitude: {}'.format(alt))
    print('ISS Azimuth: {}'.format(az))
    print('ISS Distance: {}'.format(dis))

'''
def degrees_to_decimal(latDeg, lonDeg):
    latParts = re.split('[^\d\w]+', latDeg)
    latDegD = latParts[0]
    latDegM = latParts[1]
    latDegS = latParts[2]
    latDegDir = latParts[3]

    latParts = re.split('[^\d\w]+', lonDeg)
    lonDegD = lonParts[0]
    lonDegM = lonParts[1]
    lonDegS = lonParts[2]
    lonDegDir = lonParts[3]

    latDec = float(latDegD) + float(latDegM)/60 + float(latDegS)/(60*60);
    if latDegDir == 'E' or latDegDir == 'N':
        latDec *= -1

    lonDec = float(lonDegD) + float(lonDegM)/60 + float(lonDegS)/(60*60);
    if lonDegDir == 'E' or lonDegDir == 'N':
        lonDec *= -1

    return latDec, lonDec
'''


def main():
    #clear console screen
    os.system('clear')
    #variable to retreive magnetic declination in local area on first run
    MAGsensor = setup_mag()
    GPSsensor = setup_gps()
    setup_stepper()
    pwm = setup_servo()

    heading = get_heading(MAGsensor)
    trueNorth, lat, lon = get_gps(GPSsensor, heading)
    align_stepper_north(trueNorth)
    print('Lat: ', lat)
    print('Lon: ', lon)
    print(' ')

    print('loading planets')
    earth, mars, moon = load_planet()
    #mars =  load_planet()
    print('loading satellites')
    iss = load_satellites()
    print('loading current location')
    loc_ast, loc_top = load_current_location(earth, lat, lon)
    get_planet_location(loc_ast, mars)
    print(' ')
    get_planet_location(loc_ast, moon)
    print(' ')
    get_satellite_location(loc_top, iss)

    time.sleep(1)

if __name__ == '__main__':
    try:
        main()
    except KeyboardError:
        pass
    except KeyboardInterrupt:
        GPIO.cleanup()
        quit()
    except StopIteration:
        session = None
    finally:
        GPIO.cleanup()
