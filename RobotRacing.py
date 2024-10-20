from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pybricks.iodevices import XboxController 

################################################################################
# Einstellungen
################################################################################

# Roboter-Aufbau - Car-Steering oder Tank-Steering
IsTank = False

# Steuerungseinstellungen
VideoGameSteering = True

################################################################################
# Deklarationen
################################################################################
hub = PrimeHub()




################################################################################
# Funktionen
################################################################################

def GetSteeringValues(): # Returns a Tupel of Speed and Steering
    
    CurrSpeed = 50 # Speed Value from -100 to 100
    CurrSteering = 0 # Steering Value from -180 to 180

    return CurrSpeed, CurrSteering

def MotorControll():



################################################################################
# Main Loop
################################################################################
while True:
    # Auslesen XBOX-Controller - Erfassen von Geschwindigkeit & Lenken
    MySteering = GetsteeringValues() # Returns a Tupel

    # Verarbeiten der Werte
    MotorSpeed = MySteering[0]
    MotorSteer = MySteering[1]

    # Übergabe der Werte an die Motor-Steuerung
    MotorControll(MotorSpeed,MotorSteer)
