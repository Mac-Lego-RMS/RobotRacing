from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase, Car
from pybricks.tools import wait, StopWatch
from pybricks.iodevices import XboxController 


'''
 Vorgehen:
1. Programm herunterladen und Programm starten
2. XBOX-Controller starten und pairing-starten
3. Spike sollte sich jetzt automatisch verbinden und das Programm starten
Danach verbindet sich der XBOX-Controller automatisch bei Programmstart
'''

################################################################################
# Einstellungen
################################################################################

# Steuerungseinstellungen
VideoGameSteering = True

# Roboter-Aufbau - Car-Steering oder Tank-Steering
IsTank = True

# Geschwindigkeits-Einstellungen
Max_Speed = 1000 # in °/s 
CapNorm = 0.5 # used for Boost-Function
BoostmaxTime = 2 # in Sec, states how long the boost is available for
BoostCooldownTime = 6 # in Sec, Cooldown time for boost function



if IsTank:
    drive_base = DriveBase(
        left_motor  = Motor(Port.A),
        right_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE),
        wheel_diameter=56,
        axle_track=96
    )
    drive_base.settings(
        straight_speed=900,
        straight_acceleration=1000,
        turn_rate=500,
        turn_acceleration=1000
    )

else:
    car = Car(
        steer_motor=Motor(Port.A),
        drive_motors=Motor(Port.B),
        torque_limit=100
    )
    car.drive_speed(0)

################################################################################
# restliche Deklarationen
################################################################################
hub = PrimeHub()

hub.speaker.volume(80)
hub.speaker.beep(400,100)
MyController = XboxController()
hub.speaker.beep(500,100)
MyController.rumble(
    power=50,
    duration=200,
    count=2,
    delay=50
)
BoostActive=False
BoostSW = StopWatch()
################################################################################
# Funktionen
################################################################################

def GetSteeringValues(): # Returns a Tupel of Speed and Steering
    global Pressed
    CurrSteering = MyController.joystick_left()[0]

    if (Button.RB in Pressed): # direction determined by right bumper
        CurrSpeed = -1 * MyController.triggers()[1] # Speed Value from -100 to 100
    else:
        CurrSpeed = MyController.triggers()[1] # Speed Value from -100 to 100
    
    # Speed Value from -100 to 100
    # Steering Value from -100 to 100
    return CurrSpeed, CurrSteering


def MotorControll(MySpeed,MySteer):
    if IsTank:    
        drive_base.drive(
            speed=MySpeed,
            turn_rate=MySteer
        )
    else:
        car.drive_speed(
            speed=MySpeed
        )
        car.steer(
            percentage=MySteer
        )
def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

################################################################################
# Main Loop
################################################################################
while True:
    # Auslesen XBOX-Controller - Erfassen von Geschwindigkeit & Lenken
    Pressed = MyController.buttons.pressed()
    MySteering = GetSteeringValues() # Returns a Tupel

    # Verarbeiten der Werte

    MotorSpeed = MySteering[0] * 10 # -1000 bis 1000


    if (not BoostActive) and (Button.LB in Pressed) and (BoostSW.time() > BoostCooldownTime*1000):
        BoostSW.reset()
        BoostActive = True
        print("started boost")
        hub.speaker.beep(900,200)
        hub.light.blink(color=Color.CYAN,durations=[150,100])

    if not BoostActive:
        MotorSpeed = MotorSpeed * CapNorm
        hub.light.on(color=Color.YELLOW)
        if (BoostSW.time() > BoostCooldownTime*1000):
            hub.light.blink(color=Color.GREEN,durations=[200,500])
    else:
        print(BoostSW.time())
        if BoostSW.time() < BoostmaxTime*1000:
            MotorSpeed=MotorSpeed
        else:
            BoostActive = False
            print("boost over")
            hub.speaker.beep(800,100)

    MotorSpeed = int(MotorSpeed)


    MotorSteer = MySteering[1] * -1 # -100 bis 100
    if abs(MotorSteer) <20:
        MotorSteer=0
    else:
      MotorSteer = int(translate(MotorSteer,-100,100,-300,300))


    #print("Speed",format(MotorSpeed,000),"Steering:",MotorSteer)

    print("Speed:","{: 05d}".format(MotorSpeed),"Steering","{: 04d}".format(MotorSteer),"   ",BoostSW.time())
    # Übergabe der Werte an die Motor-Steuerung
    MotorControll(MotorSpeed,MotorSteer)

    wait(10)