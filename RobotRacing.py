from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase, Car
from pybricks.tools import wait, StopWatch
from pybricks.iodevices import XboxController 
from pybricks.geometry import Matrix


'''
 Vorgehen zur Nutzung:
1. Programm herunterladen und Programm starten
2. XBOX-Controller starten und pairing-starten
3. Spike sollte sich jetzt automatisch verbinden und das Programm starten
Danach verbindet sich der XBOX-Controller automatisch bei Programmstart
'''

################################################################################
# Anpassungen vornehmen
################################################################################
'''
Um die Autos am besten nutzen zu können, müssen evtl. einige Parameter eingestellt werden:
1) Motoren:
    Welche Motoren habe ich? --> SteeringMotor & Antrieb
    2 oder 1 Antriebsmotoren --> MehrereAntriebsMotoren
2) Lenkung einstellen
    Drehmoment_Limits_finden einstellen (0-100)
    Zu gering: Roboter findet Lenkbereich nicht bzw. ist zu klein
    Zu hoch: Roboter droht, sich auf Dauer zu beschädigen
3) Geschwindigkeit & Boost Einstellen
'''

# Geschwindigkeits- & Boost-Einstellungen
Max_Speed = 100         # in %    Maximum Speed
CapNorm = 0.75          # in %    used for Boost-Function
BoostmaxTime = 2        # in Sec  states how long the boost is available for
BoostCooldownTime = 6   # in Sec  Cooldown time for boost function

# Roboter-Aufbau
IsTank = False #Car-Steering oder Tank-Steering
MehrereAntriebsMotoren = False 
Drehmoment_Limits_finden = 70 # Wie hart soll der Motor gegen die Endpunkte fahren, um die Limits seiner Steuerung zu finden?

################################################################################
# restliche Deklarationen
################################################################################

h = 100 # helligkeit der Matrize
Bluetooth_icon = Matrix(
    [
    [h,h,h,0,0],
    [h,0,0,h,0],
    [h,h,h,0,0],
    [h,0,0,h,0],
    [h,h,h,0,0]
    ]
)
Go_icon = Matrix(
    [
    [0,0,h,0,0],
    [0,h,0,h,0],
    [h,0,h,0,h],
    [0,h,0,h,0],
    [h,0,0,0,h]
    ]
)
Boost_icon = Matrix(
    [
    [0,0,h,0,0],
    [0,h,h,h,0],
    [h,0,h,0,h],
    [0,0,h,0,0],
    [0,h,h,h,0]
    ]
)
hub = PrimeHub()

hub.speaker.volume(70)
hub.speaker.beep(300,150)
hub.speaker.beep(400,150)
hub.speaker.beep(500,150)

hub.display.icon(Bluetooth_icon)

MyController = XboxController()

hub.display.icon(Go_icon)
hub.speaker.beep(600,150)
hub.speaker.beep(700,150)

MyController.rumble(
    power    = 50,
    duration = 200,
    count    = 2,
    delay    = 50
)

################################################################################
# Globale Variable, die von mehreren Funktionen aufgerufen werden:
################################################################################
BoostActive = False
boostready  = False
BoostSW     = StopWatch()

################################################################################
# Steuerung
################################################################################

if IsTank:
    print("Tank")
    drive_base = DriveBase(
        left_motor  = Motor(Port.A),
        right_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE),
        wheel_diameter= 56,
        axle_track = 96
    )
    drive_base.settings(
        straight_speed = 900,
        straight_acceleration = 1000,
        turn_rate = 500,
        turn_acceleration = 1000
    )

else:
    print("Car")
    SteeringMotor = Motor(Port.A,Direction.COUNTERCLOCKWISE)
    
    if MehrereAntriebsMotoren:
        Antrieb = [
            Motor(Port.B,Direction.COUNTERCLOCKWISE),
            Motor(Port.C)
        ]
    else:
        Antrieb = Motor(Port.B, Direction.COUNTERCLOCKWISE)

    car = Car(
        steer_motor  = SteeringMotor,
        drive_motors = Antrieb,
        torque_limit = Drehmoment_Limits_finden # wenn der Wert zu niedrig ist, werden die harten Limits der Lenkung nicht erkannt
    )
    car.drive_speed(0)
    car.drive_power(0)



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

def translate(value, leftMin, leftMax, rightMin, rightMax): # "map"-function
    # Figure out how 'wide' each range is
    leftSpan  = leftMax  - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

def Calculate_Boost(SpeedIN):
    global Pressed, BoostActive, boostready, BoostSW, BoostCooldownTime
    ReturnSpeed = 0
    if (not BoostActive) and (Button.LB in Pressed) and (BoostSW.time() > BoostCooldownTime*1000):
        BoostSW.reset()
        BoostActive = True
        print("started boost")
        hub.speaker.beep(900,200)
        hub.light.blink(color=Color.CYAN,durations=[150,100])
        MyController.rumble(
            power=50,
            duration=50,
            count=1,
            delay=50
        )

    if not BoostActive:
        ReturnSpeed = SpeedIN * CapNorm
        if (BoostSW.time() > BoostCooldownTime*1000):
            hub.light.blink(color=Color.GREEN,durations=[200,500])
            hub.display.icon(Boost_icon)
            if not boostready:
                MyController.rumble(
                    power=100,
                    duration=75,
                    count=1,
                    delay=50
                )
            boostready = True
    else:
        print(BoostSW.time())
        if BoostSW.time() < BoostmaxTime*1000:
            ReturnSpeed=MotorSpeed
        else:
            BoostActive = False
            print("boost over")
            hub.light.on(color=Color.YELLOW)
            hub.speaker.beep(800,100)
            hub.display.icon(Go_icon)
            boostready = False

    return int(ReturnSpeed)
    

def MotorControl(MySpeed,MySteer):
    if IsTank:    
        drive_base.drive(
            speed = MySpeed, 
            turn_rate = MySteer
        )
    else:
        car.drive_power(
            power = MySpeed # Number in %
        )
        car.steer(
            percentage = MySteer
        )


################################################################################
# Main Loop
################################################################################

while True:
    # Auslesen XBOX-Controller - Erfassen von Geschwindigkeit & Lenken
    Pressed = MyController.buttons.pressed()
    Control_Values = GetSteeringValues() # Returns a Tupel of Speed and Steering

    # Verarbeiten der Werte
    MotorSpeed = Control_Values[0]      # -100% bis 100%
    MotorSteer = Control_Values[1] * -1 # -100% bis 100%

    # Boost verarbeiten und Geschwindigkeit anpassen
    MotorSpeed = int(Calculate_Boost(MotorSpeed))

    # Wenn zu wenig gelenkt wird, soll der Lenk-Motor ausgeschaltet werden.
    if abs(MotorSteer) < 10:
        MotorSteer=0
        if not IsTank:
            SteeringMotor.stop

    print("Speed:","{: 05d}".format(MotorSpeed),"Steering:","{: 04d}".format(MotorSteer),"   ",BoostSW.time())
    
    # Übergabe der Werte an die Motor-Steuerung
    MotorControl(MotorSpeed,MotorSteer)

    # Ausschalten über XBOX-Button
    if Button.GUIDE in Pressed:
        raise SystemExit

    wait(10)