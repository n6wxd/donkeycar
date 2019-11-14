"""RosBot - interface to RosBot interface hat
Usage:
    RosBot.py
    RosBot.py <command>
    RosBot.py -h|--help
    RosBot.py -v|--version
Options:
    <command>  Command to exicute. 
        -  Stop - stop motor
            [' ', 'Run Motor %d.motorIdx at speed %d.motorPWM','runMotor', "M1", 100],
            [' ','pin Mode %n %d.pinSet','setPinmode',13,"input"],//设置pin状态
            [' ','digital write %n %d.pinMode','digitalwritePin',13,"high"],
            [' ','analog write %d.analogPin %n','analogwritePin',"13",100 ],
            ['r','digital read %n','digitalRead',11],
            ['r','analog read %d.Apin','analogRead',"A0"],
        - 
    -h --help  Show this screen.
    -v --version  Show version.

"""

from datetime import datetime
import donkeycar as dk
import re
import time

class RosBot:
    '''
    RosBot general purpose robot controller
    url: http://kittenbot.cc/
    '''
    import threading

    rosbot_device = None
    rosbot_lock = threading.Lock()

    def __init__(self, dev='/dev/ttyUSB0', baud=115200):
        from serial import Serial

        if RosBot.rosbot_device == None:
            RosBot.rosbot_device = Serial(dev, baud, timeout = 0.05)
            time.sleep(1)
            

        self.connected = False
        self.notifyConnection = False
        self.flag = 0
        self.motorMap = {'M1':0, 'M2':1, 'M3':2, 'M4':3}
        self.pwmMap = {'THROTTLE':0, 'STEERING':1}

        self.setup_port()
        print("Ready...")

        
    def setup_port(self):
        print("port open")
        print(self.send_msg("M0"))
        self.connected = True
        

    def get_status(self):
        if self.connected == False:
            status = 1
            msg = "Disconnected"
            print("M0");    # check if host app online
            self.send_msg("M0")
            return (status, msg)
        else:
            status = 2
            msg = "Connected"
            return (status, msg)

    # "m0" - sw version  
    def get_sw_version(self):
        return self.send_msg("M0")
        
        
     # "m1" - doPinMode       
    def set_pin_mode(self, pin, pinSet):
        if pinSet == "input": 
            return self.send_msg("M1 "+str(pin)+' 0')
       
        if pinSet=="output":
            return self.send_msg("M1 "+str(pin)+' 1')

        
     # "m2" - doDigitalWrite 
    def write_digital_pin(self, pin, pinMode):
        if pinMode == "high":
            return self.send_msg("M2 "+str(pin)+' 1')

        if pinMode == "low":
            return self.send_msg("M2 "+str(pin)+' 0')

            
    # "m3" - doDigitalRead 
    def read_digital_pin(self, pin):
        return self.send_msg("M3 "+str(pin)+' 1')


    # "m4" - doAnalogWrite 
    def write_analog_pin(self, apin, pin):
        return self.send_msg("M4 "+str(apin)+' '+str(pin))

        
    # "m5" - doAnalogRead 
    def read_analog_pin(self, pin):
        return self.send_msg("M5 "+str(pin)+' 2')


    # "m6" - doTone 
    def set_tone(self, pin, freq, time):
        return self.send_msg("M6 "+str(pin)+' '+str(freq)+' '+str(time))


    # "m7" - doServo 
    def set_servo(self, idx, degree, speed):
        return self.send_msg("M7 "+str(idx)+' '+str(degree)+' '+str(speed))


    # m8: - doEchoVin - read vin voltage
    def read_vin(self):
        return self.send_msg("M8")


    #"M9" - doRgb - rgb led
    def set_rgb_led(self, pin, pix, red, green, blue):
        return self.send_msg("M9 "+str(pin)+' '+str(pix)+' '+str(red)+' '+str(green)+' '+str(blue))    


    #"M10" - doButton   
    def read_button(self, pin):
        return self.send_msg("M10 "+str(pin))   
       
        
    #"M11" - doRgbBrightness - rgb brightness 
    def set_rgb_led(self, brightness):
        return self.send_msg("M11 "+str(brightness))      

        
    #"M30" - doExtIO   
    def ext_io(self, d12, d10, time):
        return self.send_msg("M30 "+str(d12)+' '+str(d10)+' '+str(time))
    
    
    #"M31" - doExtIOSet   
    def set_ext_io(self, brightness):
        return self.send_msg("M31 "+str(time))

    
    #"M100" - doStepperSingle - single stepper movement
    def set_stepper_single(self, idx, degree, speed):
        return self.send_msg("M100 "+str(idx)+' '+str(degree)+' '+str(speed))

    
    #"M101" - doStepperLine - move in distance
    def set_stepper_line(self, length):
        return self.send_msg("M101 "+str(length))

    
    #"M102" - doStepperTurn - turn in degree
    def set_stepper_turn(self, brightness):
        return self.send_msg("M102 "+str(angle))
    
    
    #"M103" - doStepperArc - draw arc
    def set_stepper_arc(self, diameter, ppm):
        return self.send_msg("M103 "+str(diameter)+' '+str(angle))
    
    
    #"M104" - doSetPPM -  set ppm
    def set_ppm(self, ppm):
        return self.send_msg("M104 "+str(ppm))
    
    
    #"M105" - doSetWheelBase - set wheel base
    def set_wheel_base(self, baseWidth):
        return self.send_msg("M105 "+str(baseWidth))


    #"M200" - doDcSpeed - set wheel base
    def set_motor(self, motor, speed):
        print("run motor "+motor+" "+str(speed)+"\n")
        return self.send_msg("M200 "+str(self.motorMap[motor])+' '+str(speed));


    #"M201" - doCarMove       
    def start_motor(self, motor, speed):
        print("run motor "+motor+" "+str(speed)+"\n")
        return self.send_msg("M201 "+str(self.motorMap[motor])+' '+str(speed));

    
    #"M203" - doMotorStop - motor stop        
    def stop_motor(self):
        return self.send_msg("M203");

        
    #"M204" - doMotorDualDelay - set wheel base        
    def start_motor_dual_delay(self, motor1, motor2, speed):
        print("run motor "+motor1+" "+motor2+" "+str(speed)+"\n")
        self.send_msg("M204 "+str(self.motorMap[motor])+' '+str(self.motorMap[motor])+' '+str(speed));


    #"M205" - doMotorFour - set wheel base        
    def start_motor_dual_delay(self, motor1, motor2, motor3, motor4):
        print("run motor "+motor1+" "+motor2+" "+motor3+" "+motor4+"\n")
        self.send_msg("M205 "+str(self.motorMap[motor])+' '+str(self.motorMap[motor])+' '+str(self.motorMap[motor])+' '+str(self.motorMap[motor]));

    #"M212" - doServoArray - set servo        
    def set_servo(self, idx, degree=90, speed=0, wait=0):
        print("set servo "+str(idx)+" "+str(degree)+" "+str(speed)+' '+str(wait)+"\n")
        self.send_msg("M212 "+str(idx)+' '+str(degree)+' '+str(speed)+' '+str(wait));

    #"M213" - doGeekServoArray - set servo (geek)       
    def set_servo2(self, idx, degree, speed=0):
        print("set servo (geeky) "+str(idx)+" "+str(degree)+" "+str(speed)+"\n")
        self.send_msg("M213 "+str(idx)+' '+str(degree)+' '+str(speed));

    #"M250" - Ping - ping      
    def ping(self):
        print("PING! \n")
        return self.send_msg("M250");

    #"M300" - rpm read      
    def read_rpm(self):
        return self.send_msg("M300");
    
    #"M999" - doReset - Reset RosBot Board       
    def do_reset(self):
        return self.send_msg("M999");

            
    def send_msg(self, msg):
        with RosBot.rosbot_lock:
            RosBot.rosbot_device.write((msg+"\n").encode('ascii'))
        ret = self.rosbot_readline()
        return ret

    
    def rosbot_readline(self):
        ret = None
        with RosBot.rosbot_lock:
            # expecting lines like
            # E n nnn n
            if RosBot.rosbot_device.inWaiting() > 8:
                ret = RosBot.rosbot_device.readline()

        if ret != None:
            ret.decode('utf-8')
            ret = ret.rstrip()
#            print(ret)

        return ret

    
class RosBotRCin:
    def __init__(self):
        self.inSteering = 0.0
        self.inThrottle = 0.0

        self.sensor = dk.parts.actuator.RosBot(0)

        RosBotRCin.LEFT_ANGLE = -1.0
        RosBotRCin.RIGHT_ANGLE = 1.0
        RosBotRCin.MIN_THROTTLE = -1.0
        RosBotRCin.MAX_THROTTLE =  1.0

        RosBotRCin.LEFT_PULSE = 496.0
        RosBotRCin.RIGHT_PULSE = 242.0
        RosBotRCin.MAX_PULSE = 496.0
        RosBotRCin.MIN_PULSE = 242.0


        self.on = True

        
    def map_range(self, x, X_min, X_max, Y_min, Y_max):
        '''
        Linear mapping between two ranges of values
        '''
        X_range = X_max - X_min
        Y_range = Y_max - Y_min
        XY_ratio = X_range/Y_range

        return ((x-X_min) / XY_ratio + Y_min)

    
    def update(self):
        rcin_pattern = re.compile('^I +([.0-9]+) +([.0-9]+).*$')

        while self.on:
            start = datetime.now()

            l = self.sensor.rosbot_readline()

            while l:
                # print("mw RosBotRCin line= " + l.decode('utf-8'))
                m = rcin_pattern.match(l.decode('utf-8'))

                if m:
                    i = float(m.group(1))
                    if i == 0.0:
                        self.inSteering = 0.0
                    else:
                        i = i / (1000.0 * 1000.0) # in seconds
                        i *= self.sensor.frequency * 4096.0
                        self.inSteering = self.map_range(i,
                                                         RosBotRCin.LEFT_PULSE, RosBotRCin.RIGHT_PULSE,
                                                         RosBotRCin.LEFT_ANGLE, RosBotRCin.RIGHT_ANGLE)

                    k = float(m.group(2))
                    if k == 0.0:
                        self.inThrottle = 0.0
                    else:
                        k = k / (1000.0 * 1000.0) # in seconds
                        k *= self.sensor.frequency * 4096.0
                        self.inThrottle = self.map_range(k,
                                                         RosBotRCin.MIN_PULSE, RosBotRCin.MAX_PULSE,
                                                         RosBotRCin.MIN_THROTTLE, RosBotRCin.MAX_THROTTLE)

                    # print("matched %.1f  %.1f  %.1f  %.1f" % (i, self.inSteering, k, self.inThrottle))
                l = self.sensor.rosbot_readline()

            stop = datetime.now()
            s = 0.01 - (stop - start).total_seconds()
            if s > 0:
                time.sleep(s)

                
    def run_threaded(self):
        return self.inSteering, self.inThrottle

    
    def shutdown(self):
        # indicate that the thread should be stopped
        self.on = False
        print('stopping RosBotRCin')
        time.sleep(.5)
