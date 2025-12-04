from pyb import Pin,Timer

class Motor:
    '''A motor driver interface encapsulated in a Python class. Works with
       motor drivers using separate PWM and direction inputs such as the DRV8838
       drivers present on the Romi chassis from Pololu.'''
    
    def __init__(self, PWM_Pin: Pin, DIR_Pin: Pin, nSLP_Pin: Pin, tim:Timer, chan:int): # setup given appropriate parameters like pins and timers
        
        self.nSLP_pin = Pin(nSLP_Pin, mode=Pin.OUT_PP)
        self.DIR_pin = Pin(DIR_Pin, mode = Pin.OUT_PP)
        self.PWM_chan = tim.channel(chan, pin=PWM_Pin, mode=Timer.PWM,pulse_width_percent = 0)
       
    def set_effort(self, effort):    # sets signed positive or negative effort valus between -100 and 100 
       if (effort >0):
           self.DIR_pin.low()
           self.PWM_chan.pulse_width_percent(effort)
       else:
           self.DIR_pin.high()
           self.PWM_chan.pulse_width_percent(-effort)
    def enable(self): # put motor into coast mode
        self.nSLP_pin.high()
        
            
    def disable(self): # effort is 0 
        self.nSLP_pin.low()



