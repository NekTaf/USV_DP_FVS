import RPi.GPIO as GPIO
import geopy.distance
from time import sleep
import serial, pynmea2, time, math
import time
from threading import Thread
import re
import adafruit_bno055
import board
from magnetic_field_calculator import MagneticFieldCalculator
from math import radians, cos, sin, asin, sqrt
from datetime import datetime



 
file = open("data","r+")
file.truncate(0)
file.close()

file = open("GPS","r+")
file.truncate(0)
file.close()

'''
file = open("data2","r+")
file.truncate(0)
file.close()
'''

latitude=0
longitude=0
imu=0
 

 

 
c=0
now=0
 
#from geographiclib.geodesic import Geodesic

#import RPi.GPIO as GPIO




in1=24
in2=23
en =25

enb=4
in4=20
in3=21

enc=26
in5=5
in6=6


end=16
in7=7
in8=8


GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


GPIO.setup(in1,GPIO.OUT)
GPIO.setwarnings(False)

GPIO.setup(in2,GPIO.OUT)
GPIO.setwarnings(False)

GPIO.setup(en,GPIO.OUT)
GPIO.setwarnings(False)

GPIO.setup(enc,GPIO.OUT)
GPIO.setwarnings(False)

GPIO.setup(in5,GPIO.OUT)
GPIO.setwarnings(False)

GPIO.setup(in6,GPIO.OUT)
GPIO.setwarnings(False)

GPIO.setup(in3,GPIO.OUT)
GPIO.setwarnings(False)

GPIO.setup(in4,GPIO.OUT)
GPIO.setwarnings(False)

GPIO.setup(enb,GPIO.OUT)
GPIO.setwarnings(False)




GPIO.setup(in7,GPIO.OUT)
GPIO.setwarnings(False)

GPIO.setup(in8,GPIO.OUT)
GPIO.setwarnings(False)

GPIO.setup(end,GPIO.OUT)
GPIO.setwarnings(False)





GPIO.output(in1,GPIO.LOW)
GPIO.setwarnings(False)

GPIO.output(in2,GPIO.LOW)
GPIO.setwarnings(False)

GPIO.output(in5,GPIO.LOW)
GPIO.setwarnings(False)

GPIO.output(in6,GPIO.LOW)
GPIO.setwarnings(False)

GPIO.output(in3,GPIO.LOW)
GPIO.setwarnings(False)

GPIO.output(in4,GPIO.LOW)
GPIO.setwarnings(False)

GPIO.output(in7,GPIO.LOW)
GPIO.setwarnings(False)

GPIO.output(in8,GPIO.LOW)
GPIO.setwarnings(False)


p=GPIO.PWM(en,25)
p2=GPIO.PWM(enb,25)
p3=GPIO.PWM(enc,25)
p4=GPIO.PWM(end,25)






#coords_1 = (54.9792, -1.6147) #Desired Destination
#coords_1 = (90, 0) #Desired Destination
#coords_1 =(54.979616352, -1.61617874392)

#coords_1 = ( 54.978140, -1.622371 )

#coords_1 = (54.97813017, -1.6224125)

#coords_1 = (54.97813033, -1.62239233)

coords_1 =(54.978200,-1.622482)



def calculate_initial_compass_bearing(pointA, pointB):

    if (type(pointA) != tuple) or (type(pointB) != tuple):
        raise TypeError("Only tuples are supported as arguments")

    lat1 = math.radians(pointA[0])
    lat2 = math.radians(pointB[0])
    diffLong = math.radians(pointB[1] - pointA[1])

    x = math.sin(diffLong) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1)
        * math.cos(lat2) * math.cos(diffLong))

    initial_bearing = math.atan2(x, y)
    initial_bearing = math.degrees(initial_bearing)
    compass_bearing = (initial_bearing + 360) % 360

    return compass_bearing





def GPS1():
    
    global longitude
    global latitude
    
    port_gps = "/dev/ttyACM0"
    GPS = serial.Serial(port_gps, baudrate=9600)
    
    while True:
        message = GPS.readline().decode()
        if message.find('GPGGA')>0:
            msg=pynmea2.parse(message)
            #print(round(msg.latitude,8),round(msg.longitude,8),msg.num_sats)
            latitude=msg.latitude
            longitude=msg.longitude
   
  
  
  

def IMU():
    
    i2c = board.I2C()
    sensor = adafruit_bno055.BNO055_I2C(i2c)
    
    global imu
    
    while True:
        sleep(0.005)
        try:
            imu=int(float(re.sub('[()]', '', str(sensor.euler)).split(',')[0]))
        except:
            print("Error Retrieving IMU Data")
            







def calculate_field_value(result):
    """Get and print field value."""
    field_value = result['field-value']
    declination = field_value['declination']
    #print('Declination:', declination['value'], declination['units'])
    #return(declination)
    return(declination['value'])



def haversine(cord1, cord2):
    """
    Calculate the great circle distance in kilometers between two points
    on the earth (specified in decimal degrees)
    """
    # convert decimal degrees to radians
    lon1=cord1[1]
    lat1=cord1[0]
    lon2 = cord2[1]
    lat2 = cord2[0]
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    # haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a))
    r = 6371 # Radius of earth in kilometers. Use 3956 for miles. Determines return value units.
    return c * r











def errorCalc(b,imu1):
    er=0
    
    if(0<b<90):
        if(0<=imu1<=90):
            er=b-imu1
        elif(90<imu1<=180):
            if(abs(b-imu1)<90):
                er=b-imu1
            else:
                er=b-imu1+180
        elif(180<=imu1<270):
            if(abs(b-imu1)<90):
                er=b-imu1
            else:
                er=b-imu1+180
        else:
            if(abs(b-imu1+180)<90):
                er=b-imu1+180
            else:
                er=b-imu1+360



    elif(90<=b<180):
        if(0<=imu1<90):
            if(abs(b-imu1)<90):
                er=b-imu1
            else:
                er=b-imu1-180
        elif(90<=imu1<180):
            er=b-imu1
        elif(180<=imu<270):
            if(abs(b-imu1)<90):
                er=b-imu1
            else:
                er=b-imu1+180
        else:
            er=b-imu1+180
            
            
            
    elif(180<=b<270):
        if(0<=imu1<90):
            er=b-imu1-180
        elif(90<=imu<180):
            if(abs(b-imu1)<90):
                er=b-imu1
            else:
                er=b-imu1-180
        elif(180<=b<270):
            if(abs(b-imu1)<90):
                er=b-imu1
            else:
                er=b-imu1+180
                
        else:
            if(abs(b-imu1)<90):
                er=b-imu1
            else:
                er=b-imu1+180
            
            
            
    else:
        if(0<=imu1<90):
            if(abs(b-imu1-180)<90):
                er=b-imu1-180
            else:
                er=b-imu1-360
        elif(90<=imu1<=180):
            er=(b-imu1-180)
        elif(180<imu1<=270):
            if(abs(b-imu1)<90):
                er=b-imu1
            else:
                er=b-imu1-180
        else:
            er=b-imu1
    
    return er




            
def control():
    
    start_time = time.time()
    
    errorsum=0
    lastError=0
    
    error=0
    
    direction=0
    c=0.05
    
    kp=130
    kd=23.4
    ki=0

    '''
    80, kd=20,ki=5

    #ku=250
    #tu=1.25
    
    #ku=200
    #tu=1.412 (1.4)
    
    p--> 0.6*Ku
    ti-->0.5*Tu
    td-->0.125*Tu

    Kp=K
    Ki=K/ti
    Kd=K*td
    '''
    
    global now
    
    loop=0
    error4=0
    now1=time.time()
    j=0
    
    Avrage_Distance=[0]*10
    
    while (True):
        now=time.time()
        
        t=time.time()-now1
        t = "{:.3f}".format(t)
        
        
        dt=c

    
        #print("IMU: "+str(imu))
        
        
        
        gps_coord=(latitude,longitude)
        
        #Distance=(float(geopy.distance.distance(coords_1, gps_coord).m))
        Distance=float(haversine(coords_1, gps_coord))*1000
        
        Bearing=(int(calculate_initial_compass_bearing((gps_coord),(coords_1))))#Bearing from North
        
        
        """
        calculator = MagneticFieldCalculator()
        
        wmm_calc_result = calculator.calculate(
            latitude=gps_coord[0],
            longitude=gps_coord[1],
            altitude=0,
            date=datetime.today().strftime('%Y-%m-%d')
        )
        """
        #print((calculate_field_value(wmm_calc_result)))
        
        #Bearing= Bearing +calculate_field_value(wmm_calc_result)
        

        
        b=Bearing
        
        if(j>9):
            j=0
         
        Avrage_Distance[j]=(Distance)
        Distance=sum(Avrage_Distance)/10
        
         
        Distance_error=abs(2-Distance)*10
        if (Distance_error>50):
            Distance_error=50
         
            
        print("Distance: ",Distance,"m ","Bearing: ",Bearing,"° ","IMU",imu)
        
         
        #print(str(loop)+'###')
        
        
        """
        if(loop<100):
            b=270
        else:
            b=180
            print(t)
         
        
        if(loop>=200):
            loop=0
        
        #print(loop)
        """
        
         
        error=errorCalc(b,imu)


       
         
        b1=b+90
        if (b1>360):
            b1-=360
        b2=b-90
        if (b2<0):
            b2+=360
        
        if(b1>b2):
        
            if(b2<imu<b1):
                direction=2 #Backwards
            else:
                direction=1 #Forward
                
        else:
            
            if(b1<imu<b2):
                direction=1 #Backwards
            else:
                direction=2 #Forward

        
 
       
       
        
       ### CCW is negative CW is positive
        
        error=error*1/9
        
        #integral clamping
        """
        if(ki*errorsum<=95 and ki*errorsum>=-95):
            errorsum+=(error*dt)
        
        else:
            errorsum=errorsum
        
        """
        
        errorsum+=(error*dt)
        
        Derror=(error-lastError)/dt
        
        lastError=error
         
        
        E = kp * error  + kd * Derror
            
        
        
        
        if(E<0):
            E1=E-12
            if(E1<-100):
                E1=-100

            #print(E1)
            p2.start(-E1)
            p.start(-E1)
            GPIO.output(in1,GPIO.HIGH)
            GPIO.output(in2,GPIO.LOW)
            GPIO.output(in3,GPIO.LOW)
            GPIO.output(in4,GPIO.HIGH)
            
        elif(E>0):
            E1=E+20
            
            if(E1>100):
                E1=100
            p2.start(E1)
            p.start(E1)
            GPIO.output(in3,GPIO.HIGH)
            GPIO.output(in4,GPIO.LOW)
            GPIO.output(in1,GPIO.LOW)
            GPIO.output(in2,GPIO.HIGH)
        
        elif(E==0):
            p2.start(E)
            p.start(E)
            GPIO.output(in3,GPIO.LOW)
            GPIO.output(in4,GPIO.LOW)
            GPIO.output(in1,GPIO.LOW)
            GPIO.output(in2,GPIO.LOW)
        
        #print(direction)
            
        #dist_error=Distance*50
        dist_error=0  
            
        if(Distance>0.5):
            
            if(direction==2):
                p3.start(40)
                GPIO.output(in5,GPIO.HIGH)
                GPIO.output(in6,GPIO.LOW)
                
                p4.start(40)
                GPIO.output(in7,GPIO.LOW)
                GPIO.output(in8,GPIO.HIGH)
            
            else:
                
                p3.start(40)
                GPIO.output(in5,GPIO.LOW)
                GPIO.output(in6,GPIO.HIGH)
                
                p4.start(40)
                GPIO.output(in7,GPIO.HIGH)
                GPIO.output(in8,GPIO.LOW)
        
        elif(Distance<0.5):
            
            if(direction==2):
                p3.start(dist_error)
                GPIO.output(in5,GPIO.HIGH)
                GPIO.output(in6,GPIO.LOW)
                
                p4.start(dist_error)
                GPIO.output(in7,GPIO.LOW)
                GPIO.output(in8,GPIO.HIGH)
            
            else:
                
                p3.start(dist_error)
                GPIO.output(in5,GPIO.LOW)
                GPIO.output(in6,GPIO.HIGH)
                
                p4.start(dist_error)
                GPIO.output(in7,GPIO.HIGH)
                GPIO.output(in8,GPIO.LOW)
                
        
        """  
        else:
                p3.start(0)
                GPIO.output(in5,GPIO.LOW)
                GPIO.output(in6,GPIO.LOW)
                
                p4.start(0)
                GPIO.output(in7,GPIO.LOW)
                GPIO.output(in8,GPIO.LOW)
        """
          
         
        
        E="{:.3f}".format(E)
        #print("PID Error: ",E)
        
        
        if(0<=imu<=360):
            with open("data", "a") as datafile:
                datafile.write(str(imu)+" "+str(t)+"\n")   
        '''        
        with open("data2", "a") as datafile:
            datafile.write(str(E)+" "+str(t)+"\n")
        '''   
        with open("GPS", "a") as datafile:
            datafile.write(str(gps_coord[0])+" "+str(gps_coord[1])+"\n")

        
        if(-time.time()+0.05+now>0):
            sleep(0.05-time.time()+now)
        else:
            continue 
        
        c=time.time()-now
        loop+=1
        
        #print(c)
        if(c>0.055):
            print(c)
        #print(loop)
        print("")
        #print(dt)
        
        j+=1
    
    
    
def main():
    
    Thread(target = IMU).start()
    
    
    Thread(target = GPS1).start()
    sleep(2)
    
    
    Thread(target = control).start()
    


if __name__ == "__main__":
    main()




 


       #SOSOSOS #https://stackoverflow.com/questions/1878907/how-can-i-find-the-difference-between-two-angles
        #https://www.electronicshub.org/raspberry-pi-l298n-interface-tutorial-control-dc-motor-l298n-raspberry-pi/
        #https://www.youtube.com/watch?v=2bganVdLg5Q


