#!/usr/bin/env python
# -*- encoding: UTF-8 -*-



import qi
import argparse
import sys
import time
import math





def main(session):


    motion_service  = session.service("ALMotion")
   
    motion_service.setStiffnesses("LArm", 1.0)
    motion_service.setStiffnesses("RArm", 1.0)
    motion_service.setMoveArmsEnabled(False,False)  

    


    names  = ["LShoulderPitch","RShoulderPitch"] 
    angles  = [math.radians(10),math.radians(10)]
    fractionMaxSpeed  = 0.2
    motion_service.setAngles(names, angles, fractionMaxSpeed)
    time.sleep(5)

    names  = ["LElbowRoll","RElbowRoll"] 
    angles  = [math.radians(-60),math.radians(60)]
    fractionMaxSpeed  = 0.2
    motion_service.setAngles(names, angles, fractionMaxSpeed)
    time.sleep(5)

    names  = ["LShoulderRoll","RShoulderRoll"] 
    angles  = [math.radians(40),math.radians(-40)]
    fractionMaxSpeed  = 0.2
    motion_service.setAngles(names, angles, fractionMaxSpeed)
    time.sleep(5)


    names  = ["LElbowYaw","RElbowYaw"] 
    angles  = [math.radians(0),math.radians(0)]
    fractionMaxSpeed  = 0.2
    motion_service.setAngles(names, angles, fractionMaxSpeed)
    time.sleep(5)

    names  = ["LWristYaw","RWristYaw"] 
    angles  = [math.radians(-90),math.radians(90)]
    fractionMaxSpeed  = 0.2
    motion_service.setAngles(names, angles, fractionMaxSpeed)
    time.sleep(5)
    
    

    




if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="172.18.33.122",
                        help="Robot IP address. On robot or Local Naoqi: use '172.18.33.122'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()
    session = qi.Session()
    

    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    main(session)
