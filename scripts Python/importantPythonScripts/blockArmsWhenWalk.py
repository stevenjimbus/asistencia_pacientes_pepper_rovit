#!/usr/bin/env python
# -*- encoding: UTF-8 -*-



import qi
import argparse
import sys
import time
import math






def disableAutonomousLife(session):
    # Get the service ALAutonomousLife.

    life_service = session.service("ALAutonomousLife")
    if life_service.getAutonomousAbilityEnabled("BasicAwareness") == True:
        life_service.setAutonomousAbilityEnabled("BasicAwareness", False)

    if life_service.getAutonomousAbilityEnabled("ListeningMovement") == True:
        life_service.setAutonomousAbilityEnabled("ListeningMovement", False)

    if life_service.getAutonomousAbilityEnabled("BackgroundMovement") == True:
        life_service.setAutonomousAbilityEnabled("BackgroundMovement", False)

    if life_service.getAutonomousAbilityEnabled("AutonomousBlinking") == True:
        life_service.setAutonomousAbilityEnabled("AutonomousBlinking", False)

    if life_service.getAutonomousAbilityEnabled("SpeakingMovement") == True:
        life_service.setAutonomousAbilityEnabled("SpeakingMovement", False)

def main(session):


    motion_service  = session.service("ALMotion")
   
    motion_service.setStiffnesses("LArm", 1.0)
    motion_service.setStiffnesses("RArm", 1.0)
    motion_service.setMoveArmsEnabled(False,False)




    
    




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
