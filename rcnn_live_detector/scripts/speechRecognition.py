#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import unicodedata
import rospy
from std_msgs.msg import String
import unidecode

import speech_recognition as sr
import os
#from playsound import playsound

#os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = '/home/steven/test_google_cloud/SpeechRecognitionService-432566cc4585.json'


def myhook():
  print "shutdown time!"


def talker():
    
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('/speechclassObject', String, queue_size=10)
    pubPepperSpeech = rospy.Publisher('/speech', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    i=0

    while True:
    #while not rospy.is_shutdown():
    	while True:
                    print("*********************")
		    r = sr.Recognizer()
		    with sr.Microphone() as source:
		        print("Decir trigger 'okay pepper' ")
		        audio = r.listen(source)

		    try:
		        respuesta_google_online=r.recognize_google(audio)
		        respuesta_google_online=respuesta_google_online.encode('utf-8')
		        
		        print("Google Speech Recognition thinks you said " + respuesta_google_online )

		        if respuesta_google_online == "exit":
		        	print("killing this node")
		        	rospy.signal_shutdown()
		        	#rospy.on_shutdown(myhook)


		        if respuesta_google_online == "okay paper":
		            print("Trigger reconocido")
		            pubPepperSpeech.publish("si?")
		            #playsound('sample1.mp3')
		            #print("acabar")
		            r2 = sr.Recognizer()
		            with sr.Microphone() as source2:
		                print("Decir instruccion en espanol")
		                audio2 = r2.listen(source2)


		            GOOGLE_CLOUD_SPEECH_CREDENTIALS = None
		            try:
		            	respuesta_google_cloud=r.recognize_google_cloud(audio2, credentials_json=GOOGLE_CLOUD_SPEECH_CREDENTIALS,language="es-CR")
		            	respuesta_google_cloud=respuesta_google_cloud.encode('utf-8')
		            	print("Google Cloud Speech thinks you said " + respuesta_google_cloud)
		            	
		            	listapalabras = respuesta_google_cloud.split()
		            	palabraDECO = unicodedata.normalize('NFKD', listapalabras[1].decode('unicode-escape')).encode('ASCII', 'ignore')
		            	print "Palabra deco:" + palabraDECO
		            	if len(listapalabras)==2:
		            		if listapalabras[0]=="traer":
		            			if listapalabras[1]=="botella":
			            			pub.publish(listapalabras[1])
			            		elif palabraDECO=="cafA":
			            			pub.publish("cafe")
			            		elif listapalabras[1]=="palomitas":
			            			pub.publish(listapalabras[1])
			            		elif listapalabras[1]=="chocolate":
			            			pub.publish(listapalabras[1])	
			            		else:
			            			print("objeto no disponible")
			            			pubPepperSpeech.publish("Instrucción no valida")

		            	


		            	
		                rospy.loginfo(respuesta_google_cloud)
		                
		                rate.sleep()
		            except sr.UnknownValueError:
		                print("Google Cloud Speech could not understand audio")
		            except sr.RequestError as e:
		                print("Could not request results from Google Cloud Speech service; {0}".format(e))	            
		        else:
		            print("No coincide el trigger 'okay pepper', usted dijo "+ respuesta_google_online)

		        
		        
		        
		    except sr.UnknownValueError:
		        print("Google Speech Recognition could not understand audio")
		    except sr.RequestError as e:
		        print("Could not request results from Google Speech Recognition service; {0}".format(e))











if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
