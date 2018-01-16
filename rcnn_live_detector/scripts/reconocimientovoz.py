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


#respuesta_google_online = r.recognize_google_cloud(audio, credentials_json=GOOGLE_CLOUD_SPEECH_CREDENTIALS,language="es-CR")






def myhook():
  print "shutdown time!"


def talker():
    
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('/speechclassObject', String, queue_size=10)
    pubPepperSpeech = rospy.Publisher('/speech', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    i=0
    condicional = False



    
    r = sr.Recognizer()
    m = sr.Microphone()

    try:
      #print("A moment of silence, please...")
      #with m as source: r.adjust_for_ambient_noise(source)
      #print("Set minimum energy threshold to {}".format(r.energy_threshold))

      while True:
        #print("A moment of silence, please...")
        with m as source: r.adjust_for_ambient_noise(source)
        #print("Set minimum energy threshold to {}".format(r.energy_threshold))


        #print("####################################")
        #print("####################################")
        #print("Decir okay pepper ")
        with m as source: audio = r.listen(source)
        #print("Got it! Now to recognize it...")
        try:
          GOOGLE_CLOUD_SPEECH_CREDENTIALS = None
          respuesta_google_online = r.recognize_google(audio)
          #respuesta_google_online = r.recognize_google_cloud(audio, credentials_json=GOOGLE_CLOUD_SPEECH_CREDENTIALS,language="es-CR")
          respuesta_google_online=respuesta_google_online.encode('utf-8')

          #print("Google Speech Recognition thinks you said " + respuesta_google_online )

          if respuesta_google_online == "exit":
            print("killing this node")
            rospy.signal_shutdown()

          if respuesta_google_online == "okay paper":
            print("Trigger reconocido")
            pubPepperSpeech.publish("si?")
            condicional = True
            #pubPepperSpeech.publish("si?")
          #respuesta_google_online = r.recognize_google_cloud(audio, credentials_json=GOOGLE_CLOUD_SPEECH_CREDENTIALS,language="es-CR")
      
        except sr.UnknownValueError:
          pass
        except sr.RequestError as e:
          pass

        #print(condicional)


        if condicional:
          print("Decir instruccion en espanol")
          with m as source: audio = r.listen(source)
          #print("Got it! Now to recognize it...")
          try:
            GOOGLE_CLOUD_SPEECH_CREDENTIALS = None            
            respuesta_google_cloud = r.recognize_google_cloud(audio, credentials_json=GOOGLE_CLOUD_SPEECH_CREDENTIALS,language="es-CR")
            respuesta_google_cloud=respuesta_google_cloud.encode('utf-8')
            print("Google Speech Recognition thinks you said " + respuesta_google_cloud )
            listapalabras = respuesta_google_cloud.split()
            palabraDECO = unicodedata.normalize('NFKD', listapalabras[1].decode('unicode-escape')).encode('ASCII', 'ignore')
            print "Palabra deco:" + palabraDECO
            if len(listapalabras)==2:
              if listapalabras[0]=="traer":
                if listapalabras[1]=="botella":
                  print("publicando" + listapalabras[1])
                  pub.publish(listapalabras[1])
                  
                elif palabraDECO=="cafA":
                  print("publicando" + "cafe" )
                  pub.publish("cafe")

                  
		elif listapalabras[1]=="palomitas":
                  print("publicando" + listapalabras[1])
                  pub.publish(listapalabras[1])

                  
		elif listapalabras[1]=="chocolate":
                  print("publicando" + listapalabras[1] )
                  pub.publish(listapalabras[1])

                  
		else:
                  print("objeto no disponible")
                  print("Instrucción no valida")
		  pubPepperSpeech.publish("Instrucción no valida")
		  
	    rospy.loginfo(respuesta_google_cloud)


            
          except sr.UnknownValueError:
            pass
          except sr.RequestError as e:
            pass




          condicional = False





      
    except KeyboardInterrupt:
      pass



    
    print("#######")

if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass





























