# coding: utf-8



import paho.mqtt.client as mqtt_client
from time import sleep
# Configuration 
MQTT_BROKER = "10.0.0.24"
MQTT_PORT   = 1883
KEEP_ALIVE  = 45 # interval en seconde

def on_log( client, userdata, level, buf ):
    print( "log: ",buf)
client = mqtt_client.Client( client_id="client007" )
#client.username_pw_set( username="pusr103", password="21052017" )
client.connect( host=MQTT_BROKER, port=MQTT_PORT, keepalive=KEEP_ALIVE )

# traitement des message

for i in range(4000):
    r = client.publish( "Mower/COMMAND", "START" )
    print( "Start envoyé" if r[0] == 0 else "  echec" ) 
    sleep( 10 )

    r = client.publish( "Mower/COMMAND", "OFF" )
    print( "Off  envoyé" if r[0] == 0 else "  echec" ) 
    sleep( 10 )

    r = client.publish( "Mower/COMMAND", "HOME" )
    print( "Home  envoyé" if r[0] == 0 else "  echec" ) 
    sleep( 10 )

    r = client.publish( "Mower/COMMAND", "OFF" )
    print( "Off  envoyé" if r[0] == 0 else "  echec" ) 
    sleep( 10 )



    
