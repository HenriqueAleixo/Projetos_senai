# -*- coding: cp1252 -*-
import paho.mqtt.client as mqtt
import requests

# Mosquitto MQTT
username 	= "admin"
password 	= "admin"
broker_address 	= "localhost"
port 		= 1883

def on_connect(client, userdata, flags, rc):
    print("rc: " + str(rc))

def on_message(client, obj, msg):
    print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload) )

def on_publish(client, obj, mid):
    print("mid: " + str(mid))

def on_subscribe(client, obj, mid, granted_qos):
    print("Subscribed: " + str(mid) + " " + str(granted_qos))

def on_log(client, obj, level, string):
    print(string)

mqttc = mqtt.Client()

mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_publish = on_publish
mqttc.on_subscribe = on_subscribe

# Uncomment to enable debug messages
#mqttc.on_log = on_log

# Connecta ao Mosquitto MQTT. 
# Informe o nome do usuário e senha. 
mqttc.username_pw_set(username=username, password=password)

# Carrega o certificado TLS.
#mqttc.tls_set("C:/Program Files (x86)/mosquitto/certs/ca.crt")

# Informe o endereço IP ou DNS do servidor Mosquitto. 
# Use 1883 para conexão SEM TLS, e 8883 para conexão segura TLS.
mqttc.connect(broker_address, port=port)

# Assina o tópico, com QoS level 0 (pode ser 0, 1 ou 2)
mqttc.subscribe("#", 0)

# Publicação para testes. 
#mqttc.publish(umidade, "25")

# Continue the network loop, exit when an error occurs
while True:
    rc = 0
    while rc == 0:
        rc = mqttc.loop()
    break

