# -*- coding: cp1252 -*-
import paho.mqtt.client as mqtt
import requests
import sys
import mysql.connector
from datetime import date, datetime, timedelta
import time

# Mosquitto MQTT
username 	= "user1"
password 	= "321"
hostname 	= "localhost"
port 		= 1883

# Banco de dados MySql
user_mysql 	= "root"
pwd_mysql 	= ""
db_mysql	= "esp32"

# Funções de Callback dos eventos
# on_connect = função chamada quando ocorre a conexão entre o cliente e o broker MQTT.
# on_message = função chamada quando uma mensagem de um tópico assinado for recebido.
# on_publish = função chamada quando uma mensagem for publicada. 
# on_subscribe = função chamada quando um tópico for assinado pelo cliente MQTT.
# on_log = função para debug do Paho.

def on_connect(client, userdata, flags, rc):
    print("rc: " + str(rc))

def on_message(client, obj, msg):
    # Imprime o conteudo da messagem.
    print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload) )
	
    # Conexão com o banco de dados Mysql
    # Todos os parâmetros de configuração do Connector Mysql pode ser encontrado
    # aqui: https://dev.mysql.com/doc/connector-python/en/connector-python-connectargs.html
	
    cnx = mysql.connector.connect(user=user_mysql, password=pwd_mysql, database=db_mysql)
    cursor = cnx.cursor()
    query = ("INSERT INTO sensores "
            "(nome, valor, timestamp) "
            "VALUES (%s, %s, %s)")
    # msg.topico = armazena o tópico
    # msg.payload = armzena o conteúdo da mensagem.
    # time.time = corresponde ao timestamp (UTC).
    dados_recebidos = (msg.topic, msg.payload, str(int(time.time())))
	
    # Carrega e executa a query.
    cursor.execute(query, dados_recebidos)
    cnx.commit()
	
    # Encerra a conexão com o banco de dados.
    cursor.close()
    cnx.close()

def on_publish(client, obj, mid):
    print("mid: " + str(mid))

def on_subscribe(client, obj, mid, granted_qos):
    print("Subscribed: " + str(mid) + " " + str(granted_qos))

def on_log(client, obj, level, string):
    print(string)

#------------------------------------------------
mqttc = mqtt.Client()

# Assina as funções de callback
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_publish = on_publish
mqttc.on_subscribe = on_subscribe

# Uncomment to enable debug messages
#mqttc.on_log = on_log

# Connecta ao Mosquitto MQTT. 
# Informe o nome do usuário e senha. 
mqttc.username_pw_set(username, password)

# Carrega o certificado TLS.
#mqttc.tls_set("C:/Program Files (x86)/mosquitto/certs/ca.crt")

# Informe o endereço IP ou DNS do servidor Mosquitto. 
# Use 1883 para conexão SEM TLS, e 8883 para conexão segura TLS.
mqttc.connect(hostname, port)

# Assina o tópico, com QoS level 0 (pode ser 0, 1 ou 2)
mqttc.subscribe("temperatura", 0)

# Para publicar em um tópico, utilize a função a seguir.
#mqttc.publish(umidade, "25")

# Permanece em loop mesmo se houver erros. 
while True:
    try:
        rc = 0
        while rc == 0:
            rc = mqttc.loop()
        print("rc: " + str(rc))
    except OtherExceptions:
        print("erro de exceptions")
