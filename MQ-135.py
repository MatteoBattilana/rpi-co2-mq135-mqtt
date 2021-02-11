#!/usr/bin/python3

"""
MQ135 gas sensor

Datasheet can be found here: https://www.olimex.com/Products/Components/Sensors/SNS-MQ135/resources/SNS-MQ135.pdf

Application
They are used in air quality control equipments for buildings/offices, are suitable for detecting of NH3, NOx, alcohol, Benzene, smoke, CO2, etc

Original creator of this library: https://github.com/GeorgK/MQ135 -> https://github.com/KRIPT4/MQ135-ADS1115-Python
"""

import sys, math, operator, json, time
import paho.mqtt.client as mqtt

# TODO: use actual values
t = 22 # assume current temperature. Recommended to measure with DHT22
h = 65 # assume current humidity. Recommended to measure with DHT22

"""
GeorgK, 2014
TODO: Review the correction factor calculation. This currently relies on
the datasheet but the information there seems to be wrong...
"""

#
# Sensor Model Params
#

# The load resistance on the board
RLOAD = 10.0
# Calibration resistance at atmospheric CO2 level
RZERO = 76.63
# Parameters for calculating ppm of CO2 from sensor resistance
PARA = 116.6020682
PARB = 2.769034857

# Parameters to model temperature and humidity dependence
CORA = 0.00035
CORB = 0.02718
CORC = 1.39538
CORD = 0.0018
CORE = -0.003333333
CORF = -0.001923077
CORG = 1.130128205

# Atmospheric CO2 level for calibration purposes
ATMOCO2 = 397.13


#
# MQTT
#

# Hey, If you don't want to fork it, let's create env variable config
MQTT_HOST = '192.168.1.68' 
MQTT_TOPIC = '/bedroom/weather/gas'
# https://www.home-assistant.io/integrations/sensor.mqtt/
# https://www.home-assistant.io/docs/mqtt/discovery#motion-detection-binary-sensor


"""
@brief  Get the correction factor to correct for temperature and humidity

@param[in] t  The ambient air temperature
@param[in] h  The relative humidity

@return The calculated correction factor
"""

def getCorrectionFactor(t,h,CORA,CORB,CORC,CORD,CORE,CORF,CORG):
	# Linearization of the temperature dependency curve under and above 20 degree C
	# below 20degC: fact = a * t * t - b * t - (h - 33) * d
	# above 20degC: fact = a * t + b * h + c
	# this assumes a linear dependency on humidity
	if t < 20:
		return CORA * t * t - CORB * t + CORC - (h-33.)*CORD
	else:
		return CORE * t + CORF * h + CORG

"""
@brief  Get the resistance of the sensor, ie. the measurement value

@return The sensor resistance in kOhm
"""

def getResistance(value_pin,RLOAD):
	return ((1023./value_pin) - 1.)*RLOAD

"""
@brief  Get the resistance of the sensor, ie. the measurement value corrected
        for temp/hum

@param[in] t  The ambient air temperature
@param[in] h  The relative humidity

@return The corrected sensor resistance kOhm
"""

def getCorrectedResistance(t,h,CORA,CORB,CORC,CORD,CORE,CORF,CORG,value_pin,RLOAD):
	return getResistance(value_pin,RLOAD) / getCorrectionFactor(t,h,CORA,CORB,CORC,CORD,CORE,CORF,CORG)

"""
@brief  Get the ppm of CO2 sensed (assuming only CO2 in the air)

@return The ppm of CO2 in the air
"""

def getPPM(PARA,RZERO,PARB,value_pin,RLOAD):
	return PARA * math.pow((getResistance(value_pin,RLOAD)/RZERO), -PARB)

"""
@brief  Get the ppm of CO2 sensed (assuming only CO2 in the air), corrected
        for temp/hum

@param[in] t  The ambient air temperature
@param[in] h  The relative humidity

@return The ppm of CO2 in the air
"""

def getCorrectedPPM(t,h,CORA,CORB,CORC,CORD,CORE,CORF,CORG,value_pin,RLOAD,PARA,RZERO,PARB):
	return PARA * math.pow((getCorrectedResistance(t,h,CORA,CORB,CORC,CORD,CORE,CORF,CORG,value_pin,RLOAD)/RZERO), -PARB)

"""
@brief  Get the resistance RZero of the sensor for calibration purposes

@return The sensor resistance RZero in kOhm
"""

def getRZero(value_pin,RLOAD,ATMOCO2,PARA,PARB):
	return getResistance(value_pin,RLOAD) * math.pow((ATMOCO2/PARA), (1./PARB))

"""
@brief  Get the corrected resistance RZero of the sensor for calibration
        purposes

@param[in] t  The ambient air temperature
@param[in] h  The relative humidity

@return The corrected sensor resistance RZero in kOhm
"""

def getCorrectedRZero(t,h,CORA,CORB,CORC,CORD,CORE,CORF,CORG,value_pin,RLOAD,ATMOCO2,PARA,PARB):
	return getCorrectedResistance(t,h,CORA,CORB,CORC,CORD,CORE,CORF,CORG,value_pin,RLOAD) * math.pow((ATMOCO2/PARA), (1./PARB))

"""
Re-maps a number from one range to another. That is, a value of fromLow would get mapped to toLow, 
a value of fromHigh to toHigh, values in-between to values in-between, etc.

# Arduino: (0 a 1023)
# Raspberry Pi: (0 a 26690)

More Info: https://www.arduino.cc/reference/en/language/functions/math/map/
"""

def map(x,in_min,in_max,out_min,out_max):
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


print("[mqtt] initing...")
mqttc = mqtt.Client(client_id = MQTT_TOPIC, clean_session = False)

def pub_mqtt(jsonrow):
    #cmd = ['mosquitto_pub', '-h', MQTT_HOST, '-t', MQTT_TOPIC, '-s']
    #print('Publishing using:', cmd)
    #with subprocess.Popen(cmd, shell=False, bufsize=0, stdin=subprocess.PIPE).stdin as f:
    #    json.dump(jsonrow, f)
    payload = json.dumps(jsonrow)
    mqttc.publish(MQTT_TOPIC, payload, retain=True)
    print('>mqtt', payload)

def on_connect(mqttc, userdata, flags, rc):
    global is_mqtt_connected

    print("Connected to mqtt with result code "+str(rc))
    #print(f"[mqtt] subscribing... {MQTT_TOPIC_CMD}")
    #mqttc.subscribe(MQTT_TOPIC_CMD)
    #mqttc.subscribe(MQTT_TOPIC_SW_CMD)
    #print("[mqtt] subscribed")
    is_mqtt_connected = True

mqttc.enable_logger(logger=None)
mqttc.on_connect = on_connect
#mqttc.on_message = on_message
print("[mqtt] connecting...")
mqttc.connect(MQTT_HOST)

# mqttc.loop_forever()
mqttc.loop_start() # loop thread

if __name__ == "__main__":
    #print('auto conf ha mqtt sensor')
    #pub_mqtt(HA_MQTT_CONF)
    print("start MQ135 loopback...")
    while True:
	value_ads = 3300 # value obtained by ADS1115
	value_pin = map((value_ads - 565), 0, 26690, 0, 1023) # 565 / 535 fix value
	rzero = getRZero(value_pin,RLOAD,ATMOCO2,PARA,PARB)
	correctedRZero = getCorrectedRZero(t,h,CORA,CORB,CORC,CORD,CORE,CORF,CORG,value_pin,RLOAD,ATMOCO2,PARA,PARB)
	resistance = getResistance(value_pin,RLOAD)	
	ppm = getPPM(PARA,RZERO,PARB,value_pin,RLOAD)	
	correctedPPM = getCorrectedPPM(t,h,CORA,CORB,CORC,CORD,CORE,CORF,CORG,value_pin,RLOAD,PARA,RZERO,PARB)
	print("\n MQ135 Gas Sensor:\n")
	print("\t MQ135 RZero: %s" % round(rzero))
	print("\t Corrected RZero: %s" % round(correctedRZero))
	print("\t Resistance: %s" % round(resistance))
	print("\t PPM: %s" % round(ppm))
	print("\t Corrected PPM: %s ppm" % round(correctedPPM))
	pub_mqtt({ co2_ppm: correctedPPM })

	print("Going to sleep for 1 min...")
        time.sleep(60)
