#!/usr/bin/python3
import time
import smbus
import pyfiap
import datetime
import math

i2c = smbus.SMBus(1)
i2c_addr = 0x45

def sht31_read():
  i2c.write_byte_data(i2c_addr, 0x24, 0x00) # Single shot clock stretch disable
  time.sleep(2)
  i2c.write_byte_data(i2c_addr, 0xE0, 0x00)
  reg = i2c.read_i2c_block_data(i2c_addr, 0x00, 6)
  temp = -45 + 175 * ((reg[0] << 8) | reg[1]) / 0xffff
  humi =100 * ((reg[3] << 8) | reg[4]) / 0xffff
  return(temp, humi)
#    print('temp {:.4f} {:.4f}'.format(temp, temp2))

def sht31_heater_on():
  while( not heater_status() ):
    i2c.write_byte_data(i2c_addr, 0x30, 0x6D) # Heat on
    time.sleep(1)
  
def sht31_heater_off():
  while( heater_status() ):
    i2c.write_byte_data(i2c_addr, 0x30, 0x66) # Heat off
    time.sleep(1)
  
def heater_status():
  i2c.write_byte_data(i2c_addr, 0xF3, 0x2D)
  data = i2c.read_i2c_block_data(i2c_addr, 0x00, 3)
  if( data[0] & 0x20 > 0 ):
    return True
  else:
    return False

def sht31_reset():
  i2c.write_byte_data(i2c_addr, 0x30, 0xA2) # reset
  time.sleep(2)

def temp2svp( temp ):
  temp = temp+273.15
  a = -6096.9385 / temp
  b = 21.2409642
  c = -2.711193 / 100 * temp
  d = 1.673952 / 100000 * temp * temp
  e = 2.433502 * math.log(temp)
  return( math.exp( a + b + c + d + e ) )

def calc_vpd( temp, humi ):
  svp = temp2svp(temp)   # Saturated Vapour Pressure [Pa]
  vp  = svp * humi / 100 # Vapour Pressure [Pa]
  vpd = (svp-vp)/1000    # Vapour Pressure Dificit [kPa]
  return(vpd)


#i2c.write_byte_data(i2c_addr, 0xF3, 0x2D)
#data = i2c.read_i2c_block_data(i2c_addr, 0x00, 3)
#print("SREG {:08b}".format(data[0]))

#i2c.write_byte_data(i2c_addr, 0x30, 0x6D) # Heat on
#i2c.write_byte_data(i2c_addr, 0x30, 0x66) # Heat off
#i2c.write_byte_data(i2c_addr, 0x30, 0xA2) # reset

#i2c.write_byte_data(i2c_addr, 0x20, 0x32) # Periodic Mode / 2sec
#i2c.write_byte_data(i2c_addr, 0x2c, 0x06) # Single shot clock stretch enable
temp, humi = sht31_read()
while( humi <= 0 ): 
  sht31_reset()
  temp, humi = sht31_read()

vpd = calc_vpd(temp, humi)
print('temp {:.4f} / humi {:.4f} / vpd {:.4f}'.format(temp, humi, vpd))

today = datetime.datetime.now()
fiap = pyfiap.fiap.APP("http://iot.info.nara-k.ac.jp/fiapd/axis2/services/FIAPStorage?wsdl")
fiap.write([['http://tomato.fukuoka.lab/sht31/temperature', "{:.2f}".format(temp), today],
            ['http://tomato.fukuoka.lab/sht31/humidity', "{:.2f}".format(humi), today],
            ['http://tomato.fukuoka.lab/sht31/VPD', "{:.4f}".format(vpd), today],
            ])
