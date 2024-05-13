from machine import Pin, PWM
from machine import Timer
import _thread
import bluetooth as bt
from BLE import BLEUART
import time
from motor_nema import configuraDriver, giroDerecha, giroIzquerda, detener, enable
import dht

""" Segundo hilo para la oscilacion """
def secondThread():
    bandera_mp = False  # Deshabilita oscilacion
    enable(bandera_mp, en)  # Habilita modo manual driver

    while True:
        giroDerecha(pulses, direction, 2133, 2000)
        detener(pulses)
        giroIzquerda(pulses, direction, 2133, 2000)
        detener(pulses)

def on_rx():  # Evento de interrupcion de recepcion de BLE
    rx_buffer = uart.read().decode().strip()
    print(rx_buffer)
    global flag_ang, ang
    comando = rx_buffer[0:3]

    if comando == 'ang':
        dato = rx_buffer[3:5]
        ang = dato
        flag_ang = True
    elif comando == 'osc':
        print('Oscilacion: ' + dato)
    #  uart.write('ESP32 says: ' + str(rx_buffer) + '\n')

def interrupcionTimer(tim0):
    global minutos
    global humedad
    global temperatura

    if minutos < 10:
        minutos += 1
        print('Han a pasado: ' + str(minutos) + ' min')
    else:
        sensor.measure()
        temperatura = sensor.temperature()
        humedad = sensor.humidity()
        uart.write('temp'+str(temperatura))

        if flag_auto is True:
            if temperatura >= 30:
                print('Maxima velocidad')
            elif temperatura < 30 and temperatura >= 25:
                print('75 % de velocidad')
            elif temperatura < 25 and temperatura >= 20:
                print('50 % de velocidad')
            elif temperatura < 20 and temperatura >= 15:
                print('30 % de velocidad')
            else:
                print('Apagar ventilador')

        print('Han pasado 10 min')
        minutos = 0

""" Declarar variables globales """
bandera_mp = False  # Bandera de ON/OFF Motor a pasos
flag_auto = False  # Bandera de modo automatica
flag_ang = False  # Bandera ang servo
minutos = 0  # Cuenta tiempo de sensor temp
temperatura = None
humedad = None
ang = None
dang = 0.0
dutyC = 51.0

""" Bloque de configuracion Pines"""
rele = Pin(10, Pin.OUT)

""" Bloque configuracion BLE """
name = 'VentSmart_BLE'  # Define nombre de dispositivo
ble = bt.BLE()  # Crea instancia BLE
uart = BLEUART(ble, name)  # Crea instancia UARTBLE
uart.irq(handler=on_rx)  # Declara y define requerimiento interrupcion Rx BLE

""" Bloque configuracion de sensor temperatura"""
sensor = dht.DHT11(Pin(5))  # Sensor conectado en Pin 5

""" Bloque de configuracion de timer """
tim0 = Timer(0)
tim0.init(period=60000, mode=Timer.PERIODIC, callback=interrupcionTimer)

""" Bloque configuracion spray modo aroma y humidificador"""

""" Bloque de configuracion PWM Boost """
pwm_PFC = PWM(Pin(13))  # Configura pin 13 como salida pwm para pfc
pwm_PFC.freq(100000)  # Frecuencia de 100kHz para PFC
pwm_PFC.duty(0)  # 0 -- 100% = 1023

""" Bloque configuracion PWM Servo"""
servo = PWM(Pin(15))  # Configura Pin 15 para PWM de servo
servo.freq(100)  # Frecuencia de servomotor 330
servo.duty(int(dutyC))  # Para 45 grados totalmnt horizontal
print('Se termino conf servo')

""" Bloque de configuracion Driver Motor a pasos"""
pulses, direction, en = configuraDriver(16, 4, 2)  # Configura pines 16,4,2 para driver
_thread.start_new_thread(secondThread, ())  # Inicio ejecucion hilo 2

""" Ejecucion de hilo principal """
while True:
    if flag_ang is True:
        dang = float(ang)
        if dang < 1.0:
            dang = 1.0
        print('El angulo es: ' + str(dang))
        dutyC = dang*1.13  # (102.0/90.0)=1.13
        dutyC = dutyC + 51
        print(dutyC)
        servo.duty(int(dutyC))
        flag_ang = False
    time.sleep_ms(100)
