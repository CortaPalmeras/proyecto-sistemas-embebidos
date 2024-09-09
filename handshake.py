from time import perf_counter, sleep
from struct import pack, unpack
import math
import serial

# se configura el puerto y el baud_rate
PORT = '/dev/ttyUSB0'  # esto depende del sistema operativo
BAUD_RATE = 115200  # debe coincidir con la configuracion de la esp32
SAMPLES = 50

# se abre la conexion serial
ser = serial.Serial(PORT, BAUD_RATE, timeout = 1)

# reinicia el programa en caso de que estaba en curso
_ = ser.write('q'.encode())
_ = ser.readall()

# heandshake
print("\n-- heandshake --")
_ = ser.write('++++++++++*'.encode())
resultado = ser.read(4).decode()
print(resultado)

print("\n-- output inicial --")
while True:
    data = ser.read(1).decode()
    if len(data) > 0:
        print(data, end="")
    else:
        break
        


print("\n-- estraccion de datos --")
_ = ser.write('g'.encode())

sum_temp = 0
sum_pres = 0

for _ in range(SAMPLES):
    data = ser.read(8)
    temp, pres = unpack("II", data)
    norm_temp = temp // 100
    norm_pres = pres // 1000
    sum_temp += norm_temp * norm_temp
    sum_pres += norm_pres * norm_pres
    print(f"temp: {temp / 100}\tpres: {pres}")


local_rms_temp = math.sqrt(sum_temp / SAMPLES)
local_rms_pres = math.sqrt(sum_pres / SAMPLES)
print(f"\n-- resultados RMS locales --")
print(f"temp: {local_rms_temp}")
print(f"pres: {local_rms_pres}")


data = ser.read(16)
esp_rms_temp, esp_rms_pres = unpack("dd", data)
print(f"\n-- resultados RMS esp --")
print(f"temp: {esp_rms_temp}")
print(f"pres: {esp_rms_pres}")


print(f"\n-- fin del output --")
print(ser.readall().decode())

