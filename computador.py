from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QLineEdit, QMessageBox, QProgressDialog
from PyQt5.QtCore import Qt
import sys
import serial
from struct import pack, unpack
import matplotlib.pyplot as plt

# Se configura el puerto y el BAUD_Rate
PORT = '/dev/ttyUSB0'  # Esto depende del sistema operativo
BAUD_RATE = 115200  # Debe coincidir con la configuración de la ESP32

# Se abre la conexión serial
ser = serial.Serial(PORT, BAUD_RATE, timeout=1)

# reinicia el programa en caso de que estaba en curso
_ = ser.write('q'.encode())
_ = ser.readall()

# handshake para sincronizar el computador con el ESP
print("\n-- handshake --")
_ = ser.write('++++++++++*'.encode())
resultado = ser.read(4).decode()
print(resultado)

# imprimir a la terminal logs de inicio
print("\n-- output inicial --")
while True:
    data = ser.readall()
    if len(data) > 0:
        print(data.decode(), end="")
    else:
        break

# se lee el tamanho de la ventana de datos
def solicitar_tamano_ventana():
    print("\n-- leer tamanho del sample --")
    _ = ser.write('s'.encode())
    raw_sample_size = ser.read(4)

    if len(raw_sample_size) == 4:
        new_sample_size = unpack('I', raw_sample_size)[0]
        print(f"el tamanho previo de la muestra es {new_sample_size}")
        return new_sample_size
    else:
        print(f"se leyeron {len(raw_sample_size)} bytes")
        exit(1)

samples = solicitar_tamano_ventana()


def generar_graficos(tiempo, lista_temp, esp_rms_temp, lista_pres, esp_rms_pres):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))
    ax1.plot(tiempo, lista_temp, marker='o', linestyle='-', color='b',  label=f'Temperatura (°C) RMS={esp_rms_temp}')
    ax1.set_title('Variación de la Temperatura en el Tiempo')
    ax1.set_xlabel('Tiempo (segundos)')
    ax1.set_ylabel('Temperatura (°C)')

    ax2.plot(tiempo, lista_pres, marker='o', linestyle='-', color='b',  label=f'Pascal (kPa) RMS={esp_rms_pres}')
    ax2.set_title('Variación de la presión en el tiempo')
    ax2.set_xlabel('Tiempo (segundos)')
    ax2.set_ylabel('Kilo Pascal (kPa)')
    plt.tight_layout()
    plt.show()

def solicitar_ventana_datos(progress_dialog):
    """Función para solicitar la ventana de datos"""
    global samples

    print("\n-- estraccion de datos --")

    _ = ser.write('s'.encode())
    data = ser.read(4)

    _ = ser.write('g'.encode())
    lista_temp = []
    lista_pres = []
    tiempo = list(range(samples))

    # Actualizar la barra de progreso
    for i in range(samples):
        data = ser.read(8)
        temp, pres = unpack("II", data)
        lista_temp.append(temp/100)
        lista_pres.append(pres/1000)
        print(f"temp: {temp / 100}\tpres: {pres}")

        progress_dialog.setValue(i+1)
        if progress_dialog.wasCanceled():
            while True:
                data = ser.readall()
                if len(data) > 0:
                    print(data.decode(), end="")
                else:
                    break
            return
            

    data = ser.read(16)
    esp_rms_temp, esp_rms_pres = unpack("dd", data)

    print(f"\n-- resultados RMS esp --")
    print(f"temp: {esp_rms_temp}")
    print(f"pres: {esp_rms_pres}")
    
    progress_dialog.close()
     
    generar_graficos(tiempo, lista_temp, esp_rms_temp, lista_pres, esp_rms_pres)



def cambiar_tamanho_ventana(tamanho: int) -> bool:
    """Función para cambiar el tamanho de la ventana de datos"""
    global samples
    
    print("\n-- cambiar tamaño del sample --")
    _ = ser.write('c'.encode())
    _ = ser.write(pack('I', tamanho))

    # en caso de el esp devuelva errores estos se imprimen en la terminal
    while True:
        data = ser.readall()
        if len(data) > 0:
            print(data.decode(), end="")
        else:
            break

    samples = solicitar_tamano_ventana()
    return samples == tamanho





# Clase principal para la interfaz PyQt5
class ESP32App(QWidget):
    def __init__(self):
        super().__init__()

        self.initUI()

    def initUI(self):
        """Configuración de la interfaz gráfica"""

        # Layout principal
        layout = QVBoxLayout()

        # Botones
        self.label = QLabel("Seleccione una opción:")
        self.boton_ventana = QPushButton("Solicitar ventana de datos", self)
        self.cambiar_tamanho_ventana_button = QPushButton("Cambiar tamaño de la ventana", self)
        self.reiniciar_conexion_button = QPushButton("Cerrar conexión", self)

        # Input para cambiar el tamanho de la ventana
        self.window_size_input = QLineEdit(self)
        self.window_size_input.setPlaceholderText("Nuevo tamaño de ventana")

        # Eventos de los botones
        self.boton_ventana.clicked.connect(self.solicitar_ventana_datos)
        self.cambiar_tamanho_ventana_button.clicked.connect(self.cambiar_tamanho_ventana)
        self.reiniciar_conexion_button.clicked.connect(self.reiniciar_conexion)

        # Anhadir los widgets al layout
        layout.addWidget(self.label)
        layout.addWidget(self.boton_ventana)
        layout.addWidget(self.window_size_input)
        layout.addWidget(self.cambiar_tamanho_ventana_button)
        layout.addWidget(self.reiniciar_conexion_button)

        # Configuración final de la ventana
        self.setLayout(layout)
        self.setWindowTitle('ESP32 Interface')
        self.setGeometry(300, 300, 400, 200)
        self.show()

    def solicitar_ventana_datos(self):
        """Solicitar la ventana de datos a la ESP32"""
        # Crear el cuadro de progreso
        progress_dialog = QProgressDialog("Recopilando datos...", "Cancelar", 0, samples, self)
        progress_dialog.setWindowModality(Qt.WindowModal) 
        progress_dialog.setMinimumDuration(0)
        progress_dialog.setValue(0)
         # Asegurar que el diálogo esté al frente y tenga el foco
        progress_dialog.raise_()
        progress_dialog.activateWindow()

        solicitar_ventana_datos(progress_dialog)


    def cambiar_tamanho_ventana(self):
        """Cambiar el tamanho de la ventana de datos"""
        tamanho = self.window_size_input.text()
        if tamanho.isdigit():
            if cambiar_tamanho_ventana(int(tamanho)):
                QMessageBox.information(self, 'Éxito', "Se guardó el tamaño de la ventana exitosamente.")
            else:
                QMessageBox.information(self, 'Error', "Ocurrió un error al cambiar el tamaño de la ventana de datos.")
        else:
            QMessageBox.information(self, 'Numero no valido', "Numero no valido")


    def reiniciar_conexion(self):
        _ = ser.write('q'.encode())
        QMessageBox.information(self, 'Reinicio', 'La conexión se ha reiniciado.')
        ser.close()
        print("Saliendo del programa")
        sys.exit(0)



# Función principal para ejecutar la aplicación
if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = ESP32App()
    sys.exit(app.exec_())
