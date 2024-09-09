import sys
import serial
from struct import pack, unpack
import time
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QLineEdit, QMessageBox
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

# Se configura el puerto y el BAUD_Rate
PORT = '/dev/ttyUSB0'  # Esto depende del sistema operativo
BAUD_RATE = 115200  # Debe coincidir con la configuración de la ESP32

# Se abre la conexión serial
ser = serial.Serial(PORT, BAUD_RATE, timeout=1)

# Funciones
def send_message(message):
    """Función para enviar un mensaje a la ESP32"""
    ser.write(message)

def receive_response():
    """Función para recibir un mensaje de la ESP32"""
    response = ser.readline()
    return response

def receive_data():
    """Función que recibe tres floats (fff) de la ESP32 
    y los imprime en consola"""
    data = receive_response()
    data = unpack("dI", data)
    print(f'Se leyo: {data}')
    return data

def send_end_message():
    """Función para enviar un mensaje de finalización a la ESP32"""
    end_message = pack('4s', 'END\0'.encode())
    ser.write(end_message)
listaTempPress=list()

def inicializar():
    """Función para solicitar la ventana de datos"""
    mensaje = pack('6s','BEGIN\0'.encode())
    send_message(mensaje)
    contador=0
    while(True):
        contador+=1
        if(contador>=10):
            print("contador = 10")
            break
        print("Entra al ciclo while")
        if ser.in_waiting > 0:
            print("Se esta dando una respuesta")
            try:
                message = receive_data()
                print("Recibiendo informacion")
                listaTempPress.append(message)
                
            except:
                try:
                    message=ser.readline()
                    message=unpack("8s",message)
                    print("Se ha terminado de enviar los datos")
                    break
                except:
                    print("Ha ocurrido un error")
    
    


def change_window_size(size):
    """Función para cambiar el tamaño de la ventana de datos"""
    command = f"CHANGE_WINDOW_SIZE {size}\n"
    ser.write(command.encode())
    print(f"Sent window size: {size}")


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
        self.data_window_button = QPushButton("Solicitar ventana de datos", self)
        self.change_window_size_button = QPushButton("Cambiar tamaño de la ventana", self)
        self.close_connection_button = QPushButton("Cerrar conexión", self)
        self.exit_button = QPushButton("Salir", self)

        # Input para cambiar el tamaño de la ventana
        self.window_size_input = QLineEdit(self)
        self.window_size_input.setPlaceholderText("Nuevo tamaño de ventana")

        # Eventos de los botones
        self.data_window_button.clicked.connect(self.inicializar)
        self.change_window_size_button.clicked.connect(self.change_window_size)
        self.close_connection_button.clicked.connect(self.close_connection)
        self.exit_button.clicked.connect(self.exit_program)

        # Añadir los widgets al layout
        layout.addWidget(self.label)
        layout.addWidget(self.data_window_button)
        layout.addWidget(self.window_size_input)
        layout.addWidget(self.change_window_size_button)
        layout.addWidget(self.close_connection_button)
        layout.addWidget(self.exit_button)

        # Configuración final de la ventana
        self.setLayout(layout)
        self.setWindowTitle('ESP32 Interface')
        self.setGeometry(300, 300, 400, 200)
        self.show()

    def inicializar(self):
        """Solicitar la ventana de datos a la ESP32"""
        QMessageBox.information(self, 'Se esta recopilando información', "Se esta recopilando información")
        inicializar()

        # self.canvas = FigureCanvas(Figure())

        # self.layout.addWidget(self.canvas)

        # ax = self.canvas.figure.add_subplot(111)

        # self.canvas.draw()

    def change_window_size(self):
        """Cambiar el tamaño de la ventana de datos"""
        size = self.window_size_input.text()
        if size.isdigit():
            change_window_size(int(size))
            QMessageBox.information(self, 'Cambio de tamaño', f'Tamaño de ventana cambiado a: {size}')
        else:
            QMessageBox.warning(self, 'Error', 'Por favor, ingrese un número entero válido.')

    def close_connection(self):
        """Cerrar la conexión serial"""
        send_end_message()
        QMessageBox.information(self, 'Conexión cerrada', 'La conexión se ha cerrado y la ESP32 se ha reseteado.')

    def exit_program(self):
        """Salir del programa"""
        ser.close()
        print("Exiting program.")
        sys.exit(0)


# Función principal para ejecutar la aplicación
if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = ESP32App()
    sys.exit(app.exec_())
