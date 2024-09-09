# Iteración 1 del proyecto 1
## CC5328 - Sistemas Embedidos y Sensores

Esta tarea fue Probada en Debain 12, por lo que el puerto usado fue `/dev/ttyUSB0`, para cambierlo se debe cambiar la variable `PORT` en el archivo `computador.py`.

Para ejecutar la tarea es necesario cargar el programa en el ESP, ejecutando el comando:

```bash
idf.py flash
```

Luego se deben instalar las dependencias del programa en python:

```bash
pip install -r requirements.txt
```

Finalmente se puede ejecutar la aplicación usando python

```bash
python3 computador.py
```
