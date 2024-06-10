# SECO 3ª entrega

Esta carpeta contiene el código empleado en la realización de la 3ª entrega de la asignatura. Todas la figuras representadas a lo largo de la memoria han sido realizadas con los scripts presentes en esta carpeta. En ella se adjunta también un archivo `requirements.txt`, que contiene las dependencias necesarias para su ejecución. Para su correcta instalación se recomienda ejecutar:

```
# Create a virtual environment
$ python3 -m venv venv

# Activate the new environment
$ source venv/bin/activate

# Install dependencies
$ pip install -r requirements.txt
```

La carpeta tiene la siguiente distribución:

```
code
├── README.md
├── python
│   ├── D|PID_analysis.ipynb
│   ├── PIDControllers.py
│   ├── design.ipynb
│   ├── rlocus.ipynb
│   └── time_response.py
└── requirements.txt
```

En la subcarpeta de python se encuentran todos los archivos de código. Nótese que algunos se han realizado mediante cuadernos de jupyter para facilitar la creación de figuras.

# Apartado a)

Para la creación de figuras del primer apartado se han creado los archivos `PIDControllers.py` y `time_response.py`. En el archivo PIDControllers se ha definido una interfaz llamada Controlador. De esta forma, cada nuevo controlador creado es suficiente crear una nueva clase que hereda de la primera. Esta interfaz tiene un método para devolver la función de transferencia, otra para la etiqueta y otra para su nombre.

Las figuras se hacen desde el archivo `time_response.py`. Aquí se selecciona el controlador a probar, la función de referencia, y las constantes a probar.

# Apartado b)

Se ha empleado el archivo `rlocus.ipynb` para la representación del lugar de polos. Por otro lado, para el diseño se ha utilizado `design.ipynb` y `D|PID_analysis.ipynb` para su simulación final. En estos archivos se encuentra específicado el modo de uso.
