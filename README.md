# Laboratorio 2: Navegación reactiva con filtrado y fusión de sensores en Webots

**Integrantes del grupo:**
* Hans Silva
* Inti Liberona
* Kevin Alvarez
* Renato Mujica
* Zhiheng Lei

## 1. Objetivo del Trabajo
Implementar un sistema básico de navegación reactiva en Webots para un robot móvil diferencial, utilizando sensores de distancia y encoders de rueda. El objetivo principal es aplicar técnicas de filtrado sobre las mediciones crudas y emplear un filtro de Kalman para fusionar la información de movimiento y percepción, estimando de manera robusta la proximidad a los obstáculos para mejorar la toma de decisiones.

## 2. Cómo ejecutar
1. **Requisitos:** Tener instalado **Webots** y **Python 3.x**.
2. **Importar el mundo:** Importar el mundo "lab2.wbt" que se encuentra en la carpeta "worlds".
3. **Configuración:** * Seleccionar el robot e-puck en el árbol de escena.
    * En el campo `controller`, crear un nuevo controlador personalizado y pegarle el código que se encuentra en el repositorio. (Código debe ser en Python)
5. **Simulación:** Presionar el botón `Play` para iniciar el movimiento.

## 3. Descripción del Robot y Sensores Utilizados
En este laboratorio se utilizó el robot diferencial **e-puck** simulado en Webots. Para lograr la navegación y la percepción del entorno, se emplearon los siguientes sensores:
* **Sensores de distancia frontales:** `ps7` (izquierdo) y `ps0` (derecho), utilizados para medir la proximidad frontal a los obstáculos.
* **Sensores de distancia laterales:** `ps5` (izquierdo) y `ps2` (derecho), empleados para determinar la dirección de giro al evadir un obstáculo.
* **Encoders de rueda:** `left wheel sensor` y `right wheel sensor`, que miden el desplazamiento angular de las ruedas para estimar la odometría del robot.

## 4. Frecuencia de Muestreo
El controlador sincroniza la lectura de datos extrayendo el timeStep básico del robot. Al ejecutar la simulación, el sistema calcula automáticamente los valores:
* **Tiempo de muestreo (Ts):** Calculado como timeStep / 1000.0 segundos.
* **Frecuencia de muestreo (fs):** Calculado como 1 / Ts Hz.
*(Los valores exactos y el total de muestras registradas se imprimen en la consola al finalizar la simulación).*

## 5. Análisis de las Señales Registradas
Las señales crudas obtenidas de los sensores infrarrojos (ps7 y ps0) presentan fluctuaciones, especialmente al interactuar con objetos irregulares del entorno. Estas variaciones abruptas justifican el uso de filtros para evitar que el robot tome decisiones erráticas.

## 6. Estimación del Avance mediante Encoders
Para estimar el avance lineal del robot a partir de los encoders, se utiliza la relación geométrica s = r * theta. En la función calcularAvanceRuedas, se calcula la diferencia de posición angular de cada rueda, se multiplica por el radio de la rueda (r = 0.0205 m) y se promedia el avance izquierdo y derecho para obtener el desplazamiento lineal central (avance) del robot.

## 7. Filtro Simple Aplicado
Se implementó un **Filtro de Mediana** con un tamaño de ventana de 5 muestras (largoFiltro = 5). Este filtro almacena el historial reciente de las lecturas, las ordena y selecciona el valor central. Esto permite eliminar de manera efectiva los valores atípicos (outliers) y el ruido impulsivo de las lecturas crudas antes de enviarlas al filtro de Kalman.
