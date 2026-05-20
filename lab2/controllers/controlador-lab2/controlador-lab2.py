from controller import Robot
from math import sin, cos, degrees
import csv

robot = Robot()
timeStep = int(robot.getBasicTimeStep())

motorRuedaIzq = robot.getDevice("left wheel motor")
motorRuedaIzq.setPosition(float("inf"))
motorRuedaIzq.setVelocity(0.0)

motorRuedaDer = robot.getDevice("right wheel motor")
motorRuedaDer.setPosition(float("inf"))
motorRuedaDer.setVelocity(0.0)

distFrontIzq = robot.getDevice("ps7")
distFrontIzq.enable(timeStep)

distFrontDer = robot.getDevice("ps0")
distFrontDer.enable(timeStep)

distLatIzq = robot.getDevice("ps5")
distLatIzq.enable(timeStep)

distLatDer = robot.getDevice("ps2")
distLatDer.enable(timeStep)

encoderRuedaIzq = robot.getDevice("left wheel sensor")
encoderRuedaIzq.enable(timeStep)

encoderRuedaDer = robot.getDevice("right wheel sensor")
encoderRuedaDer.enable(timeStep)

radioRuedas = 0.0205
distanciaEjes = 0.052
velocidadMax = 6.28

tiempoMuestreoTs = timeStep / 1000.0
frecuenciaMuestreoFs = 1.0 / tiempoMuestreoTs

pos = {
    "theta": 0.0,
    "theta_angulo": 0.0,
    "actual": 0.0,
    "x": 0.0,
    "y": 0.0,
    "encoderIzqPrevio": 0.0,
    "encoderDerPrevio": 0.0,
}

historialFrontIzq = []
historialFrontDer = []
largoFiltro = 5

estimacionKalman = 0.0
covarianzaError = 1.0
ruidoProceso = 0.05
ruidoMedicion = 0.5

estadoGiro = False
direccionGiro = 0
conteoGiro = 0
pasosMinimosGiro = 15

sumaRaw = 0.0
sumaFiltrado = 0.0
sumaKalman = 0.0
conteoMuestrasTotales = 0
tiempo_simulacion = 0.0

archivo_info = open("datos-lab2.csv", mode="w", newline='')
escritor = csv.writer(archivo_info)
escritor.writerow(['tiempo_s', 'distancia_recorrida_m', 'posicion_x', 'posicion_y', 'angulo_theta', 'ps7_izq_raw', 'pS0_der_raw', 'ps7_izq_filtrado', 'ps0_der_filtrado', 'distancia_kalman'])

def calcularAvanceRuedas(pos, posActualIzq, posActualDer):
    deltaIzq = posActualIzq - pos["encoderIzqPrevio"]
    deltaDer = posActualDer - pos["encoderDerPrevio"]
    avanceRuedaIzq = deltaIzq * radioRuedas
    avanceRuedaDer = deltaDer * radioRuedas
    avance = (avanceRuedaIzq + avanceRuedaDer) / 2
    deltaTheta = (avanceRuedaDer - avanceRuedaIzq) / distanciaEjes

    pos["actual"] += avance
    pos["theta"] += deltaTheta
    pos["theta_angulo"] = degrees(pos["theta"]) % 360.0
    pos["x"] += avance * cos(pos["theta"])
    pos["y"] += avance * sin(pos["theta"])
    return avance

def aplicarFiltroMediana(historial, nuevaLectura):
    historial.append(nuevaLectura)
    if len(historial) > largoFiltro:
        historial.pop(0)
    
    historialOrdenado = sorted(historial)
    n = len(historialOrdenado)
    mitad = n // 2
    
    if n % 2 == 1:
        return historialOrdenado[mitad]
    else:
        return (historialOrdenado[mitad - 1] + historialOrdenado[mitad]) / 2

def ejecutarFiltroKalman(avanceRobot, medicionSensor):
    global estimacionKalman, covarianzaError
    
    factorEscalaProximidad = avanceRobot * 2000.0
    prediccionDistancia = estimacionKalman + factorEscalaProximidad
    
    covarianzaPrediccion = covarianzaError + ruidoProceso
    
    gananciaKalman = covarianzaPrediccion / (covarianzaPrediccion + ruidoMedicion)
    estimacionKalman = prediccionDistancia + gananciaKalman * (medicionSensor - prediccionDistancia)
    covarianzaError = (1 - gananciaKalman) * covarianzaPrediccion
    
    if estimacionKalman < 0.0:
        estimacionKalman = 0.0
        
    return estimacionKalman

while robot.step(timeStep) != -1:
    tiempo_simulacion += tiempoMuestreoTs
    print(pos)
    posEncoderIzq = encoderRuedaIzq.getValue()
    posEncoderDer = encoderRuedaDer.getValue()
    
    avanceRobot = calcularAvanceRuedas(pos, posEncoderIzq, posEncoderDer)
    
    pos["encoderIzqPrevio"] = posEncoderIzq
    pos["encoderDerPrevio"] = posEncoderDer
    
    lecturaIzqRaw = distFrontIzq.getValue()
    lecturaDerRaw = distFrontDer.getValue()
    
    lecturaIzqFiltrada = aplicarFiltroMediana(historialFrontIzq, lecturaIzqRaw)
    lecturaDerFiltrada = aplicarFiltroMediana(historialFrontDer, lecturaDerRaw)
    
    medicionFrontalCombinada = (lecturaIzqFiltrada + lecturaDerFiltrada) / 2
    
    distanciaEstimada = ejecutarFiltroKalman(avanceRobot, medicionFrontalCombinada)
    
    sumaRaw += lecturaIzqRaw
    sumaFiltrado += lecturaIzqFiltrada
    sumaKalman += distanciaEstimada
    conteoMuestrasTotales += 1
    
    umbralObstaculo = 95.0
    umbralDespejado = 75.0
    
    if not estadoGiro:
        if distanciaEstimada > umbralObstaculo:
            estadoGiro = True
            conteoGiro = 0
            if distLatIzq.getValue() > distLatDer.getValue():
                direccionGiro = 1
            else:
                direccionGiro = -1
                
    if estadoGiro:
        conteoGiro += 1
        if direccionGiro == 1:
            motorRuedaIzq.setVelocity(velocidadMax * 0.4)
            motorRuedaDer.setVelocity(-velocidadMax * 0.4)
        else:
            motorRuedaIzq.setVelocity(-velocidadMax * 0.4)
            motorRuedaDer.setVelocity(velocidadMax * 0.4)
            
        if distanciaEstimada < umbralDespejado and conteoGiro >= pasosMinimosGiro:
            estadoGiro = False
    else:
        motorRuedaIzq.setVelocity(velocidadMax * 0.5)
        motorRuedaDer.setVelocity(velocidadMax * 0.5)
    
    escritor.writerow([
        tiempo_simulacion,
        pos["actual"],
        pos["x"],
        pos["y"],
        pos["theta"],
        lecturaIzqRaw,
        lecturaDerRaw,
        lecturaIzqFiltrada,
        lecturaDerFiltrada,
        distanciaEstimada
    ])

promedioRaw = sumaRaw / conteoMuestrasTotales
promedioFiltrado = sumaFiltrado / conteoMuestrasTotales
promedioKalman = sumaKalman / conteoMuestrasTotales

print("\n=== REPORTE FINAL DE SIMULACION (30 SEGUNDOS) ===")
print(f"Tiempo Ts: {tiempoMuestreoTs} s | Frecuencia Fs: {frecuenciaMuestreoFs:.2f} Hz")
print(f"Muestras Registradas: {conteoMuestrasTotales}")
print(f"Posicion Final -> X: {pos['x']:.4f} | Y: {pos['y']:.4f} | Theta: {pos['theta']:.4f}")
print(f"Distancia Total Avanzada: {pos['actual']:.4f} metros")
print(f"Promedio Senales -> RAW: {promedioRaw:.2f} | MEDIANA: {promedioFiltrado:.2f} | KALMAN: {promedioKalman:.2f}")
print("=================================================\n")
