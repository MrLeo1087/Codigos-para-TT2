# -*- coding: utf-8 -*-
"""
Trabajo Terminal 1 - CINVESTAV
Validación de Modelo y Control PD de Servomotor Harmonic Drive
Autor: Leonardo Mondragón Ruíz

"""

import control as ct
import numpy as np
import matplotlib.pyplot as plt
import sympy as sp


# =========================================================
# 1. DEFINICIÓN DE PARÁMETROS DEL HARDWARE
# =========================================================
# Parámetros Empíricos (Modelo Real)s


b = 27
a = 9


# Parámetros Teóricos Motor 1

R1  = 2.7;     
L1  = 0.0011;   
Kt1 = 2.92;     
Kb1 = 2.865;    
J1  = 0.0216;  
Bm1  = 0.0334; 

# Parámetros Teóricos Motor 1  
 
R2  = 4.7;        
L2  = 0.0016;   
Kt2 = 2.46;     
Kb2 = 2.4828;   
J2  = 0.011;    
Bm2  = 0.1719;

# =========================================================
# 2. DEFINICIÓN DEL ESPACIO DE ESTADOS
# =========================================================

A1 = np.array([
    [0, 1],
    [0, -(Kt2*Kb2+ Bm2*R2)/(J2*R2)]
    ])

A2 = np.array([
    [0, 1],
    [0, -a]
    ])

B1 = np.array([
    [0],
    [(Kt2)/(J2*R2)]
    ])

B2 = np.array([
    [0],
    [b]
    ])

C1 = np.array([
       [1, 0]
       ])

C2 = np.array([
       [1, 0]
       ])

D1 = np.array([[0]])

D2 = np.array([[0]])


sys_teorico = ct.ss(A1, B1, C1, D1, outputs=['theta_1'], name="Servomotor Harmonic Drive_teorico")
sys_real = ct.ss(A2, B2, C2, D2, outputs=['theta_1'], name="Servomotor Harmonic Drive_real")

titulo = " Espacio de estados modelo teorico "
print(f"\n{titulo.center(60, '/')}\n")
print(sys_teorico)

titulo = " Espacio de estados modelo real "
print(f"\n{titulo.center(60, '/')}\n")
print(sys_real)

# =========================================================
# 3. ANÁLISIS DE DINÁMICA NATURAL (POLOS LAZO ABIERTO)
# =========================================================

s = sp.symbols('s')
M = sp.Matrix([[s, 0], 
               [0, s]])

resta_teorico = M-A1
resta_real = M-A2

determinante_teorico = resta_teorico.det()
determinante_real = resta_real.det()

# =========================================================
# 4. ANÁLISIS DE CONTROLABILIDAD
# =========================================================

Controlabilidad_1 = ct.ctrb(A1, B1)
Controlabilidad_2 = ct.ctrb(A2, B2)


rango1 = np.linalg.matrix_rank(Controlabilidad_1)
rango2 = np.linalg.matrix_rank(Controlabilidad_2)

n1 = A1.shape[0]
n2 = A2.shape[0]

print(f"\n{' 3. MATRICES DE CONTROLABILIDAD '.center(70, '=')}")
print(f"Rango Teórico: {rango1} (Debe ser igual a {A1.shape[0]})")
print(f"Rango Real   : {rango2} (Debe ser igual a {A2.shape[0]})")

# =========================================================
# 5. CÁLCULO DE GANANCIAS (MÉTODO DE ACKERMANN)
# =========================================================

# Determinacion de polos en funcion del comportamiento deseado

coeficiente_amortiguamiento = 1 # Comportamiento sobreamoortiguado
tiempo_establecimiento = 0.2 # Criterio del disenador
frecuencia_natural = 4/(tiempo_establecimiento * coeficiente_amortiguamiento)
coeficientes = [1, 2*frecuencia_natural*coeficiente_amortiguamiento, frecuencia_natural**2]

raices = np.roots(coeficientes)

polos_deseados = [raices[0], raices[1]]

# Cálculo de la matriz de realimentación K usando el modelo real
# Esto equivale al método de Ackermann K = acker(A, B, J)
K_teorico = np.atleast_2d(ct.acker(A1, B1, polos_deseados))   # (1, 2)
K_real    = np.atleast_2d(ct.acker(A2, B2, polos_deseados))   # (1, 2)

## Simulacion del controlador 
# Partiendo de la realcion xp = Ax + Bu
# en su forma nominal la ley de control se ve de la forma u = -k1x1 -k2x2
# Con el estado x1 siendo la posicion y x2 siendo la velocidad
# Con la e = r - y
# ep = -yp
# epp = -ypp
# Introducimos la nueva ley de control 
# u = -KX + K1r donde r es la referencia a seguir
# Sustituyendo en el espacio de estados, nos queda de la siguiente forma
# xp = AX +B(-KX + K1r) -> (A - BK)X + K1r

print(f"\n{' 4. GANANCIAS DEL CONTROLADOR (MATRIZ K) '.center(70, '=')}")
print(f"Polos Deseados: {polos_deseados}")
print(f"\nK Teórico: {K_teorico}")
print(f"K Real   : {K_real}")

# Comprobacion de las ganancias calculadas

coef_1 = [1, (a+b*K_real[0,1]), b*K_real[0,0]]
poles = np.roots(coef_1)

titulo = " Comprobacion de polos "
# Centra el texto en un ancho de 60 caracteres rellenando con "/"
print(f"\n{titulo.center(60, '/')}\n")
print("Las raíces del polinomio son:", poles)


# =========================================================
# 4. ANÁLISIS DE OBSERVABILIDAD
# =========================================================

# 2. Calcular la matriz de observabilidad (O)
# La función 'obsv' calcula: [C; C*A; C*A^2; ... ; C*A^(n-1)]
obs_real = ct.obsv(A2, C2)
titulo = " Matriz de observabilidad "
print(f"\n{titulo.center(60, '/')}\n")
print(f"La matriz de observabilidad es: {obs_real}")

A2_t = A2.T
C2_t = C2.T
B2_t = B2.T
D2_t = D2.T

#Formacion del espacio de estados

sys_obersavador = ct.ss(A2_t, C2_t, B2_t, D2_t, outputs=['voltaje'], name="Espacio de estados del observador")

# Comportamiento sobreamoortiguado
tiempo_establecimiento_obs = 0.05 # Criterio del disenador
frecuencia_natural_obs = 4/(tiempo_establecimiento_obs * coeficiente_amortiguamiento)
coeficientes_obs = [1, 2*frecuencia_natural_obs*coeficiente_amortiguamiento, frecuencia_natural_obs**2]

raices_obs = np.roots(coeficientes_obs)

polos_deseados_obs = [raices_obs[0], raices_obs[1]]

# Cálculo de la matriz de realimentación K usando el modelo real
# Esto equivale al método de Ackermann K = acker(A, B, J)  # (1, 2)
K_real_obs = np.atleast_2d(ct.acker(A2_t, C2_t, polos_deseados_obs))   # (1, 2)

print(f"\n{' 4. GANANCIAS DEL OBSERVADOR (MATRIZ Ko) '.center(70, '=')}")
print(f"Polos Deseados: {polos_deseados_obs}")
print(f"K Real   : {K_real_obs}")


# =========================================================
# 4. Desarrollo de ADRC
# =========================================================

# Definimos al termino =ax2 como un perturbacion que se agrupa conD
# xip = x2
# x2p = bu + x3
# x3p = dp

# El tercer estado se define como la perturbacion de la cual no ceonocemos basicamente nada
# En la matriz A tendra un peso de cero



A3 = np.array([
    [0, 1, 0],
    [0, 0, 1],
    [0, 0, 0]
    ])

B3 = np.array([
    [0],
    [b],
    [0]
    ])


C3 = np.array([
       [1, 0, 0]
       ])

D3 = np.array([[0]])
 
sys_real_ADRC = ct.ss(A3, B3, C3, D3, outputs=['theta_1_ADRC'], name="Servomotor Harmonic Drive_teorico, modelo con perturbaciones")

## Analizamos controlabilidad del sistema

Controlabilidad_3 = ct.ctrb(A3, B3)

rango3 = np.linalg.matrix_rank(Controlabilidad_3)

n3 = A3.shape[0]

print(f"\n{' 3. MATRICES DE CONTROLABILIDAD (ADRC) '.center(70, '=')}")
print(f"Rango Teórico: {rango3} (Debe ser igual a {A3.shape[0]})")

## Obtenemos que el sistema no es controlable

## Analizamos la observabilidad del sistema

Observabilidad_3 = ct.obsv(A3, C3)
rango_obs_ADRC = np.linalg.matrix_rank(Observabilidad_3)

det_obs_ADRC = np.linalg.det(Observabilidad_3)

print(f"Determinante de la matriz de observabildas", det_obs_ADRC)

## Determinante es diferente de cero, podemos estimar la perturbacion del sistema

## Calculamos el polimnomio caracteristico del observador, se tienen tres ganancias
# como incognitas, L1, L2 y L3. con un tiempo de establecimiento de 0.05 segundos
# calculamos la frecuencia natrual del sistema
ts_ADRC = 0.175

wn_ADRC = 4/(1*ts_ADRC)

#Calculamos los coeficientes del polinomio caracteristico

coef_3 = [1, 3*wn_ADRC, 3*wn_ADRC**2, wn_ADRC**3]
 
poles_obs_ADRC = np.roots(coef_3)

polos_deseados_obs_ADRC = [poles_obs_ADRC[0], poles_obs_ADRC[1], poles_obs_ADRC[2]]

# Cálculo de la matriz de realimentación K usando el modelo real
# Esto equivale al método de Ackermann K = acker(A, B, J)  # (1, 2)
K_real_obs_ADRC = np.atleast_2d(ct.acker(A3.T, C3.T, polos_deseados_obs_ADRC))   # (1, 2)

print(f"\n{' 4. GANANCIAS DEL OBSERVADOR (MATRIZ Ko del ADRC) '.center(70, '=')}")
print(f"K Real ADRC   : {K_real_obs_ADRC}")











