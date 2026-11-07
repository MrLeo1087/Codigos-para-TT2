# -*- coding: utf-8 -*-
"""
Created on Thu Oct 22 12:33:57 2026

@author: mondr
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline

# Datos del Amplificador 2 (motor 2) - Extraídos de motor2_amplificador2.txt
datos_amp2 = np.array([
    [8.406152, 18.524535],
    [9.509849, 27.456846],
    [9.284615, 26.972272],
    [9.542406, 18.399878],
    [13.122919, 14.466918],
    [14.699140, 11.668602]
])

# Datos del Amplificador 3 (motor 2) - Extraídos de motor2_amplificador3.txt
datos_amp3 = np.array([
    [4.939703, 5.542520],
    [6.521412, 8.392665],
    [7.478186, 10.717371],
    [7.919120, 12.216347],
    [8.389961, 12.631539],
    [8.929847, 12.315700],
    [9.717068, 11.685352],
    [10.454252, 10.880779],
    [11.151454, 10.464020],
    [11.527060, 10.027939],
    [11.849501, 9.747947],
    [12.119605, 9.552781],
    [12.204751, 9.209454],
    [12.374013, 9.262185]
])

def graficar_suavizado(datos, titulo):
    # Eje X: Generar un vector secuencial basado en el número de filas (Pruebas 1 a N)
    x = np.arange(1, len(datos) + 1)
    a = datos[:, 0]
    b = datos[:, 1]

    # Crear un eje X más denso con 300 puntos para calcular la curva suave
    x_suave = np.linspace(x.min(), x.max(), 300)

    # Interpolar utilizando Splines Cúbicos
    spline_a = make_interp_spline(x, a, k=3)
    spline_b = make_interp_spline(x, b, k=3)

    a_suave = spline_a(x_suave)
    b_suave = spline_b(x_suave)

    # Configuración de la figura
    plt.figure(figsize=(10, 6))
    
    # Graficar las líneas continuas y suaves
    plt.plot(x_suave, a_suave, label='Parámetro $a$ (Suave)', color='#1f77b4', linewidth=2.5)
    plt.plot(x_suave, b_suave, label='Parámetro $b$ (Suave)', color='#ff7f0e', linewidth=2.5)
    
    # Superponer los puntos reales medidos
    plt.scatter(x, a, color='#1f77b4', s=60, zorder=5, label='Datos reales $a$')
    plt.scatter(x, b, color='#ff7f0e', s=60, zorder=5, label='Datos reales $b$')

    # Detalles visuales
    plt.title(titulo, fontsize=14, fontweight='bold')
    plt.xlabel('Número de Prueba', fontsize=12)
    plt.ylabel('Valor del Parámetro', fontsize=12)
    plt.xticks(x) # Forzar que el eje X muestre solo números enteros
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend(fontsize=11)
    plt.tight_layout()
    plt.show()

# Ejecutar la función para ambos conjuntos de datos
graficar_suavizado(datos_amp2, 'Evolución de Parámetros: Motor 2 - Amplificador 2')
graficar_suavizado(datos_amp3, 'Evolución de Parámetros: Motor 2 - Amplificador 3')