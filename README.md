# Control No Lineal y Estabilización en Tiempo Real de un Sistema Subactuado utilizando Arquitectura C2000

[![MATLAB](https://img.shields.io/badge/MATLAB-R2024b+-ED7D31?style=for-the-badge&logo=mathworks&logoColor=white)](https://www.mathworks.com/products/matlab.html)
[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org/)
[![C2000 TI](https://img.shields.io/badge/Texas_Instruments-TMS320F280049C-CC0000?style=for-the-badge&logo=texas-instruments&logoColor=white)](https://www.ti.com/product/es-mx/TMS320F280049C)
[![Languages](https://img.shields.io/badge/Languages-C%20%2F%20Python%20%2F%20MATLAB-00599C?style=for-the-badge&logo=cplusplus&logoColor=white)](./)

Este repositorio contiene el ecosistema completo de software, firmware y herramientas de análisis desarrolladas para mi Trabajo de Final de Grado (TFG) en el **Grado en Inteligencia Robótica** de la **Universitat Jaume I (UJI)**.

El proyecto aborda de forma integral el modelado matemático, la identificación paramétrica experimental, la simulación híbrida, el control en tiempo real estricto y la telemetría 3D de un **sistema subactuado de péndulo invertido sobre carro (Cart-Pole)** utilizando la plataforma de control en tiempo real **TI C2000**.

<p align="center">
  <img src="memoria/Pictures/modelo_3D.png" width="550" alt="Modelo 3D del Sistema Cart-Pole Péndulo Invertido">
  <br>
  <em>Gometría y diseño conceptual del sistema.</em>
</p>

---

## Características Clave

* **Firmware:** Implementación en C sobre el microcontrolador `TMS320F280049C`, configurando de forma nativa periféricos críticos como `ePWM` (control del motor), `eQEP` (lectura de encoders en cuadratura por hardware) y `SCI` (comunicación serie asíncrona).
* **Estrategias de Control Avanzado:**
  * **Swing-up:** Control no lineal basado en energía y linealización parcial por realimentación (*Partial Feedback Linearization - PFL*).
  * **Estabilización:** Control por realimentación del estado ($Kx$) diseñado mediante LQR, incluyendo variantes con acción integral.
  * **Control Híbrido Conmutado:** Transición suave y segura en tiempo real entre el algoritmo de swing-up y el control de estabilización en la vecindad del punto de equilibrio inestable.
* **Identificación Experimental:** Ensayos específicos orientados a la obtención del modelo dinámico real del sistema:
  * **Caracterización de la Zona Muerta del Actuador:** Análisis del comportamiento estático del motor y el puente en H a diferentes frecuencias de PWM (de 200 Hz a 10 kHz) para determinar el umbral mínimo de tensión necesario para vencer la fricción estática.
  * **Relación Par-Voltaje:** Ensayos dinámicos utilizando controladores P y PI de velocidad para caracterizar la constante del motor y modelar de forma precisa la conversión entre el par de control calculado y el voltaje aplicado.
  * **Identificación en Bucle Cerrado del Carro:** Ensayos experimentales aplicando un control proporcional (P) de posición sobre el carro para excitar el sistema de forma segura en bucle cerrado, permitiendo estimar la masa efectiva y sus coeficientes de fricción.
  * **Identificación del Rozamiento del Péndulo:** Ensayos de oscilación libre del péndulo y aplicación del método de decremento logarítmico en MATLAB para aislar y modelar el coeficiente de amortiguamiento viscoso del eje.
* **Gemelo Digital y Telemetría en ROS 2:** Visualización 3D en tiempo real del estado del sistema utilizando un modelo `URDF` en `RViz`. Implementa una arquitectura versátil con doble vía de entrada de datos:
  * **Modo Simulación:** Conexión directa con **MATLAB/Simulink** para validar la respuesta de los controladores en el entorno virtual.
  * **Modo Sistema Real:** Monitorización del prototipo físico mediante un nodo dedicado en Python (`serial_node.py`) que monitoriza de forma eficiente los datos de telemetría recibidos por el puerto serie (`SCI`) del microcontrolador. 

---

## Estructura del Repositorio

El proyecto está organizado de manera modular para separar las fases de análisis, simulación, despliegue y documentación:
```text
├── analisis_ensayos/          # Scripts de MATLAB para procesar datos experimentales reales
│   ├── controladores_Kx_cartpole/     # Comparativa de transitorios con diferentes matrices Q y R
│   ├── control_Kx_integrador_cartpole/# Ensayos del controlador con acción integral
│   ├── conversion_par_voltaje/        # Caracterización del motor y puente en H
│   ├── ident_BC_carro/                # Identificación de la dinámica del carro
│   ├── rozamiento_pendulo/            # Ensayos de oscilación libre para modelar la fricción
│   └── zona_muerta_frec_PWM/          # Análisis del comportamiento del motor según la freq del PWM
│
├── firmware_C2000/            # Código fuente (Code Composer Studio)
│   ├── experimentos/                  # Controladores específicos probados de forma aislada (código .c)
│   └── f280049c_ws/                   # Workspace de CCS con los drivers HAL y periféricos configurados
│
├── simulacion/                # Entornos de simulación
│   ├── simulaciones_codigo/           # Scripts .m de control híbrido, swing-up y LQR
│   └── simulink/                      # Modelos .slx y pasarela de comunicación con ROS
│
├── ros2_ws/                   # Workspace de ROS 2 para la monitorización en tiempo real
│   └── src/cartpole_digital_twin/
│       ├── launch/                    # Launch files para telemetría y gemelo digital
│       ├── rviz/                      # Configuración del entorno visual de RViz
│       ├── urdf/                      # Modelo geométrico y físico del Péndulo-Carro
│       └── cartpole_digital_twin/     # Nodo Python para lectura del puerto serie e hilos de ejecución
│
└── docs/                      # Memoria técnica del TFG, planos y vídeos demostrativos
