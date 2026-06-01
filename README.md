# Control No Lineal y Estabilización en Tiempo Real de un Sistema Subactuado utilizando Arquitectura C2000

[![MATLAB](https://img.shields.io/badge/MATLAB-R2024b+-ED7D31?style=for-the-badge&logo=mathworks&logoColor=white)](https://www.mathworks.com/products/matlab.html)
[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org/en/humble/index.html)
[![C2000 TI](https://img.shields.io/badge/Texas_Instruments-TMS320F280049C-CC0000?style=for-the-badge&logo=texas-instruments&logoColor=white)](https://www.ti.com/product/es-mx/TMS320F280049C)
[![Languages](https://img.shields.io/badge/Languages-C%20%2F%20Python%20%2F%20MATLAB-00599C?style=for-the-badge&logo=cplusplus&logoColor=white)](./)

Este repositorio contiene el ecosistema completo de software, firmware y herramientas de análisis desarrolladas para mi Trabajo de Final de Grado (TFG) en el **Grado en Inteligencia Robótica** de la **Universitat Jaume I (UJI)**.

El proyecto aborda de forma integral el modelado matemático, la identificación experimental, la simulación, el control en tiempo real y la telemetría 3D de un **sistema subactuado de péndulo invertido sobre carro (Cart-Pole)** utilizando la plataforma de control en tiempo real **TI C2000**.

<p align="center">
  <img src="memoria/Pictures/modelo_3D.png" width="550" alt="Modelo 3D del Sistema Cart-Pole Péndulo Invertido">
  <br>
  <em>Gometría y diseño conceptual del sistema.</em>
</p>

---

## Características Clave del Sistema

### Firmware
Implementación nativa en **C** sobre el microcontrolador de control en tiempo real `TMS320F280049C`. Configuración a bajo nivel de periféricos críticos para garantizar determinismo estricto:
* `ePWM`: Modulación por ancho de pulsos para el control del motor de continua.
* `eQEP`: Decodificación por hardware de encoders en cuadratura para la lectura posicional.
* `SCI`: Interfaz de comunicación serie asíncrona dedicada a la transmisión de telemetría de alta velocidad.

### Estrategias de Control Avanzado
* **Algoritmo de Swing-Up:** Ley de control no lineal basada en energía y linealización parcial por realimentación (*Partial Feedback Linearization - PFL*) para elevar el péndulo desde su posición de reposo.
* **Control de Estabilización:** Regulación robusta mediante realimentación del estado ($Kx$) diseñada a través de un regulador óptimo lineal cuadrático (**LQR**), incluyendo extensiones con acción integral.
* **Control Híbrido Conmutado:** Autómata de control que gestiona la transición entre el lazo de balanceo y el lazo de estabilización dentro de la región de atracción del punto de equilibrio inestable.

### Gemelo Digital (ROS 2 Humble)
Desacoplo de la capa de control crítico de la capa de visualización 3D en `RViz` mediante un modelo geométrico `URDF`, operando bajo una arquitectura dual:
* **Modo Simulación:** Co-simulación directa con **MATLAB/Simulink** a través de *ROS Toolbox* para la validación previa de los algoritmos en entornos virtuales.
* **Modo Sistema Real:** Monitorización del prototipo físico mediante el nodo `serial_node.py` en Python, encargado de monitorizar eficientemente de las tramas provenientes del periférico `SCI`.

---
## Requisitos del Sistema

Para compilar, simular y desplegar el proyecto, es necesario contar con el siguiente entorno de hardware y software:

| Categoría | Componente / Herramienta | Versión Recomendada | Notas Adicionales |
| :--- | :--- | :--- | :--- |
| **Hardware** | <img src="https://upload.wikimedia.org/wikipedia/commons/thumb/c/c0/Texas_Instruments_logo.svg/512px-Texas_Instruments_logo.svg.png" width="18" align="center"/> TI LAUNCHXL-F280049C | Rev. A o superior | Placa de desarrollo principal. |
| **Hardware** | Módulo Puente en H | - | Compatible con señales PWM de 3.3V. |
| **Software** | <img src="https://cdn.jsdelivr.net/gh/devicons/devicon@latest/icons/ubuntu/ubuntu-original.svg" width="18" align="center"/> Ubuntu Linux | 22.04 LTS (Jammy) | Requerido para la compatibilidad con ROS 2. |
| **Software** | <img src="https://upload.wikimedia.org/wikipedia/commons/b/bb/Ros_logo.svg" width="18" align="center"/> ROS 2 | Humble | Para el Gemelo Digital y telemetría. |
| **Software** | <img src="https://cdn.jsdelivr.net/gh/devicons/devicon@latest/icons/matlab/matlab-original.svg" width="18" align="center"/> MATLAB & Simulink | R2024b o superior | Incluyendo *ROS Toolbox* y *Control System Toolbox*. |
| **Software** | <img src="https://cdn.jsdelivr.net/gh/devicons/devicon@latest/icons/c/c-original.svg" width="18" align="center"/> Code Composer Studio | 20.x o superior | IDE para la programación, compilación y flasheo del firmware en C. |

---

## Caracterización e Identificación Experimental

Con el objetivo de obtener las constantes físicas precisas para el modelo dinámico formal, se diseñó e implementó la siguiente metodología de ensayos:

| Módulo del Sistema | Método Experimental | Variable Identificada | Propósito en el Modelo |
| :--- | :--- | :--- | :--- |
| **Actuador y Puente en H** | Análisis estático variando frecuencias de ciclo de trabajo en el rango de `200 Hz` a `10 kHz`. | **Zona Muerta ($ZM$)** | Determinar el umbral mínimo de tensión necesario para vencer la fricción estática del motor para cada frecuencia. |
| **Planta Motriz** | Control de posición del eje del motor aplicando el concepto de palanca con pesos calibrados. | **Relación Par-Voltaje ($K_m$)** | Caracterizar la ganancia electromecánica del motor y modelar la conversión par-tensión. |
| **Dinámica del Carro** | Control de posición tipo P. | **Masa  ($M$) y Fricción ($c_x$)** | Estimar la masa del carro y su coeficiente de rozamiento viscoso. |
| **Eje del Péndulo** | Ensayos de oscilación libre desde condiciones iniciales no nulas | **Amortiguamiento ($c_p)$)** | Aislar el coeficiente de fricción viscosa del eje rotatorio mediante **decremento logarítmico**. |

--- 
## Estructura del Repositorio

El proyecto está organizado de manera modular para separar las fases de análisis, simulación, despliegue y documentación:
```diff
TFG_Control_MiguelPorcar/
+ analisis_ensayos/                  # Scripts de procesamiento de datos experimentales reales
│   ├── controladores_Kx_cartpole/     # Comparativas de transitorios con matrices Q y R
│   ├── control_Kx_integrador_cartpole # Ensayos del controlador con acción integral
│   ├── conversion_par_voltaje/        # Caracterización del motor y puente en H
│   ├── ident_BC_carro/                # Identificación de la dinámica y masa del carro
│   ├── rozamiento_pendulo/            # Ensayos de oscilación libre y amortiguamiento
│   └── zona_muerta_frec_PWM/          # Comportamiento del motor según la frecuencia de PWM
│
+ firmware_C2000/                    # Código fuente embebido (Code Composer Studio)
│   ├── experimentos/                  # Módulos lógicos y archivos de control .c individuales
│   └── f280049c_ws/                   # Workspace de CCS con drivers HAL y periféricos
│
+ simulacion/                        # Entornos virtuales previos al despliegue físico
│   ├── simulaciones_codigo/           # Algoritmos de control híbrido, swing-up y LQR (.m)
│   └── simulink/                      # Modelos de bloques .slx y pasarela ROS
│
+ ros2_ws/                           # Workspace de ROS 2 Humble (Nodos de visualización)
│   └── src/cartpole_digital_twin/
│       ├── launch/                    # Lanzadores duales (sistema real y simulación)
│       ├── rviz/                      # Configuraciones gráficas de RViz
│       ├── urdf/                      # Modelo geométrico y físico del robot
│       └── cartpole_digital_twin/     # Nodos e hilos en Python de lectura serie
│
+ docs/                              # Memoria técnica del TFG, planos y vídeos demostrativos
