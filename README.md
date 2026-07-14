# SUMO-UAV-Py Extended

Entorno de simulación para el estudio del uso de UAVs para el despliegue de redes temporales.

Este repositorio extiende [SUMO-UAV-Py](https://github.com/TsioutisCh/SUMO-UAV-Py), un plugin de código abierto en Python que integra la simulación de UAVs en el simulador de tráfico [SUMO](https://eclipse.dev/sumo/) mediante la interfaz [TraCI](https://sumo.dlr.de/docs/TraCI.html). Se añaden un modelado realista del UAV (batería, viento, retorno a base, relevo), un módulo de optimización de cobertura y la co-simulación con el simulador de red [OMNeT++](https://omnetpp.org/) vía [Veins](https://veins.car2x.org/).

---

## Características

- **UAVs como vehículos TraCI** — Los UAVs se registran como objetos `vehicle` en SUMO (`traci.vehicle.add()`), haciéndolos visibles para otros clientes TraCI y simuladores de red.
- **Modelo de batería dependiente de la velocidad** — Consumo energético diferenciado entre hovering, velocidad de crucero y velocidad máxima.
- **Modelo de viento** — Penalización configurable en dirección e intensidad que afecta a la tasa de descarga.
- **Retorno automático a base** — Máquina de estados que gestiona el ciclo misión → retorno → recarga → reanudación, con umbral dinámico calculado en cada paso.
- **Modo relevo** — Duplica la flota en UAVs primarios y sustitutos; el sustituto asume la posición cuando el primario retorna.
- **Optimización de cobertura** — Algoritmo greedy con modelo de propagación log-distance para maximizar la cobertura de señal sobre una zona definida.
- **Co-simulación con OMNeT++** — El script `sumo-launchd-multi.py` permite que SUMO-UAV-Py y OMNeT++ compartan la misma instancia de SUMO como clientes TraCI simultáneos.

---

## Estructura del repositorio

```
.
├── main_.py                    # Módulo principal — clase UAVSimulation
├── _utils.py                   # Cálculos geométricos, FoV, POIs y polígonos
├── coverage_optimizer.py       # Módulo de optimización de posicionamiento
├── uavpy_gui.py                # Interfaz gráfica (Tkinter) de configuración
├── config.json                 # Configuración de la simulación y waypoints
├── sumo-launchd-multi.py       # Proxy multi-cliente para co-simulación con OMNeT++
├── MadridScenario/
│   ├── madrid.net.xml          # Red viaria de Madrid (SUMO)
│   ├── madrid.rou.xml          # Rutas de tráfico vehicular
│   ├── madrid.poly.xml         # Polígonos del escenario
│   ├── madrid.vtypes.xml       # Tipos de vehículos
│   └── madrid.sumocfg          # Configuración SUMO del escenario
└── simulations/                # Archivos OMNeT++ / Veins
    ├── DronesNetwork.ned       # Definición de la red OMNeT++
    └── omnetpp.ini             # Configuración de la simulación OMNeT++
```

---

## Requisitos

- **Python 3.8+**
- **SUMO** (>= 1.14) con `sumolib` y `traci`
- Dependencias Python: `numpy`, `ujson`, `Pillow`, `tkinter`
- **Para co-simulación** (opcional):
  - [OMNeT++ 6](https://omnetpp.org/)
  - [INET Framework](https://inet.omnetpp.org/)
  - [Veins 5.2+](https://veins.car2x.org/)

---

## Instalación

```bash
# Clonar el repositorio
git clone https://github.com/emiliomartinezz/TFG.git
cd TFG

# Instalar dependencias Python
pip install numpy ujson Pillow
```

Asegúrate de que SUMO está instalado y de que la variable de entorno `SUMO_HOME` apunta al directorio de instalación.

---

## Uso

### Simulación básica (SUMO + SUMO-UAV-Py)

1. Editar `config.json` con los parámetros deseados (modelo de UAV, número de drones, waypoints, modos activos, etc.).
2. Ejecutar la simulación:

```bash
python main_.py
```

### Co-simulación con OMNeT++

La co-simulación requiere tres pasos en orden:

```bash
# 1. Lanzar el proxy multi-cliente (queda escuchando en los puertos 9999 y 9998)
python sumo-launchd-multi.py

# 2. Iniciar la simulación en OMNeT++
#    Veins se conecta al puerto 9999 y envía el launch.xml

# 3. Ejecutar SUMO-UAV-Py con "OMNeT Mode": true en config.json
python main_.py
```

El proxy gestiona la sincronización de forma que ambos clientes se conecten antes de que comience ningún paso de simulación, garantizando que `setOrder()` se ejecute correctamente en los dos.

---

## Configuración

Todos los parámetros se definen en `config.json`. Los más relevantes:

| Parámetro | Tipo | Descripción |
|---|---|---|
| `Uav Model` | `string` | Modelo de UAV: `"Mavic 2e"`, `"Mini 3 pro"` o `"Manual"` |
| `Number of UAVs` | `int` | Número de UAVs en la simulación |
| `Battery Mode` | `bool` | Activa el modelo de batería dependiente de la velocidad |
| `Wind Mode` | `bool` | Activa la penalización por viento |
| `Wind Speed (m/s)` | `float` | Velocidad del viento constante |
| `Wind Direction (deg)` | `float` | Dirección del viento en grados |
| `Relay Mode` | `bool` | Activa el modo relevo (duplica la flota) |
| `Optimization Mode` | `bool` | Activa la optimización de posicionamiento |
| `OMNeT Mode` | `bool` | Conecta como segundo cliente TraCI (co-simulación) |
| `Uav Mode` | `string` | `"Hovering"`, `"Sampling"` o `"Spinning"` |
| `Movement` | `string` | `"Continuous"` o `"Discrete"` |
| `Hovering autonomy (s)` | `float` | Autonomía en hovering (s) |
| `Cruise autonomy (s)` | `float` | Autonomía a velocidad de crucero (s) |
| `Path loss Exponent` | `float` | Exponente de pérdidas del modelo de propagación |
| `Coverage threshold (dB)` | `float` | Umbral de SNR para considerar cobertura |
| `Grid resolution (m)` | `float` | Resolución del grid de optimización |
| `Step length (s)` | `float` | Duración del paso de simulación |
| `Total time (s)` | `float` | Tiempo total de simulación |
| `uav_data` | `dict` | Waypoints 5D por UAV con formato `[t, x, y, z, yaw]` |

---

## Modelos de UAV predefinidos

| Modelo | FoV (°) | Velocidad (m/s) | Vel. yaw (°/s) | Batería (s) |
|---|---|---|---|---|
| Mavic 2 Enterprise | 68,06 × 40,05 | 13,8 | 10 | 1500 |
| Mini 3 Pro | 66,92 × 40,25 | 10 | 10 | 1800 |
| Manual | Configurable | Configurable | Configurable | Configurable |

Seleccionando `"Manual"` en `Uav Model` se pueden definir manualmente todos los parámetros físicos del UAV en `config.json`.

---

## Escenario

El escenario por defecto es un área del centro de Madrid importada a SUMO. Los archivos de red viaria y rutas se encuentran en el directorio `MadridScenario/`. Para usar otro escenario, basta con actualizar las rutas `Network file` y `Sumocfg file` en `config.json`.

---

## Créditos

Este proyecto extiende [SUMO-UAV-Py](https://github.com/TsioutisCh/SUMO-UAV-Py) de Tsioutis et al. El script `sumo-launchd-multi.py` está basado en `veins_launchd` de Christoph Sommer (GPL-2.0-or-later).
