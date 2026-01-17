# Optimización de la Trayectoria de un Lanzamiento
**Autores:** Pablo Rodríguez Soria y José Carlos Riego

Este proyecto presenta una simulación computacional diseñada para modelar y analizar la trayectoria de un proyectil en tres dimensiones. El objetivo fundamental es estudiar el comportamiento del objeto bajo la influencia de diversas fuerzas físicas —como la gravedad, la resistencia aerodinámica y el viento— para predecir con exactitud su punto de impacto y optimizar los parámetros de lanzamiento.

## Descripción del Modelo

El núcleo del simulador resuelve las ecuaciones de movimiento de Newton utilizando el método numérico de Runge-Kutta de cuarto orden (RK4). Este algoritmo permite integrar las ecuaciones diferenciales ordinarias con alta precisión, determinando la posición y velocidad del proyectil en cada instante de la simulación.

El modelo considera una dinámica tridimensional completa e incorpora fuerzas realistas, incluyendo una resistencia del aire que depende de la velocidad, el área y el coeficiente aerodinámico del objeto, así como un campo de viento vectorial variable. Para determinar el punto de impacto exacto en el plano objetivo (definido en x=0), el sistema emplea técnicas de interpolación lineal, asegurando que la detección del cruce sea precisa y no dependa únicamente de la discretización temporal.

## Estructura del Código

El código fuente, ubicado en el directorio `src/`, está organizado en módulos funcionales escritos en MATLAB:

- **`rungeKutta.m`**: Constituye el motor físico de la simulación. Implementa el algoritmo de integración para resolver las ecuaciones diferenciales de segundo orden que rigen el movimiento.
- **`ImpactPoint.m`**: Es la función principal de alto nivel. Coordina la simulación de la trayectoria completa basándose en los parámetros iniciales y calcula las coordenadas finales del impacto.
- **`force.m`**: Define el modelo físico del sistema, calculando la suma vectorial de todas las fuerzas que actúan sobre el proyectil en función de su estado cinemático y las condiciones ambientales.
- **`Vwind.m`**: Modela el comportamiento del viento, definiendo su magnitud y dirección en el espacio.
- **`bisectionRoot.m`**: Proporciona herramientas auxiliares para el cálculo de raíces, útiles para resolver ecuaciones trascendentes o encontrar condiciones de contorno específicas.

## Instrucciones de Uso

Para realizar una simulación, se debe invocar la función `ImpactPoint`. Esta función toma como entrada las condiciones iniciales del disparo y las propiedades del proyectil, devolviendo las coordenadas del punto de impacto en el plano YZ y el tiempo de vuelo.

La firma de la función es:
```matlab
[y, z, t01] = ImpactPoint(alpha, beta, m, Cd, A, x0, Vtotal, tmax, dt)
```

Los parámetros requeridos son los ángulos de lanzamiento (elevación `alpha` y azimut `beta` en radianes), la masa `m` y el área transversal `A` del proyectil, el coeficiente de arrastre `Cd`, la posición inicial en el eje X `x0` y la velocidad inicial `Vtotal`. También se deben especificar los parámetros de control de la simulación: el tiempo máximo `tmax` y el paso de integración `dt`.

## Requisitos y Referencias

El código ha sido desarrollado para funcionar en MATLAB, sin necesidad de toolboxes adicionales. El integrador numérico implementado se basa en los algoritmos descritos por W.H. Press et al. en *Numerical Recipes* (Cambridge University Press), adaptados para este propósito por J.M. Soler.
