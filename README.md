SO2 - 2021 - UNC

## Introduccion
Toda aplicación de ingenierı́a que posea requerimientos rigurosos de tiempo, y
que esté controlado por un sistema de computación, utiliza un Sistema Operativo
de Tiempo Real (RTOS, por sus siglas en inglés). Una de las caracterı́sticas
principales de este tipo de SO, es su capacidad de poseer un kernel preemtive y
un scheduler altamente configurable. Numerosas aplicaciones utilizan este tipo
de sistemas tales como aviónica, radares, satélites, etc. lo que genera un gran
interés del mercado por ingenieros especializados en esta área.

## Ejecucion
Dentro de /src correr:

```
make

qemu-system-arm -serial stdio -machine lm3s811evb -kernel gcc/RTOSDemo.axf
```

## Tareas

1. Una tarea que simule un sensor de temperatura. Generando valores aleato-
rios, con una frecuencia de 100 Hz.
```C
static void vTempSensorTask(void *p);
```
2. Una tarea que reciba los valores del sensor y aplique un filtro pasa bajos.
Donde cada valor resultante es el promedio de las ultimas N mediciones.

3. Una tarea que grafica en el display los valores de temperatura en el tiempo.
   
4. Se debe poder recibir comandos por la interfaz UART para cambiar el N
del filtro.

5. Calcular el stack necesario para cada task. Realizar el análisis utilizando
uxTaskGetStackHighWaterMark o vApplicationStackOverflowHook.

6. Implementar una tarea tipo top de linux, que muestre periódicamente
estadı́sticas de las tareas (uso de cpu, uso de memoria, etc).