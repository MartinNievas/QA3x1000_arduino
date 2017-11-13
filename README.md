# QA3x1000_arduino
Software para controlar el QA3x1000 (quadrorrotor de 1m de largo). 

Se utiliza una arduino [promicro](http://arduino.cl/pro-micro/) y el sensor [MPU6050](https://playground.arduino.cc/Main/MPU-6050)

## Dependencias
* Las bibliotecas están en la carpeta libs
* Para compilar se utiliza [Platformio](http://platformio.org/)

## Como compilar
Dentro de la carpeta del repositorio: 
```c 
platformio run
```

## Como cargar a la placa
Dentro de la carpeta del repositorio: 
```c 
platformio run -t upload
```
