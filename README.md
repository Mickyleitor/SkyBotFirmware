# Practica Final : Microbótica

Se pide realizar la programación y comportamiento de un robot basado en el microprocesador TM4C123GPH6M. 

El robot incorpora dos sensores distancia, uno de larga distancia (hasta 30 cm) y otro de corta distancia (hasta 15 cm) y varios sensores tipo CNY90 para encoders para contar las vueltas de la rueda, sensores TCRT1000 para detectar los extremos del tatami y varios interruptores tipo whisker para contacto directo con objetos externos. El microbot deberá ser capaz de navegar por el tatami de forma reactiva, sin salirse del mismo, detectando y reaccionando ante posibles obstáculos (otros robots o bloques fijos colocados para pruebas). 

El microbot deberá incorporar TAMBIÉN alguna estrategia de alto nivel.

## Guías
#### [Practica 1 : Control de ServoMotores del Skybot](https://github.com/Mickyleitor/SkyBot/blob/master/Docs/Pr%C3%A1cticaServos2017v5.pdf)
#### [Practica 2 : Caracterización y uso de sensores en el Skybot (Parte 1)](https://github.com/Mickyleitor/SkyBot/blob/master/Docs/Pr%C3%A1cticaSensores2017v4.pdf)
#### [Practica 2 : Caracterización y uso de sensores en el Skybot (Parte 2)](https://github.com/Mickyleitor/SkyBot/blob/master/Docs/Pr%C3%A1cticaSensores2017Parte2v4.pdf)
#### [Practica final: “MINI-SUMO”](https://github.com/Mickyleitor/SkyBot/blob/master/Docs/Especificaciones%20de%20la%20practica%20final%20-%20mini-Sumo.pdf)

## Brainstorming para generación de [informe final](https://github.com/Mickyleitor/SkyBot/blob/master/Docs/Informe.pdf)

![](https://github.com/Mickyleitor/SkyBot/blob/master/Docs/Skybot_PostIt.png)

## Implementación con maquina de estados

![](https://github.com/Mickyleitor/SkyBot/blob/master/Docs/Skybot_FSM.png)