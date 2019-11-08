# SkyBot Firmware

Se pide realizar la programaci�n y comportamiento de un robot basado en el microprocesador TM4C123GPH6M. 

El robot incorpora dos sensores distancia, uno de larga distancia (hasta 30 cm) y otro de corta distancia (hasta 15 cm) y varios sensores tipo CNY90 para encoders para contar las vueltas de la rueda, sensores TCRT1000 para detectar los extremos del tatami y varios interruptores tipo whisker para contacto directo con objetos externos. El microbot deber� ser capaz de navegar por el tatami de forma reactiva, sin salirse del mismo, detectando y reaccionando ante posibles obst�culos (otros robots o bloques fijos colocados para pruebas). 

El microbot deber� incorporar TAMBI�N alguna estrategia de alto nivel.

El desarrollo de la programaci�n de este Firmware ha sido realizado para la asignatura **Microb�tica**, en la Escuela de T�cnica Superior de Telecomunicaci�n de la **Universidad de M�laga** (https://www.uma.es/etsi-de-telecomunicacion/).

## Paso a Paso
Se muestran las siguientes gu�as proporcionadas para la realizaci�n del proyecto:
- [Control de ServoMotores del Skybot](https://github.com/Mickyleitor/SkyBot/blob/master/Docs/Pr%C3%A1cticaServos2017v5.pdf)
- [Caracterizaci�n y uso de sensores en el Skybot (Parte 1)](https://github.com/Mickyleitor/SkyBot/blob/master/Docs/Pr%C3%A1cticaSensores2017v4.pdf)

<p align="center">
  <url=https://youtu.be/SMUNZbP2yN4><img src="https://img.youtube.com/vi/SMUNZbP2yN4/hqdefault.jpg" width=350/></url>
</p>

- [Caracterizaci�n y uso de sensores en el Skybot (Parte 2)](https://github.com/Mickyleitor/SkyBot/blob/master/Docs/Pr%C3%A1cticaSensores2017Parte2v4.pdf)

<p align="center">
  <url=https://youtu.be/Gbu0nRUPFR4><img src="https://img.youtube.com/vi/Gbu0nRUPFR4/hqdefault.jpg" width=350/></url>
</p>

- [Adaptaci�n a �MINI-SUMO�](https://github.com/Mickyleitor/SkyBot/blob/master/Docs/Especificaciones%20de%20la%20practica%20final%20-%20mini-Sumo.pdf)

<p align="center">
  <url=https://youtu.be/mdWAT61AjUI><img src="https://img.youtube.com/vi/mdWAT61AjUI/hqdefault.jpg" width=350/></url>
</p>

- [Generaci�n de Informe final](https://github.com/Mickyleitor/SkyBot/blob/master/Docs/Informe.pdf)

<p align="center">
  <url=https://github.com/Mickyleitor/SkyBot/blob/master/Docs/Skybot_PostIt.png><img src="https://github.com/Mickyleitor/SkyBot/blob/master/Docs/Skybot_PostIt.png" width=640/></url>
</p><p align="center">Brainstorming para generaci�n del informe final</p>

## Implementaci�n con maquina de estados

![](https://github.com/Mickyleitor/SkyBot/blob/master/Docs/Skybot_FSM.png)
