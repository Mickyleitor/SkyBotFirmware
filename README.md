# SkyBot Firmware

Se pide realizar la programación y comportamiento de un robot basado en el microprocesador TM4C123GPH6M. 

El robot incorpora dos sensores distancia, uno de larga distancia (hasta 30 cm) y otro de corta distancia (hasta 15 cm) y varios sensores tipo CNY90 para encoders para contar las vueltas de la rueda, sensores TCRT1000 para detectar los extremos del tatami y varios interruptores tipo whisker para contacto directo con objetos externos. El microbot deberá ser capaz de navegar por el tatami de forma reactiva, sin salirse del mismo, detectando y reaccionando ante posibles obstáculos (otros robots o bloques fijos colocados para pruebas). 

El microbot deberá incorporar TAMBIÉN alguna estrategia de alto nivel.

El desarrollo de la programación de este Firmware ha sido realizado para la asignatura **Microbótica**, en la Escuela de Técnica Superior de Telecomunicación de la **Universidad de Málaga** (https://www.uma.es/etsi-de-telecomunicacion/).

## Paso a Paso
Se muestran las siguientes guías proporcionadas para la realización del proyecto:
- [Control de ServoMotores del Skybot](https://github.com/Mickyleitor/SkyBot/blob/master/Docs/Pr%C3%A1cticaServos2017v5.pdf)
- [Caracterización y uso de sensores en el Skybot (Parte 1)](https://github.com/Mickyleitor/SkyBot/blob/master/Docs/Pr%C3%A1cticaSensores2017v4.pdf)

<p align="center">
  <url=https://youtu.be/SMUNZbP2yN4><img src="https://img.youtube.com/vi/SMUNZbP2yN4/hqdefault.jpg" width=350/></url>
</p>

- [Caracterización y uso de sensores en el Skybot (Parte 2)](https://github.com/Mickyleitor/SkyBot/blob/master/Docs/Pr%C3%A1cticaSensores2017Parte2v4.pdf)

<p align="center">
  <url=https://youtu.be/Gbu0nRUPFR4><img src="https://img.youtube.com/vi/Gbu0nRUPFR4/hqdefault.jpg" width=350/></url>
</p>

- [Adaptación a “MINI-SUMO”](https://github.com/Mickyleitor/SkyBot/blob/master/Docs/Especificaciones%20de%20la%20practica%20final%20-%20mini-Sumo.pdf)

<p align="center">
  <url=https://youtu.be/mdWAT61AjUI><img src="https://img.youtube.com/vi/mdWAT61AjUI/hqdefault.jpg" width=350/></url>
</p>

- [Generación de Informe final](https://github.com/Mickyleitor/SkyBot/blob/master/Docs/Informe.pdf)

<p align="center">
  <url=https://github.com/Mickyleitor/SkyBot/blob/master/Docs/Skybot_PostIt.png><img src="https://github.com/Mickyleitor/SkyBot/blob/master/Docs/Skybot_PostIt.png" width=640/></url>
</p><p align="center">Brainstorming para generación del informe final</p>

## Implementación con maquina de estados

![](https://github.com/Mickyleitor/SkyBot/blob/master/Docs/Skybot_FSM.png)
