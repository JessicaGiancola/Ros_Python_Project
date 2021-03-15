# <b>Ros Python Project</b>

[![Python 3.8](https://img.shields.io/badge/Python-3.8-blue.svg)](https://www.python.org/downloads/release/python-380/)
[![ROS](https://img.shields.io/badge/Ros-noetic-green.svg)](http://wiki.ros.org/noetic)
[![Raspberry Pi](https://img.shields.io/badge/RaspberryPi-3B+-violet.svg)](https://www.raspberrypi.org/software/)
[![Freenove Smart car kit](https://img.shields.io/badge/Freenove-smartcarkit-yellow.svg)](https://www.freenove.com/)

# Obiettivo
Il progetto prevede l'installazione di ROS su Raspberry Pi 3.<br>
L'obiettivo è quello di guidare il robot attraverso un percorso e renderlo in grado di aggirare gli ostacoli.<br>
In particolare abbiamo utilizzato un Raspberry Pi 3B+ con architettura armhf, montato sul robot Freenove 4WD smar car kit.<br>
Lo potete acquistare a questo link:<br>
https://www.amazon.it/Freenove-Raspberry-Tracking-Avoidance-Ultrasonic/dp/B07YD2LT9D?ref_=ast_sto_dp

# Installazione 
Per iniziare, occorre avere una micro SD card da almeno 8GB sulla quale andrete a installare il sistema operativo per il vostro Raspberry.<br>
Potrete installarlo utilizzando il Raspberry Pi Imager scaricabile a questo link:<br>
https://www.raspberrypi.org/software/<br>
Noi abbiamo installato il sistema operativo Ubuntu 20.04 LTS Focal 32-bit per architetture armhf. <br>
<br>
Una volta installato il sistema operativo, andremo ad installare ROS seguendo questa guida <br>
http://wiki.ros.org/noetic/Installation<br>
<br>
Successivamente dobbiamo configurare il robot, nel nostro caso lo facciamo seguendo il tutorial fornito dal produttore del robot:<br>
https://github.com/Freenove/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi<br>
<br>
Ora potete clonare questa repository nella directory src del vostro catkin workspace.<br>
Da terminale posizionatevi nella directory catkin_ws e lanciate il comando: <br>
```markdown
catkin_make
```
# Lanciare il programma

Per lanciare il nostro programma aprite un terminale ed eseguite il comando:
```markdown
roscore
```
per lanciare il cuore di ROS.<br>
<b>In un altro terminale</b> eseguire i seguenti comandi:
```markdown
sudo -i
cd /path/to/catkin/workspace
source ./devel/setup.bash
roslaunch myros project.launch
```
A questo punto il nostro programma è partito.<br>
<br>
Se volete visualizzare i log, potrete farlo aprendo in un altro terminale la console di ROS, attraverso il comando:
```markdown
rosrun rqt_console rqt_console
```

# <em>Importante!</em>
Dopo aver aggiunto file al progetto, è necessario modificare opportunamente i file CMakeLists.txt e package.xml nella cartella myros. Per sapere come, consulta la guida di ROS:<br>
http://wiki.ros.org/ROS/Tutorials

Successivamente è necessario recarsi nella directory del catkin workspace da terminale e lanciare il comando:
```markdown
catkin_make
```

# Conclusioni
A causa di un malfunzionamento del robot durante lo sviluppo di questo progetto, non abbiamo potuto testare a pieno il nostro codice, quindi <b>NON GARANTIAMO</b> un corretto funzionamento del programma.


<!---
Code
```markdown
pip3 install pyswip
```
--->

<!--- immagine --->
<!--- ![alt text](https://github.com/LorisNanni91/ProgettoRobotica/blob/master/python.gif?raw=true) --->
<!--- ![alt text](https://github.com/LorisNanni91/ProgettoRobotica/blob/master/unity.gif?raw=true) --->
