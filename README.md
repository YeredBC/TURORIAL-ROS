# {Tutorial: Simulación del UR5 con MoveIt y Gazebo en ROS}

Este tutorial explica paso a paso cómo instalar y configurar el entorno necesario para simular y controlar el robot UR5 con un gripper Robotiq 85 en ROS. Al finalizar, el lector podrá lanzar correctamente el modelo del UR5 con su gripper en Gazebo, visualizarlo en RViz mediante MoveIt y comprobar el funcionamiento de los controladores. Todo está preparado para luego usar el sistema en tareas más complejas como pick and place o control avanzado.

---

## 📋 Requisitos Previos

- Uso básico de terminal en Linux.

- Conocimientos elementales de ROS (nodos, launch files, catkin).

- Familiaridad básica con URDF y MoveIt.

## 🛠️ Herramientas y software requeridos:
- Ubuntu 20.04

- ROS Noetic

- Gazebo (incluido con ROS Noetic)

- MoveIt

- Python 3

---

## 📖  Introducción

Este tutorial tiene como objetivo documentar el proceso completo para simular y configurar el robot UR5 con un gripper Robotiq 85 en ROS, utilizando herramientas como Gazebo, RViz y MoveIt. A lo largo de este instructivo, se explicará cómo preparar el entorno de trabajo, cargar los modelos del robot, la mesa y los objetos del entorno, y cómo ajustar los controladores y archivos necesarios para que todo funcione de forma integrada.

El enfoque principal es lograr que el robot UR5 y su gripper funcionen correctamente dentro del simulador, con la capacidad de ser controlados desde MoveIt. Este entorno sirve como base para desarrollos más avanzados, como tareas de manipulación, pruebas de algoritmos o integración con visión artificial.



---

## 💾 Instalación

A continuación se detallan los pasos necesarios para instalar y preparar el entorno de trabajo del robot UR5 con gripper Robotiq 85 en ROS.

✅ PASO 1: Crear el espacio de trabajo
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
```

✅ PASO 2: Clonar los paquetes necesarios
```
cd ~/catkin_ws/src
```
# Paquetes base del robot y gripper

git clone https://github.com/ros-industrial/universal_robot.git

git clone https://github.com/ros-industrial/robotiq.git

# Paquetes del proyecto
git clone https://github.com/YeredBC/TURORIAL-ROS.git

📌 NOTA:
Este repositorio contiene los paquetes personalizados utilizados en este proyecto,
como ur5_v1 y ur_gripper_moveit_config. Asegúrate de que, al clonarlo,
queden dentro de la carpeta src de tu workspace (catkin_ws/src).

✅ PASO 3: Instalar dependencias del workspace
```
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

✅ PASO 4: Compilar el workspace
```
catkin_make
```
✅ PASO 5: Configurar el entorno en el terminal
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```



## 🛠️ Configuración del Entorno

Pasos para configurar el entorno de desarrollo:

# Proyecto UR5 Pick and Place en Gazebo

Este proyecto tiene como objetivo configurar el entorno de ROS para simular el robot UR5 en Gazebo y realizar una tarea de "Pick and Place". Se utilizan paquetes de ROS industriales y MoveIt! para el control del robot.

## Requisitos

- **Sistema Operativo**: Ubuntu 20.04 (para ROS Noetic).
- **ROS**: Instalación de ROS Noetic.
- **Gazebo**: Para simulaciones en 3D.
- **UR5 Simulation**: Paquetes de ROS para el UR5 y MoveIt!.

## Paso 1: Instalar ROS

### En Ubuntu 20.04 (para ROS Noetic)
bash
sudo apt update
sudo apt install curl gnupg lsb-release
curl -sSL http://packages.ros.org/ros2/ubuntu/gpg.key | sudo tee /etc/apt/trusted.gpg.d/ros.asc
echo "deb [arch=amd64] http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros-latest.list
sudo apt update
sudo apt install ros-noetic-desktop-full

### Paso 2: Inicializar ROS y configurar el entorno
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc  # Para ROS Noetic
source ~/.bashrc

### Paso 3: Instalar dependencias para Gazebo y control de robots

### Paso 4: Instalar los paquetes del robot UR5

### Paso 5: Crear un workspace de ROS

### Paso 6: Clonar el repositorio de UR5 (opcional)

### Paso 7: Compilar el workspace

### Paso 8: Configurar y lanzar Gazebo con el UR5

### Paso 9: Configurar MoveIt! para el control del robot

### Paso 10: Código para el "Pick and Place"

### Paso 11: Verificación y pruebas


## 🏗️ Instrucciones
**Paso 1:** Descripción del primer paso

Instrucciones detalladas y código de ejemplo:


**Paso 2:** Descripción del segundo paso

Más instrucciones y ejemplos según sea necesario.

---
## ✅ Conclusión

Este tutorial documenta todo el proceso necesario para simular el robot UR5 con un gripper Robotiq 85 en ROS,
incluyendo la instalación de dependencias, la clonación de paquetes, la configuración de MoveIt y el lanzamiento
correcto en Gazebo y RViz.

Gracias a esta guía, es posible tener un entorno funcional para pruebas con el UR5 en ROS, dejando todo listo para
futuros desarrollos como scripts de movimiento, integración con visión, o secuencias de pick and place.

Además, se explicaron los archivos clave que deben modificarse, como los controladores, URDFs, y launch files, 
lo que ayuda a entender cómo se estructura un sistema completo de simulación con ROS.


---

## 📚 Referencias y Recursos Adicionales

Enlace a documentación oficial:  
https://wiki.ros.org/noetic

Tutoriales relacionados:  
https://ros-planning.github.io/moveit_tutorials/  
http://wiki.ros.org/Industrial/Tutorials

Repositorio de código fuente:  
https://github.com/YeredBC/TURORIAL-ROS.git

---

## 📬 Contacto

Para preguntas o sugerencias:
* Nombre: Juan Pablo Rosas Pineda:
* 📧 Correo electrónico: juan.rosaspa@udlap.mx
* Nombre:Cesar Maximiliano Gutierrez Velazquez
* 📧 Correo electrónico: cesar.gutierrezvz@udlap.mx
* Nombre:Antonio De Jesus Xicali Arriaga
* 📧 Correo electrónico: antonio.xicaliaa@udlap.mx
* Nombre:Yered Yosshiel Bojorquez Castillo
* 📧 Correo electrónico: yered.bojorquezco@udlap.mx

---
