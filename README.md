# Tutorial: Simulación del UR5 con MoveIt y Gazebo en ROS

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

✅ PASO 0: Plugin MIMIC

Se debe instalar el plugin de mimic. (Si ya lo tienes instalado no hace falta deguir estos pasos)
Para ver si lo tienes instalado: 
    
    find /usr/ -name "libroboticsgroup_gazebo_mimic_joint_plugin.so"

Para clonar el repositorio, instalalo dentro de tu ROS workspace: 

    cd ~/catkin_ws/src
		git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git
		cd roboticsgroup_gazebo_plugins
		mkdir build && cd build
		cmake ..
		make
		sudo make install
		
Para verificar que se instalo correctamente se ejecuta:
    
    find /usr/ -name "libroboticsgroup_gazebo_mimic_joint_plugin.so"

A continuación se detallan los pasos necesarios para instalar y preparar el entorno de trabajo del robot UR5 con gripper Robotiq 85 en ROS.

✅ PASO 1: Crear el espacio de trabajo
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
✅ PASO 2: Crear tu package
```
cd src
catkin_create_pkg ur5_v1 controller_manager joint_state_controller robot_state_publisher roscpp rospy std_msgs urdf
```

✅ PASO 3: Clonar los paquetes necesarios
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

Para probar que aparezca el UR5, use tres terminales diferentes y ejecute los siguientes comandos:

Prepara el entorno en Gazebo. Lanza el modelo del UR5 en Gazebo:
```
roslaunch ur_gazebo ur5_bringup.launch
```

Configura MoveIt para planificación. Lanza MoveIt con el UR5:
```
roslaunch ur5_moveit_config moveit_planning_execution.launch sim:=true
```

Lanza Rviz para visualizar: (En fixed frame seleccione "base_linK", en la parte inferior de rviz dar click en "add" y seleccionar "RobotModel"
```
roslaunch ur5_moveit_config moveit_rviz.launch config:=true
```

✅ PASO 4: Instalar dependencias del workspace
```
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

✅ PASO 5: Compilar el workspace
```
catkin_make
```
✅ PASO 6: Configurar el entorno en el terminal
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
```
bash
sudo apt update
sudo apt install curl gnupg lsb-release
curl -sSL http://packages.ros.org/ros2/ubuntu/gpg.key | sudo tee /etc/apt/trusted.gpg.d/ros.asc
echo "deb [arch=amd64] http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros-latest.list
sudo apt update
sudo apt install ros-noetic-desktop-full
```

### Paso 2: Inicializar ROS y configurar el entorno
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc  # Para ROS Noetic
source ~/.bashrc
```
### Paso 3: Instalar dependencias para Gazebo y control de robots
```
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
sudo apt install ros-noetic-industrial-core ros-noetic-ur5-ros-control
```
### Paso 4: Instalar los paquetes del robot UR5
```
sudo apt install ros-noetic-ur5-moveit-config
sudo apt install ros-noetic-ur5-gazebo
```
### Paso 5: Crear un workspace de ROS
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```
### Paso 6: Clonar el repositorio de UR5 (opcional)
```
cd ~/catkin_ws/src
git clone https://github.com/ros-industrial/ur5.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
### Paso 7: Compilar el workspace
```
cd ~/catkin_ws
catkin_make
```
### Paso 8: Configurar y lanzar Gazebo con el UR5
```
roslaunch ur5_gazebo ur5_world.launch
```
### Paso 9: Configurar MoveIt! para el control del robot
```
roslaunch ur5_moveit_config demo.launch
```
### Paso 10: Código para el "Pick and Place"
```
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from moveit_commander.robot_trajectory import RobotTrajectory
```
# Inicializar el nodo ROS y MoveIt!
```
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('pick_and_place_node')
```
# Inicializar el brazo UR5
```
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")
```
# Mover el robot a una posición inicial
```
group.set_named_target("home")
group.go(wait=True)
```
# Definir una nueva posición de objetivo
```
pose_target = Pose()
pose_target.position.x = 0.5
pose_target.position.y = -0.5
pose_target.position.z = 0.5
group.set_pose_target(pose_target)
```
# Planificar y ejecutar el movimiento
```
plan = group.plan()
group.go(wait=True)
```
# Detener el robot
```
moveit_commander.roscpp_shutdown()
```
### Paso 11: Verificación y pruebas
```
roslaunch ur5_moveit_config demo.launch
```

## 🏗️ Instrucciones

## Paso 1: Instalar el plugin de mimic. (Si ya lo tienes instalado no hace falta deguir estos pasos)
-Para ver si lo tienes instalado: find /usr/ -name "libroboticsgroup_gazebo_mimic_joint_plugin.so"
-Para clonar el repositorio, tienes 2 opciones:
	• Instalar globalmente en el sistema (recomendado si tienes permisos sudo) o si usaras en diferentes archivos:cd ~
	• Instalarlo dentro de tu ROS workspace: cd ~/ros_ws/src
```
git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git
    cd roboticsgroup_gazebo_plugins
    mkdir build && cd build
    cmake ..
    make
    sudo make install
```
-Para verificar que se instalo correctamente se ejecuta:
```
    find /usr/ -name "libroboticsgroup_gazebo_mimic_joint_plugin.so"
```
-A veces, Gazebo necesita ayuda para encontrar plugins en carpetas no estándar. 
		Para ello ejecutamos estas lineas en la terminal:
```
echo 'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/local/lib' >> ~/.bashrc
			source ~/.bashrc
```
## Paso 2: Crear tu catkin_ws_6
```
mkdir -p ~/catkin_ws_6/src
	cd catkin_ws_6
	catkin_make
	source devel/setup.bash
	echo "source ~/catkin_ws_6/devel/setup.bash" >> ~/.bashrc
```
## Paso 3: Crear tu package
```
cd src
	catkin_create_pkg ur5_v1 controller_manager joint_state_controller robot_state_publisher roscpp rospy std_msgs urdf
		 Comentario -->   	catkin_create_pkg  <name_of_package> <dependencies of package>
	cd ..
	catkin_make
```
## Paso 4: Clonar el repositorio de UR para cargar el UR5
```
udo apt-get install ros-noetic-universal-robots
	cd src
	git clone -b noetic-devel https://github.com/ros-industrial/universal_robot.git
	cd ..
	rosdep update
	rosdep install --rosdistro noetic --ignore-src --from-paths src
	catkin_make
```



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
* 🔗 GitHub: https://github.com/RosasJP17
* 📧 Correo electrónico: juan.rosaspa@udlap.mx
  
  
* Nombre: Cesar Maximiliano Gutierrez Velazquez
* 📧 Correo electrónico: cesar.gutierrezvz@udlap.mx

  
* Nombre: Antonio De Jesus Xicali Arriaga
* 🔗 GitHub: https://github.com/AntonioXicali101
* 📧 Correo electrónico: antonio.xicaliaa@udlap.mx

   
* Nombre: Yered Yosshiel Bojorquez Castillo
* 🔗 GitHub: https://github.com/YeredBC
* 📧 Correo electrónico: yered.bojorquezco@udlap.mx

---
