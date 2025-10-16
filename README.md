Proyecto: Vehículo Seguidor de Línea con STM32
Este repositorio contiene el código fuente y el modelo de control para un vehículo autónomo seguidor de línea, desarrollado como mini-proyecto para la materia de Sistemas Embebidos de la Facultad de Ingeniería, UNMdP. 

El objetivo del proyecto es que el vehículo complete un circuito cerrado de línea negra sobre fondo blanco en el menor tiempo posible, utilizando un sistema de control basado en una máquina de estados de Harel. 

Hardware Requerido
Para replicar este proyecto, necesitarás los siguientes componentes: 
Placa de control: STM32F407G-DISC1 
Driver de Motores: L298N 
Sensores: 2 x Sensores Infrarrojos (IR) digitales 
Actuadores: 2 x Motores DC con ruedas 
Alimentación: Batería externa (compatible con el driver L298N y los motores) 
Chasis: Estructura para montar todos los componentes.
Cableado: Cables para realizar las conexiones.

Software y Herramientas
IDE: STM32CubeIDE
Modelado: Itemis CREATE (Anteriormente Yakindu Statechart Tools)
Librerías: STM32 HAL (gestionadas a través de CubeIDE).

Claro, aquí tienes un borrador para el archivo README.md del proyecto.

Proyecto: Vehículo Seguidor de Línea con STM32
Este repositorio contiene el código fuente y el modelo de control para un vehículo autónomo seguidor de línea, desarrollado como mini-proyecto para la materia de Sistemas Embebidos de la Facultad de Ingeniería, UNMdP. 

El objetivo del proyecto es que el vehículo complete un circuito cerrado de línea negra sobre fondo blanco en el menor tiempo posible, utilizando un sistema de control basado en una máquina de estados de Harel. 


Hardware Requerido
Para replicar este proyecto, necesitarás los siguientes componentes: 


Placa de control: STM32F407G-DISC1 


Driver de Motores: L298N 


Sensores: 2 x Sensores Infrarrojos (IR) digitales 


Actuadores: 2 x Motores DC con ruedas 


Alimentación: Batería externa (compatible con el driver L298N y los motores) 

Chasis: Estructura para montar todos los componentes.

Cableado: Cables para realizar las conexiones.

Software y Herramientas
IDE: STM32CubeIDE

Modelado: Itemis CREATE (Anteriormente Yakindu Statechart Tools)

Librerías: STM32 HAL (gestionadas a través de CubeIDE).

Instalación y Configuración
1. Configuración del Proyecto en STM32CubeIDE
Clonar el Repositorio:

Bash

git clone [URL-del-repositorio]
Importar Proyecto: Abre STM32CubeIDE y selecciona File > Import.... Elige General > Existing Projects into Workspace y selecciona la carpeta del repositorio clonado.

Generar Código: Una vez importado, haz clic derecho sobre el archivo .ioc y selecciona Generate Code. Esto asegurará que todas las configuraciones de periféricos (GPIO, TIM4 para PWM) estén correctamente inicializadas.

2. Modelo de Máquina de Estados
El modelo de control (.ysc) se encuentra en la carpeta /prueba/. Este modelo fue creado con Itemis CREATE. El código C correspondiente a la máquina de estados ya está generado e incluido en el proyecto.

Si deseas realizar modificaciones en el modelo de control:

Abre el archivo del modelo en Itemis CREATE.

Realiza los cambios en los estados o transiciones.

Vuelve a generar el código C desde Itemis CREATE (Click derecho > Generate Code).

Reemplaza los archivos generados en las carpetas /Core/Inc y /Core/Src del proyecto en STM32CubeIDE.

Compilación y Carga
Conectar la Placa: Conecta tu placa STM32F407G-DISC1 al computador mediante el cable USB.

Compilar el Proyecto: En STM32CubeIDE, haz clic en el ícono del martillo (Build) o presiona Ctrl+B. El proyecto debería compilar sin errores.

Cargar el Firmware: Haz clic en el ícono de reproducción verde (Run) o presiona Ctrl+F11. Esto cargará el programa en la memoria flash del microcontrolador.

Instrucciones de Uso
Verificar Conexiones: Asegúrate de que todos los componentes de hardware (sensores, motores, driver L298N) estén conectados a los pines GPIO y de alimentación correctos según la configuración del archivo .ioc.

Encender el Vehículo: Coloca el vehículo sobre la pista de línea negra. Asegúrate de que ambos sensores estén inicialmente sobre el fondo blanco.

Iniciar la Marcha: Alimenta el sistema con la batería externa. El programa main.c está diseñado para iniciar la lógica de control inmediatamente. El vehículo comenzará a moverse y a seguir la línea.

Depuración Visual: La placa STM32F407G-DISC1 tiene LEDs de usuario. El código está configurado para que los LEDs parpadeen según las lecturas de los sensores, lo cual es útil para verificar su funcionamiento:

LED Azul (LD6): Parpadea cuando el sensor izquierdo detecta la línea negra.

LED Naranja (LD3): Parpadea cuando el sensor derecho detecta la línea negra.

LED Rojo (LD5): Parpadea cuando el sensor izquierdo detecta el fondo blanco.

LED Verde (LD4): Parpadea cuando el sensor derecho detecta el fondo blanco.
