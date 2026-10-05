# an5_mock_sim

Modo de simulacion (mock) del robot AN5/FR5, para desarrollar/probar contra
Unity + RosSharp sin el brazo real conectado. Paquete **100% aditivo**: no
modifica `fr_ros2`, `code` ni `frhal_msgs`.

## Quick start: correr el robot simulado

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch an5_mock_sim sim.launch.py
```

Con eso quedan arriba los 4 nodos esperados (`mock_cmd_server`,
`robot_publisher`, `rosapi`, `rosbridge_websocket` en el puerto **9090**
para Unity). No hace falta ningun argumento extra: dejalo con los
defaults (en particular `xmlrpc_mock_enabled:=false`, que es el default y
evita el problema de fuga de sockets documentado mas abajo) ni el alias de
IP -- Unity ya deberia poder enviar comandos (`JNTPoint`/`MoveJ`/`SplinePTP`)
y ver el movimiento reflejado.

Si hay que recompilar el paquete despues de tocar
`mock_cmd_server.py` (o para aplicar una actualizacion):

```bash
cd ~/ros2_ws
colcon build --packages-select an5_mock_sim
source install/setup.bash
ros2 launch an5_mock_sim sim.launch.py
```

Para cortar la simulacion, `Ctrl+C` en la terminal del `ros2 launch` alcanza
(cierra los 4 nodos juntos, ya que estan agrupados en el mismo launch).

## Arquitectura (resumen)

En el stack real, `fr_ros2/ros2_cmd_server` corre dos nodos:
- `FR_ROS_API_node`: expone el servicio `/FR_ROS_API_service`
  (`frhal_msgs/srv/ROSCmdInterface`) y habla por TCP crudo con el
  controlador real en `192.168.58.2:8080/8082`.
- `FR_recv_data_thread`: publica `nonrt_state_data`
  (`frhal_msgs/msg/FRState`) cada 100 ms, leyendo del controlador por TCP
  en el puerto `8083`.

`code/publisher_subscriber` (nodo `robot_publisher`) es el puente: se
suscribe a `/api_command` (`std_msgs/String`) y reenvia cada mensaje como
`cmd_str` al servicio `/FR_ROS_API_service`. Ademas, **por su cuenta**,
consulta al robot por XML-RPC (SDK oficial Fairino, puerto `20003`) cada
20 ms y publica `current_joint_position`/`current_cartesian_position`
(`std_msgs/String`, CSV).

**Unity (RosSharp) no lee `/joint_states`: lee `current_joint_position` /
`current_cartesian_position`.** Ver `Assets/JointPositionSubscriber.cs` y
`Assets/CartesianPositionSubscriber.cs` en el proyecto Unity
(`Interfaz-Unity-AN5`). En modo real esos dos topics los llena
`publisher_subscriber` via XML-RPC al controlador; en modo mock esa IP
nunca responde, asi que **`mock_cmd_server` los publica el mismo,
directamente desde su estado interno** (ver seccion siguiente) -- no hace
falta ningun alias de IP ni servidor XML-RPC para que Unity vea el
movimiento simulado.

**Unity tampoco necesita el servicio para enviar comandos: publica
`JNTPoint(...)`/`MoveJ(...)` como texto en `/api_command`** (ver
`Assets/Ros2CommandSender.cs`, metodo `SendCommand`), esperando que
`robot_publisher` los reenvie a `/FR_ROS_API_service`. Por el mismo
bloqueo de `robot_publisher` (XML-RPC a la IP inalcanzable), ese reenvio
no sucede en modo mock. Por eso `mock_cmd_server` **tambien se suscribe
directo a `/api_command`** y procesa esos comandos el mismo, con la misma
gramatica que expone por el servicio (ver seccion siguiente) -- Unity no
necesita ningun cambio, sigue publicando al mismo topico de siempre.

Importante: **hoy no existe ningun publisher de `/joint_states`
(`sensor_msgs/JointState`) en ningun paquete del workspace.** `an5_mock_sim`
lo agrega como capacidad nueva (para RViz/Unity), ademas de replicar
`nonrt_state_data` para paridad exacta con el driver real.

`an5_mock_sim` reemplaza **solo** al primer nodo (`ros2_cmd_server`).
`publisher_subscriber` sigue siendo el mismo, sin tocar, en ambos modos.

## Como alternar entre modo real y modo simulado

```bash
# Modo SIMULADO (sin robot fisico, sin controlador conectado)
ros2 launch an5_mock_sim sim.launch.py

# Modo REAL (requiere el controlador FR5/AN5 accesible en 192.168.58.2)
ros2 launch an5_mock_sim real.launch.py
```

`real.launch.py` no duplica codigo: ejecuta, tal cual, el mismo binario
`fr_ros2/ros2_cmd_server` y el mismo `code/publisher_subscriber` que ya
usabas manualmente en terminales separadas, mas `rosbridge_websocket`.
Antes no existia ningun launch file para esto en el workspace (`fr_ros2` y
`code` no tienen carpeta `launch/`); este archivo solo los agrupa.

### ⚠️ No corras ambos modos al mismo tiempo

`sim.launch.py` y `real.launch.py` **compiten por la misma interfaz ROS2**:

- Ambos intentan ofrecer el servicio `/FR_ROS_API_service`. ROS2 permite
  que existan dos servidores con el mismo nombre, pero las llamadas quedan
  repartidas de forma **ambigua** entre ambos (a veces te contesta el real,
  a veces el mock) -- comportamiento indefinido para vos, no un error que
  se vea claramente en el log.
- Ambos publican en `nonrt_state_data` y `an5_mock_sim` publica en
  `/joint_states` -- vas a tener datos mezclados/parpadeando entre el
  estado real y el simulado.

`mock_cmd_server` intenta detectar esto al arrancar: si ya existe un
servidor de `/FR_ROS_API_service` en el grafo ROS2, o publishers externos
en `/joint_states`/`nonrt_state_data`, imprime una advertencia bien visible
en el log (no bloquea el arranque, solo avisa). Prestale atencion a esa
advertencia si algo se ve raro.

## Unity / RosSharp: no requiere ningun cambio

Unity sigue apuntando al mismo `rosbridge_websocket` en el puerto **9090**
en ambos modos -- `sim.launch.py` y `real.launch.py` levantan exactamente
el mismo `rosbridge_server` con la configuracion por defecto. Los nombres
de topic/servicio y tipos de mensaje son identicos en ambos modos
(`/FR_ROS_API_service`, `nonrt_state_data`, `/joint_states`,
`current_joint_position`, `current_cartesian_position`); lo unico que
cambia es quien los sirve -- en modo real es `publisher_subscriber`, en
modo sim es `mock_cmd_server`.

## Que hace mock_cmd_server

- Sirve `/FR_ROS_API_service` replicando la gramatica real de comandos
  (`fr_ros2/src/ROS_API.cpp::_selectfunc`): `JNTPoint(idx,j1..j6)` /
  `CARTPoint(idx,...)` guardan un punto; `MoveJ(JNT<idx>,speed)` /
  `MoveL(JNT<idx>,speed)` disparan el movimiento hacia ese punto guardado;
  `GET(JNT,idx)`/`GET(CART,idx)` lo consultan; `StopMotion()` cancela el
  movimiento en curso. El resto de los ~45 comandos reales (`SetSpeed`,
  `RobotEnable`, `SetDO`, `ActGripper`, etc.) se aceptan y responden exito
  inmediato, sin efecto -- para no romper el flujo de quien los llama.
- **Simplificaciones deliberadas** (documentadas en el codigo,
  `mock_cmd_server.py`):
  - No hay cinematica INVERSA: comandos `MoveJ(CART<idx>,...)` /
    `MoveL(CART<idx>,...)` se rechazan con un warning claro en vez de
    moverse (no se puede ir de una pose cartesiana deseada a los joints
    que la producen).
  - `MoveJ` y `MoveL` no distinguen forma de trayectoria: ambas son una
    interpolacion suave en espacio articular hacia el punto guardado.
  - `nonrt_state_data.cart_*` SI se calcula (cinematica DIRECTA: joints ->
    pose del TCP), a partir de las transformaciones `<origin>` de
    `j1..j6`/`tool` en el mismo urdf que `JOINT_LIMITS`
    (`_forward_kinematics_mm_deg` en `mock_cmd_server.py`, sin dependencia
    nueva de numpy). `x,y,z` en mm, `rx,ry,rz` en grados (RPY, misma
    convencion `Rz*Ry*Rx` de los `<origin rpy>` del URDF) -- no hay forma
    de confirmar bit-a-bit contra el controlador real sin el robot fisico,
    pero el orden de magnitud (alcance ~845mm en pose cero) es consistente
    con la geometria del FR5.
  - Convencion de `cmd_res`: `"0"` = exito, `"-1"` = fallo (la del `.srv`).
    El servidor real tiene una inconsistencia interna (devuelve
    tambien `"0"` cuando no reconoce el comando); el mock NO replica ese
    detalle a proposito.
- Publica `/joint_states` (`sensor_msgs/JointState`) interpolando entre la
  posicion actual y el setpoint objetivo, a **50 Hz por defecto**
  (parametro `joint_states_rate_hz`; no habia ninguna frecuencia
  documentada para `/joint_states` en el workspace porque el topic no
  existe en el stack real -- 50 Hz es un valor razonable de partida, no un
  valor confirmado). Interpolacion `ease_in_out` (smoothstep) por defecto;
  cambiala a lineal con el parametro `easing:=linear`.
- Publica `nonrt_state_data` (`frhal_msgs/msg/FRState`) cada 100 ms, igual
  que el driver real, con `j1_cur_pos`..`j6_cur_pos` en grados.
- Publica `current_joint_position` y `current_cartesian_position`
  (`std_msgs/String`, CSV, mismo formato que usa `publisher_subscriber.py`
  en modo real) junto con `/joint_states`, a la misma tasa
  (`joint_states_rate_hz`). Son los topics que **realmente** consume Unity
  (ver arriba); `current_cartesian_position` viaja con la pose calculada
  por cinematica directa (mismo valor que `nonrt_state_data.cart_*`, ver
  simplificaciones deliberadas mas arriba), en vez de ceros.
- Se suscribe directo a `api_command` (`std_msgs/String`) y procesa cada
  mensaje con la misma gramatica que el servicio (`_process_command_str`,
  compartida entre ambos caminos). No hay respuesta hacia quien publico
  (es un topico, no un servicio) -- el resultado solo queda en el log
  (`get_logger().info`), igual que en el stack real `robot_publisher`
  tampoco reenvia la respuesta a Unity. Esto es lo que permite que los
  comandos que Unity publica en `/api_command` (`Ros2CommandSender.cs`)
  muevan el robot simulado sin pasar por `robot_publisher`.
  Nota: `robot_publisher` sigue suscripto al mismo topico y en teoria
  podria, si alguna vez su XML-RPC bloqueado se destranca, reenviar hasta
  10 comandos viejos acumulados en su cola (`queue depth` por defecto) al
  servicio -- inofensivo en la practica porque `JNTPoint`/`MoveJ`
  duplicados son idempotentes o solo reinician el mismo movimiento.
- **`SplineStart()`/`SplinePTP(JNT<idx>,speed)`/`SplineEnd()` encolan una
  trayectoria de varios puntos que se ejecutan EN ORDEN, uno despues de
  que termina el anterior** -- igual que el robot real, sin que Unity
  necesite agregar ningun delay. No son no-ops como el resto de los
  comandos `Spline*`/`NewSpline*`: `ControlArticular.cs` (modo "sin
  delay", el default cuando `DelayModeController` esta inactivo) mueve el
  robot mandando `SplineStart()` + una tanda de `SplinePTP(JNT<idx>,speed)`
  (uno por punto en el lote) + `SplineEnd()`, en vez de `MoveJ`. Antes de
  este cambio esos comandos respondian `cmd_res=0` (sin error visible)
  pero el robot no se movia; despues de un primer fix (rutear `SplinePTP`
  igual que `MoveJ`) se movia pero todos los puntos del lote pisaban el
  mismo `_move_target`, o sea saltaba directo al ultimo punto. La
  implementacion actual usa una cola (`_spline_queue`, ver
  `_spline_ptp`/`_tick_joint_states` en `mock_cmd_server.py`): cada
  `SplinePTP` guarda una **copia** del punto (no el indice -- Unity reusa
  `JNTPoint(1..5)` para cada lote nuevo, asi que el proximo lote no debe
  corromper puntos todavia pendientes de un lote anterior) y
  `_tick_joint_states` arranca el siguiente punto de la cola recien cuando
  termina el movimiento actual. `SplineStart()` vacia la cola pendiente
  (no interrumpe el movimiento EN CURSO) para que un lote nuevo no quede
  mezclado con las sobras de uno viejo si Unity manda lotes mas rapido de
  lo que el robot tarda en moverse. El mock sigue sin trazar una curva
  spline real: cada segmento es una interpolacion recta e independiente
  hacia el siguiente punto (no hay blending entre segmentos), pero el
  robot SI visita cada punto de la cola en orden.
- Nombres de joints usados en `/joint_states`: `j1, j2, j3, j4, j5, j6`
  (confirmados en `code/code/frcobot_description/urdf/fr5v6.urdf` y
  `fr5_robot.urdf`, identicos en ambos). Limites de posicion/velocidad
  tomados de los mismos archivos.

### Parametros relevantes (`ros2 launch an5_mock_sim sim.launch.py <arg>:=<valor>`)

| Argumento | Default | Descripcion |
|---|---|---|
| `include_publisher_subscriber` | `true` | Si `false`, no levanta `publisher_subscriber` (solo rosbridge + mock). |
| `joint_states_rate_hz` | `50.0` | Frecuencia de `/joint_states`. |
| `easing` | `ease_in_out` | `ease_in_out` o `linear`. |
| `xmlrpc_mock_enabled` | `false` | Ver seccion siguiente. |
| `initial_joint_positions_deg` | `'0,-90,90,-90,90,0'` | Pose articular de arranque en grados, CSV `'j1,j2,j3,j4,j5,j6'`. Se clampea a `JOINT_LIMITS` si algun valor se pasa de rango. Pasar `''` para arrancar en todos los joints en 0. |

## Simular la IP del robot para publisher_subscriber (obsoleto / desaconsejado)

`code/publisher_subscriber.py` tiene la IP del controlador (`192.168.58.2`)
**hardcodeada** para sus llamadas XML-RPC (`GetActualJointPosDegree`,
`GetActualTCPPose`). Como no podemos tocar ese archivo, en modo sim esas
llamadas fallan (excepcion atrapada) y el propio `publisher_subscriber`
nunca llega a publicar `current_joint_position`/`current_cartesian_position`
con datos reales.

**Esto ya no es un problema:** desde que `mock_cmd_server` publica esos
mismos dos topics directamente (ver seccion "Que hace mock_cmd_server"),
Unity los recibe igual sin que `publisher_subscriber` tenga que lograrlo
por su cuenta. **No hace falta ningun workaround para que Unity vea el
movimiento.**

Este workspace todavia trae un mecanismo para hacer que las llamadas XML-RPC
de `publisher_subscriber.py` *tambien* respondan en modo sim (alias de IP +
`xmlrpc_mock_enabled:=true`, scripts `enable_robot_ip_alias.sh` /
`disable_robot_ip_alias.sh`), mantenido solo por si algo distinto de Unity
depende especificamente de que sea `publisher_subscriber` (y no
`mock_cmd_server`) quien resuelva esas llamadas.

### ⚠️ Riesgo real probado: fuga de sockets, no solo choque de puerto

Ademas del choque de puerto documentado originalmente (`mock_cmd_server` y
el `TCPServer` propio de `publisher_subscriber.py` compiten por
`0.0.0.0:20003`, y el que arranca segundo falla con "address already in
use"), se confirmo en la practica (sesion 2026-07-03) que activar el alias
sin que el mock XML-RPC llegue a bindear produce algo peor que un log sucio:
`publisher_subscriber` termina conectandose por HTTP a su **propio** servidor
TCP (que no habla XML-RPC), y cada intento fallido cada ~20 ms deja un
socket/file-descriptor sin cerrar. En cuestion de segundos se acumularon
~11.800 file descriptors y ~8.460 sockets en el puerto 20003, agotando casi
toda la RAM del host (con el mock y `publisher_subscriber` corriendo en la
misma maquina). No es cosmetico: puede colgar la maquina si se lo deja
corriendo mas de un par de minutos.

**Recomendacion:** no actives `xmlrpc_mock_enabled` ni corras los scripts de
alias de IP mientras `robot_publisher` y `mock_cmd_server` esten en el mismo
host, salvo que tengas una razon especifica (no relacionada con Unity) para
hacerlo, y en ese caso monitoreá `ps aux` / `free -h` en los primeros
segundos y frenalo ante cualquier crecimiento descontrolado de FDs/sockets.

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select an5_mock_sim
source install/setup.bash
```
