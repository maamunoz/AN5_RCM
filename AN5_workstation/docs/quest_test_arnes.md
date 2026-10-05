# Escena `Quest_test_arnes`

Es el entorno de `AN5_sim` (robot fr5v6, laboratorio, conexión ROS2 y toda la UI)
llevado a un visor Meta Quest 2, con locomoción y con la simulación XR del editor
activa para poder validarlo antes de subirlo a las gafas.

## Cómo se genera

La escena **no se edita a mano**: se reconstruye desde `AN5_sim` con

> **AN5 ▸ Reconstruir Quest_test_arnes desde AN5_sim**

(`Assets/Editor/QuestSceneBuilder.cs`), o en batch:

```bash
Unity -batchmode -nographics -quit \
      -projectPath . \
      -executeMethod AN5.EditorTools.QuestSceneBuilder.BuildBatch
```

`AN5_sim` sigue siendo la única fuente de verdad del entorno: cuando cambie, se vuelve
a correr el constructor y `Quest_test_arnes` queda al día. Lo que se retoque a mano
sobre la escena Quest se pierde en la siguiente reconstrucción — si un ajuste debe
sobrevivir, va en el constructor.

## Qué cambia respecto de `AN5_sim`

| | `AN5_sim` (escritorio) | `Quest_test_arnes` (visor) |
|---|---|---|
| Punto de vista | `mainCamera` + `Camera_aux1/2`, orbitables con el DPad | `XR Origin (XR Rig)` en (0, 0, −2), mirando al robot; las tres cámaras de escritorio quedan desactivadas |
| Desplazamiento | teclado/ratón sobre la cámara | locomoción **continua**: stick izquierdo mueve, derecho gira. Sin teleport |
| UI | 7 canvas en Screen Space, una pantalla plana con una pestaña visible a la vez | descompuesta en 10 ventanas World Space: 5 en un arco alrededor del operador, 2 en pantallas grandes sobre la pared del fondo, 2 pegadas a los muros laterales y el header anclado al visor |
| Interacción con la UI | ratón | rayo y toque directo desde los mandos (`TrackedDeviceGraphicRaycaster` + `XRUIInputModule`) |
| Suelo | `Floor` desactivado | `Floor` activo y ajustado a la sala |
| Simulación | — | prefab `XR Interaction Simulator` en la escena |

Se desactivan además los controles que solo tienen sentido con una ventana de
escritorio: `DPad_Orbit`, `DPad_Translate`, `ZoomSlider` y los botones de
minimizar/pantalla completa/salir del header. En VR el punto de vista se cambia
caminando.

### El suelo

`AN5_sim` trae el objeto `Floor` **desactivado**: la cámara de escritorio apunta
siempre al robot y nunca mira al piso, así que allí no hace falta. En VR sí, y no solo
por lo que se ve — sin collider bajo los pies el `CharacterController` del rig se cae
al vacío en cuanto la gravedad lo empuja. El constructor lo activa y lo ajusta a la
sala: medida sobre los colliders de las paredes, va de x −2.5 a 2.5 y de z −3 a 2, así
que el plano queda 5 × 5 m centrado en z −0.5. Con la escala que traía de `AN5_sim`
quedaban dos franjas destapadas contra las paredes.

### Las ventanas sueltas

En `AN5_sim` toda la UI es una sola superficie: `TabContainer` apila los tres paneles de
pestaña (la aplicación enseña uno a la vez) y encima va `PersistentLayer` con el header,
la barra de pestañas y el panel lateral, más `LoadingOverlay`. En el visor esa pantalla
se **descompone**: cada pieza pasa a ser un canvas World Space independiente, con su
sitio y su escala, y las pestañas conviven en vez de turnarse.

Cinco ventanas se reparten por un arco centrado en el punto de vista inicial del operador;
dos son las pantallas de la pared del fondo, dos van pegadas a los muros laterales y la
última va anclada al visor (las cinco, más abajo). Salvo la del visor, cada una se gira
**solo en yaw**, así que su plano contiene siempre el eje vertical del mundo y queda
perpendicular a la tapa de la mesa, esté donde esté. Desde arriba se ven todas de canto.

El arco tiene que dejar libres dos cosas: el robot, que desde el punto de vista inicial
ocupa un cono de ±9° en azimut y de −22° a −5.5° en elevación, y las pantallas, que se
ven por encima de él entre −26° y +26°. Así que todo el frente queda despejado y el arco
se abre a los lados:

| ventana | yaw | radio | altura | escala | tamaño |
|---|---|---|---|---|---|
| `SecTraj` (control de trayectorias) | 0° | 0.95 m | 1.05 m | 0.00184 | 0.45 × 0.29 m |
| `Footer` (barra de pestañas) | 0° | 1.10 m | 0.62 m | 0.0009 | 1.73 × 0.04 m |
| `Panel_ppal` | −95° | 2.15 m | 1.45 m | 0.0007 | 1.34 × 0.76 m |
| `Panel_trayectorias` | +50° | 1.90 m | 1.45 m | 0.0009 | 1.73 × 0.97 m |
| `LoadingOverlay` | 0° | 1.30 m | 1.55 m | 0.0006 | 1.15 × 0.65 m |

`SecTraj` —cargar/guardar el `.csv`, ejecutar, pausar, parar— es lo único del conjunto
que se maneja en vez de mirarse, así que va al frente y al alcance de la mano, a 0.95 m
y a la altura del pecho. Cae de −22° para abajo en elevación, justo por debajo del
robot, así que no tapa ni al robot ni a las pantallas. Es además la única ventana que el
operador puede **recolocar con los mandos**: lo que dice su fila es de dónde parte, no
dónde se queda (ver «Mover el panel de trayectorias»).

El `Footer` se queda flotando en el mundo, en el punto donde arranca la aplicación,
haciendo **consola** justo debajo del control de trayectorias: al estar anclado ahí uno
puede alejarse y volver, al revés que el header, que va pegado a la cabeza. Va por debajo
de `SecTraj`, que llega hasta −35.3° de elevación, con 5° de separación.

Ojo con ese margen: mirando hacia abajo, los extremos de una tira ancha y plana quedan más
**altos** que su centro, así que su borde superior no es la elevación de la tabla sino
unos grados por encima — el `Footer` está centrado en −42.3° pero sus esquinas suben a
−34.4°. Esas esquinas caen en ±38° de azimut, donde `SecTraj` (que solo abarca ±13.3°) ya
no llega, así que no se tocan: comprobado por franjas de azimut, no por caja envolvente,
que ahí daba un falso positivo.

`LoadingOverlay` sí se queda al frente, tapando las pantallas: es modal y solo se ve
mientras carga, que es justo lo que tiene que hacer.

**Los yaw del arco están medidos, no calculados.** En un panel plano y ancho el borde que
queda cerca del eje de la vista abarca mucho más ángulo que el lejano, así que su huella
angular no es `yaw ± semiángulo` sino claramente asimétrica: el `Header`, cuando todavía
estaba en el arco, a −48° llegaba hasta −12.8° y se comía la esquina de la pantalla de
gráficos. Si mueves algo, comprueba la huella real en vez de fiarte de la cuenta.

La tabla está tal cual en `k_Windows`, en la cabecera de `QuestSceneBuilder.cs`. Para
mover una ventana basta con retocar su fila: `yaw` la desplaza por el arco, `radius` la
acerca o la aleja, `height` la sube o la baja y `pixelToMeter` la agranda (más legible,
más giro de cabeza) o la encoge.

Tres detalles que no son obvios:

- **El rect de cada canvas raíz se fija a 1920 × 1080 antes de medir nada.** Mientras un
  canvas está en Screen Space, Unity conduce su `RectTransform` desde la resolución del
  Game View, así que `Header` (anclado a lo ancho de su padre) o `LeftPanel` (a lo alto)
  medirían lo que midiera la ventana del editor en vez de la resolución de referencia
  con la que está anclada toda la UI.
- **Cada ventana se centra sobre su contenido visible, no sobre su rect.** Las pestañas
  siguen siendo rects de 1920 × 1080 con el contenido pegado a la derecha, porque la
  franja izquierda la ocupaba el `LeftPanel`, que ahora es otra ventana. Y al header le
  falta el bloque de botones de ventana, desactivado más arriba.
- **`Panel_ppal` va más pequeña** porque a −95° su ancho se reparte casi todo en Z, y por
  ahí solo hay 1 m hasta la pared del fondo. Sigue además **inactiva**: está retirada desde
  `TabController` y su contenido duplica al que estaba en monitoreo, así que se coloca
  por si algún día se reactiva, pero activarla sería un cambio de UI y no de colocación. Si la
  quieres visible, ponla activa en la escena (o añade el `Activate` en
  `ReleaseTabSwitching`).

`TabContainer` y `PersistentLayer` se quedan vacíos y el constructor los apaga.

### Las dos pantallas de la pared del fondo

Lo que solo se mira va en grande contra la **pared que el operador tiene enfrente**,
detrás del robot, como dos pantallas que se leen desde cualquier punto de la sala y en
escorzo. De izquierda a derecha:

| pantalla | qué es | escala | tamaño | campo visual a 4 m |
|---|---|---|---|---|
| `SecTrend` | gráficos de error de posición X/Y/Z, real contra setpoint (ver `SecTrendGraphController`) | 0.0044 | 1.07 × 1.85 m | 14° × 25° |
| `CenterBottom` | `SecJoints` (posiciones articulares) y `SecPosition` (cartesianas del efector final) | 0.0034 | 2.31 × 1.79 m | 30° × 24° |

Cada una lleva su propia escala porque sus proporciones no se parecen en nada: `SecTrend`
es una columna de 244 × 421 px y las lecturas un bloque de 698 × 545. La lista está en
`k_WallWindows`; el reparto lo controlan `k_WallGap`, `k_WallCenterX`, `k_WallCenterY` y
`k_WallClearance`.

#### Las lecturas van en rejilla de 3 × 4

De escritorio vienen como dos tiras de seis —seis articulaciones en fila y seis
coordenadas en fila—, que ampliadas a pantalla de pared serían una banda larguísima e
incómoda de barrer con la vista. `ReflowReadouts` las replantea a **tres columnas**, así
que cada tira ocupa dos filas: 2 de articulares (BASE/SHOULDER/ELBOW y
WRIST1/WRIST2/WRIST3) y 2 de cartesianas (X/Y/Z y Rz/Ry/Rx), 3 × 4 en total. El bloque
pasa de 1370 × 280 px a 698 × 545, casi cuadrado.

Se cambia el layout, no las cajas: cada `Body` pasa de `HorizontalLayoutGroup` a
`GridLayoutGroup` conservando su espaciado y sus márgenes, y las celdas se recalculan
para llenar el ancho nuevo. `SecJointsDisplay` y `SecPositionDisplay` buscan sus textos
por nombre relativo, así que no se enteran. El ancho lo manda la sección que más pida con
tres columnas, para que las dos rejillas queden alineadas entre sí.

Dos trampas que costaron una pasada:

- **`sizeDelta` no es el tamaño cuando el anclaje es a estiramiento.** En `AN5_sim` el
  bloque de lecturas viene anclado así, y escribirle 698 dentro de un canvas de 1920
  dejaba un rect de 2618. Por eso todo el redimensionado va por `SetSize`, que usa
  `SetSizeWithCurrentAnchors`.
- **Las cabeceras no encogían solas.** Vienen maquetadas para 1350 px con anchos fijos
  —en `SecJoints`, un espaciador de 356 y un «6 DOF» de 386 que centran el título— y su
  grupo no controla el ancho de los hijos, así que a 698 se habrían salido del panel.
  `FitHead` detecta el desbordamiento y activa `childControlWidth` solo cuando hace
  falta.

Tres cosas condicionan dónde pueden estar:

- **La pared se busca por geometría, no por nombre.** En `AN5_sim` los muros se llaman
  Front/Back/Left/Right sin ninguna relación con la orientación del puesto: el del fondo
  resulta ser `wall Right`. `FindFacingWall` coge el de z mayor, que es el que le da la
  cara a un operador que arranca en z −2 mirando a +Z.
- **La pared se despeja al construir.** `ClearWall` apaga lo que cuelgue de ese muro —hoy
  `Logo_GIA v1` y `LogoUnicauca 2`— para que no se mezcle con las pantallas. Se
  identifican por geometría (hijos de `Laboratory` que no son muros y caen sobre su
  plano), no por nombre, y se desactivan en vez de borrarse: devolverlos es quitar esa
  llamada. El `LogoUnicauca` de la pared lateral no se toca.
- **Cuanto más grandes, más altas.** El robot se cruza por delante del borde inferior en
  cuanto levanta el brazo: llega a y 1.40, y la visual al borde bajo de `SecTrend` pasa a
  y 1.48. A este tamaño, `k_WallCenterY = 2.30` es lo más bajo que admiten.

Subir las escalas las agranda más, a cambio de que se ensanchen y vuelvan a chocar con el
arco: hoy llegan a ±26° de azimut y el muro entero solo abarca ±32° desde el punto de
vista inicial.

### La cola de coordenadas, junto a los sliders de jog

`SecCoord` —el log de la cola y sus botones ADD/CLEAR, SEND y Save TXT— sube del panel
derecho de su pestaña a ponerse **al lado de `SecJoints`** dentro del mismo
`Panel_trayectorias/CenterBottom`. Encolar una pose es leer esos sliders, así que las dos
mitades acaban formando un solo panel:

```
[ SecJoints    ] [          ]
[ SecCartInput ] [ SecCoord ]
```

`JogRow` es la fila; dentro va `JogColumn` con las articulaciones y, justo debajo, las
entradas cartesianas. **`SecCartInput` se estrecha de 1602 a los 1350 px de `SecJoints`**
—se lo impone el layout group de la columna— y cae en el hueco que deja `SecCoord` por
ser 126 px más alta que los sliders. Sus seis cajas caben de sobra: piden 1002 px.

**El sitio para la cola sale de ensanchar el bloque, no de encoger los sliders.**
Estrecharlos no es opción: sus siete cajas declaran un mínimo que deja a `SecJoints` en
1350 px, y el layout se salta cualquier `preferredWidth` menor que se le ponga. Así que
`CenterBottom` pasa de 1370 a 1622 px de ancho (1350 + 8 + 244, más sus márgenes) y crece
**hacia la izquierda**: por la derecha ya tocaba el panel lateral de la pestaña, que
empieza en x 1660, y por la izquierda sobraba sitio hasta el borde. De alto pasa de 408 a
436 — solo lo que la fila es más alta que lo que antes apilaba.

Con eso el **panel derecho de la pestaña se queda sin nada propio** —la cola se ha ido con
los sliders y lo único que le quedaba, su `SecTraj`, duplica el que flota al frente— así
que `RetireTrajectorySidePanel` lo apaga entero: la aplicación usa un solo panel para las
trayectorias. `Panel_trayectorias` pasa a ser solo su `CenterBottom`, 0.93 × 0.32 m.

Va después de `PairQueueWithJog` a propósito: mientras la cola siga colgando de ahí hay
que poder medirla, y dentro de una rama desactivada los layout groups no corren y los
tamaños se leen rancios.

De paso deshace una ambigüedad que el propio `SecTrajController` documenta: la escena
traía varios `SecTraj` vivos y solo uno acaba gobernando la ejecución —de ahí que publique
el setpoint por un `static`, porque un suscriptor de fuera no tiene forma fiable de elegir
la instancia buena. Apagado este, queda uno solo activo.

### El header, anclado al visor

`Header` (logo, pastillas de estado ROS/robot, `ModeToggle`) no va en el mundo:
`PinToHeadset` lo cuelga de la **cámara del rig**
(`XR Origin (XR Rig)/Camera Offset/Main Camera`), así que se queda clavado en el campo de
visión y acompaña al operador mire donde mire. Su fila está en `k_HudWindows`, que admite
más de una si algún día hace falta.

Va a `k_HudDistance` = 1.60 m por delante de los ojos, centrado en azimut y arriba del
todo: elevación pedida 30°, huella real azim ±28.4° y elev 26.3–27.5°.

Tres cosas que costaron una vuelta:

- **No se gira solo en yaw.** Es la única ventana que **no** queda perpendicular a la tapa
  de la mesa: al ir pegada a la cabeza acompaña también su cabeceo y su alabeo, que es
  justo lo que se le pide a un HUD.
- **Aquí no se recentra sobre el contenido**, al revés que en las ventanas del mundo. Lo
  que se ve es su barra de fondo, que es un `Image` del propio rect y no un hijo, así que
  `ContentOffset` no la mide: centrar por los hijos dejaba la barra corrida 13° a la
  derecha. Una barra de header se centra ella, con su logo a la izquierda y su estado a la
  derecha, que es como está maquetada.
- **La elevación necesita margen.** Es la del centro de la tira, pero los extremos de una
  tira ancha y plana quedan más lejos que su centro y su ángulo "cae" — 3.4° en este caso.
  A 26° el borde bajo se metía hasta 22.6° y rozaba la cabecera de las pantallas de la
  pared, que suben hasta 21.9°. Con 30° el borde más bajo queda en 26.3° y las deja libres.

Al ir pegado a la cabeza tapa **permanentemente** su franja del campo visual: 26.3° a
27.5° por encima de la vista. Esa franja está vacía con la vista al frente, pero en cuanto
se levanta la cabeza el mundo pasa por debajo — es inevitable con un HUD. Bajar
`k_HudDistance` o su elevación lo acerca a la postura natural a cambio de tapar más.

### Las dos pegadas a los muros laterales

`LeftPanel` y `SecCamara` tampoco flotan en el arco: van planas contra los muros de los
lados, 1 cm por delante y mirando hacia dentro de la sala. Su rotación es un giro de 90°
solo en yaw, así que siguen perpendiculares a la tapa de la mesa. La lista está en
`k_SideWallWindows`, y cada fila lleva la **dirección** del muro, que sirve para las dos
cosas: buscarlo y orientar la ventana.

| ventana | muro | en `AN5_sim` | z | altura | escala | tamaño |
|---|---|---|---|---|---|---|
| `LeftPanel` | derecha (x 2.5) | `wall Back` | −1.75 m | 1.45 m | 0.0021 | 0.61 × 2.07 m |
| `SecCamara` | izquierda (x −2.5) | `wall Front` | −0.84 m | 1.72 m | 0.0035 | 0.39 × 0.14 m |

`SecCamara` se queda en el mismo azimut que ocupaba flotando, −65°, que sobre ese muro cae
en z −0.84, y a la altura de la vista. No pisa el `LogoUnicauca` de esa pared, que está
más arriba (y 2.42–2.46).

Sobre el panel lateral:

**Su escala está puesta para que se lea igual que las lecturas de la pared del fondo.** Lo
que iguala la legibilidad no es el tamaño sino el ángulo que abarca cada píxel: las
lecturas van a 0.0034 m/px pero a 4.10 m, y ese muro está a 2.51, así que
`0.0034 × 2.51 / 4.10` da los **0.0021 m/px** de la tabla y deja a los dos en 2.85 minutos
de arco por píxel. El panel acaba midiendo 0.61 × 2.07 m, entre y 0.43 y 2.53.

Va bastante atrás en z a propósito: `Panel_trayectorias` llega hasta 74° de azimut y está
a la mitad de distancia, así que más adelante se le pondría por delante — a la escala
anterior el margen era de décimas de grado y al agrandarlo se lo comía. Con `AlongZ` en
−1.75 el panel ocupa de 77.2° a 91.0° y no lo tapa nada.

Los muros, como el del fondo, se buscan **por geometría** —`FindWall(scene, dirección)`,
el más lejano en esa dirección— y no por nombre: los `wall Front/Back/Left/Right` de
`AN5_sim` no siguen la orientación del puesto. El de la derecha del operador resulta ser
`wall Back` y el de su izquierda, `wall Front`.

### Lo que queda de `Panel_monitoreo`

Nada: sus tres piezas —los gráficos, las lecturas y el control de trayectorias— son ahora
ventanas sueltas, así que el constructor **apaga la pestaña entera**. Es seguro aunque
`MonitoreoActivation` viva ahí: con el panel inactivo desde el principio sus
`OnEnable`/`OnDisable` no llegan a correr nunca, y quien pone `driveRobotModel` en `true`
es `TrayectoriasActivation`, en un panel que sí queda activo.

`Panel_trayectorias` traía su propio `SecTraj`, así que durante un tiempo el control de
trayectorias salía en dos sitios a la vez. Ya no: ese panel derecho está apagado (ver más
arriba) y el `SecTraj` que flota al frente es el único activo de la escena.

### Interacción con los mandos

Todo lo que en escritorio se maneja con el ratón se maneja con el mando, salvo escribir
texto. La cadena completa, verificada sobre la escena reconstruida:

- El `EventSystem` lleva **`XRUIInputModule`** y ningún otro módulo de entrada
  (`SetUpEventSystem` borra los que traía `AN5_sim`).
- Los **9 canvas activos** llevan `GraphicRaycaster` + `TrackedDeviceGraphicRaycaster` y
  su `worldCamera` apuntando a la cámara del rig — se lo pone `DetachAsWindow` a cada
  ventana que saca.
- Los **56 controles activos** (34 `Button`, 7 `Slider`, 15 `InputField`) cuelgan todos de
  un canvas con el raycaster del rayo. Ninguno se queda fuera.
- Los mandos traen **`NearFarInteractor`** (el rayo lejano de XRI 3.x) y `XRPokeInteractor`
  (toque directo), los dos con `enableUIInteraction`. Ojo al buscarlos: el rayo de UI no es
  el `XRRayInteractor`, que aquí es el del teleport y va con la UI **desactivada**.
- El rayo alcanza 10 m y el control más lejano está a 2.77 m.
- Lanzando rayos de prueba contra los raycasters desde la posición del mando, **15 de 15
  controles muestreados devuelven el objeto apuntado**.

#### Mover el panel de trayectorias

El reparto del arco es el mismo para todo el mundo, y para lo que solo se mira eso vale.
`SecTraj` no: se maneja con las manos, y la altura y la distancia cómodas dependen de
quién lleve el visor y de si trabaja de pie o sentado. Así que lleva **asa**
(`VrWindowGrab`, en `Assets/Scripts/VrWindowGrab.cs`): una tira vertical pegada a su
costado derecho, rotulada `MOVER`, que se coge con el gatillo —de lejos con el rayo, de
cerca con la mano— y arrastra la ventana entera hasta que se suelta. La tira se enciende
al apuntarla y cambia de color mientras se sostiene.

Quién la lleva sale de `k_MovableWindows`, en la cabecera de `QuestSceneBuilder.cs`; hoy
solo `SecTraj`. Dejar mover las demás sería dejar deshacer un reparto que está medido
para no tapar ni el robot ni las pantallas.

Como el teclado, el asa **se monta sola en runtime** y no desde la escena: la escena se
reconstruye entera en cada pasada, así que una jerarquía dejada a mano se perdería. Al
constructor le basta con añadir el componente a la ventana, después de colocarla.

Cuatro decisiones que no son obvias:

- **El asa va al costado, no arriba.** `SecTraj` está encajada entre el robot, que empieza
  en −22° de elevación, y el `Footer`, cuyas esquinas suben a −34.4°: por arriba o por
  abajo, una tira de ~0.16 m a 0.95 m son ~9° que se comerían uno de esos dos márgenes. A
  lo ancho sobra sitio — la ventana solo abarca ±13.3° de azimut, y con el asa (96 px de
  ancho y 12 de separación, o sea 0.20 m a esta escala) llega a ~24°, con lo próximo que
  hay a la derecha, `Panel_trayectorias`, empezando en 26° y además 3° más arriba.
- **La ventana solo gira en yaw.** Se congela en el marco del mando al agarrarla, pero
  descontando su cabeceo y su alabeo, así que la muñeca no la vuelca: su plano sigue
  conteniendo el eje vertical del mundo —perpendicular a la tapa de la mesa, como todas
  las demás— la dejes donde la dejes. Ese yaw se saca proyectando el eje del mando sobre
  el plano horizontal y no de `eulerAngles.y`, que apuntando a plomo (justo como se coge
  una ventana que está por debajo de la vista) pega saltos.
- **El agarre entra por un collider, no por el raycaster de UI.** El asa se dibuja con
  `raycastTarget` desactivado y lleva su propia caja, de 48 px de grosor hacia el lado por
  el que llega la mano. Si fuese además blanco de UI, el rayo tendría dos cosas que
  golpear en el mismo sitio. Los botones de la ventana no se tocan: el volumen del asa
  queda fuera de ellos.
- **La altura está acotada** entre 0.35 y 2.2 m, para que no se pueda dejar bajo el suelo.
  En horizontal no hay tope: el alcance del brazo ya es el límite.

Dos límites conocidos: la posición **no se guarda** —cada arranque vuelve a dejar la
ventana donde diga `k_Windows`—, y en cuanto el operador la mueve dejan de valer las
holguras medidas del arco, que es justo lo que se está cediendo a cambio.

Lo que está verificado es que compila y que el constructor le pone el componente; el
agarre en sí hay que probarlo en Play con el simulador XR o en el visor.

#### El teclado virtual

Escribir era el único hueco: los 15 `InputField` se pueden señalar con el rayo, pero el
visor no tiene teclado y un `InputField` heredado bajo OpenXR pelado no levanta el del
sistema de Meta. Lo cubre **`VrKeyboard`** (`Assets/Scripts/VrKeyboard.cs`), que el
constructor deja en la escena como un `GameObject` suelto.

Vigila el foco y, en cuanto se selecciona uno de esos campos, abre un teclado flotante
delante del operador. Se monta **por código en runtime**, no desde la escena: la escena se
reconstruye entera desde `AN5_sim` en cada pasada, así que una jerarquía montada a mano se
perdería. Son 44 teclas —dígitos, QWERTY, `.` `-` `_` `/`, espacio, borrar, Cancelar y
OK— en 0.66 × 0.39 m con teclas de 5.9 cm, alcanzables tanto con el rayo como con el toque
directo. Se coloca a 0.80 m y 28° por debajo de la vista, girando solo en yaw, o sea
perpendicular a la mesa como el resto.

**Lo delicado no era el teclado sino no disparar `onEndEdit` a destiempo.** A ese evento le
cuelga `SecCartInputController`, que pide cinemática inversa y aplica el resultado a los
sliders de las articulaciones. Si cada tecla lo disparara, se estarían pidiendo poses a
medio escribir. Por eso:

- Al abrir, el campo se suelta del `EventSystem`. Si se quedara seleccionado, la primera
  tecla le robaría el foco y Unity dispararía su `onEndEdit` a mitad de escritura.
- Mientras se teclea se escribe con `SetTextWithoutNotify`: no se dispara ni
  `onValueChanged` ni `onEndEdit`. Lo que se ve es la vista previa del propio teclado.
- Al aceptar se asigna el texto y se invoca `onEndEdit` **una sola vez**, con el valor
  definitivo. `Cancelar` restaura el original sin avisar a nadie.

Queda un efecto que no se puede evitar del todo: soltar el campo al abrir hace que Unity
dispare su `onEndEdit` una vez **con el valor sin cambiar**, o sea una re-petición de la
pose que ya estaba. Debería ser inocuo —esos campos se refrescan solos con la pose en
vivo— pero conviene confirmarlo en el visor antes de darlo por bueno.

**Lo que está verificado y lo que no.** Que el teclado se construye con sus 44 teclas, su
canvas World Space y sus dos raycasters, sí. El ciclo completo abrir → teclear → aceptar
solo se puede comprobar en Play con el simulador XR; queda pendiente.

### Las pestañas ya no se turnan

`TabController` vive en el `Footer` —porque el footer *es* la barra de pestañas— y en
`Start` apaga todos los paneles menos el activo. Con las pestañas convertidas en
ventanas separadas eso las haría desaparecer, así que el constructor **destruye el
componente**: los botones de la barra se quedan con un destino nulo y sus `onClick`
pasan a ser inocuos. La barra sigue ahí como indicador, pero ya no cambia nada.

Los dos paneles vivos pueden estar activos a la vez sin pelearse: `MonitoreoActivation`
y `TrayectoriasActivation` solo comparten `driveRobotModel` y los dos lo quieren en
`true`; el conflicto estaba en el `OnDisable` del saliente, que ya no llega a ocurrir.

Lo que sí hubo que tocar es `SecCoordQueueController`. Buscaba **el primer `SecJoints`
activo de toda la escena** para engancharle sus sliders, cosa que solo funcionaba porque
las pestañas se turnaban y nunca había más de uno activo; con todas visibles hay tres y
el barrido podía atar los controles al panel equivocado.

Ahora busca **el `SecJoints` que tiene los sliders**, que es lo único que de verdad
necesita y el único discriminador estable: de los tres de la escena solo el de
`Panel_trayectorias` los lleva (`SecJointsSliderSync`, con una `S` por articulación),
mientras que los de monitoreo y `Panel_ppal` son lecturas de solo mostrar
(`SecJointsDisplay`, cajas `N/ValBox/Unit`, y ni siquiera nombran las cajas igual). El
cambio vale igual para `AN5_sim`, donde el resultado no varía.

`PANEL` (con su canvas `Main_Panel/Canvas`) está inactivo en `AN5_sim` y se deja tal
cual, en Screen Space: si alguna vez se reactiva, hay que pasarlo a World Space a mano.

## Validar en el editor

1. Abrir `Assets/Scenes/Quest_test_arnes.unity` y darle a Play. No hace falta visor:
   el `XR Interaction Simulator` inyecta un HMD y dos mandos simulados.
2. Controles del simulador (los recuerda su UI en pantalla):
   - ratón: mirar; `W`/`A`/`S`/`D`: desplazar.
   - `Tab`: alterna entre mover la cabeza (modo FPS) y manipular un dispositivo.
   - `[`, `]`: activan mando izquierdo / derecho (dos pulsaciones alternan entre
     mando y mano).
   - con un mando activo, el gatillo simulado dispara el `Select`, que es lo que
     acciona los botones y sliders de la UI a través del rayo.
3. Para que la UI responda al rayo hace falta que el `EventSystem` de la escena tenga
   el `XRUIInputModule` (lo pone el constructor) y que cada canvas tenga el
   `TrackedDeviceGraphicRaycaster` (idem).
4. Para probar que el panel de trayectorias se mueve: apuntar con el rayo a la tira
   `MOVER` del costado derecho de `SecTraj` —tiene que encenderse— y mantener el gatillo
   mientras se gira o se desplaza el mando. La ventana debe acompañarlo sin volcarse y
   quedarse donde se suelte.

## Pendiente antes de subirlo a las gafas

Nada de esto está hecho: la escena está configurada **para simulación**, como se pidió.

1. **Perfil de interacción de OpenXR para Android.** El loader XR de Android ya es
   OpenXR, pero en `Assets/XR/Settings/OpenXRPackageSettings.asset` no hay ninguna
   *feature* habilitada para Android — ni el grupo Meta Quest ni un
   *interaction profile* (p. ej. `Oculus Touch Controller Profile`). Sin al menos un
   perfil, los mandos no dan entrada en el dispositivo. Se habilita en
   **Project Settings ▸ XR Plug-in Management ▸ OpenXR ▸ Android**.
2. **Quitar el `XR Interaction Simulator` de la escena.** Su componente solo compila
   en editor y standalone (`AR_FOUNDATION_5_2_OR_NEWER && (UNITY_EDITOR || …)`), así
   que en una build de Android queda como script perdido.
3. **Rendimiento.** El proyecto usa el pipeline integrado y espacio de color Gamma.
   Para Quest 2 conviene revisar espacio de color Lineal y las opciones de calidad
   antes de dar por buen el framerate en el visor.
4. **Red.** En el visor, `RosConnector` tiene que apuntar a una IP de rosbridge
   alcanzable desde la WiFi de las gafas: `localhost` deja de servir.
