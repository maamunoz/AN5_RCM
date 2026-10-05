using System;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.SceneManagement;
using UnityEngine.UI;
using UnityEngine.XR.Interaction.Toolkit;
using UnityEngine.XR.Interaction.Toolkit.UI;

namespace AN5.EditorTools
{
    /// Reconstruye Assets/Scenes/Quest_test_arnes.unity a partir de AN5_sim.unity.
    ///
    /// La escena Quest es una copia literal del entorno de AN5_sim (robot fr5v6,
    /// laboratorio, conexión ROS2 y toda la UI) con las adaptaciones mínimas para
    /// que funcione en un visor: XR Origin con locomoción
    /// continua, la UI pasada a World Space e interacción por rayo desde los mandos.
    ///
    /// Está escrito como reconstrucción y no como edición incremental a propósito:
    /// AN5_sim sigue siendo la única fuente de verdad del entorno, así que cuando
    /// cambie basta con volver a correr esto (menú AN5) y la escena Quest se
    /// regenera. Cualquier retoque hecho a mano sobre Quest_test_arnes se pierde al
    /// reconstruir; si un ajuste debe sobrevivir, va aquí.
    public static class QuestSceneBuilder
    {
        const string k_SourceScene = "Assets/Scenes/AN5_sim.unity";
        const string k_TargetScene = "Assets/Scenes/Quest_test_arnes.unity";

        const string k_RigPrefab =
            "Assets/Samples/XR Interaction Toolkit/3.4.1/Starter Assets/Prefabs/XR Origin (XR Rig).prefab";
        const string k_SimulatorPrefab =
            "Assets/Samples/XR Interaction Toolkit/3.4.1/XR Interaction Simulator/XR Interaction Simulator.prefab";

        // --- Geometría del puesto de trabajo virtual (metros, mundo de AN5_sim) ---
        //
        // El laboratorio va de x -2.5..2.5 y z -3..2, con la mesa en el origen (su tapa
        // a y 0.762) y el robot encima. El operador arranca a 2 m de la mesa mirando
        // hacia +Z.
        static readonly Vector3 k_UserStart = new Vector3(0f, 0f, -2f);
        const float k_EyeHeight = 1.6f;

        // Resolución de referencia de la UI de AN5_sim: todas las secciones están
        // ancladas contra ella, así que los canvas raíz se fijan a este rect para
        // reproducir la maquetación de escritorio exactamente.
        const float k_CanvasWidthPx = 1920f;
        const float k_CanvasHeightPx = 1080f;

        /// Una ventana suelta del panel: qué RectTransform de AN5_sim es y dónde se
        /// planta en el arco alrededor del operador. Las que van pegadas a la pared del
        /// fondo no usan esto, ver k_WallWindows y PlaceOnBackWall.
        ///
        /// La orientación no es un dato: toda ventana se gira solo en yaw, así que su
        /// plano contiene siempre el eje vertical del mundo y queda perpendicular a la
        /// tapa de la mesa, esté a la altura que esté.
        readonly struct WindowSpec
        {
            public readonly string Path;          // ruta en la jerarquía de AN5_sim
            public readonly float YawDeg;         // + = a la derecha del frente del operador
            public readonly float Radius;         // m del punto de vista al centro de la ventana
            public readonly float Height;         // m, altura del centro de la ventana
            public readonly float PixelToMeter;   // escala px -> m de esta ventana

            public WindowSpec(string path, float yawDeg, float radius, float height, float pixelToMeter)
            {
                Path = path;
                YawDeg = yawDeg;
                Radius = radius;
                Height = height;
                PixelToMeter = pixelToMeter;
            }
        }

        // Desde el punto de vista inicial hay dos cosas que el arco no puede tapar: el
        // robot, que ocupa un cono de ±9° en azimut y de -22° a -5.5° en elevación, y las
        // dos pantallas de la pared del fondo (ver k_WallWindows), que se ven por encima
        // de él entre -26° y +26° de azimut. Como están pegadas a un muro que desde aquí
        // solo abarca ±32°, no se las puede correr a un hueco: es el arco el que se
        // reparte alrededor, dejando ese frente despejado.
        //
        // Para mover una ventana basta con retocar su fila: yaw la desplaza por el arco,
        // radius la acerca o la aleja, height la sube o la baja y pixelToMeter la agranda
        // (más legible, más giro de cabeza) o la encoge.
        const string k_TrajectoryWindow = "TabContainer/Panel_monitoreo/RightPanel/Content/SecTraj";
        const string k_TrajectoriesTabWindow = "TabContainer/Panel_trayectorias";

        static readonly WindowSpec[] k_Windows =
        {
            // El control de trayectorias (cargar/guardar el .csv, ejecutar, pausar) es lo
            // único de aquí que se maneja en vez de mirarse, así que va al frente y al
            // alcance de la mano: a 0.95 m y a la altura del pecho, mirando al operador.
            // Queda por debajo del robot —de -23° para abajo en elevación, y el robot
            // empieza en -22°— así que no tapa ni al robot ni a las pantallas.
            //
            // Esto es solo dónde aparece: es además la única ventana que el operador
            // puede recolocar con los mandos, ver k_MovableWindows.
            new WindowSpec(k_TrajectoryWindow,                  0f, 0.95f, 1.05f, 0.00184f),

            // La pestaña de trayectorias, a la derecha. Una vez retirado su panel derecho
            // (ver RetireTrajectorySidePanel) lo que le queda visible es su CenterBottom,
            // 1622x436 px, que a 0.0009 m/px son 1.46 x 0.39 m: a 1.9 m y puesta a 50°
            // ocupa de 29.0° a 71.0° de azimut, así que deja pasar las pantallas de la
            // pared, que terminan en 28.5°.
            //
            // También se puede recolocar con los mandos, ver k_MovableWindows.
            new WindowSpec(k_TrajectoriesTabWindow,            50f, 1.90f, 1.45f, 0.0009f),

            // Panel_ppal está retirada (ver TabController): se coloca por si algún día
            // se reactiva, aparcada contra la pared izquierda, pero se deja inactiva.
            // A 95° su ancho se reparte casi todo en z, y por ahí solo hay 1 m hasta la
            // pared del fondo (el operador arranca en z -2 y el muro está en z -3): a
            // 0.0009 m/px la atravesaría, así que esta va más pequeña, 1.34 x 0.76 m.
            new WindowSpec("TabContainer/Panel_ppal",         -95f, 2.15f, 1.45f, 0.0007f),

            // (El panel lateral y el selector de cámara se pegan a los muros laterales,
            // ver k_SideWallWindows y PlaceOnSideWalls. Header y Footer no están en
            // ningún lado: se desactivan enteros en DisableDesktopOnly, son inútiles en
            // este puesto de VR.)

            // El velo de carga es modal: al frente y más cerca que todo lo demás, para
            // que tape el resto mientras está visible.
            new WindowSpec("LoadingOverlay",                    0f, 1.30f, 1.55f, 0.0006f),
        };

        // Ventanas que el operador puede recolocar con los mandos: se les añade un asa
        // (VrWindowGrab, en Assets/Scripts) por la que arrastrarlas con el gatillo. La
        // fila de k_Windows sigue mandando dónde aparecen; esto solo dice cuáles se
        // pueden mover después.
        //
        // Las dos con las que se trabaja: SecTraj (cargar el .csv, ejecutar, pausar) y la
        // pestaña de trayectorias (los sliders de jog, las entradas cartesianas y la cola
        // de coordenadas). En las dos la altura y la distancia cómodas dependen de quién
        // lleve el visor y de si trabaja de pie o sentado.
        //
        // Las demás no: lo que solo se mira vive donde lo pone el reparto del arco, que
        // está medido para no tapar ni el robot ni las pantallas de la pared, y dejarlas
        // mover sería dejar deshacerlo.
        static readonly string[] k_MovableWindows =
        {
            k_TrajectoryWindow,
            k_TrajectoriesTabWindow,
        };

        /// Una de las pantallas de la pared del fondo. Solo lleva escala: la posición la
        /// reparte PlaceOnBackWall a lo largo del muro.
        readonly struct WallWindowSpec
        {
            public readonly string Path;
            public readonly float PixelToMeter;

            public WallWindowSpec(string path, float pixelToMeter)
            {
                Path = path;
                PixelToMeter = pixelToMeter;
            }
        }

        // Lo que solo se mira va a la pared del fondo, en grande, como dos pantallas: se
        // leen desde cualquier punto de la sala sin acercarse.
        //
        // De izquierda a derecha: los gráficos de error de posición X/Y/Z (real contra
        // setpoint, ver SecTrendGraphController) y las lecturas del robot — SecJoints,
        // las posiciones articulares, y SecPosition, las cartesianas del efector final,
        // que viven las dos en CenterBottom.
        //
        // Cada una lleva su propia escala porque sus proporciones no se parecen en nada:
        // SecTrend es una columna de 244x421 px y CenterBottom una banda de 1350x295. Las
        // escalas de abajo las dejan en 1.07 x 1.85 m y 2.73 x 0.60 m: desde el punto de
        // vista inicial, a 4 m, eso son 15° x 26° y 37° x 8° de campo visual.
        static readonly WallWindowSpec[] k_WallWindows =
        {
            new WallWindowSpec("TabContainer/Panel_monitoreo/RightPanel/Content/SecTrend", 0.0044f),
            new WallWindowSpec(k_ReadoutWindow,                                           0.0034f),
        };

        // Las lecturas del robot vienen de escritorio como dos tiras de seis: seis
        // articulaciones en fila y seis coordenadas en fila. Eso, ampliado a pantalla de
        // pared, es una banda larguísima e incómoda de barrer con la vista. Cada tira se
        // replantea en rejilla de tres columnas, así que pasa a ocupar dos filas: 3 x 4
        // entre las dos, y el bloque queda casi cuadrado en vez de apaisado.
        const string k_ReadoutWindow = "TabContainer/Panel_monitoreo/CenterBottom";
        const int k_ReadoutColumns = 3;
        const string k_JointsSection = "SecJoints";
        static readonly string[] k_ReadoutSections = { k_JointsSection, "SecPosition" };

        // Mismo mecanismo (ToGrid/NaturalGridWidth) que k_ReadoutColumns, pero para los
        // sliders de jog y las cajas cartesianas de Panel_trayectorias -- ver
        // ReflowJointsAndCart. Tres columnas: J1-J3/J4-J6 y X-Y-Z/Rx-Ry-Rz quedan cada
        // una en dos filas parejas.
        const int k_JogColumns = 3;

        // La cola de coordenadas sube del panel derecho de su pestaña a ponerse al lado
        // de los sliders de jog, dentro del mismo CenterBottom: encolar una pose es leer
        // esos sliders, así que las dos mitades quedan en un solo panel.
        //
        // Las dos pasan a colgar de una fila horizontal; SecCartInput se queda debajo.
        const string k_QueueSection = "TabContainer/Panel_trayectorias/RightPanel/Content/SecCoord";
        const string k_QueueHost = "TabContainer/Panel_trayectorias/CenterBottom";
        const string k_QueueRow = "JogRow";
        const string k_QueueColumn = "JogColumn";
        const string k_CartInputSection = "SecCartInput";

        // El panel derecho de la pestaña de trayectorias se queda sin nada propio: la
        // cola de coordenadas se va con los sliders y lo único que le quedaba, su
        // SecTraj, es un duplicado del que ya flota al frente. La aplicación usa un solo
        // panel para las trayectorias, así que este se apaga entero.
        const string k_TrajectorySidePanel = "TabContainer/Panel_trayectorias/RightPanel";

        // Objeto que hospeda el teclado virtual (VrKeyboard, en Assets/Scripts).
        const string k_KeyboardObject = "VR Keyboard";
        const float k_QueueSpacing = 8f;        // px entre las piezas de la fila

        // El panel lateral se pega al muro de la derecha del operador —en AN5_sim se
        // llama "wall Back", en x 2.5— en vez de flotar en el arco. Conserva la escala
        // que tenía allí, así que sigue midiendo 0.36 x 1.23 m.
        //
        // Va bastante atrás en z para que Panel_trayectorias, que llega hasta 71° de
        // azimut y está mucho más cerca, no se le ponga por delante.
        /// Una ventana pegada a un muro lateral. La dirección dice a cuál: es la que se
        /// usa para buscarlo y también hacia donde apunta la cara de atrás del canvas.
        ///
        /// Los dos muros laterales de la sala son perpendiculares a X, así que el reparto
        /// a lo largo de cada uno se da en Z.
        readonly struct SideWallWindowSpec
        {
            public readonly string Path;
            public readonly Vector3 Direction;    // +X = muro de la derecha, -X = el de la izquierda
            public readonly float AlongZ;         // m, a lo largo del muro
            public readonly float Height;         // m, altura del centro
            public readonly float PixelToMeter;

            public SideWallWindowSpec(string path, Vector3 direction, float alongZ, float height, float pixelToMeter)
            {
                Path = path;
                Direction = direction;
                AlongZ = alongZ;
                Height = height;
                PixelToMeter = pixelToMeter;
            }
        }

        static readonly SideWallWindowSpec[] k_SideWallWindows =
        {
            // El panel lateral, contra el muro de la derecha (en AN5_sim, "wall Back").
            //
            // Su escala está puesta para que se lea igual de grande que las lecturas de
            // la pared del fondo, que es lo que se mira a su lado. Lo que iguala la
            // legibilidad no es el tamaño sino el ángulo que abarca cada píxel: las
            // lecturas van a 0.0034 m/px pero a 4.10 m, y este muro está a 2.51, así que
            // 0.0034 * 2.51 / 4.10 deja a los dos en 2.85 minutos de arco por píxel.
            //
            // Va bastante atrás en z a propósito: Panel_trayectorias llega hasta 74° de
            // azimut y está a la mitad de distancia, así que más adelante se le pondría
            // por delante.
            new SideWallWindowSpec("PersistentLayer/LeftPanel", Vector3.right, -1.75f, 1.45f, 0.0021f),

            // El selector de cámara, contra el de la izquierda ("wall Front"). Se queda a
            // la altura de la vista y en el mismo azimut que ocupaba flotando, -65°, que
            // sobre este muro cae en z -0.84.
            new SideWallWindowSpec("PersistentLayer/SecCamara", Vector3.left, -0.84f, 1.72f, 0.0035f),
        };

        const float k_WallGap = 0.15f;         // m entre pantallas
        const float k_WallCenterX = 0f;        // m, centro del conjunto a lo largo de la pared
        // Alto: cuanto más grandes son las pantallas más hay que subirlas, porque el
        // robot se cruza por delante de su borde inferior en cuanto levanta el brazo
        // (llega a y 1.40). A este tamaño, 2.30 es lo más bajo que admiten.
        const float k_WallCenterY = 2.30f;     // m, altura del centro de las pantallas
        const float k_WallClearance = 0.01f;   // m por delante del muro, para no rifar el z-buffer

        [MenuItem("AN5/Reconstruir Quest_test_arnes desde AN5_sim")]
        public static void Build()
        {
            BuildInternal();
        }

        /// Punto de entrada para -executeMethod en batch mode.
        public static void BuildBatch()
        {
            try
            {
                BuildInternal();
                EditorApplication.Exit(0);
            }
            catch (Exception e)
            {
                Debug.LogError("[QuestSceneBuilder] Falló la reconstrucción: " + e);
                EditorApplication.Exit(1);
            }
        }

        static void BuildInternal()
        {
            var source = EditorSceneManager.OpenScene(k_SourceScene, OpenSceneMode.Single);
            if (!EditorSceneManager.SaveScene(source, k_TargetScene, true))
                throw new Exception("No se pudo copiar " + k_SourceScene + " a " + k_TargetScene);

            var scene = EditorSceneManager.OpenScene(k_TargetScene, OpenSceneMode.Single);

            var rig = SetUpRig(scene);
            var xrCamera = rig.GetComponentInChildren<Camera>(true);
            if (xrCamera == null)
                throw new Exception("El prefab del XR Origin no trae cámara.");

            EnsureFloor(scene);
            // DisableDesktopOnly va antes de LayOutWindows: esta última termina
            // apagando PersistentLayer entero si no le queda nada activo adentro (ver
            // DeactivateIfEmpty), y para que esa cuenta salga bien primero tiene que
            // haber apagado ya todo lo que solo servía en escritorio -- Header y Footer
            // incluidos.
            DisableDesktopOnly(scene);
            LayOutWindows(scene, xrCamera);
            SetUpEventSystem(scene);
            SetUpKeyboard(scene);
            SetUpSimulator(scene);

            EditorSceneManager.MarkSceneDirty(scene);
            if (!EditorSceneManager.SaveScene(scene))
                throw new Exception("No se pudo guardar " + k_TargetScene);

            AddToBuildSettings();
            AssetDatabase.SaveAssets();
            Debug.Log("[QuestSceneBuilder] " + k_TargetScene + " reconstruida desde " + k_SourceScene + ".");
        }

        // -----------------------------------------------------------------
        // XR Origin, locomoción y gestor de interacción
        // -----------------------------------------------------------------
        static GameObject SetUpRig(Scene scene)
        {
            if (UnityEngine.Object.FindObjectsByType<XRInteractionManager>(FindObjectsInactive.Include).Length == 0)
            {
                var manager = new GameObject("XR Interaction Manager", typeof(XRInteractionManager));
                SceneManager.MoveGameObjectToScene(manager, scene);
            }

            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(k_RigPrefab);
            if (prefab == null)
                throw new Exception("No se encontró el prefab del rig en " + k_RigPrefab);

            var rig = (GameObject)PrefabUtility.InstantiatePrefab(prefab, scene);
            rig.transform.SetPositionAndRotation(k_UserStart, Quaternion.identity);

            ForceContinuousLocomotion(rig);
            return rig;
        }

        /// Deja el rig en locomoción continua: stick izquierdo para desplazarse y
        /// derecho para girar, sin teleport.
        ///
        /// Los dos interruptores viven en ControllerInputActionManager (Starter
        /// Assets), que es quien decide en tiempo de ejecución si el stick mueve o
        /// dispara el rayo de teleport, y habilita o deshabilita los interactores
        /// correspondientes. Por eso se tocan esos flags y no los proveedores de
        /// locomoción directamente: dejar el TeleportationProvider en su sitio pero
        /// sin nadie que lo dispare mantiene el cambio reversible desde el inspector.
        ///
        /// Se escribe por SerializedObject y no por la propiedad pública porque en
        /// modo edición hay que registrar la modificación sobre la instancia del
        /// prefab para que quede serializada en la escena.
        static void ForceContinuousLocomotion(GameObject rig)
        {
            var touched = 0;
            foreach (var behaviour in rig.GetComponentsInChildren<MonoBehaviour>(true))
            {
                if (behaviour == null || behaviour.GetType().Name != "ControllerInputActionManager")
                    continue;

                var so = new SerializedObject(behaviour);
                var smoothMotion = so.FindProperty("m_SmoothMotionEnabled");
                var smoothTurn = so.FindProperty("m_SmoothTurnEnabled");
                if (smoothMotion != null) smoothMotion.boolValue = true;
                if (smoothTurn != null) smoothTurn.boolValue = true;
                so.ApplyModifiedPropertiesWithoutUndo();
                touched++;
            }

            if (touched == 0)
                Debug.LogWarning("[QuestSceneBuilder] No se encontró ControllerInputActionManager: " +
                                 "revisa a mano que la locomoción quede continua y no en teleport.");
        }

        // -----------------------------------------------------------------
        // Suelo
        // -----------------------------------------------------------------
        /// AN5_sim deja el objeto Floor desactivado: la cámara de escritorio apunta
        /// siempre al robot y nunca mira al piso, así que no hace falta. En VR sí, y
        /// no solo por lo que se ve: sin collider bajo los pies el CharacterController
        /// del rig se cae al vacío en cuanto la gravedad lo empuja.
        static void EnsureFloor(Scene scene)
        {
            var floor = FindRoot(scene, "Floor");
            if (floor == null)
            {
                Debug.LogWarning("[QuestSceneBuilder] No hay objeto Floor: el rig se caerá al vacío.");
                return;
            }

            floor.SetActive(true);

            // El plano de Unity mide 10x10 unidades a escala 1. La sala, medida sobre
            // los colliders de las paredes, va de x -2.5..2.5 y z -3..2, así que un
            // 5x5 centrado en z -0.5 la cubre exactamente. Con la escala que traía de
            // AN5_sim quedaban destapadas dos franjas contra las paredes.
            floor.transform.localScale = new Vector3(0.5f, 1f, 0.5f);
            floor.transform.position = new Vector3(0f, 0f, -0.5f);

            if (floor.GetComponent<Collider>() == null)
                Debug.LogWarning("[QuestSceneBuilder] Floor no tiene collider: la locomoción continua no tendrá suelo.");

            // AN5_sim trae además un "floor" (minúscula) suelto, activo de fábrica y con
            // el material "pared" puesto por error -- no es este Floor, es una pieza
            // aparte que se queda tapando/asomando por debajo con la textura equivocada.
            // Se apaga en vez de borrarse, como el resto de lo que sobra: si algún día se
            // corrige en AN5_sim, basta con quitar esta línea.
            Deactivate(FindRoot(scene, "floor"));
        }

        // -----------------------------------------------------------------
        // Lo que solo tiene sentido en escritorio
        // -----------------------------------------------------------------
        static void DisableDesktopOnly(Scene scene)
        {
            // Las tres cámaras de escritorio: en el visor la cámara es la del XR
            // Origin. Se desactivan en vez de borrarse para que la escena siga
            // siendo comparable con AN5_sim, y porque Camera.main ignora las
            // cámaras inactivas, así que no compiten con la del rig.
            Deactivate(FindRoot(scene, "mainCamera"));
            Deactivate(FindRoot(scene, "Camera_aux1"));
            Deactivate(FindRoot(scene, "Camera_aux2"));

            // Controles que orbitan/trasladan esa cámara de escritorio y el botón de
            // pantalla completa: en VR el punto de vista se mueve caminando.
            var persistent = FindRoot(scene, "PersistentLayer");
            if (persistent != null)
            {
                Deactivate(Child(persistent, "DPad_Orbit"));
                Deactivate(Child(persistent, "DPad_Translate"));
                Deactivate(Child(persistent, "ZoomSlider"));

                // Header (logo, pastillas de estado, ModeToggle) y Footer (la barra de
                // pestañas) no tienen sitio en este puesto de VR: las pestañas conviven
                // todas activas a la vez (ver ReleaseTabSwitching) y el estado que
                // mostraba el header no tiene equivalente todavía acá. Enteros y no por
                // piezas, igual que el resto de lo que solo tenía sentido en escritorio.
                Deactivate(Child(persistent, "Header"));
                Deactivate(Child(persistent, "Footer"));
            }
        }

        // -----------------------------------------------------------------
        // UI: de una pantalla plana a ventanas sueltas alrededor del operador
        // -----------------------------------------------------------------
        /// Descompone la pantalla de la aplicación en ventanas independientes y las
        /// reparte por un arco centrado en el punto de vista del operador.
        ///
        /// En AN5_sim toda la UI es una sola superficie: los tres paneles de pestaña
        /// apilados (la app enseña uno a la vez) y encima PersistentLayer con el header,
        /// la barra de pestañas y el panel lateral. Aquí cada una de esas piezas pasa a
        /// ser un canvas World Space propio, con su sitio y su escala, y las tres
        /// pestañas conviven en vez de turnarse.
        static void LayOutWindows(Scene scene, Camera xrCamera)
        {
            NormalizeSourceCanvases(scene, xrCamera);
            ReleaseTabSwitching(scene);

            ReflowReadouts(scene);
            PairQueueWithJog(scene);
            RetireTrajectorySidePanel(scene);
            PlaceOnBackWall(scene, xrCamera);
            PlaceOnSideWalls(scene, xrCamera);

            // Mismo resguardo que ReflowReadouts/ReflowJointsAndCart: PlaceWindow y
            // MakeMovable miden bounds (ContentOffset, y de ahí boundsAnchor) para
            // cada ventana de esta lista, y para cuando llegan acá ya pasaron
            // PairQueueWithJog/RetireTrajectorySidePanel, que dejan cosas recién
            // reparentadas/apagadas sin que Unity haya terminado de asentarlas.
            Canvas.ForceUpdateCanvases();

            var pivot = k_UserStart + Vector3.up * k_EyeHeight;
            foreach (var spec in k_Windows)
            {
                var rect = FindRect(scene, spec.Path);
                if (rect == null)
                {
                    Debug.LogWarning("[QuestSceneBuilder] No se encontró la ventana " + spec.Path + ".");
                    continue;
                }

                DetachAsWindow(rect, xrCamera);
                PlaceWindow(rect, pivot, spec);
                MakeMovable(rect, spec.Path);
            }

            // Panel_monitoreo se queda desmontada: sus tres piezas —los gráficos, las
            // lecturas y el control de trayectorias— son ahora ventanas sueltas, y lo
            // único que le sobrevive es el fondo del RightPanel. Se apaga entera.
            //
            // Es seguro aunque MonitoreoActivation viva ahí: con el panel inactivo desde
            // el principio sus OnEnable/OnDisable no llegan a correr nunca, y quien pone
            // driveRobotModel en true es TrayectoriasActivation, en un panel que sí queda
            // activo.
            var tabContainer = FindRoot(scene, "TabContainer");
            if (tabContainer != null) Deactivate(Child(tabContainer, "Panel_monitoreo"));

            // TabContainer y PersistentLayer se quedan sin contenido vivo: dentro solo
            // les quedan los controles de escritorio que ya desactivó DisableDesktopOnly.
            // Se apagan para no dejar dos canvas vacíos con su raycaster en medio.
            DeactivateIfEmpty(tabContainer);
            DeactivateIfEmpty(FindRoot(scene, "PersistentLayer"));
        }

        /// Pasa los canvas raíz de AN5_sim a World Space y les fija el rect a 1920x1080.
        ///
        /// Tiene que correr antes de medir o mover nada: mientras un canvas está en
        /// Screen Space, Unity conduce su RectTransform desde la resolución del Game
        /// View, así que LeftPanel, anclado a lo alto de su padre, mediría lo que
        /// midiera la ventana del editor y no la maquetación de referencia con la que
        /// están ancladas todas las secciones de la UI.
        static void NormalizeSourceCanvases(Scene scene, Camera xrCamera)
        {
            var tabContainer = FindRoot(scene, "TabContainer");
            if (tabContainer != null)
            {
                foreach (var canvas in tabContainer.GetComponentsInChildren<Canvas>(true))
                    if (canvas.transform.parent == tabContainer.transform)
                        NormalizeCanvas(canvas);
            }

            NormalizeCanvas(FindRoot(scene, "PersistentLayer")?.GetComponent<Canvas>());
            NormalizeCanvas(FindRoot(scene, "LoadingOverlay")?.GetComponent<Canvas>());

            void NormalizeCanvas(Canvas canvas)
            {
                if (canvas == null) return;
                canvas.renderMode = RenderMode.WorldSpace;
                canvas.worldCamera = xrCamera;

                var rect = (RectTransform)canvas.transform;
                var half = new Vector2(0.5f, 0.5f);
                rect.anchorMin = half;
                rect.anchorMax = half;
                rect.pivot = half;
                rect.sizeDelta = new Vector2(k_CanvasWidthPx, k_CanvasHeightPx);
                rect.localScale = Vector3.one;
            }
        }

        /// Deja que las tres pestañas se vean a la vez.
        ///
        /// TabController (que vive en el Footer) es quien apaga en Start todos los
        /// paneles menos el activo -- pero el Footer ya se desactivó entero en
        /// DisableDesktopOnly (son inútiles en este puesto de VR, ver ese comentario),
        /// así que su Start() ya ni corre y no hace falta tocarlo acá para nada: alcanza
        /// con activar los paneles a mano.
        ///
        /// Los dos paneles vivos pueden estar activos a la vez sin pelearse:
        /// MonitoreoActivation y TrayectoriasActivation solo comparten driveRobotModel y
        /// los dos lo quieren en true; el conflicto estaba en el OnDisable del saliente,
        /// que con TabController inerte ya no llega a ocurrir. Panel_ppal se deja como
        /// está: está retirada, su contenido duplica al de monitoreo y activarla sería
        /// un cambio de UI, no de colocación.
        static void ReleaseTabSwitching(Scene scene)
        {
            var tabContainer = FindRoot(scene, "TabContainer");
            if (tabContainer == null) return;

            Activate(Child(tabContainer, "Panel_monitoreo"));
            Activate(Child(tabContainer, "Panel_trayectorias"));
        }

        /// Saca una sección de su canvas padre y la convierte en un canvas World Space
        /// independiente, conservando el tamaño que tenía dentro de la maquetación de
        /// 1920x1080.
        static void DetachAsWindow(RectTransform rect, Camera xrCamera)
        {
            // El tamaño se mide antes de soltar el objeto: hay rects anclados a
            // estiramiento (LeftPanel, por ejemplo, a lo alto del padre), así que en
            // cuanto pierden el padre sus anclas dejan de decir nada y hay que congelar
            // lo que medían.
            LayoutRebuilder.ForceRebuildLayoutImmediate(rect);
            var size = rect.rect.size;

            rect.SetParent(null, worldPositionStays: false);

            var half = new Vector2(0.5f, 0.5f);
            rect.anchorMin = half;
            rect.anchorMax = half;
            rect.pivot = half;
            rect.sizeDelta = size;

            var canvas = rect.GetComponent<Canvas>();
            if (canvas == null) canvas = rect.gameObject.AddComponent<Canvas>();
            canvas.renderMode = RenderMode.WorldSpace;
            canvas.worldCamera = xrCamera;

            // En World Space el CanvasScaler ya no escala nada: lo único que sigue
            // pesando es la densidad con la que se generan las mallas de texto. A 1
            // px/unidad el texto pequeño de la UI se ve borroso en el visor.
            var scaler = rect.GetComponent<CanvasScaler>();
            if (scaler == null) scaler = rect.gameObject.AddComponent<CanvasScaler>();
            scaler.dynamicPixelsPerUnit = 2f;

            if (rect.GetComponent<GraphicRaycaster>() == null)
                rect.gameObject.AddComponent<GraphicRaycaster>();

            // Sin esto los rayos de los mandos no ven la UI.
            if (rect.GetComponent<TrackedDeviceGraphicRaycaster>() == null)
                rect.gameObject.AddComponent<TrackedDeviceGraphicRaycaster>();
        }

        /// Planta una ventana en el arco.
        static void PlaceWindow(RectTransform rect, Vector3 pivot, WindowSpec spec)
        {
            // Un canvas se ve desde su cara -Z, así que su forward tiene que apuntar en
            // dirección contraria al observador: hacia fuera del arco. Como la dirección
            // es horizontal, LookRotation no mete cabeceo y el plano de la ventana
            // contiene el eje vertical del mundo — que es lo que la deja perpendicular a
            // la tapa de la mesa, esté la ventana a la altura que esté.
            var direction = Quaternion.Euler(0f, spec.YawDeg, 0f) * Vector3.forward;
            var rotation = Quaternion.LookRotation(direction, Vector3.up);

            var center = pivot + direction * spec.Radius;
            center.y = spec.Height;

            rect.localScale = Vector3.one * spec.PixelToMeter;
            rect.SetPositionAndRotation(center, rotation);

            // Las pestañas siguen siendo rects de 1920x1080 con el contenido anclado
            // donde lo dejaba la maquetación de escritorio: pegado a la derecha, porque
            // la franja izquierda la ocupaba el LeftPanel, que ahora es otra ventana. Sin
            // esto la ventana quedaría centrada en su rect y el contenido visible saldría
            // descuadrado del punto donde se la ha puesto.
            var offset = ContentOffset(rect);
            rect.position -= rotation * (new Vector3(offset.x, offset.y, 0f) * spec.PixelToMeter);
        }

        /// Le pone el asa de arrastre a la ventana si es una de k_MovableWindows, para que
        /// el operador pueda recolocarla con los mandos.
        ///
        /// Va después de PlaceWindow: lo que deja aquí el constructor es de dónde parte la
        /// ventana al arrancar la aplicación, y a partir de ahí manda quien lleve el visor.
        /// El asa se monta sola en runtime (ver VrWindowGrab), así que aquí solo hay que
        /// dejar puesto el componente.
        static void MakeMovable(RectTransform rect, string path)
        {
            if (Array.IndexOf(k_MovableWindows, path) < 0) return;

            var grab = rect.GetComponent<VrWindowGrab>();
            if (grab == null) grab = rect.gameObject.AddComponent<VrWindowGrab>();

            // Panel_trayectorias no ancla el asa a toda la ventana (VrWindowGrab.
            // ContentBounds, que uniría SecCartInput con la columna de la cola de
            // coordenadas al lado -- ver PairQueueWithJog) sino solo a SecCartInput: la
            // cola crece de alto con cada punto que se agrega, y si mandara ella el asa
            // se correría de cuadro en cuadro. SecCartInput es fija, así que el asa queda
            // siempre justo debajo del panel de posición cartesiana.
            if (path == k_TrajectoriesTabWindow)
            {
                var cartInput = rect.Find("CenterBottom/JogRow/JogColumn/SecCartInput") as RectTransform;
                if (cartInput != null)
                    grab.boundsAnchor = cartInput;
                else
                    Debug.LogWarning("[QuestSceneBuilder] No se encontró SecCartInput para anclar " +
                                      "el asa de Panel_trayectorias; queda pegada a todo el contenido.");
            }
        }

        /// Centro del contenido visible de un rect, en su espacio local. Vector2.zero si
        /// no hay hijos activos que medir.
        static Vector2 ContentOffset(RectTransform rect)
        {
            return TryGetContentBounds(rect, out var bounds) ? (Vector2)bounds.center : Vector2.zero;
        }

        /// Caja de lo que un rect dibuja de verdad, en su espacio local: la unión de sus
        /// hijos activos. No es lo mismo que el rect, que en las pestañas sigue siendo el
        /// 1920x1080 de la maquetación de escritorio con zonas vacías dentro.
        static bool TryGetContentBounds(RectTransform rect, out Bounds bounds)
        {
            bounds = new Bounds();
            var any = false;

            for (var i = 0; i < rect.childCount; i++)
            {
                var child = rect.GetChild(i);
                if (!child.gameObject.activeSelf) continue;

                var childBounds = RectTransformUtility.CalculateRelativeRectTransformBounds(rect, child);
                if (!any) { bounds = childBounds; any = true; }
                else bounds.Encapsulate(childBounds);
            }

            return any;
        }

        /// Apaga el panel derecho de la pestaña de trayectorias, que se queda sin
        /// contenido propio: la cola de coordenadas se ha ido con los sliders y su
        /// SecTraj duplica el que flota al frente.
        ///
        /// Va después de PairQueueWithJog a propósito: mientras la cola siga colgando de
        /// aquí hay que poder medirla, y dentro de una rama desactivada los layout groups
        /// no corren y los tamaños se leen rancios.
        ///
        /// De paso deshace una ambigüedad que el propio SecTrajController documenta: la
        /// escena traía varios SecTraj vivos y solo uno acaba gobernando la ejecución.
        /// Apagado este, queda uno.
        static void RetireTrajectorySidePanel(Scene scene)
        {
            var sidePanel = FindRect(scene, k_TrajectorySidePanel);
            if (sidePanel == null)
            {
                Debug.LogWarning("[QuestSceneBuilder] No se encontró " + k_TrajectorySidePanel +
                                 ": la pestaña de trayectorias se queda con sus dos paneles.");
                return;
            }

            Deactivate(sidePanel.gameObject);
        }

        /// Cuelga las pantallas de la pared del fondo: las ventanas de k_WallWindows en
        /// fila, pegadas al muro despejado y mirando al operador.
        static void PlaceOnBackWall(Scene scene, Camera xrCamera)
        {
            var wall = FindFacingWall(scene);
            if (wall == null)
            {
                Debug.LogWarning("[QuestSceneBuilder] No se encontró la pared del fondo: " +
                                 "las pantallas se quedan donde estaban.");
                return;
            }

            ClearWall(wall);

            // Primero se resuelven todas las rutas y solo después se mueve nada: las dos
            // cuelgan de Panel_monitoreo, así que sacar una podría romper la ruta de la
            // otra.
            var rects = new List<RectTransform>(k_WallWindows.Length);
            var scales = new List<float>(k_WallWindows.Length);
            foreach (var spec in k_WallWindows)
            {
                var rect = FindRect(scene, spec.Path);
                if (rect == null)
                {
                    Debug.LogWarning("[QuestSceneBuilder] No se encontró la pantalla " + spec.Path + ".");
                    continue;
                }

                rects.Add(rect);
                scales.Add(spec.PixelToMeter);
            }

            if (rects.Count == 0) return;

            for (var i = 0; i < rects.Count; i++)
            {
                DetachAsWindow(rects[i], xrCamera);
                rects[i].localScale = Vector3.one * scales[i];
            }

            // Igual que en ReflowReadouts/ReflowJointsAndCart: DetachAsWindow acaba de
            // reparentar estos rects (incluido el de las lecturas, recién reflowed por
            // ReflowReadouts), y TryGetContentBounds más abajo mide sus hijos -- sin
            // esto a veces medía de más o de menos según cómo cayera la cola de layout
            // pendiente de Unity en ese instante.
            Canvas.ForceUpdateCanvases();

            // Se reparten a lo ancho por lo que dibujan de verdad, no por su rect: hay
            // rects con relleno que no dibuja nada, y alinearlas por ahí dejaría huecos
            // entre pantallas.
            var widths = new float[rects.Count];
            var total = k_WallGap * (rects.Count - 1);
            for (var i = 0; i < rects.Count; i++)
            {
                widths[i] = (TryGetContentBounds(rects[i], out var b) ? b.size.x : rects[i].rect.width)
                            * scales[i];
                total += widths[i];
            }

            // Un canvas se ve desde su cara -Z, así que su forward tiene que apuntar
            // lejos del operador: hacia +Z, que es justo la rotación identidad. Sin
            // cabeceo, o sea perpendicular a la tapa de la mesa como el resto.
            var z = wall.bounds.center.z - k_WallClearance;
            var cursor = k_WallCenterX - total * 0.5f;

            for (var i = 0; i < rects.Count; i++)
            {
                var rect = rects[i];
                rect.SetPositionAndRotation(
                    new Vector3(cursor + widths[i] * 0.5f, k_WallCenterY, z),
                    Quaternion.identity);

                var offset = ContentOffset(rect);
                rect.position -= new Vector3(offset.x, offset.y, 0f) * scales[i];

                cursor += widths[i] + k_WallGap;
            }
        }

        /// Replantea las lecturas del robot en rejilla de k_ReadoutColumns columnas:
        /// SecJoints y SecPosition dejan de ser dos filas de seis y pasan a dos filas de
        /// tres cada una.
        ///
        /// Se cambia el layout, no las cajas: cada Body pasa de HorizontalLayoutGroup a
        /// GridLayoutGroup conservando su espaciado y sus márgenes, y las celdas se
        /// recalculan para llenar el ancho nuevo. Los scripts de display no se enteran:
        /// SecJointsDisplay y SecPositionDisplay buscan sus textos por nombre relativo,
        /// así que les da igual dónde acabe cada caja.
        ///
        /// Tiene que correr antes de PlaceOnBackWall, que mide el bloque para colocarlo.
        static void ReflowReadouts(Scene scene)
        {
            var readouts = FindRect(scene, k_ReadoutWindow);
            if (readouts == null)
            {
                Debug.LogWarning("[QuestSceneBuilder] No se encontró " + k_ReadoutWindow +
                                 ": las lecturas se quedan en dos filas de seis.");
                return;
            }

            var outer = readouts.GetComponent<VerticalLayoutGroup>();
            var outerPad = outer != null ? Copy(outer.padding) : new RectOffset();
            var outerSpacing = outer != null ? outer.spacing : 0f;

            // NormalizeSourceCanvases acaba de pasar los canvas raíz a World Space --
            // eso deja pendiente en la cola de Unity un recálculo de layout que todavía
            // no corrió. Medir FirstChildSize/NaturalGridWidth contra eso leía las cajas
            // de SecPosition (no las de SecJoints, que por lo que sea ya caían resueltas)
            // a medio asentar: NaturalGridWidth devolvía 0, ToGrid se cortaba temprano
            // sin avisar con ningún warning, y SecPosition se quedaba con el
            // HorizontalLayoutGroup viejo en vez de la rejilla de 3 columnas -- la pared
            // salía con la posición cartesiana en una sola fila de seis en vez de dos de
            // tres. Mismo defecto ya visto en VrWindowGrab.Build() y
            // TrajectoryFileList.Refresh(); mismo arreglo.
            Canvas.ForceUpdateCanvases();

            // El ancho lo manda la sección que más pida puesta en tres columnas, para que
            // las dos rejillas queden alineadas entre sí. Se calculan las dos antes de
            // tocar ninguna, porque convertir una cambia el tamaño de sus cajas.
            var sectionWidth = 0f;
            foreach (var name in k_ReadoutSections)
            {
                var body = readouts.Find(name + "/Body") as RectTransform;
                if (body != null) sectionWidth = Mathf.Max(sectionWidth, NaturalGridWidth(body, k_ReadoutColumns));
            }

            if (sectionWidth <= 0f)
            {
                Debug.LogWarning("[QuestSceneBuilder] Las lecturas no tienen cajas que medir: se quedan como estaban.");
                return;
            }

            float total = outerPad.top + outerPad.bottom;
            var laidOut = 0;

            foreach (var name in k_ReadoutSections)
            {
                var section = readouts.Find(name) as RectTransform;
                var body = section != null ? section.Find("Body") as RectTransform : null;
                if (section == null || body == null) continue;

                var head = section.Find("Head") as RectTransform;
                if (head != null) FitHead(head, sectionWidth);

                var sectionHeight = ToGrid(body, sectionWidth, k_ReadoutColumns) + (head != null ? head.rect.height : 0f);
                SetSize(section, new Vector2(sectionWidth, sectionHeight));

                // El grupo de fuera no controla la altura de sus hijos, así que este
                // LayoutElement no manda; se actualiza igual para que no diga una cosa
                // distinta del rect si alguien invierte ese flag más adelante.
                var element = section.GetComponent<LayoutElement>();
                if (element != null) element.preferredHeight = sectionHeight;

                total += sectionHeight;
                laidOut++;
            }

            if (laidOut > 1) total += outerSpacing * (laidOut - 1);

            // El bloque es un canvas raíz sin ContentSizeFitter: nadie le va a ajustar el
            // rect, así que su tamaño final se escribe aquí.
            SetSize(readouts, new Vector2(sectionWidth + outerPad.left + outerPad.right, total));
            LayoutRebuilder.ForceRebuildLayoutImmediate(readouts);
        }

        // Nombres de los hijos de SecJoints/Body, en el orden en que ya están (J1..J6),
        // más "Vel" -- que ReflowJointsAndCart saca del medio para la columna de la cola.
        static readonly string[] k_JointBoxNames =
            { "Joint_BASE", "Joint_SHOULDER", "Joint_ELBOW", "Joint_WRIST 1", "Joint_WRIST 2", "Joint_WRIST 3" };
        const string k_VelBoxName = "Vel";

        // Nombres de los hijos de SecCartInput/Body, en el orden que se quiere (X,Y,Z /
        // Rx,Ry,Rz) -- en AN5_sim vienen X,Y,Z,Rz,Ry,Rx (el grupo de rotación al revés),
        // así que ReflowJointsAndCart los reordena antes de armar la rejilla.
        static readonly string[] k_CartBoxNames = { "BoxX", "BoxY", "BoxZ", "BoxRx", "BoxRy", "BoxRz" };

        /// Sube la cola de coordenadas del panel derecho de su pestaña y la pone al lado
        /// de los sliders de jog, dentro del mismo CenterBottom.
        ///
        /// Encolar una pose es leer esos sliders (SecCoordQueueController), así que las
        /// dos mitades acaban formando un solo panel:
        ///
        ///     [ J1 J2 J3 ] [ Vel   ]
        ///     [ J4 J5 J6 ] [       ]
        ///     [ X  Y  Z  ] [ Queue ]
        ///     [ Rx Ry Rz ] [       ]
        ///
        /// A la izquierda una columna con las articulaciones (replanteadas en rejilla de
        /// 3 por ReflowJointsAndCart) y, justo debajo, las entradas cartesianas, también
        /// en rejilla de 3; a la derecha Vel arriba y la cola debajo, que entre las dos
        /// son más altas que la columna de la izquierda y la acompañan de arriba abajo.
        static void PairQueueWithJog(Scene scene)
        {
            var host = FindRect(scene, k_QueueHost);
            var queue = FindRect(scene, k_QueueSection);
            var joints = host != null ? host.Find(k_JointsSection) as RectTransform : null;
            var cart = host != null ? host.Find(k_CartInputSection) as RectTransform : null;

            if (host == null || queue == null || joints == null)
            {
                Debug.LogWarning("[QuestSceneBuilder] No se encontró " + k_QueueSection + " o su destino en " +
                                 k_QueueHost + ": la cola de coordenadas se queda en el panel derecho.");
                return;
            }

            var vel = ReflowJointsAndCart(joints, cart);

            var queueSize = queue.rect.size;
            var jointsSize = joints.rect.size;
            var cartHeight = cart != null ? cart.rect.height : 0f;
            var velSize = vel != null ? vel.rect.size : Vector2.zero;

            var outer = host.GetComponent<VerticalLayoutGroup>();

            var row = new GameObject(k_QueueRow, typeof(RectTransform)).GetComponent<RectTransform>();
            row.SetParent(host, worldPositionStays: false);
            row.SetSiblingIndex(joints.GetSiblingIndex());

            var group = row.gameObject.AddComponent<HorizontalLayoutGroup>();
            group.spacing = k_QueueSpacing;
            group.childAlignment = TextAnchor.UpperLeft;
            group.childControlWidth = false;
            group.childControlHeight = false;
            group.childForceExpandWidth = false;
            group.childForceExpandHeight = false;

            var column = new GameObject(k_QueueColumn, typeof(RectTransform)).GetComponent<RectTransform>();
            column.SetParent(row, worldPositionStays: false);

            // La columna sí manda el ancho de los suyos: así SecCartInput se estrecha o
            // ensancha para caer exactamente debajo de SecJoints, columna con columna.
            var stack = column.gameObject.AddComponent<VerticalLayoutGroup>();
            stack.spacing = k_QueueSpacing;
            stack.childAlignment = TextAnchor.UpperLeft;
            stack.childControlWidth = true;
            stack.childControlHeight = false;
            stack.childForceExpandWidth = true;
            stack.childForceExpandHeight = false;

            joints.SetParent(column, worldPositionStays: false);
            if (cart != null) cart.SetParent(column, worldPositionStays: false);

            // Columna derecha: Vel arriba, la cola debajo -- mismo mecanismo que la
            // izquierda, con la cola (más ancha) mandando el ancho y Vel estirándose para
            // igualarla.
            var velColumn = new GameObject("VelQueueColumn", typeof(RectTransform)).GetComponent<RectTransform>();
            velColumn.SetParent(row, worldPositionStays: false);

            var velStack = velColumn.gameObject.AddComponent<VerticalLayoutGroup>();
            velStack.spacing = k_QueueSpacing;
            velStack.childAlignment = TextAnchor.UpperLeft;
            velStack.childControlWidth = true;
            velStack.childControlHeight = false;
            velStack.childForceExpandWidth = true;
            velStack.childForceExpandHeight = false;

            if (vel != null) vel.SetParent(velColumn, worldPositionStays: false);
            queue.SetParent(velColumn, worldPositionStays: false);

            // Al salir del Content de su pestaña la cola pierde el layout group que la
            // dimensionaba, así que se le congela el tamaño que traía.
            SetSize(queue, queueSize);

            var columnHeight = jointsSize.y + (cart != null ? k_QueueSpacing + cartHeight : 0f);
            SetSize(column, new Vector2(jointsSize.x, columnHeight));

            var velColumnWidth = Mathf.Max(velSize.x, queueSize.x);
            var velColumnHeight = velSize.y + (vel != null ? k_QueueSpacing : 0f) + queueSize.y;
            SetSize(velColumn, new Vector2(velColumnWidth, velColumnHeight));

            var rowWidth = jointsSize.x + k_QueueSpacing + velColumnWidth;
            var rowHeight = Mathf.Max(columnHeight, velColumnHeight);
            SetSize(row, new Vector2(rowWidth, rowHeight));

            // CenterBottom no se autoajusta: hay que darle a mano lo que ha crecido su
            // contenido. Antes se calculaba como host.rect.height + rowHeight - oldContent,
            // restando lo que joints+cart medían ya reflowed -- pero eso es lo mismo que
            // rowHeight (jointsSize y cartHeight se leen DESPUÉS de ReflowJointsAndCart), así
            // que la resta se cancelaba y host se quedaba con el alto viejo, heredado del
            // layout de escritorio. La fila entera (577px en la rejilla de 3 columnas) no
            // cabía en esos 408px y JogRow se salía por abajo del propio rect de la ventana,
            // arrastrando con ella a SecCartInput y al asa que cuelga de él. Como con el
            // ancho: alto absoluto, contenido más padding, nada de restar.
            var pad = outer != null ? outer.padding : null;
            var hostWidth = rowWidth + (pad != null ? pad.left + pad.right : 0);
            var hostHeight = rowHeight + (pad != null ? pad.top + pad.bottom : 0);
            var grown = hostWidth - host.rect.width;

            SetSize(host, new Vector2(hostWidth, hostHeight));

            // Y crece hacia la izquierda: por la derecha el bloque ya toca el panel
            // lateral de la pestaña, y por la izquierda sobra sitio hasta el borde.
            host.anchoredPosition -= new Vector2(grown * 0.5f, 0f);

            LayoutRebuilder.ForceRebuildLayoutImmediate(host);
        }

        /// Replantea SecJoints y SecCartInput en rejillas de k_JogColumns columnas --
        /// mismo mecanismo (ToGrid/NaturalGridWidth) que ReflowReadouts usa para las
        /// lecturas de la pared, aplicado acá a los controles interactivos de jog.
        ///
        /// SecJoints pierde a Vel de en medio antes de armar la rejilla: con J1..J6 solos
        /// (seis cajas, tres columnas) quedan exactamente dos filas parejas -- J1 J2 J3 /
        /// J4 J5 J6 -- y Vel sale por su cuenta para la columna de la cola en
        /// PairQueueWithJog. SecCartInput no pierde nada, pero en AN5_sim sus seis cajas
        /// están en el orden X,Y,Z,Rz,Ry,Rx (el grupo de rotación al revés), así que se
        /// reordenan primero a X,Y,Z,Rx,Ry,Rz -- si no, la rejilla saldría con la segunda
        /// fila invertida.
        ///
        /// El ancho de columna es uno solo, compartido por las dos rejillas -- el que más
        /// pida de las dos, igual que ReflowReadouts hace para alinear SecJoints y
        /// SecPosition en la pared. Pero un mismo ancho de CONTENEDOR no alcanza para que
        /// las CAJAS calcen entre sí: ToGrid reparte ese ancho en celdas descontando el
        /// espaciado y el margen del propio Body, y SecJoints/SecCartInput traían cada uno
        /// los suyos (los sliders, con sus botones +/-, vienen con más aire que una caja
        /// de texto sola) -- con cada Body descontando de más o de menos por su cuenta,
        /// las celdas resultantes no salían exactamente iguales aunque el ancho total sí
        /// coincidiera. Por eso el espaciado y el margen de SecJoints (el que manda el
        /// ancho, ver más abajo) se reutilizan tal cual para las dos rejillas: misma
        /// fórmula, mismas cuentas, celdas idénticas.
        ///
        /// Devuelve el RectTransform de Vel, ya desprendido de Body (null si no se
        /// encontró SecJoints/Body o Vel).
        static RectTransform ReflowJointsAndCart(RectTransform joints, RectTransform cart)
        {
            // Mismo resguardo que ReflowReadouts: sin esto, NaturalGridWidth/ToGrid
            // pueden medir estas cajas contra un layout que Unity todavía no terminó
            // de resolver desde NormalizeSourceCanvases, y la rejilla sale mal armada
            // sin ningún warning que lo delate.
            Canvas.ForceUpdateCanvases();

            var jointsBody = joints.Find("Body") as RectTransform;
            if (jointsBody == null)
            {
                Debug.LogWarning("[QuestSceneBuilder] SecJoints no tiene Body: los sliders quedan como estaban.");
                return null;
            }

            var vel = jointsBody.Find(k_VelBoxName) as RectTransform;
            if (vel != null) vel.SetParent(null, worldPositionStays: false);

            ReorderChildren(jointsBody, k_JointBoxNames);

            var cartBody = cart != null ? cart.Find("Body") as RectTransform : null;
            if (cartBody != null) ReorderChildren(cartBody, k_CartBoxNames);

            // Espaciado/margen de referencia: los de SecJoints, leídos ANTES de que
            // ToGrid los consuma (destruye el HorizontalLayoutGroup original al convertir
            // a grilla). Se usan tal cual para las dos rejillas.
            var jointsGroup = jointsBody.GetComponent<HorizontalOrVerticalLayoutGroup>();
            var spacing = jointsGroup != null ? new Vector2(jointsGroup.spacing, jointsGroup.spacing) : Vector2.zero;
            var pad = jointsGroup != null ? Copy(jointsGroup.padding) : new RectOffset();

            var width = NaturalGridWidth(jointsBody, k_JogColumns);
            if (cartBody != null) width = Mathf.Max(width, NaturalGridWidth(cartBody, k_JogColumns));

            ReflowBody(joints, jointsBody, k_JogColumns, width, spacing, pad);
            if (cart != null && cartBody != null) ReflowBody(cart, cartBody, k_JogColumns, width, spacing, pad);

            return vel;
        }

        /// Deja los hijos de `parent` en el orden de `names`, moviendo por índice de
        /// hermano -- lo mismo que arrastrarlos a mano en la jerarquía del Editor. Los que
        /// no aparecen en `names` no se tocan.
        static void ReorderChildren(Transform parent, string[] names)
        {
            for (var i = 0; i < names.Length; i++)
            {
                var child = parent.Find(names[i]);
                if (child != null) child.SetSiblingIndex(i);
            }
        }

        /// Convierte `body` en rejilla de `columns` columnas al ancho `width` (mismo para
        /// todas las que se quieran alineadas, ver ReflowJointsAndCart) y ajusta `section`
        /// (su padre directo, con el Head del título) al tamaño resultante. Cuerpo
        /// compartido por ReflowJointsAndCart para SecJoints y SecCartInput.
        ///
        /// `spacing`/`pad`: cuando se da (no null), ToGrid los usa tal cual en vez de leer
        /// los del propio Body -- así dos Body distintos reparten el mismo `width` con la
        /// misma cuenta y sus celdas salen idénticas. Null (el uso de ReflowReadouts) dice
        /// "cada uno con lo suyo", que es lo que quiere cuando no hace falta que calcen
        /// celda a celda con otra sección, solo compartir el ancho total.
        static void ReflowBody(RectTransform section, RectTransform body, int columns, float width,
                                Vector2? spacing = null, RectOffset pad = null)
        {
            if (width <= 0f)
            {
                Debug.LogWarning("[QuestSceneBuilder] " + section.name + " no tiene cajas que medir: se deja como estaba.");
                return;
            }

            var head = section.Find("Head") as RectTransform;
            if (head != null) FitHead(head, width);

            var height = ToGrid(body, width, columns, spacing, pad) + (head != null ? head.rect.height : 0f);
            SetSize(section, new Vector2(width, height));

            LayoutRebuilder.ForceRebuildLayoutImmediate(section);
        }

        /// Ancho que pide un Body puesto en `columns` columnas conservando el tamaño de
        /// caja, el espaciado y los márgenes que ya traía de escritorio.
        static float NaturalGridWidth(RectTransform body, int columns)
        {
            var cell = FirstChildSize(body);
            if (cell.x <= 0f) return 0f;

            var group = body.GetComponent<HorizontalOrVerticalLayoutGroup>();
            var spacing = group != null ? group.spacing : 0f;
            var pad = group != null ? group.padding : new RectOffset();

            return columns * cell.x + (columns - 1) * spacing + pad.left + pad.right;
        }

        /// Convierte un Body en rejilla de `columns` columnas y devuelve la altura que
        /// necesita. `spacing`/`pad` explícitos (no null) ganan sobre los del propio
        /// Body -- ver el comentario de ReflowBody sobre por qué hace falta eso para que
        /// dos Body distintos terminen con celdas idénticas.
        static float ToGrid(RectTransform body, float width, int columns,
                             Vector2? spacing = null, RectOffset pad = null)
        {
            var cell = FirstChildSize(body);
            var count = 0;
            foreach (RectTransform child in body)
                if (child.gameObject.activeSelf) count++;

            if (count == 0 || cell.y <= 0f) return body.rect.height;

            var group = body.GetComponent<HorizontalOrVerticalLayoutGroup>();
            var resolvedSpacing = spacing ?? (group != null ? new Vector2(group.spacing, group.spacing) : Vector2.zero);
            var resolvedPad = pad ?? (group != null ? Copy(group.padding) : new RectOffset());
            if (group != null) UnityEngine.Object.DestroyImmediate(group);

            var grid = body.gameObject.AddComponent<GridLayoutGroup>();
            grid.padding = resolvedPad;
            grid.spacing = resolvedSpacing;
            grid.constraint = GridLayoutGroup.Constraint.FixedColumnCount;
            grid.constraintCount = columns;
            grid.cellSize = new Vector2(
                (width - resolvedPad.left - resolvedPad.right - (columns - 1) * resolvedSpacing.x) / columns,
                cell.y);

            var rows = Mathf.CeilToInt(count / (float)columns);
            var height = rows * cell.y + (rows - 1) * resolvedSpacing.y + resolvedPad.top + resolvedPad.bottom;

            SetSize(body, new Vector2(width, height));
            return height;
        }

        /// Evita que la cabecera de una sección se salga por los lados al estrecharse.
        ///
        /// Los Head vienen maquetados para 1350 px con anchos fijos —en SecJoints, un
        /// espaciador de 356 y un "6 DOF" de 386 que centran el título— y su grupo no
        /// controla el ancho de los hijos, así que a 698 px no encogerían: se saldrían
        /// del panel. Pasarlo a childControlWidth deja que el título flexible absorba la
        /// diferencia.
        static void FitHead(RectTransform head, float width)
        {
            var group = head.GetComponent<HorizontalLayoutGroup>();
            if (group == null || group.childControlWidth) return;

            float natural = group.padding.left + group.padding.right;
            var children = 0;
            foreach (RectTransform child in head)
            {
                if (!child.gameObject.activeSelf) continue;
                natural += child.rect.width;
                children++;
            }

            if (children > 1) natural += group.spacing * (children - 1);
            if (natural <= width) return;

            group.childControlWidth = true;
        }

        static Vector2 FirstChildSize(RectTransform body)
        {
            foreach (RectTransform child in body)
                if (child.gameObject.activeSelf) return child.rect.size;

            return Vector2.zero;
        }

        static RectOffset Copy(RectOffset source)
        {
            return new RectOffset(source.left, source.right, source.top, source.bottom);
        }

        /// Fija el tamaño de un rect sea cual sea su anclaje.
        ///
        /// Escribir sizeDelta directamente no vale: en AN5_sim varias de estas piezas
        /// vienen ancladas a estiramiento, y ahí sizeDelta no es el tamaño sino la
        /// diferencia contra el padre — poner 698 en un hijo de un canvas de 1920
        /// dejaba un rect de 2618.
        static void SetSize(RectTransform rect, Vector2 size)
        {
            rect.SetSizeWithCurrentAnchors(RectTransform.Axis.Horizontal, size.x);
            rect.SetSizeWithCurrentAnchors(RectTransform.Axis.Vertical, size.y);
        }

        /// Despeja la pared: apaga lo que esté colgado de ella (los logos) para que no se
        /// mezcle con las pantallas.
        ///
        /// Se identifican por geometría —hijos de Laboratory que no son muros y que caen
        /// sobre el plano de este— y no por nombre, para que valga igual si mañana se
        /// cuelga otra cosa. Se desactivan en vez de borrarse: la escena se reconstruye
        /// desde AN5_sim en cada pasada, así que devolverlos es quitar esta llamada.
        static void ClearWall(Renderer wall)
        {
            var lab = wall.transform.parent;
            if (lab == null) return;

            foreach (Transform child in lab)
            {
                if (child == wall.transform || child.name.StartsWith("wall")) continue;

                var decal = child.GetComponentInChildren<Renderer>();
                if (decal == null) continue;

                // Sobre el plano del muro: lo que cuelga de él, no lo que hay en las otras
                // paredes de la sala.
                if (Mathf.Abs(decal.bounds.center.z - wall.bounds.center.z) > 0.1f) continue;

                child.gameObject.SetActive(false);
            }
        }

        /// Pega a los muros laterales las ventanas de k_SideWallWindows.
        static void PlaceOnSideWalls(Scene scene, Camera xrCamera)
        {
            foreach (var spec in k_SideWallWindows)
            {
                var wall = FindWall(scene, spec.Direction);
                var rect = FindRect(scene, spec.Path);
                if (wall == null || rect == null)
                {
                    Debug.LogWarning("[QuestSceneBuilder] No se encontró " + spec.Path +
                                     " o su muro: se queda donde estaba.");
                    continue;
                }

                DetachAsWindow(rect, xrCamera);
                rect.localScale = Vector3.one * spec.PixelToMeter;

                // El forward de un canvas apunta al lado contrario del observador, o sea
                // hacia dentro del muro: justo la dirección con la que se ha buscado. Es
                // un giro de 90° solo en yaw, con lo que la ventana sigue perpendicular a
                // la tapa de la mesa como todo lo demás.
                var rotation = Quaternion.LookRotation(spec.Direction, Vector3.up);

                // Restar la dirección separa la ventana del muro hacia dentro de la sala,
                // valga el muro que valga: la de la derecha se queda en x 2.49 y la de la
                // izquierda en -2.49.
                var position = wall.bounds.center - spec.Direction * k_WallClearance;
                position.y = spec.Height;
                position.z = spec.AlongZ;

                rect.SetPositionAndRotation(position, rotation);

                var offset = ContentOffset(rect);
                rect.position -= rotation * (new Vector3(offset.x, offset.y, 0f) * spec.PixelToMeter);
            }
        }

        /// El muro de la sala que queda más lejos en la dirección dada.
        ///
        /// Se busca por geometría y no por nombre a propósito: en AN5_sim los muros se
        /// llaman Front/Back/Left/Right sin ninguna relación con la orientación del
        /// puesto — el del fondo es "wall Right" y el de la derecha del operador es
        /// "wall Back"— así que ir por el nombre sería escribir a mano una coincidencia
        /// que nadie garantiza.
        static Renderer FindWall(Scene scene, Vector3 direction)
        {
            var lab = FindRoot(scene, "Laboratory");
            if (lab == null) return null;

            Renderer wall = null;
            var best = float.NegativeInfinity;

            foreach (var candidate in lab.GetComponentsInChildren<Renderer>())
            {
                if (!candidate.name.StartsWith("wall")) continue;

                var distance = Vector3.Dot(candidate.bounds.center, direction);
                if (distance <= best) continue;

                best = distance;
                wall = candidate;
            }

            return wall;
        }

        /// La pared que el operador tiene enfrente, detrás del robot: arranca en z -2
        /// mirando a +Z, luego es la de z mayor.
        static Renderer FindFacingWall(Scene scene)
        {
            return FindWall(scene, Vector3.forward);
        }

        /// Deja en la escena el teclado virtual.
        ///
        /// En escritorio los InputField se rellenan tecleando; en el visor no hay teclado
        /// y un InputField heredado bajo OpenXR pelado no levanta el del sistema de Meta,
        /// así que los campos se podían señalar con el rayo pero no escribir. VrKeyboard
        /// se monta a sí mismo en runtime — aquí solo hay que dejarlo puesto.
        static void SetUpKeyboard(Scene scene)
        {
            if (FindRoot(scene, k_KeyboardObject) != null) return;

            var keyboard = new GameObject(k_KeyboardObject, typeof(VrKeyboard));
            SceneManager.MoveGameObjectToScene(keyboard, scene);
        }

        static void SetUpEventSystem(Scene scene)
        {
            var eventSystem = FindRoot(scene, "EventSystem");
            if (eventSystem == null)
            {
                eventSystem = new GameObject("EventSystem", typeof(EventSystem));
                SceneManager.MoveGameObjectToScene(eventSystem, scene);
            }

            foreach (var module in eventSystem.GetComponents<BaseInputModule>())
                UnityEngine.Object.DestroyImmediate(module);

            eventSystem.AddComponent<XRUIInputModule>();
        }

        // -----------------------------------------------------------------
        // Simulación XR en el editor
        // -----------------------------------------------------------------
        /// El simulador solo sirve para probar en modo Play dentro del Editor sin
        /// ponerse el visor: suplanta la cabeza y los mandos con teclado y ratón. Si
        /// viaja al build de Quest sus dispositivos virtuales de XR compiten con los
        /// reales por los mismos bindings, y el resultado es la vista pegada a la
        /// cabeza y los mandos sin representar — ver EditorOnlyGameObject, que es lo
        /// que lo saca del build sin sacarlo de la escena.
        static void SetUpSimulator(Scene scene)
        {
            var existing = FindRoot(scene, "XR Interaction Simulator");
            if (existing != null)
            {
                if (existing.GetComponent<EditorOnlyGameObject>() == null)
                    existing.AddComponent<EditorOnlyGameObject>();
                return;
            }

            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(k_SimulatorPrefab);
            if (prefab == null)
            {
                Debug.LogWarning("[QuestSceneBuilder] No está importado el sample " +
                                 "'XR Interaction Simulator' de XRI; la escena queda sin simulador.");
                return;
            }

            var simulator = (GameObject)PrefabUtility.InstantiatePrefab(prefab, scene);
            simulator.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
            simulator.AddComponent<EditorOnlyGameObject>();
        }


        static void AddToBuildSettings()
        {
            var scenes = EditorBuildSettings.scenes.ToList();
            if (scenes.Any(s => s.path == k_TargetScene))
                return;

            scenes.Add(new EditorBuildSettingsScene(k_TargetScene, true));
            EditorBuildSettings.scenes = scenes.ToArray();
        }

        // -----------------------------------------------------------------
        static GameObject FindRoot(Scene scene, string name)
        {
            return scene.GetRootGameObjects().FirstOrDefault(go => go.name == name);
        }

        static GameObject Child(GameObject parent, string path)
        {
            var child = parent.transform.Find(path);
            return child != null ? child.gameObject : null;
        }

        /// Resuelve una ruta "Raiz/Hijo/Nieto" contra la escena y devuelve su
        /// RectTransform. Null si falta algún tramo o si el objeto no es de UI.
        static RectTransform FindRect(Scene scene, string path)
        {
            var slash = path.IndexOf('/');
            var rootName = slash < 0 ? path : path.Substring(0, slash);

            var root = FindRoot(scene, rootName);
            if (root == null) return null;

            var target = slash < 0 ? root : Child(root, path.Substring(slash + 1));
            return target != null ? target.transform as RectTransform : null;
        }

        static void Deactivate(GameObject go)
        {
            if (go != null) go.SetActive(false);
        }

        static void Activate(GameObject go)
        {
            if (go != null) go.SetActive(true);
        }

        /// Apaga un contenedor que ya no tiene ningún hijo activo dentro.
        static void DeactivateIfEmpty(GameObject go)
        {
            if (go == null) return;

            foreach (Transform child in go.transform)
                if (child.gameObject.activeSelf)
                    return;

            go.SetActive(false);
        }
    }
}
