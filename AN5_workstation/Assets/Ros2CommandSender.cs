/*******************
Autores:    Angel Garzon Sarzosa (ahgarzon@unicauca.edu.co)
            Jhoan Simei Sarria (simei@unicauca.edu.co)                   
*******************/

using UnityEngine;
using UnityEngine.UI;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Std;
using System.Collections;
using System.Collections.Generic;

// Alias para diferenciar entre RosSharp.RosBridgeClient.MessageTypes.Std.String y System.String
using StringMsg = RosSharp.RosBridgeClient.MessageTypes.Std.String;

public class Ros2CommandSender : MonoBehaviour
{
    public RosConnector rosConnector; // Referencia al RosConnector compartido (asignar en el Inspector)
    private RosSocket rosSocket;
    private Dictionary<string, string> advertisedTopics = new Dictionary<string, string>();

    // Detección propia de reconexión: cada reconexión de RosConnector crea un RosSocket
    // nuevo, y el rosSocket capturado en WaitForConnectionAndAdvertise queda apuntando
    // al socket viejo (ya cerrado) -- Publish() sobre ese socket no lanza excepción pero
    // tampoco llega a nadie. JointPositionSubscriber/CartesianPositionSubscriber ya
    // resuelven esto mismo para las suscripciones; esto replica el mismo patrón para el
    // lado de publicación (comandos de Unity hacia el robot/mock).
    private RosSocket lastSeenSocket;
    private bool hasSeenFirstSocket = false;

    public string commandTopic = "api_command"; // Tópico para enviar comandos a la API del robot
    public string inverseInputTopic = "input_cartesian_position"; // Tópico que envía posiciones a la cinemática inversa (cartesianas)
    public string directaInputTopic = "input_joint_position"; // Tópico que envía posiciones a la cinemática directa (articulares)

    private InputField commandInputField; // Asigna este campo en el Inspector
    private Button sendCommandButton;      // Asigna este botón en el Inspector
    public Button stopCommandButton;         // Botón original para stop
    public Button duplicateStopCommandButton; // Botón duplicado para stop

    public GameObject modeManualPanel; // Panel para modo manual, asigna en el Inspector
    public GameObject modeAutoPanel;   // Panel para modo automático, asigna en el Inspector

    private bool lastManualState; // Estado anterior del panel manual
    private bool lastAutoState;   // Estado anterior del panel automático

    void Start()
    {
        // Usar el RosConnector compartido (misma conexión que usan los subscribers) en vez de abrir un socket propio
        if (rosConnector == null)
        {
            rosConnector = FindObjectOfType<RosConnector>();
        }
        if (rosConnector == null)
        {
            Debug.LogError("Ros2CommandSender: no se encontró ningún RosConnector en la escena.");
            return;
        }

        // Asignar referencias de UI si están disponibles
        if (sendCommandButton != null && commandInputField != null)
        {
            sendCommandButton.onClick.AddListener(() => SendCommand(commandInputField.text.Trim()));
        }

        // Asignar funcionalidad al botón stop original
        if (stopCommandButton != null)
        {
            stopCommandButton.onClick.AddListener(() => SendCommand("SplineEnd()"));
            stopCommandButton.onClick.AddListener(() => SendCommand("StopMotion()"));
            stopCommandButton.onClick.AddListener(() => SendCommand("ResetAllError()"));
            stopCommandButton.onClick.AddListener(() => StartCoroutine(SendJogCommandsWithDelay()));
        }

        // Asignar la misma funcionalidad al botón stop duplicado
        if (duplicateStopCommandButton != null)
        {
            duplicateStopCommandButton.onClick.AddListener(() => SendCommand("SplineEnd()"));
            duplicateStopCommandButton.onClick.AddListener(() => SendCommand("StopMotion()"));
            duplicateStopCommandButton.onClick.AddListener(() => SendCommand("ResetAllError()"));
            duplicateStopCommandButton.onClick.AddListener(() => StartCoroutine(SendJogCommandsWithDelay()));
        }

        lastManualState = modeManualPanel.activeSelf;
        lastAutoState = modeAutoPanel.activeSelf;

        StartCoroutine(WaitForConnectionAndAdvertise());
        StartCoroutine(WatchForReconnect());
    }

    // Espera a que el RosConnector compartido esté conectado antes de anunciar los tópicos
    private IEnumerator WaitForConnectionAndAdvertise()
    {
        while (!rosConnector.IsConnected.WaitOne(0))
        {
            yield return null;
        }

        rosSocket = rosConnector.RosSocket;

        rosSocket.Advertise<StringMsg>(commandTopic);
        advertisedTopics[commandTopic] = commandTopic;
        Debug.Log("Conectado y tópico anunciado: " + commandTopic);

        rosSocket.Advertise<StringMsg>(inverseInputTopic);
        advertisedTopics[inverseInputTopic] = inverseInputTopic;
        Debug.Log("Tópico anunciado: " + inverseInputTopic);

        rosSocket.Advertise<StringMsg>(directaInputTopic);
        advertisedTopics[directaInputTopic] = directaInputTopic;
        Debug.Log("Tópico anunciado: " + directaInputTopic);
    }

    // Vigila el RosSocket vigente; si cambia (reconexión), vuelve a anunciar todos los
    // tópicos ya anunciados sobre el socket nuevo. La primera conexión la maneja
    // WaitForConnectionAndAdvertise, así que aquí solo se actúa ante cambios posteriores.
    private IEnumerator WatchForReconnect()
    {
        var wait = new WaitForSeconds(0.5f);
        while (true)
        {
            yield return wait;

            if (rosConnector == null) continue;
            RosSocket currentSocket = rosConnector.RosSocket;
            if (currentSocket == null) continue;

            if (!hasSeenFirstSocket)
            {
                hasSeenFirstSocket = true;
                lastSeenSocket = currentSocket;
                continue;
            }

            if (currentSocket != lastSeenSocket)
            {
                lastSeenSocket = currentSocket;
                rosSocket = currentSocket;
                foreach (var topic in advertisedTopics.Values)
                    rosSocket.Advertise<StringMsg>(topic);
                Debug.Log("[Ros2CommandSender] RosSocket reconectado, tópicos re-anunciados: " + string.Join(", ", advertisedTopics.Values));
            }
        }
    }

    void Update()
    {
        // Detectar cambios en el estado del panel de modo manual
        if (modeManualPanel.activeSelf != lastManualState)
        {
            lastManualState = modeManualPanel.activeSelf;
            if (lastManualState) // Si el modo manual se activa
            {
                modeAutoPanel.SetActive(false);
                SendCommand("DragTeachSwitch(0)");
                Debug.Log("Modo manual activado, enviando comando: DragTeachSwitch(0)");
                SendCommand("SplineEnd()");
                SendCommand("ResetAllError()");
                SendCommand("StartJOG(0,6,0,100)");
                SendCommand("StartJOG(0,6,1,100)");
            }
        }

        // Detectar cambios en el estado del panel de modo automático
        if (modeAutoPanel.activeSelf != lastAutoState)
        {
            lastAutoState = modeAutoPanel.activeSelf;
            if (lastAutoState) // Si el modo automático se activa
            {
                modeManualPanel.SetActive(false);
                SendCommand("DragTeachSwitch(1)");
                Debug.Log("Modo automático activado, enviando comando: DragTeachSwitch(1)");
            }
        }
    }

    // Método para enviar un comando al tópico principal de comandos.
    // En modo Simulacion no hay ROS: el comando lo ejecuta LocalRobotSimulator.
    public void SendCommand(string command)
    {
        if (RobotMode.IsSimulation)
        {
            if (LocalRobotSimulator.Instance != null)
                LocalRobotSimulator.Instance.ProcessCommand(command);
            else
                Debug.LogWarning("Ros2CommandSender: modo Simulacion sin LocalRobotSimulator en la escena, comando descartado: " + command);
            return;
        }

        if (rosSocket == null)
        {
            Debug.LogWarning("Ros2CommandSender: RosSocket aún no está conectado, comando descartado: " + command);
            return;
        }
        Debug.Log("Preparando para enviar comando: " + command);
        rosSocket.Publish(commandTopic, new StringMsg { data = command });
    }

    // Método para enviar un comando a un tópico específico
    public void SendCommandToTopic(string topic, string command)
    {
        if (RobotMode.IsSimulation)
        {
            LocalRobotSimulator.Instance?.ProcessTopic(topic, command, directaInputTopic, inverseInputTopic);
            return;
        }

        if (rosSocket == null)
        {
            Debug.LogWarning("Ros2CommandSender: RosSocket aún no está conectado, comando descartado para " + topic);
            return;
        }
        if (!advertisedTopics.ContainsKey(topic))
        {
            rosSocket.Advertise<StringMsg>(topic);
            advertisedTopics[topic] = topic;
            Debug.Log("Tópico anunciado: " + topic);
        }
        Debug.Log("Preparando para enviar comando a " + topic + ": " + command);
        rosSocket.Publish(topic, new StringMsg { data = command });
    }

    // Corrutina para enviar comandos de JOG con retraso
    private IEnumerator SendJogCommandsWithDelay()
    {
        SendCommand("StartJOG(0,6,0,100)");
        yield return new WaitForSeconds(1.32f); // Retraso de 1.32 segundos
        SendCommand("StartJOG(0,6,1,100)");
    }

    void OnDestroy()
    {
        // Desanunciar los tópicos propios; NO cerrar el RosSocket porque es compartido con el RosConnector
        if (rosSocket != null)
        {
            foreach (var topic in advertisedTopics.Values)
            {
                rosSocket.Unadvertise(topic);
                Debug.Log("Tópico desanunciado: " + topic);
            }
        }
    }
}
