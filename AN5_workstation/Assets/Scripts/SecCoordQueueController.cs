using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Text;
using UnityEngine;
using UnityEngine.UI;

public class SecCoordQueueController : MonoBehaviour
{
    [Header("Data Sources")]
    public Ros2CommandSender ros2CommandSender;
    public JointPositionSubscriber jointPositionSubscriber;

    [Header("SecJoints sliders (auto-resolved)")]
    public Slider j1Slider;
    public Slider j2Slider;
    public Slider j3Slider;
    public Slider j4Slider;
    public Slider j5Slider;
    public Slider j6Slider;
    public Slider velSlider;

    [Header("Speed")]
    public float speed = 10f;

    [Header("UI — auto-resolved from hierarchy if left empty")]
    public Button addButton;
    public Button clearButton;
    public Button sendButton;
    public Button saveButton;
    public Text logText;
    public ScrollRect scrollRect;

    private readonly List<float[]> _joints = new List<float[]>();
    // Speed captured at the moment each point was added (not the current slider
    // value at send time), so the whole queue no longer sends at a single,
    // last-set speed -- each point moves at whatever speed was showing when it
    // was added.
    private readonly List<float> _speeds = new List<float>();

    // Cada ADD (o el acople del fulcro) entra como un bloque: un punto suelto, o un
    // tramo de varios puntos (camino restringido por el trocar). CLEAR deshace el
    // ultimo bloque entero y el log muestra los tramos resumidos.
    private readonly List<int> _blockSizes = new List<int>();
    private readonly List<string> _blockTags = new List<string>();

    [Tooltip("Fulcro: con el fulcro activo, ADD guarda el camino restringido desde el ADD anterior, no solo la pose final.")]
    public FulcroController fulcro;

    void Start()
    {
        ResolveSliders();

        if (ros2CommandSender == null)
            ros2CommandSender = FindObjectOfType<Ros2CommandSender>();
        if (jointPositionSubscriber == null)
            jointPositionSubscriber = FindObjectOfType<JointPositionSubscriber>();
        if (fulcro == null)
            fulcro = FindObjectOfType<FulcroController>();

        if (logText    == null) logText    = transform.Find("Body/QueueLog/Viewport/Content/LogText")?.GetComponent<Text>();
        if (scrollRect == null) scrollRect = transform.Find("Body/QueueLog")?.GetComponent<ScrollRect>();
        if (addButton   == null) addButton   = transform.Find("Body/BtnRow_AddClear/Btn_Add")?.GetComponent<Button>();
        if (clearButton == null) clearButton = transform.Find("Body/BtnRow_AddClear/Btn_Clear")?.GetComponent<Button>();
        if (sendButton  == null) sendButton  = transform.Find("Body/BtnRow_Send/Btn_Send")?.GetComponent<Button>();
        if (saveButton  == null) saveButton  = transform.Find("Body/BtnRow_SaveTxt/Btn_SaveTxt")?.GetComponent<Button>();

        addButton?.onClick.AddListener(AddCurrentPosition);
        clearButton?.onClick.AddListener(ClearLastPosition);
        sendButton?.onClick.AddListener(() => StartCoroutine(SendQueue()));
        saveButton?.onClick.AddListener(SaveToTxt);

        // Keep 'speed' (used by SendQueue/SaveToTxt) in sync with the Vel slider --
        // without this it stayed stuck at its 10f default regardless of what the
        // slider showed, since nothing ever wrote back to this field.
        if (velSlider != null)
        {
            speed = velSlider.value;
            velSlider.onValueChanged.AddListener(v => speed = v);
        }
    }

    private static readonly string[] jointNames =
        { "Joint_BASE", "Joint_SHOULDER", "Joint_ELBOW", "Joint_WRIST 1", "Joint_WRIST 2", "Joint_WRIST 3" };

    private void ResolveSliders()
    {
        // Bind to the SecJoints that actually carries the jog sliders, identified by
        // having them. The scene holds three: Panel_trayectorias' has a Slider per joint
        // (SecJointsSliderSync), while Panel_monitoreo's and Panel_ppal's are read-only
        // readouts (SecJointsDisplay, no "S" child, and their boxes aren't even named the
        // same). This used to sweep for the first *active* SecJoints, which worked only
        // because the tabs took turns: exactly one was ever active. In the Quest scene
        // every tab is visible at once, so that sweep could bind these controls to a
        // readout that has no sliders at all. What this needs is the sliders, so that is
        // what it looks for.
        Transform body = null;
        foreach (var t in Resources.FindObjectsOfTypeAll<Transform>())
        {
            if (t.name != "SecJoints" || !t.gameObject.scene.IsValid()) continue;

            var candidate = t.Find("Body");
            if (candidate?.Find(jointNames[0] + "/S")?.GetComponent<Slider>() == null) continue;

            body = candidate;
            break;
        }
        if (body == null) { Debug.LogWarning("[SecCoordQueueController] No SecJoints with jog sliders found."); return; }

        Slider[] targets = { null, null, null, null, null, null };
        for (int i = 0; i < jointNames.Length; i++)
        {
            var s = body.Find(jointNames[i] + "/S")?.GetComponent<Slider>();
            if (s == null) Debug.LogWarning($"[SecCoordQueueController] Slider not found for {jointNames[i]}");
            targets[i] = s;
        }
        if (j1Slider == null) j1Slider = targets[0];
        if (j2Slider == null) j2Slider = targets[1];
        if (j3Slider == null) j3Slider = targets[2];
        if (j4Slider == null) j4Slider = targets[3];
        if (j5Slider == null) j5Slider = targets[4];
        if (j6Slider == null) j6Slider = targets[5];

        if (velSlider == null)
        {
            // Vel ya no cuelga de este Body: QuestSceneBuilder.ReflowJointsAndCart lo
            // saca de en medio de los joints para la rejilla de 3 columnas, y
            // PairQueueWithJog lo reparenta a VelQueueColumn, hermano de JogColumn --
            // los dos cuelgan de JogRow, tres niveles arriba de este Body
            // (Body -> SecJoints -> JogColumn -> JogRow).
            var jogRow = body.parent?.parent?.parent;
            velSlider = jogRow?.Find("VelQueueColumn/Vel/S")?.GetComponent<Slider>();
            if (velSlider == null)
                Debug.LogWarning("[SecCoordQueueController] Vel slider not found.");
        }
    }

    private void AddCurrentPosition()
    {
        Slider[] sliders = { j1Slider, j2Slider, j3Slider, j4Slider, j5Slider, j6Slider };
        for (int i = 0; i < sliders.Length; i++)
        {
            if (sliders[i] == null)
            {
                ResolveSliders();
                if (sliders[i] == null)
                {
                    Debug.LogError($"[SecCoordQueueController] J{i + 1} slider not found.");
                    return;
                }
            }
        }

        // Con el fulcro activo, entre dos ADD el instrumento siguio un camino que respeta
        // el trocar; guardar solo la pose final dejaria que la reproduccion interpole en
        // espacio articular y saque el instrumento del trocar a mitad de tramo.
        if (fulcro != null && fulcro.TryTakeConstrainedPath(out List<float[]> rcmPath) && rcmPath.Count > 0)
        {
            AppendBlock(rcmPath, "RCM");
            return;
        }

        float[] joints = new float[6];
        for (int i = 0; i < 6; i++)
            joints[i] = sliders[i].value;

        AppendBlock(new List<float[]> { joints }, null);
    }

    /// Agrega un bloque de puntos (un ADD) a la cola, todos con la velocidad actual.
    /// `tag` nombra el tramo en el log (p. ej. "ACOPLE" o "RCM"); null = punto suelto.
    public void AppendBlock(List<float[]> points, string tag)
    {
        if (points == null || points.Count == 0) return;
        foreach (var p in points)
        {
            _joints.Add((float[])p.Clone());
            _speeds.Add(speed);
        }
        _blockSizes.Add(points.Count);
        _blockTags.Add(tag);
        RefreshLog();
    }

    /// Deshace el ultimo ADD completo (un tramo entero, no un punto suelto del tramo).
    private void ClearLastPosition()
    {
        if (_joints.Count == 0) return;
        int n = _blockSizes.Count > 0 ? _blockSizes[_blockSizes.Count - 1] : 1;
        n = Mathf.Min(n, _joints.Count);
        _joints.RemoveRange(_joints.Count - n, n);
        _speeds.RemoveRange(_speeds.Count - n, n);
        string tag = null;
        if (_blockSizes.Count > 0)
        {
            tag = _blockTags[_blockTags.Count - 1];
            _blockSizes.RemoveAt(_blockSizes.Count - 1);
            _blockTags.RemoveAt(_blockTags.Count - 1);
        }
        // Un tramo del fulcro deshecho: el proximo ADD debe volver a cubrir ese camino.
        if (tag != null) fulcro?.OnQueueBlockRemoved();
        RefreshLog();
    }

    private void RefreshLog()
    {
        if (logText == null) return;

        var sb = new StringBuilder();
        int first = 0;
        for (int b = 0; b < _blockSizes.Count && first < _joints.Count; b++)
        {
            int n = _blockSizes[b];
            int last = Mathf.Min(first + n, _joints.Count) - 1;
            float[] j = _joints[last];
            float[] cart = LocalForwardKinematics.CartesianFromJointsDeg(j);
            if (n == 1 && _blockTags[b] == null)
            {
                sb.AppendLine(
                    $"P {first + 1}: {j[0]:F1}, {j[1]:F1}, {j[2]:F1}, {j[3]:F1}, {j[4]:F1}, {j[5]:F1}  Speed: {_speeds[last]:F0}");
            }
            else
            {
                // Tramo: una linea resumen con la pose final, no un punto por linea.
                sb.AppendLine($"P {first + 1}-{last + 1} {_blockTags[b] ?? "TRAMO"}: {n} puntos  Speed: {_speeds[last]:F0}");
            }
            sb.AppendLine(
                $"    : X:{cart[0]:F1}  Y:{cart[1]:F1}  Z:{cart[2]:F1}  Rx:{cart[3]:F1}  Ry:{cart[4]:F1}  Rz:{cart[5]:F1}");
            first = last + 1;
        }

        logText.text = sb.ToString();

        if (scrollRect != null)
        {
            Canvas.ForceUpdateCanvases();
            scrollRect.verticalNormalizedPosition = 0f;
        }
    }

    // Guards against a second SendQueue() coroutine starting while the first one is
    // still waiting on WaitForRobotToReachPosition: with that wait in place a single
    // send can now take many seconds, so a Send click during that window used to spin
    // up an overlapping coroutine that interleaved its ROS commands with the one still
    // running (both touching the shared _joints list and issuing SplineStart/PTP/End
    // concurrently), which on the robot showed up as only a small, partial motion.
    private bool _isSending = false;

    private IEnumerator SendQueue()
    {
        if (_isSending || _joints.Count == 0 || ros2CommandSender == null) yield break;
        _isSending = true;
        if (sendButton != null) sendButton.interactable = false;

        try
        {
            // Required: WaitForRobotToReachPosition() below polls GetLastKnownPositions(),
            // which only updates while isUpdating is true. Without this, a prior manual
            // slider edit (which correctly calls StopUpdating() to pause live sync while
            // editing) leaves position data frozen, so the wait can never detect arrival
            // and just spins for its full timeout before re-enabling the Send button.
            if (jointPositionSubscriber != null)
                jointPositionSubscriber.StartUpdating();

            // Mientras se ejecuta la trayectoria, refrescar el modelo fr5 desde ROS2 cada 100ms
            if (jointPositionSubscriber != null)
                jointPositionSubscriber.StartLiveTracking(0.1f);

            ros2CommandSender.SendCommand("DragTeachSwitch(0)");
            yield return new WaitForSeconds(0.05f);
            ros2CommandSender.SendCommand("StopMotion()");
            yield return new WaitForSeconds(0.05f);
            ros2CommandSender.SendCommand("ResetAllError()");
            yield return new WaitForSeconds(0.05f);
            ros2CommandSender.SendCommand("StartJOG(0,6,0,100)");
            yield return new WaitForSeconds(0.5f);
            ros2CommandSender.SendCommand("StartJOG(0,6,1,100)");
            yield return new WaitForSeconds(1.5f);

            int batchSize = 5;
            int batches = Mathf.CeilToInt(_joints.Count / (float)batchSize);

            for (int b = 0; b < batches; b++)
            {
                int start = b * batchSize;
                int end   = Mathf.Min(start + batchSize, _joints.Count);

                for (int i = start; i < end; i++)
                {
                    int localIdx = i - start + 1;
                    float[] j = _joints[i];
                    ros2CommandSender.SendCommand(
                        $"JNTPoint({localIdx},{j[0]},{j[1]},{j[2]},{j[3]},{j[4]},{j[5]})");
                    yield return new WaitForSeconds(0.05f);
                }

                ros2CommandSender.SendCommand("SplineStart()");
                yield return new WaitForSeconds(0.5f);

                for (int i = start; i < end; i++)
                {
                    int localIdx = i - start + 1;
                    ros2CommandSender.SendCommand($"SplinePTP(JNT{localIdx},{_speeds[i]:F0})");
                    yield return new WaitForSeconds(0.5f);
                }

                ros2CommandSender.SendCommand("SplineEnd()");
                yield return new WaitForSeconds(0.1f);
            }

            // The commands above are fired with fixed short delays that do not reflect how
            // long the real robot takes to physically reach the last waypoint (depends on
            // distance/speed). Without this wait, StopLiveTracking() below would cut the
            // fr5v6 live-tracking loop after ~1-2s while the robot keeps moving for longer,
            // making the model appear to freeze mid-motion.
            if (jointPositionSubscriber != null)
                yield return StartCoroutine(WaitForRobotToReachPosition(_joints[_joints.Count - 1], 1f));

            if (jointPositionSubscriber != null)
            {
                jointPositionSubscriber.StopLiveTracking();
                // Pause live ROS sync now that the robot has settled. ControlArticular's own
                // "isArticularModeActive" guard (which normally pauses sync while the user edits
                // a slider) only ever gets set once and is never reset back to false by this
                // panel's flow, so without this, live feedback would keep overwriting the shared
                // j1Slider..j6Slider every time a new position arrives, fighting any manual edit
                // you make while composing the next point. StartUpdating() above re-enables it
                // for the next send.
                jointPositionSubscriber.StopUpdating();
            }

            _joints.Clear();
            _speeds.Clear();
            _blockSizes.Clear();
            _blockTags.Clear();
            RefreshLog();
            Debug.Log("[SecCoordQueueController] Queue sent and cleared.");
        }
        finally
        {
            _isSending = false;
            if (sendButton != null) sendButton.interactable = true;
        }
    }

    private IEnumerator WaitForRobotToReachPosition(float[] targetPositions, float tolerance)
    {
        float timeout = 30f;
        float elapsedTime = 0f;

        while (elapsedTime < timeout)
        {
            float[] currentPositions = jointPositionSubscriber.GetLastKnownPositions();
            if (currentPositions == null || currentPositions.Length != targetPositions.Length)
            {
                yield return new WaitForSeconds(0.1f);
                elapsedTime += 0.1f;
                continue;
            }

            bool positionReached = true;
            for (int i = 0; i < targetPositions.Length; i++)
            {
                if (Mathf.Abs(currentPositions[i] - targetPositions[i]) > tolerance)
                {
                    positionReached = false;
                    break;
                }
            }

            if (positionReached)
                yield break;

            yield return new WaitForSeconds(0.1f);
            elapsedTime += 0.1f;
        }
    }

    // Writes the queue's cartesian pose (computed locally from each point's joints
    // via LocalForwardKinematics, not fetched from ROS) in the format
    // SecTrajController's loader expects: "x,y,z,rx,ry,rz,speed,delay" per point,
    // e.g. "-572.000,-177.000,302.000,90.00,45.00,0.00,15,0.000". Speed is each
    // point's own captured value (see _speeds); this panel has no delay concept,
    // so that column is always defaulted to 0.
    private void SaveToTxt()
    {
        if (_joints.Count == 0)
        {
            Debug.LogWarning("[SecCoordQueueController] Queue is empty, nothing to save.");
            return;
        }

        var sb = new StringBuilder();

        for (int i = 0; i < _joints.Count; i++)
        {
            float[] cart = LocalForwardKinematics.CartesianFromJointsDeg(_joints[i]);
            sb.AppendLine($"{cart[0]:F3},{cart[1]:F3},{cart[2]:F3},{cart[3]:F2},{cart[4]:F2},{cart[5]:F2},{_speeds[i]:F0},{0f:F3}");
        }

        string dir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "routines"));
        Directory.CreateDirectory(dir);
        string path = Path.Combine(dir, $"trajectory_{System.DateTime.Now:yyyyMMdd_HHmmss}.txt");
        File.WriteAllText(path, sb.ToString());

        Debug.Log($"[SecCoordQueueController] Saved {_joints.Count} point(s) to {path}");
    }
}
