using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;
using UnityEngine.UI;

public class SecTrajController : MonoBehaviour
{
    [Header("Data Source")]
    public Ros2CommandSender ros2CommandSender;
    public JointPositionSubscriber jointPositionSubscriber;
    public CartesianPositionSubscriber cartesianPositionSubscriber;

    [Header("Delay between commands (seconds)")]
    public float commandDelay = 0.05f;

    [Header("Joint arrival tolerance (deg)")]
    public float jointToleranceDeg = 1f;

    [Header("Arrival mode")]
    [Tooltip("When true, sends MoveJ per point and waits for current_joint_position to confirm arrival before " +
             "the next one (needs live, responsive feedback) — slower, but fully confirmed point-by-point. " +
             "When false (default), the whole file is queued via SplineStart/SplinePTP/SplineEnd (same mechanism " +
             "ControlArticular.cs already uses for joint trajectories): the robot/mock executes the queue in order " +
             "on its own, so long files run fluidly instead of paying a flat per-point delay. Either way, motion is " +
             "sent in JOINT space (JNTPoint/MoveJ/SplinePTP(JNT...)), with IK resolved by ROS/MATLAB per point " +
             "before execution starts (see ResolveJointTrajectory) -- CARTPoint/MoveL(CART...)/SplinePTP(CART...) " +
             "depend on the real robot controller's own onboard GetInverseKin, which has been failing on real " +
             "hardware, so cartesian commands are never sent directly to the robot.")]
    public bool waitForCartesianArrival = true;

    [Header("Loading overlay — shown while ResolveJointTrajectory runs (local IK, near-instant)")]
    public LoadingOverlayController loadingOverlay;

    [Header("UI — auto-resolved from hierarchy if null")]
    public Button    cargarButton;
    public InputField inputNombre;
    public Button    execButton;
    public Button    pauseButton;
    public Button    stopButton;
    public RectTransform progressFill;
    public Text      progressLabel;

    private List<(float[] cart, float speed, float delay)> _points = new List<(float[] cart, float speed, float delay)>();
    // Joint-space solution (degrees) for each entry in _points, same index, resolved
    // once at load time by ResolveJointTrajectory() -- see that method's comment.
    private List<float[]> _jointPoints = new List<float[]>();
    private Coroutine    _execCoroutine;
    private bool         _paused;
    private bool         _sendingCommands;
    private int          _execGeneration;

    private bool _fullyWired;
    private bool _resolveSucceeded;

    // The cartesian point currently being driven toward, or null while idle. Static
    // because the real robot's fr_ros2 driver never publishes /setpoint_cartesian_position
    // (only an5_mock_sim does -- see SetpointCartesianPositionSubscriber.HasReceivedSetpoint),
    // so on real hardware SecTrendGraphController had no setpoint feed at all and its
    // setpoint trace sat frozen at zero forever. This component already resolves and
    // sends each waypoint's cartesian target one at a time, so it's the natural source
    // of truth to publish it from directly instead, without touching the vendor driver.
    // A plain static (rather than an instance the graph controller holds a reference to)
    // sidesteps the scene's multiple duplicate "SecTraj" GameObjects (only one of which
    // ever actually gets its buttons wired and runs ExecutePoints; the others' EnsureWired()
    // never resolves, so they never touch this field).
    private static float[] s_liveCartesianSetpoint;

    public static float[] GetLiveCartesianSetpoint()
    {
        return s_liveCartesianSetpoint != null ? (float[])s_liveCartesianSetpoint.Clone() : null;
    }

    void Start()
    {
        EnsureWired();
        SetProgress(0f);
    }

    // FindObjectOfType<T>() depends on the target (Ros2CommandSender,
    // JointPositionSubscriber, CartesianPositionSubscriber -- all on other
    // GameObjects) already existing/active at the moment THIS Start() runs.
    // Unity does not guarantee Start()-call order across different scripts,
    // so on some sessions this raced and silently resolved everything to
    // null forever (Start() only runs once) -- the trajectory UI looked
    // wired (buttons present) but every send silently no-op'd. Retrying from
    // Update() until it succeeds removes the race: by any frame's Update(),
    // every object that will exist at startup has already had its own
    // Start() called, so the retry is guaranteed to eventually succeed.
    void Update()
    {
        if (!_fullyWired)
            EnsureWired();
    }

    private void EnsureWired()
    {
        if (ros2CommandSender == null)
            ros2CommandSender = FindObjectOfType<Ros2CommandSender>();
        if (jointPositionSubscriber == null)
            jointPositionSubscriber = FindObjectOfType<JointPositionSubscriber>();
        if (cartesianPositionSubscriber == null)
            cartesianPositionSubscriber = FindObjectOfType<CartesianPositionSubscriber>();
        if (loadingOverlay == null)
            loadingOverlay = FindObjectOfType<LoadingOverlayController>();

        if (cargarButton == null)
        {
            var body     = transform.Find("Body");
            var loadSave = body?.Find("LoadSave");
            cargarButton = loadSave?.Find("Btn_⬆  CARGAR")?.GetComponent<Button>();
        }
        if (inputNombre   == null) inputNombre   = transform.Find("Body/Input_nombre_archivo.csv")?.GetComponent<InputField>();
        if (execButton    == null) execButton    = transform.Find("Body/ExRow/Exec")?.GetComponent<Button>();
        if (pauseButton   == null) pauseButton   = transform.Find("Body/ExRow/Pause")?.GetComponent<Button>();
        if (stopButton    == null) stopButton    = transform.Find("Body/ExRow/Stop2")?.GetComponent<Button>();
        if (progressFill  == null) progressFill  = transform.Find("Body/ProgWrap/Track/Fill")?.GetComponent<RectTransform>();
        if (progressLabel == null) progressLabel = transform.Find("Body/ProgWrap/PLabel/V")?.GetComponent<Text>();

        if (!_listenersWired && (cargarButton != null || execButton != null || pauseButton != null || stopButton != null))
        {
            _listenersWired = true;
            if (cargarButton != null)
                cargarButton.onClick.AddListener(() =>
                {
                    // No hay diálogo nativo de archivos en Android/Quest (ni tendría cómo
                    // pintarse dentro de la sesión inmersiva de OpenXR aunque lo hubiera) --
                    // ver el comentario de ShowNativeFileDialog. Ahí se usa en cambio
                    // TrajectoryFileList, que cuelga al lado de esta misma ventana y se lee
                    // con el rayo de los mandos.
                    if (Application.platform == RuntimePlatform.Android)
                    {
                        Debug.Log("[SecTrajController] Btn_CARGAR clicked (Android) — toggling file list");
                        ToggleAndroidFileList();
                    }
                    else
                    {
                        Debug.Log("[SecTrajController] Btn_CARGAR clicked — starting OpenFileDialog coroutine");
                        StartCoroutine(OpenFileDialog());
                    }
                });
            execButton  ?.onClick.AddListener(OnExec);
            pauseButton ?.onClick.AddListener(OnPause);
            stopButton  ?.onClick.AddListener(OnStop);
        }

        if (cargarButton == null) Debug.LogError("[SecTrajController] Btn_CARGAR not found — check hierarchy path 'Body/LoadSave/Btn_⬆  CARGAR'");
        if (execButton   == null) Debug.LogWarning("[SecTrajController] Exec button not found");

        // cartesianPositionSubscriber is no longer required for ExecutePoints (motion is
        // sent in joint space and confirmed via jointPositionSubscriber), so it's not
        // part of this gate -- it stays as an auto-resolved field only in case some
        // other consumer of this component reads it later. IK resolution is local
        // (FR5AnalyticIK, see ResolveJointTrajectory) and needs no ROS wiring of its own.
        _fullyWired = ros2CommandSender != null && jointPositionSubscriber != null
                      && cargarButton != null && execButton != null && pauseButton != null && stopButton != null;
    }

    private bool _listenersWired;

    // Dónde viven los archivos de trayectorias. En escritorio, la carpeta "routines" al
    // lado del ejecutable (para que sea fácil de encontrar y copiar cosas a mano). En
    // Android no hay un "al lado del .apk" utilizable: persistentDataPath es la única
    // carpeta de la app que Unity puede leer/escribir sin permisos extra en cualquier
    // versión de Android, y sigue siendo alcanzable desde una PC por USB/adb sin rootear
    // el visor -- ver el aviso que registra LoadByTypedFileName cuando no encuentra nada
    // ahí, que imprime esta misma ruta resuelta.
    private static string GetRoutinesDir()
    {
        string dir = Application.platform == RuntimePlatform.Android
            ? Path.Combine(Application.persistentDataPath, "routines")
            : Path.GetFullPath(Path.Combine(Application.dataPath, "..", "routines"));

        Directory.CreateDirectory(dir);
        return dir;
    }

    /// Sustituto de OpenFileDialog para Android: sin diálogo nativo de archivos (no hay
    /// uno, y aunque lo hubiera no se pintaría dentro de la sesión inmersiva de OpenXR),
    /// esto abre/cierra TrajectoryFileList colgada al lado de esta ventana, con un botón
    /// por archivo encontrado en GetRoutinesDir() -- tocable con el rayo de los mandos.
    /// Los archivos llegan ahí copiándolos por USB con
    /// `adb push archivo.txt "<GetRoutinesDir()>"` mientras la app está corriendo; la
    /// lista se relee del disco cada vez que se abre o se toca su botón "Refrescar", así
    /// que no hace falta reinstalar nada para verlos.
    private void ToggleAndroidFileList()
    {
        var windowRoot = (RectTransform)transform;
        if (TrajectoryFileList.IsVisible(windowRoot))
        {
            TrajectoryFileList.Hide(windowRoot);
            return;
        }

        TrajectoryFileList.Show(windowRoot, GetRoutinesDir(), chosen =>
        {
            Debug.Log($"[SecTrajController] Archivo elegido de la lista: {chosen}");
            StartCoroutine(LoadTrajectoryFile(chosen));
        });
    }

    private IEnumerator OpenFileDialog()
    {
        string routinesDir = GetRoutinesDir();
        string chosen = "";
        bool   done   = false;

        var bgThread = new System.Threading.Thread(() =>
        {
            try   { chosen = ShowNativeFileDialog(routinesDir); }
            catch (System.Exception e)
            {
                UnityEngine.Debug.LogError($"[SecTrajController] File dialog error: {e.Message}\n{e.StackTrace}");
            }
            finally { done = true; }
        });
        bgThread.Start();

        while (!done) yield return null;
        Debug.Log($"[SecTrajController] Dialog done. chosen='{chosen}'");

        if (string.IsNullOrEmpty(chosen) || !File.Exists(chosen)) yield break;

        yield return StartCoroutine(LoadTrajectoryFile(chosen));
    }

    /// Loads and IK-resolves a trajectory file by path -- everything the CARGAR button
    /// does once a file has actually been chosen. Split out of OpenFileDialog, which
    /// now just picks the path and delegates here.
    private IEnumerator LoadTrajectoryFile(string chosen)
    {
        if (string.IsNullOrEmpty(chosen) || !File.Exists(chosen))
        {
            Debug.LogWarning($"[SecTrajController] Archivo inexistente: '{chosen}'");
            yield break;
        }

        // Cancel any run still in progress from a PREVIOUSLY loaded file before
        // touching _points/_jointPoints -- see StopExecutionIfRunning's comment for
        // the crash this prevents.
        StopExecutionIfRunning();

        _points.Clear();
        _jointPoints.Clear();
        int skipped = 0;
        foreach (var raw in File.ReadAllLines(chosen))
        {
            var line = raw.Trim();
            if (string.IsNullOrEmpty(line)) continue;

            if (TryParseCartesianLine(line, out float[] cart, out float speed, out float delay))
                _points.Add((cart, speed, delay));
            else
                skipped++;
        }

        if (inputNombre != null)
            inputNombre.text = Path.GetFileName(chosen);

        SetProgress(0f);
        Debug.Log($"[SecTrajController] Loaded {_points.Count} points from {chosen} ({skipped} line(s) skipped)");

        if (_points.Count > 0)
        {
            // ResolveJointTrajectory can take several seconds (each point is a round-trip
            // to the ROS/MATLAB IK node, see that method's comment) -- block the rest of
            // the UI behind a modal overlay for that stretch so the user can't press
            // another button mid-load and think the app is frozen. Hide() runs no matter
            // how the coroutine below finished (success, rejected point, or timeout).
            loadingOverlay?.Show();
            yield return StartCoroutine(ResolveJointTrajectory());
            loadingOverlay?.Hide();

            if (!_resolveSucceeded)
            {
                // All-or-nothing, mirroring the MATLAB file validator: a file with any
                // unreachable point is not loaded at all rather than executing a partial
                // or silently-approximated trajectory on the real robot.
                _points.Clear();
                _jointPoints.Clear();
                SetProgress(0f);
            }
        }
    }

    // Resolves every loaded cartesian waypoint to joint angles (degrees) locally
    // (FR5AnalyticIK) instead of the ROS/MATLAB round-trip this used before --
    // input_cartesian_position/output_joint_position only ever worked in simulation
    // anyway (the real driver has no subscriber for those topics), and resolving a
    // whole file one point at a time over that bridge meant one network round-trip
    // per waypoint. FR5AnalyticIK solves against the exact same DH table
    // LocalForwardKinematics already uses for FK, so it's consistent with everything
    // else in the app, and was validated against the real waypoints in routines/*.txt
    // (see its header comment for the theta6 bug that was found and fixed there) --
    // it's a different, verified solver from the geometric heuristic
    // (RobotKinematics.MgiAn5) tried here previously and abandoned for producing
    // unreliable positions; FR5AnalyticIK re-verifies every candidate against its own
    // forward kinematics before returning it, so a wrong-but-silent point is not
    // possible here either.
    //
    // Each point is solved relative to the previous one's joints (seeded from the
    // robot's actual current position for the first point) so the chosen kinematic
    // configuration (shoulder/elbow/wrist branch) stays continuous along the file
    // instead of jumping between equally-valid-but-different arm configurations
    // point to point.
    private IEnumerator ResolveJointTrajectory()
    {
        _resolveSucceeded = false;

        float[] currentDeg = GetSeedJointsDeg();

        for (int i = 0; i < _points.Count; i++)
        {
            float[] c = _points[i].cart;
            var result = FR5AnalyticIK.Solve(c[0], c[1], c[2], c[3], c[4], c[5], currentDeg);

            if (!result.Success)
            {
                Debug.LogError($"[SecTrajController] Punto {i + 1}/{_points.Count} rechazado: {result.FailureReason}. " +
                                "Se cancela la carga del archivo completo.");
                yield break;
            }

            _jointPoints.Add(result.JointsDeg);
            currentDeg = result.JointsDeg;

            // Resolving is local math (no network wait), but still yield periodically
            // on long files so loading one doesn't stall a frame for the whole file.
            if (i % 25 == 24) yield return null;
        }

        Debug.Log($"[SecTrajController] IK local resuelto para {_jointPoints.Count} puntos.");
        _resolveSucceeded = true;
    }

    // Starting joint pose for the first waypoint's IK solve -- the robot's actual
    // current position when available (real or sim), so the loaded trajectory
    // continues from wherever the arm already is instead of an arbitrary pose that
    // might favor a different (but equally valid) kinematic configuration than the
    // one the arm would naturally continue in.
    private float[] GetSeedJointsDeg()
    {
        float[] real = jointPositionSubscriber != null ? jointPositionSubscriber.GetLastKnownPositions() : null;
        if (real != null && real.Length == 6) return real;
        return new float[] { 0f, -90f, 90f, -90f, -90f, 0f };
    }

    // Parses a "x,y,z,rx,ry,rz,speed,delay" line, e.g.
    // "-572.000,-177.000,302.000,90.00,45.00,0.00,15,0.000". Any line that isn't
    // exactly 8 comma-separated numbers (e.g. a stray header line) is skipped.
    private static bool TryParseCartesianLine(string line, out float[] cart, out float speed, out float delay)
    {
        cart = null;
        speed = 0f;
        delay = 0f;

        var parts = line.Split(',');
        if (parts.Length != 8) return false;

        var parsed = new float[6];
        for (int i = 0; i < 6; i++)
        {
            if (!float.TryParse(parts[i].Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out parsed[i]))
                return false;
        }

        if (!float.TryParse(parts[6].Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out speed))
            return false;
        if (!float.TryParse(parts[7].Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out delay))
            return false;

        cart = parsed;
        return true;
    }

    private static string ShowNativeFileDialog(string initialDir)
    {
        if (System.Runtime.InteropServices.RuntimeInformation.IsOSPlatform(
                System.Runtime.InteropServices.OSPlatform.Windows))
            return ShowFileDialogWindows(initialDir);

        if (System.Runtime.InteropServices.RuntimeInformation.IsOSPlatform(
                System.Runtime.InteropServices.OSPlatform.Linux))
            return ShowFileDialogLinux(initialDir);

        if (System.Runtime.InteropServices.RuntimeInformation.IsOSPlatform(
                System.Runtime.InteropServices.OSPlatform.OSX))
            return ShowFileDialogMacOS(initialDir);

        return "";
    }

    private static string ShowFileDialogWindows(string initialDir)
    {
        // EnableVisualStyles lets WinForms render correctly in a redirected process.
        // OpenFileDialog has no TopMost property of its own (only Form does) -- a
        // previous version set "$d.TopMost = $true" directly on the dialog, which
        // PowerShell accepted as a non-terminating error and printed to stdout ahead
        // of the real Write-Output line. Since the C# side only read the FIRST stdout
        // line (see below), it picked up that error text instead of the chosen file
        // path on every single run, so the dialog opened and a file could be picked,
        // but nothing ever loaded (confirmed via Player.log). The fix for "appear
        // above the Unity window" is to give ShowDialog() a hidden TopMost owner Form
        // instead, which IS a valid property there.
        string safeDir = initialDir.Replace("\\", "\\\\").Replace("'", "\\'");
        string psScript =
            "[System.Windows.Forms.Application]::EnableVisualStyles();" +
            "$owner = New-Object System.Windows.Forms.Form -Property @{ TopMost = $true; ShowInTaskbar = $false; StartPosition = 'CenterScreen'; Width = 0; Height = 0 };" +
            "$d = New-Object System.Windows.Forms.OpenFileDialog;" +
            "$d.Title = 'Select trajectory file';" +
            $"$d.InitialDirectory = [System.IO.Path]::GetFullPath('{safeDir}');" +
            "$d.Filter = 'Text files (*.txt)|*.txt|All files (*.*)|*.*';" +
            "if ($d.ShowDialog($owner) -eq [System.Windows.Forms.DialogResult]::OK) { Write-Output $d.FileName }";
        string psArgs = $"-NoProfile -Sta -Command \"Add-Type -AssemblyName System.Windows.Forms; {psScript}\"";

        UnityEngine.Debug.Log($"[SecTrajController] Launching PowerShell dialog. InitialDir={initialDir}");
        var psi = new System.Diagnostics.ProcessStartInfo("powershell", psArgs)
        {
            UseShellExecute        = false,
            RedirectStandardOutput = true,
            CreateNoWindow         = false
        };
        using (var proc = System.Diagnostics.Process.Start(psi))
        {
            // Read every stdout line (not just the first -- a stray non-terminating
            // error from the script would otherwise shadow the real result the same
            // way the TopMost bug above did) and take the last non-empty one, which
            // is always the Write-Output'd file path when a file was chosen.
            string result = "";
            string line;
            while (proc != null && (line = proc.StandardOutput.ReadLine()) != null)
            {
                if (!string.IsNullOrWhiteSpace(line))
                    result = line;
            }
            proc?.WaitForExit();
            UnityEngine.Debug.Log($"[SecTrajController] PowerShell exited with code {proc?.ExitCode}. chosen='{result}'");
            return result;
        }
    }

    private static string ShowFileDialogLinux(string initialDir)
    {
        // Try zenity (GNOME/GTK), then kdialog (KDE)
        if (TryRunProcess("zenity",
                $"--file-selection --title=\"Select trajectory file\" --filename=\"{initialDir}/\"",
                out string zenityResult))
            return zenityResult;

        if (TryRunProcess("kdialog",
                $"--getopenfilename \"{initialDir}\" \"Text files (*.txt)\"",
                out string kdialogResult))
            return kdialogResult;

        UnityEngine.Debug.LogWarning(
            "[SecTrajController] No file dialog tool found. Install zenity (GNOME) or kdialog (KDE), " +
            "or type the full file path directly into the filename input field.");
        return "";
    }

    private static string ShowFileDialogMacOS(string initialDir)
    {
        string escaped = initialDir.Replace("\"", "\\\"");
        string script  = $"POSIX path of (choose file with prompt \"Select trajectory file\" default location POSIX file \"{escaped}\")";
        TryRunProcess("osascript", $"-e '{script}'", out string result);
        return result.Trim();
    }

    private static bool TryRunProcess(string exe, string args, out string output)
    {
        output = "";
        try
        {
            var psi = new System.Diagnostics.ProcessStartInfo(exe, args)
            {
                UseShellExecute        = false,
                RedirectStandardOutput = true,
                RedirectStandardError  = true,
                CreateNoWindow         = true
            };
            using (var proc = System.Diagnostics.Process.Start(psi))
            {
                if (proc == null) return false;
                output = proc.StandardOutput.ReadToEnd().Trim();
                proc.WaitForExit();
                return proc.ExitCode == 0 && !string.IsNullOrEmpty(output);
            }
        }
        catch { return false; }
    }

    private void OnExec()
    {
        if (_points.Count == 0)
        {
            Debug.LogWarning("[SecTrajController] No file loaded.");
            return;
        }
        // Guards re-entrancy only while actively sending commands (batches of
        // JNTPoint/SplinePTP), NOT for the whole lifetime of _execCoroutine.
        // The no-delay path keeps that coroutine alive well past that point,
        // through a cosmetic settle-wait (see ExecutePoints) that can run
        // many seconds for slow/large files -- blocking on _execCoroutine
        // itself here made loading and running a NEW trajectory silently
        // do nothing for as long as the previous file's settle-wait lasted.
        if (_sendingCommands) return;
        _paused          = false;
        _execGeneration++;
        _execCoroutine   = StartCoroutine(ExecutePoints(_execGeneration));
    }

    private void OnPause()
    {
        _paused = !_paused;
        Debug.Log($"[SecTrajController] {(_paused ? "Paused" : "Resumed")}");
    }

    private void OnStop()
    {
        StopExecutionIfRunning();
        Debug.Log("[SecTrajController] Stopped.");
    }

    // Cancels whatever ExecutePoints() coroutine is currently running (if any) and
    // resets all the state it owns. Shared by OnStop() and OpenFileDialog(): loading
    // a new file used to just clear/repopulate _points/_jointPoints directly while a
    // PREVIOUS file's ExecutePoints() coroutine was still mid-iteration over the old
    // list (e.g. waiting on WaitForRobotToReachJointPosition for a slow real-robot
    // move) -- that coroutine kept its own stale index into a list that just got
    // swapped out from under it, so as soon as it resumed and indexed _points[i]
    // again it could land past the end of the NEW (possibly shorter) file and throw
    // (confirmed in practice: an 81-point file got reloaded with a 17-point one
    // mid-run and crashed with ArgumentOutOfRangeException) -- and because Unity
    // coroutines killed by an uncaught exception never reach their own cleanup, that
    // also left _sendingCommands stuck true forever, silently blocking every future
    // Exec() press on that panel. Stopping any in-progress run before touching
    // _points/_jointPoints removes the race entirely: only one coroutine ever reads
    // those lists at a time.
    private void StopExecutionIfRunning()
    {
        if (_execCoroutine != null)
        {
            // Unity coroutines stopped via StopCoroutine() never run pending finally
            // blocks, so live tracking (started inside ExecutePoints) must be torn
            // down explicitly here or it would keep polling forever.
            StopCoroutine(_execCoroutine);
            _execCoroutine = null;
        }
        // Stopping mid-send would otherwise leave this stuck true forever (the
        // coroutine is killed outright, so it never reaches its own reset).
        _sendingCommands = false;
        s_liveCartesianSetpoint = null;
        if (jointPositionSubscriber != null)
        {
            jointPositionSubscriber.StopLiveTracking();
            jointPositionSubscriber.StopUpdating();
        }
        _paused = false;
        SetProgress(0f);
    }

    // Translates the loaded waypoints (already resolved to joint angles at load time
    // by ResolveJointTrajectory) into the robot's real command protocol, batched in
    // groups of 5 since the robot only exposes JNT1..JNT5 slots per batch (same limit
    // SecCoordQueueController/ControlArticular hit).
    //
    // When waitForCartesianArrival is true: JNTPoint/MoveJ per point, waiting for
    // current_joint_position to confirm arrival before the next one (mirrors
    // Record_Panel.EnviarComandosConDelay()'s delay-confirmed branch).
    //
    // When false (default): JNTPoint/SplinePTP, mirroring ControlArticular.cs's "modo
    // sin delay". SplineStart() is sent only ONCE for the whole file, not once per
    // batch — the mock (and, per the FR-series API, the real robot) clears any
    // not-yet-executed queued points on SplineStart(), so restarting it every batch
    // would silently drop the tail of the previous one before it got a chance to run.
    // JNTPoint/SplinePTP still cycle through the 5 slots per batch, but they all
    // append to the same running queue, which the robot drains in order on its own —
    // that's what keeps long files fluid instead of paying a flat delay after every point.
    private IEnumerator ExecutePoints(int myGeneration)
    {
        if (ros2CommandSender == null)
        {
            Debug.LogError("[SecTrajController] Ros2CommandSender not assigned.");
            _execCoroutine = null;
            yield break;
        }
        if (jointPositionSubscriber == null)
        {
            Debug.LogError("[SecTrajController] JointPositionSubscriber not assigned.");
            _execCoroutine = null;
            yield break;
        }
        if (_jointPoints.Count != _points.Count)
        {
            Debug.LogError("[SecTrajController] No hay solución articular para todos los puntos cargados " +
                            "(¿se canceló la carga?). Volvé a presionar CARGAR.");
            _execCoroutine = null;
            yield break;
        }

        _sendingCommands = true;

        // Drive the fr5 model from the robot's real-time joint feedback while this
        // sequence runs, same as SecCoordQueueController.SendQueue().
        if (jointPositionSubscriber != null)
        {
            jointPositionSubscriber.StartUpdating();
            jointPositionSubscriber.StartLiveTracking(0.1f);
        }

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

        if (!waitForCartesianArrival)
        {
            ros2CommandSender.SendCommand("SplineStart()");
            yield return new WaitForSeconds(commandDelay);
        }

        int batchSize = 5;
        int batches = Mathf.CeilToInt(_points.Count / (float)batchSize);

        for (int b = 0; b < batches; b++)
        {
            int start = b * batchSize;
            int end   = Mathf.Min(start + batchSize, _points.Count);

            for (int i = start; i < end; i++)
            {
                while (_paused) yield return null;

                int localIdx = i - start + 1;
                float[] q = _jointPoints[i];
                string jntCommand = $"JNTPoint({localIdx},{q[0]},{q[1]},{q[2]},{q[3]},{q[4]},{q[5]})";
                ros2CommandSender.SendCommand(jntCommand);
                Debug.Log($"[SecTrajController] ({i + 1}/{_points.Count}) {jntCommand}");
                yield return new WaitForSeconds(commandDelay);
            }

            if (waitForCartesianArrival)
            {
                for (int i = start; i < end; i++)
                {
                    while (_paused) yield return null;

                    int localIdx = i - start + 1;
                    float speed = _points[i].speed;
                    string moveCommand = $"MoveJ(JNT{localIdx},{speed:F0})";
                    ros2CommandSender.SendCommand(moveCommand);
                    Debug.Log($"[SecTrajController] ({i + 1}/{_points.Count}) {moveCommand}");
                    s_liveCartesianSetpoint = (float[])_points[i].cart.Clone();

                    yield return StartCoroutine(WaitForRobotToReachJointPosition(_jointPoints[i], jointToleranceDeg, i + 1, _points.Count));

                    float pointDelay = _points[i].delay;
                    if (pointDelay > 0f)
                        yield return new WaitForSeconds(pointDelay);

                    SetProgress((float)(i + 1) / _points.Count);
                }
            }
            else
            {
                for (int i = start; i < end; i++)
                {
                    while (_paused) yield return null;

                    int localIdx = i - start + 1;
                    float speed = _points[i].speed;
                    string splineCommand = $"SplinePTP(JNT{localIdx},{speed:F0})";
                    ros2CommandSender.SendCommand(splineCommand);
                    Debug.Log($"[SecTrajController] ({i + 1}/{_points.Count}) {splineCommand}");
                    // Fire-and-forget mode has no arrival confirmation, so this is only an
                    // approximation: it advances to each point as it's ENQUEUED (commandDelay
                    // apart), not as the robot's own queue actually reaches it, so the graph's
                    // setpoint can race ahead of the true in-progress target while this batch
                    // is being sent. Still strictly better than the alternative (no setpoint
                    // signal at all on real hardware -- see the field's own comment).
                    s_liveCartesianSetpoint = (float[])_points[i].cart.Clone();
                    yield return new WaitForSeconds(commandDelay);
                }
                SetProgress((float)end / _points.Count);
            }
        }

        if (!waitForCartesianArrival)
        {
            ros2CommandSender.SendCommand("SplineEnd()");
            yield return new WaitForSeconds(commandDelay);

            // All commands are sent; a new Exec() press is safe to start its own
            // batch from here on (see the comment on _sendingCommands in OnExec).
            // What's left below is a cosmetic wait plus this generation's own
            // cleanup, gated further down so a superseded run can't tear down a
            // newer one's live tracking.
            _sendingCommands = false;

            // The queue drains on its own after SplineEnd(); this is just so
            // "Execution complete" doesn't log (and live tracking doesn't stop)
            // while the arm is still visibly finishing the queued moves. NOT
            // gated on current_cartesian_position reaching the last point:
            // for closed/self-intersecting paths (e.g. a shape whose last point
            // equals its first, or simply re-running the same file twice in a
            // row) the arm can already BE at that cartesian position before the
            // queue has drained at all, which made this resolve almost
            // instantly and cut live tracking off seconds before the real
            // motion actually finished (confirmed against /joint_states, which
            // has no such conflict, during a fluidity/lag investigation).
            //
            // Instead, sum a per-point estimate that scales with each point's
            // OWN recorded speed, mirroring the mock's own duration formula
            // (mock_cmd_server._estimate_duration: worst-case per-joint delta
            // divided by that joint's max speed * speed_pct/100, clamped to
            // [0.2s, 8s]). Unity has no IK, so it can't know the actual joint
            // delta -- 200/speed assumes a generously large ~360 degree swing
            // at ~180 deg/s (this robot's slowest joint's max speed), which is
            // still less than the true worst case for a couple of joints whose
            // range spans ~350 degrees, so this can occasionally undershoot in
            // an extreme case, but is far safer than a flat per-point constant:
            // a real file at speed=5 needed ~33s across 5 points (individual
            // segments hit the mock's own 8s cap) while a flat 1.5s/point
            // estimate only waited 7.5s, cutting live tracking off after
            // barely the first point. At high speed this still errs long
            // (e.g. 2.5s/point at speed=80), which is the safe direction to
            // err in -- it only costs time, not correctness.
            float settleSecondsTotal = 0f;
            foreach (var pt in _points)
            {
                float speedPct = Mathf.Clamp(pt.speed, 1f, 100f);
                settleSecondsTotal += Mathf.Min(8f, 200f / speedPct);
            }
            yield return new WaitForSeconds(Mathf.Max(1f, settleSecondsTotal));
        }

        _sendingCommands = false;

        // If a newer Exec() press superseded this run while it was in the settle
        // wait above, this instance is orphaned: skip tearing down live tracking
        // (the newer run's) and don't overwrite _execCoroutine/log a completion
        // that isn't real for the run the user is actually watching -- including
        // NOT clearing s_liveCartesianSetpoint, which belongs to the newer run now.
        if (myGeneration != _execGeneration) yield break;

        s_liveCartesianSetpoint = null;

        // Only stop the periodic ApplyToModel() polling here -- StopUpdating() must
        // NOT be called on this path. Nothing re-enables it except the next Exec()
        // run (see StartUpdating() above), so calling it here left fr5 frozen and
        // deaf to every subsequent current_joint_position message -- on every tab,
        // not just during playback -- until another trajectory happened to be run.
        if (jointPositionSubscriber != null)
        {
            jointPositionSubscriber.StopLiveTracking();
        }

        _execCoroutine = null;
        Debug.Log("[SecTrajController] Execution complete.");
    }

    // pointIndex1Based/totalPoints are for diagnostics only (which waypoint this was,
    // out of how many) -- see the LogWarning below. Investigating a bug where long
    // files (>10 points) would stop advancing on real hardware with zero console
    // output: this method used to return silently on timeout, indistinguishable from
    // a genuine arrival, which is exactly what made that stall look like "the whole
    // thing just stopped" instead of "still going, just never confirming arrival."
    private IEnumerator WaitForRobotToReachJointPosition(float[] targetJointDeg, float toleranceDeg,
                                                          int pointIndex1Based, int totalPoints)
    {
        float timeout = 30f;
        float elapsedTime = 0f;
        float[] lastSeenJoint = null;

        while (elapsedTime < timeout)
        {
            float[] currentJoint = jointPositionSubscriber.GetLastKnownPositions();
            if (currentJoint == null || currentJoint.Length != targetJointDeg.Length)
            {
                yield return new WaitForSeconds(0.1f);
                elapsedTime += 0.1f;
                continue;
            }
            lastSeenJoint = currentJoint;

            bool positionReached = true;
            for (int i = 0; i < targetJointDeg.Length; i++)
            {
                if (Mathf.Abs(currentJoint[i] - targetJointDeg[i]) > toleranceDeg)
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

        string targetStr = string.Join(",", targetJointDeg);
        string lastSeenStr = lastSeenJoint != null
            ? string.Join(",", lastSeenJoint)
            : "sin datos válidos de current_joint_position durante toda la espera";
        Debug.LogWarning($"[SecTrajController] Timeout ({timeout}s) esperando llegada al punto {pointIndex1Based}/{totalPoints}. " +
                          $"Target(deg)=[{targetStr}] Último current_joint_position visto=[{lastSeenStr}]. " +
                          "Se continúa con el siguiente comando de todos modos (no se confirmó la llegada).");
    }

    private void SetProgress(float t)
    {
        if (progressFill != null)
            progressFill.anchorMax = new Vector2(t, progressFill.anchorMax.y);
        if (progressLabel != null)
            progressLabel.text = $"{Mathf.RoundToInt(t * 100)}%";
    }
}
