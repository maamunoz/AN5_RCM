using System.Globalization;
using UnityEngine;
using UnityEngine.UI;

/// Attached to SecCartInput in Panel_trayectorias. Lets the user type a
/// cartesian target (X/Y/Z/Rx/Ry/Rz) and, on Enter, resolves inverse
/// kinematics LOCALLY (FR5AnalyticIK, synchronous) and applies the resulting
/// joint angles to the SAME SecJoints sliders SecCoordQueueController reads
/// from -- this both updates the fr5v6 model (ControlArticular already
/// listens on those sliders) and lets the existing Add/Send queue pick up the
/// new pose without any changes to that flow.
///
/// Used to request IK from the mock over ROS (input_cartesian_position ->
/// output_joint_position, mock_cmd_server.py) with a timeout for when nothing
/// came back. That path only ever worked in simulation (the real driver has no
/// subscriber for those topics at all) and added a full network round-trip to
/// every jog edit. FR5AnalyticIK solves against the exact same DH table
/// LocalForwardKinematics already uses for FK, so it's consistent with
/// everything else in the app that reasons about cartesian poses, and works
/// identically with or without ROS connected.
///
/// Every solution FR5AnalyticIK returns is verified against its own forward
/// kinematics before being applied (so a wrong-but-silent move is impossible);
/// validated at 100% success over 200 random within-limits poses and against
/// the real waypoints in routines/*.txt (see FR5AnalyticIK's comments for the
/// theta6 bug that was found and fixed here). A target this box still reports
/// as "posición inalcanzable" means none of the 8 kinematic configurations
/// reach it without exceeding a joint limit -- a genuine robot constraint, not
/// a solver gap.
public class SecCartInputController : MonoBehaviour
{
    public SecCoordQueueController secCoordQueueController;
    public CartesianPositionSubscriber cartesianPositionSubscriber;

    public InputField xInput, yInput, zInput, rxInput, ryInput, rzInput;

    static readonly Color ErrorColor  = new Color(0.85f, 0.25f, 0.25f, 1f);
    Color[] _normalColors;

    // Mirrors CartesianStateWriterNew.UpdateInputFieldsContinuously(), which
    // used to keep these boxes tracking the robot's live cartesian position --
    // that component is now disabled (see SendROS2/CartesianStateWriterNew)
    // because its "isManualEditing" flag latched true permanently the first
    // time any box got focus and never reset, so the boxes went stale as soon
    // as this controller started making them interactive. Refresh here
    // instead, with the same 0.5s cadence and "skip while focused" rule, but
    // without the latch bug.
    const float LiveRefreshInterval = 0.5f;
    float _nextLiveRefreshTime;

    InputField[] AllInputs => new[] { xInput, yInput, zInput, rxInput, ryInput, rzInput };

    void Start()
    {
        if (xInput  == null) xInput  = transform.Find("Body/BoxX")?.GetComponentInChildren<InputField>();
        if (yInput  == null) yInput  = transform.Find("Body/BoxY")?.GetComponentInChildren<InputField>();
        if (zInput  == null) zInput  = transform.Find("Body/BoxZ")?.GetComponentInChildren<InputField>();
        if (rxInput == null) rxInput = transform.Find("Body/BoxRx")?.GetComponentInChildren<InputField>();
        if (ryInput == null) ryInput = transform.Find("Body/BoxRy")?.GetComponentInChildren<InputField>();
        if (rzInput == null) rzInput = transform.Find("Body/BoxRz")?.GetComponentInChildren<InputField>();

        if (secCoordQueueController == null)
            secCoordQueueController = FindObjectOfType<SecCoordQueueController>();
        if (cartesianPositionSubscriber == null)
            cartesianPositionSubscriber = FindObjectOfType<CartesianPositionSubscriber>();

        var inputs = AllInputs;
        _normalColors = new Color[inputs.Length];
        for (int i = 0; i < inputs.Length; i++)
        {
            var img = inputs[i]?.GetComponent<Image>();
            _normalColors[i] = img != null ? img.color : Color.white;
            inputs[i]?.onEndEdit.AddListener(OnCartesianInputChanged);
        }
    }

    void Update()
    {
        if (Time.time >= _nextLiveRefreshTime)
        {
            _nextLiveRefreshTime = Time.time + LiveRefreshInterval;
            RefreshLivePosition();
        }
    }

    // Keeps the boxes tracking the robot's actual cartesian position (from
    // current_cartesian_position) whenever the user isn't actively typing into
    // any of them -- restores the pre-existing "boxes move with the robot"
    // behavior that CartesianStateWriterNew used to provide.
    private void RefreshLivePosition()
    {
        if (cartesianPositionSubscriber == null) return;

        var inputs = AllInputs;
        foreach (var field in inputs)
            if (field != null && field.isFocused)
                return;

        float[] positions = cartesianPositionSubscriber.GetLastKnownCartesianPositions();
        if (positions == null || positions.Length != 6) return;

        for (int i = 0; i < inputs.Length; i++)
            if (inputs[i] != null)
                inputs[i].text = positions[i].ToString("F2", CultureInfo.InvariantCulture);
    }

    // Fires on Enter (or losing focus) in any of the 6 boxes. Reads all six
    // current values (not just the one edited) since IK needs the full pose.
    private void OnCartesianInputChanged(string _)
    {
        if (secCoordQueueController == null)
        {
            Debug.LogError("[SecCartInputController] SecCoordQueueController not assigned.");
            return;
        }

        ClearError();

        float x  = ParseOr(xInput, 0f);
        float y  = ParseOr(yInput, 0f);
        float z  = ParseOr(zInput, 0f);
        float rx = ParseOr(rxInput, 0f);
        float ry = ParseOr(ryInput, 0f);
        float rz = ParseOr(rzInput, 0f);

        Slider[] sliders =
        {
            secCoordQueueController.j1Slider, secCoordQueueController.j2Slider, secCoordQueueController.j3Slider,
            secCoordQueueController.j4Slider, secCoordQueueController.j5Slider, secCoordQueueController.j6Slider,
        };
        float[] currentDeg = new float[6];
        float[] sliderMin = new float[6];
        float[] sliderMax = new float[6];
        for (int i = 0; i < 6; i++)
        {
            currentDeg[i] = sliders[i] != null ? sliders[i].value : 0f;
            sliderMin[i] = sliders[i] != null ? sliders[i].minValue : float.MinValue;
            sliderMax[i] = sliders[i] != null ? sliders[i].maxValue : float.MaxValue;
        }

        // Limites del slider como restriccion adicional: si la solucion cayera fuera
        // de su rango, Slider.value la recortaria en silencio y el modelo terminaria
        // en otra pose distinta de la pedida.
        var result = FR5AnalyticIK.Solve(x, y, z, rx, ry, rz, currentDeg, sliderMin, sliderMax);
        if (!result.Success)
        {
            ShowError(result.FailureReason);
            return;
        }

        for (int i = 0; i < 6; i++)
            if (sliders[i] != null)
                sliders[i].value = result.JointsDeg[i]; // Slider.value already clamps to [minValue, maxValue].
    }

    private static float ParseOr(InputField field, float fallback)
    {
        if (field == null || string.IsNullOrEmpty(field.text))
            return fallback;
        return float.TryParse(field.text, NumberStyles.Float, CultureInfo.InvariantCulture, out float v)
            ? v : fallback;
    }

    // Failsafe feedback: tints all 6 boxes red and logs the reason, so an
    // unreachable/failed target is visible instead of the panel just doing
    // nothing. Cleared on the next edit (ClearError), called from the top of
    // OnCartesianInputChanged.
    private void ShowError(string reason)
    {
        Debug.LogWarning($"[SecCartInputController] IK fallo: {reason}");
        var inputs = AllInputs;
        for (int i = 0; i < inputs.Length; i++)
        {
            var img = inputs[i]?.GetComponent<Image>();
            if (img != null) img.color = ErrorColor;
        }
    }

    private void ClearError()
    {
        var inputs = AllInputs;
        for (int i = 0; i < inputs.Length; i++)
        {
            var img = inputs[i]?.GetComponent<Image>();
            if (img != null) img.color = _normalColors[i];
        }
    }
}
