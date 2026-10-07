using UnityEngine;
using UnityEngine.UI;

/// Attach to the LoadingOverlay root GameObject (its own full-screen, top-sorted
/// Canvas + GraphicRaycaster). Show()/Hide() toggle the "Panel" child, which is
/// what actually dims the screen and blocks raycasts to every other UI element
/// (tab buttons, other panels) while it's active -- used by SecTrajController to
/// cover the local IK resolution in ResolveJointTrajectory, so the
/// user can't press something else mid-load and think the app froze.
public class LoadingOverlayController : MonoBehaviour
{
    public static LoadingOverlayController Instance { get; private set; }

    [Header("UI — auto-resolved from hierarchy if null")]
    public GameObject panel;
    public Text       messageText;
    public Transform  spinner;

    [Tooltip("Degrees/second the spinner icon spins while the overlay is visible.")]
    public float spinSpeed = 220f;

    const string DefaultMessage = "Cargando trayectoria...\nCalculando posiciones (IK)";

    void Awake()
    {
        Instance = this;

        if (panel       == null) panel       = transform.Find("Panel")?.gameObject;
        if (messageText == null) messageText = transform.Find("Panel/Box/Message")?.GetComponent<Text>();
        if (spinner     == null) spinner     = transform.Find("Panel/Box/Spinner");

        panel?.SetActive(false);
    }

    void Update()
    {
        if (spinner != null && panel != null && panel.activeSelf)
            spinner.Rotate(0f, 0f, -spinSpeed * Time.deltaTime);
    }

    public void Show(string message = DefaultMessage)
    {
        if (messageText != null) messageText.text = message;
        panel?.SetActive(true);
    }

    public void Hide()
    {
        panel?.SetActive(false);
    }
}
