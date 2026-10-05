using UnityEngine;
using UnityEngine.UI;

public class TabController : MonoBehaviour
{
    [Header("Panels")]
    public GameObject panelPpal;
    public GameObject panelTrayectorias;
    public GameObject panelMonitoreo;

    [Header("Tab highlights (ActiveLine children)")]
    public GameObject activeLinePpal;
    public GameObject activeLineTray;
    public GameObject activeLineMon;

    [Header("Tab texts — add one entry per tab text")]
    public Text[] textsTabPpal;
    public Text[] textsTabTray;
    public Text[] textsTabMon;

    // Tema negro/gris + naranja (#F97316 acento, #7A7570 texto inactivo).
    static readonly Color HighlightColor = new Color(0.976f, 0.451f, 0.086f, 1f);
    static readonly Color NormalColor    = new Color(0.478f, 0.459f, 0.439f, 1f);

    void Start()
    {
        // "Panel principal" (index 0) is retired — its tab button and panel are
        // deactivated in the scene, so it's unreachable from the UI. Monitoreo is
        // now the default/only landing tab (it already covers the same controls
        // Panel_ppal had, plus the cartesian/rotation readouts).
        SetTab(2);
    }

    public void ShowPpal()         => SetTab(0);
    public void ShowTrayectorias() => SetTab(1);
    public void ShowMonitoreo()    => SetTab(2);

    void SetTab(int index)
    {
        // Disable every outgoing panel before enabling the incoming one. Panels like
        // MonitoreoActivation/TrayectoriasActivation toggle shared state (driveRobotModel)
        // from OnEnable/OnDisable; doing all the disables first guarantees the incoming
        // panel's OnEnable always runs last and wins, regardless of which tab was active
        // before. Enabling the new panel first (old code) let the outgoing panel's
        // OnDisable fire afterward and clobber what OnEnable had just set.
        if (index != 0) panelPpal?.SetActive(false);
        if (index != 1) panelTrayectorias?.SetActive(false);
        if (index != 2) panelMonitoreo?.SetActive(false);

        panelPpal?.SetActive(index == 0);
        panelTrayectorias?.SetActive(index == 1);
        panelMonitoreo?.SetActive(index == 2);

        activeLinePpal?.SetActive(index == 0);
        activeLineTray?.SetActive(index == 1);
        activeLineMon?.SetActive(index == 2);

        SetTextColor(textsTabPpal, index == 0);
        SetTextColor(textsTabTray, index == 1);
        SetTextColor(textsTabMon,  index == 2);
    }

    void SetTextColor(Text[] texts, bool active)
    {
        if (texts == null) return;
        foreach (var t in texts)
            if (t != null) t.color = active ? HighlightColor : NormalColor;
    }
}
