using System;

// Parametros DH (a, alpha, d) del FR5v6, compartidos entre LocalForwardKinematics.cs
// (cinematica directa local) y FR5AnalyticIK.cs (cinematica inversa analitica + RCM).
// Ambos deben operar sobre exactamente la misma cadena -- de lo contrario un IK
// resuelto aca no seria consistente con la FK que el resto de la app ya usa para
// mostrar/verificar posiciones cartesianas.
internal static class Fr5DhTable
{
    public static readonly double[,] Params = {
        {0, Math.PI / 2, 0.152, 0},
        {-0.425, 0, 0, 0},
        {-0.395, 0, 0, 0},
        {0, Math.PI / 2, 0.102, 0},
        {0, -Math.PI / 2, 0.102, 0},
        {0, 0, 0.267, 0}
    };

    // Limites articulares mecanicos reales (grados), desde el URDF -- misma fuente que
    // JOINT_LIMITS en ros2_ws/.../mock_cmd_server.py (ahi en radianes). NO son los mismos
    // que ControlArticular.SetSliderLimits(): esos sliders restringen el rango de jog
    // manual por seguridad/comodidad (p.ej. J2 ahi solo cubre -145..-45, un recorte de
    // 350 a 100 grados), no el limite mecanico real -- usarlos aca rechazaba soluciones
    // de IK perfectamente validas (varias trayectorias de routines/ dejaron de cargar
    // hasta corregir esto). J4/J6 exceden el rango principal de atan2 (-180,180], por
    // eso FR5AnalyticIK prueba +-360 al verificar limites en vez de asumir que toda
    // solucion cae ahi.
    public static readonly float[] JointMinDeg = { -175f, -265f, -162f, -265f, -175f, -175f };
    public static readonly float[] JointMaxDeg = {  175f,   85f,  162f,   85f,  175f,  175f };
}
