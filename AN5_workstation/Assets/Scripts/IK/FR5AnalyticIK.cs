using System;
using System.Collections.Generic;
using UnityEngine;

/// Cinematica inversa analitica del FR5v6 (6R desacoplado en muneca), resuelta
/// localmente en Unity -- sin pasar por el round-trip ROS/MATLAB que usan hoy
/// SecCartInputController (jog cartesiano) y el panel de trayectorias.
///
/// Portado del motor IK con restriccion RCM de
/// github.com/Juan-Sebastian-Silva/Robot-Quir-rgico-AN5- (IK_calc.cs), adaptado asi:
///   - Misma tabla DH (Fr5DhTable, identica a la que ya usaba LocalForwardKinematics.cs
///     para cinematica directa -- por eso un IK resuelto aca es consistente con la FK
///     que el resto de la app ya usa).
///   - El objetivo se recibe como (x,y,z,rx,ry,rz) en la MISMA convencion que
///     LocalForwardKinematics.CartesianFromJointsDeg ya decodifica (R = Rz*Ry*Rx,
///     posicion en mm) en vez de leer Transforms de la escena -- evita tener que
///     calibrar una conversion mundo-Unity <-> marco-DH (ver notas de la rama rcm-lfd:
///     el muneco visual del fr5v6 ya diverge de la cadena DH en un par de grados/cm,
///     divergencia preexistente y aceptada por este proyecto, cuantificada en su momento
///     por P9KinematicConsistency; replicarla via Transforms solo la hereda sin necesidad).
///   - En vez de un selector manual de las 8 soluciones (solutionID), filtra por
///     limites articulares reales (Fr5DhTable, desde el URDF -- ver su comentario) y elige
///     automaticamente la mas cercana a la pose articular actual -- para jog/RCM en vivo
///     no hay operador mirando una lista de 8 soluciones por cuadro.
///   - El derivado original traia un bug heredado: la formula de theta6 no tenia en
///     cuenta el signo de sin(theta5), asi que la mitad de las 8 ramas (las de
///     sin(theta5) negativo) encadenaban un theta6 equivocado hacia el codo/hombro/
///     muneca1 (todos se derivan de T14, que depende de theta6) y nunca alcanzaban el
///     objetivo -- quedaba tapado en el proyecto de referencia porque ahi un operador
///     elegia a ojo entre las 8 soluciones. Corregido (+pi cuando sin(theta5) < 0,
///     antes de propagarlo); validado a <0.004mm sobre 200 poses aleatorias dentro de
///     limites y sobre los archivos de routines/ (ver conversacion). Cada candidata
///     ademas se re-verifica con su propia FK antes de aceptarla, asi que una rama
///     invalida nunca puede devolver un resultado silenciosamente incorrecto.
public static class FR5AnalyticIK
{
    public struct Result
    {
        public bool Success;
        public float[] JointsDeg; // 6 valores, grados, misma convencion que los sliders SecJoints
        public string FailureReason;
    }

    /// Construye la matriz 4x4 objetivo a partir de una pose cartesiana (x,y,z en mm,
    /// rx,ry,rz en grados), exactamente la inversa de como
    /// LocalForwardKinematics.CartesianFromJointsDeg la decodifica (R = Rz(rz)*Ry(ry)*Rx(rx),
    /// posicion en metros). Misma convencion que ya usan los campos X/Y/Z/Rx/Ry/Rz del
    /// panel de jog cartesiano.
    public static Matrix4x4 BuildTargetMatrix(float xMm, float yMm, float zMm, float rxDeg, float ryDeg, float rzDeg)
    {
        float cx = Mathf.Cos(rxDeg * Mathf.Deg2Rad), sx = Mathf.Sin(rxDeg * Mathf.Deg2Rad);
        float cy = Mathf.Cos(ryDeg * Mathf.Deg2Rad), sy = Mathf.Sin(ryDeg * Mathf.Deg2Rad);
        float cz = Mathf.Cos(rzDeg * Mathf.Deg2Rad), sz = Mathf.Sin(rzDeg * Mathf.Deg2Rad);

        var m = Matrix4x4.identity;
        m[0, 0] = cz * cy;              m[0, 1] = -sz * cx + cz * sy * sx; m[0, 2] = sz * sx + cz * sy * cx;
        m[1, 0] = sz * cy;              m[1, 1] = cz * cx + sz * sy * sx;  m[1, 2] = -cz * sx + sz * sy * cx;
        m[2, 0] = -sy;                  m[2, 1] = cy * sx;                 m[2, 2] = cy * cx;

        m[0, 3] = xMm / 1000f;
        m[1, 3] = yMm / 1000f;
        m[2, 3] = zMm / 1000f;
        return m;
    }

    /// Inversa de BuildTargetMatrix: decodifica (rx,ry,rz) en grados a partir de una
    /// matriz de rotacion, exactamente igual que LocalForwardKinematics.
    /// CartesianFromJointsDeg. Usado por el constraint RCM (FulcroController) para
    /// convertir la pose objetivo (ya resuelta como matriz por geometria vectorial)
    /// de vuelta al formato (x,y,z,rx,ry,rz) que FR5AnalyticIK.Solve espera.
    public static void DecodeRxRyRz(Matrix4x4 m, out float rxDeg, out float ryDeg, out float rzDeg)
    {
        ryDeg = Mathf.Atan2(-m[2, 0], Mathf.Sqrt(m[0, 0] * m[0, 0] + m[1, 0] * m[1, 0])) * Mathf.Rad2Deg;
        rxDeg = Mathf.Atan2(m[2, 1], m[2, 2]) * Mathf.Rad2Deg;
        rzDeg = Mathf.Atan2(m[1, 0], m[0, 0]) * Mathf.Rad2Deg;
    }

    // Filtro de colision simple contra la mesa/base: ningun eslabon intermedio (codo y
    // muneca, frames DH 2..5) puede quedar mas de esto por debajo del plano de la base.
    // No aplica al TCP/herramienta, que si trabaja por debajo del plano (las poses reales
    // de routines/ bajan el TCP a -61mm y el codo a -20mm; la muneca nunca baja de
    // ~205mm). -100mm acepta todas esas poses y descarta las configuraciones que hunden
    // el codo o la muneca bajo la mesa. Es un proxy, no un modelo de colision completo.
    public const float MinLinkHeightMm = -100f;

    // Peso de cada joint al medir "cercania" a la pose actual. Base/hombro/codo pesan
    // el doble que la muneca: moverlos barre el brazo entero por el espacio (riesgo de
    // colision) y cambia su configuracion (codo arriba/abajo, hombro adelante/atras),
    // mientras que girar la muneca solo reorienta la herramienta.
    static readonly float[] JointWeights = { 2f, 2f, 2f, 1f, 1f, 1f };

    // Preferencia (suave) por wrist2 (J5) cerca de -90: penaliza cada grado de
    // desvio con este peso, ademas de la cercania a la pose actual. Sin esto, en
    // pick and place algunas soluciones elegian J5=+90 (muneca "volteada", misma pose
    // cartesiana) y al pasar al siguiente punto la pinza giraba 180 desde wrist2.
    // Medido en routines/: peso 1.0 elimina todos los cambios de signo de J5 entre
    // puntos (2 -> 0) y ademas baja el recorrido total un 15%; con 0.5 quedaban 2
    // puntos con J5>0, y pesos mayores dan el mismo resultado que 1.0.
    public const float PreferredWrist2Deg = -90f;
    public const float Wrist2PreferenceWeight = 1f;

    // Postura "grua" (brazo por encima, codo arriba): evita que el brazo baje hacia el
    // area de trabajo sobre el fulcro y choque con lo que haya ahi.
    //  - Shoulder (J2) <= ShoulderMaxDeg: limite DURO, ninguna solucion lo supera (los
    //    sliders de J2 ya cortaban en -45; esto lo aplica tambien donde se usaban los
    //    limites del URDF, hasta +85).
    //  - Elbow (J3) > 0: preferencia SUAVE, ElbowNegativeWeight por cada grado bajo 0.
    //    Proporcional y no constante: dentro de un jog continuo no empuja a saltar de
    //    rama, pero al elegir entre ramas (acoplamiento, primer punto de una
    //    trayectoria) prefiere claramente el codo arriba.
    public const float ShoulderMaxDeg = -45f;
    public const float ElbowNegativeWeight = 2f;

    /// Resuelve IK para la pose cartesiana dada y devuelve la solucion (de hasta 8
    /// candidatas) mas cercana a currentJointsDeg -- ver Solve(Matrix4x4, ...).
    public static Result Solve(float xMm, float yMm, float zMm, float rxDeg, float ryDeg, float rzDeg,
                               float[] currentJointsDeg, float[] minDegOverride = null, float[] maxDegOverride = null)
    {
        return Solve(BuildTargetMatrix(xMm, yMm, zMm, rxDeg, ryDeg, rzDeg), currentJointsDeg, minDegOverride, maxDegOverride);
    }

    /// "Good solution": de las candidatas que alcanzan el objetivo, respetan los limites
    /// articulares y pasan el filtro de colision, elige la MAS CERCANA a la pose actual:
    /// menor suma de desplazamientos articulares ponderados por JointWeights, mas una
    /// preferencia por wrist2 cerca de -90 (ver PreferredWrist2Deg) y por el codo
    /// arriba (J3 > 0); shoulder nunca pasa de -45 (postura grua, ver ShoulderMaxDeg).
    /// Elegido comparando criterios sobre las trayectorias reales de routines/ (106
    /// puntos): frente a la suma sin ponderar redujo el giro maximo de un joint en un
    /// paso de 297 a 188 grados, el barrido del brazo (J1-J3) un 22% y el recorrido
    /// total un 12%; el minimo-del-maximo bajaba algo mas el giro maximo (180) pero
    /// barria mas el brazo (+33% vs ponderado), que es lo que arriesga colisiones.
    /// Para cada joint, de sus representaciones equivalentes (+-360) dentro de limites
    /// se usa la mas cercana a la actual. minDegOverride/maxDegOverride restringen los
    /// limites (p.ej. al rango de un slider, para que la solucion nunca se recorte al
    /// escribirla ahi).
    public static Result Solve(Matrix4x4 targetMatrix, float[] currentJointsDeg,
                               float[] minDegOverride = null, float[] maxDegOverride = null)
    {
        var candidates = SolveAll(targetMatrix, currentJointsDeg, out string reason, minDegOverride, maxDegOverride);
        if (candidates.Count == 0)
            return new Result { Success = false, JointsDeg = null, FailureReason = reason };

        float bestCost = float.MaxValue;
        float[] bestDeg = null;
        foreach (var deg in candidates)
        {
            float cost = SolutionCost(deg, currentJointsDeg);
            if (bestDeg == null || cost < bestCost) { bestCost = cost; bestDeg = deg; }
        }
        return new Result { Success = true, JointsDeg = bestDeg, FailureReason = null };
    }

    /// Costo de "good solution" de Solve: desplazamiento ponderado desde la pose actual
    /// mas la preferencia por wrist2 cerca de -90. Publico para quien elija entre las
    /// candidatas de SolveAll con un criterio extra (p.ej. el acoplamiento del fulcro).
    public static float SolutionCost(float[] deg, float[] currentJointsDeg)
    {
        float cost = Wrist2PreferenceWeight * Mathf.Abs(deg[4] - PreferredWrist2Deg)
                   + ElbowNegativeWeight * Mathf.Max(0f, -deg[2]);
        if (currentJointsDeg != null && currentJointsDeg.Length == 6)
            for (int row = 0; row < 6; row++)
                cost += Mathf.Abs(deg[row] - currentJointsDeg[row]) * JointWeights[row];
        return cost;
    }

    /// Todas las candidatas (de hasta 8) que reproducen el objetivo, respetan los limites
    /// y pasan el filtro de la mesa -- sin elegir entre ellas. Cada joint usa la
    /// representacion (+-360) mas cercana a currentJointsDeg. Si no queda ninguna,
    /// `failureReason` dice por que.
    public static List<float[]> SolveAll(Matrix4x4 targetMatrix, float[] currentJointsDeg, out string failureReason,
                                         float[] minDegOverride = null, float[] maxDegOverride = null)
    {
        double[,] solutions = InverseKinematicSolutions(targetMatrix);
        bool hasCurrent = currentJointsDeg != null && currentJointsDeg.Length == 6;

        bool anyReachesTarget = false;
        bool anyWithinLimits = false;
        var valid = new List<float[]>();

        for (int col = 0; col < 8; col++)
        {
            bool isNaN = false;
            for (int row = 0; row < 6 && !isNaN; row++)
                if (double.IsNaN(solutions[row, col])) isNaN = true;
            if (isNaN) continue;

            // El derivado cerrado de 8 soluciones tiene una rama de la muneca que no
            // siempre es valida (bug heredado del proyecto de referencia -- ahi lo
            // tapaba un selector manual con un operador mirando; aca no hay operador
            // mirando cada cuadro). En vez de intentar repararlo analizando la cadena
            // theta4/theta5/theta6, se verifica cada candidata con su propia FK: la
            // que no reproduce el objetivo (en posicion Y orientacion) se descarta,
            // sin importar cual de las 8 columnas sea.
            double[,] row0to6 = RowFromColumn(solutions, col);
            Matrix4x4 verifyFk = Matrix4x4.identity;
            for (int j = 1; j <= 6; j++) verifyFk *= ComputeTransformMatrix(j, row0to6);

            float posErrM = (new Vector3(verifyFk.m03, verifyFk.m13, verifyFk.m23) -
                             new Vector3(targetMatrix.m03, targetMatrix.m13, targetMatrix.m23)).magnitude;
            float rotErrDeg = Quaternion.Angle(verifyFk.rotation, targetMatrix.rotation);
            if (posErrM > 0.002f || rotErrDeg > 1f) continue; // no reproduce el objetivo -- rama invalida
            anyReachesTarget = true;

            float[] deg = new float[6];
            bool withinLimits = true;
            for (int row = 0; row < 6; row++)
            {
                float min = Mathf.Max(Fr5DhTable.JointMinDeg[row], minDegOverride != null ? minDegOverride[row] : float.MinValue);
                float max = Mathf.Min(Fr5DhTable.JointMaxDeg[row], maxDegOverride != null ? maxDegOverride[row] : float.MaxValue);
                if (row == 1) max = Mathf.Min(max, ShoulderMaxDeg); // postura grua, ver ShoulderMaxDeg
                float raw = (float)(solutions[row, col] * Mathf.Rad2Deg);
                float reference = hasCurrent ? currentJointsDeg[row] : 0f;
                if (!TryBringWithinLimits(raw, min, max, reference, out deg[row]))
                {
                    withinLimits = false;
                    break;
                }
            }
            if (!withinLimits) continue;
            anyWithinLimits = true;

            if (!ClearsTable(deg)) continue;

            valid.Add(deg);
        }

        failureReason = null;
        if (valid.Count == 0)
        {
            if (!anyReachesTarget) failureReason = "posicion inalcanzable (fuera del espacio de trabajo o singularidad)";
            else if (!anyWithinLimits) failureReason = "sin solucion IK dentro de los limites articulares";
            else failureReason = "todas las soluciones dentro de limites llevan el codo/muneca contra la mesa";
        }
        return valid;
    }

    // true si el codo y la muneca (origenes de los frames DH 2..5) quedan por encima de
    // MinLinkHeightMm respecto del plano de la base. Ver MinLinkHeightMm.
    public static bool ClearsTable(float[] jointsDeg)
    {
        double[,] th = new double[1, 6];
        for (int i = 0; i < 6; i++) th[0, i] = jointsDeg[i] * Mathf.Deg2Rad;

        Matrix4x4 t = Matrix4x4.identity;
        for (int j = 1; j <= 5; j++)
        {
            t *= ComputeTransformMatrix(j, th);
            if (j >= 2 && t.m23 * 1000f < MinLinkHeightMm) return false;
        }
        return true;
    }

    // atan2 devuelve el angulo en (-180,180], pero J2/J4 tienen un rango real que no
    // esta centrado en 0 (-265..85, ver Fr5DhTable) -- la misma solucion fisica puede
    // necesitar +-360 para caer dentro del limite. De las variantes que entran, devuelve
    // la mas cercana a reference (la pose actual) para no forzar una vuelta completa.
    private static bool TryBringWithinLimits(float raw, float min, float max, float reference, out float result)
    {
        result = raw;
        bool found = false;
        float bestDist = float.MaxValue;
        for (int k = -1; k <= 1; k++)
        {
            float candidate = raw + 360f * k;
            if (candidate < min || candidate > max) continue;
            float dist = Mathf.Abs(candidate - reference);
            if (dist < bestDist)
            {
                bestDist = dist;
                result = candidate;
                found = true;
            }
        }
        return found;
    }

    // ---------------------------------------------------------------------
    // Algebra de la IK analitica (6R con muneca esferica desacoplada), portada de
    // IK_calc.cs del repositorio de referencia -- sin cambios en la matematica, solo
    // en como se obtiene la tabla DH (Fr5DhTable en vez de un campo propio).
    // ---------------------------------------------------------------------

    private static Matrix4x4 ComputeTransformMatrix(int jointIndex, double[,] jointAngles)
    {
        jointIndex--;

        var rotationZ = Matrix4x4.identity;
        rotationZ.m00 = Mathf.Cos((float)jointAngles[0, jointIndex]);
        rotationZ.m01 = -Mathf.Sin((float)jointAngles[0, jointIndex]);
        rotationZ.m10 = Mathf.Sin((float)jointAngles[0, jointIndex]);
        rotationZ.m11 = Mathf.Cos((float)jointAngles[0, jointIndex]);

        var translationZ = Matrix4x4.identity;
        translationZ.m23 = (float)Fr5DhTable.Params[jointIndex, 2];

        var translationX = Matrix4x4.identity;
        translationX.m03 = (float)Fr5DhTable.Params[jointIndex, 0];

        var rotationX = Matrix4x4.identity;
        rotationX.m11 = Mathf.Cos((float)Fr5DhTable.Params[jointIndex, 1]);
        rotationX.m12 = -Mathf.Sin((float)Fr5DhTable.Params[jointIndex, 1]);
        rotationX.m21 = Mathf.Sin((float)Fr5DhTable.Params[jointIndex, 1]);
        rotationX.m22 = Mathf.Cos((float)Fr5DhTable.Params[jointIndex, 1]);

        return rotationZ * translationZ * translationX * rotationX;
    }

    private static double[,] InverseKinematicSolutions(Matrix4x4 transformMatrix)
    {
        double[,] theta = new double[6, 8];

        Vector4 p05 = transformMatrix * new Vector4(0, 0, -(float)Fr5DhTable.Params[5, 2], 1);
        float psi = Mathf.Atan2(p05[1], p05[0]);
        float phi = Mathf.Acos((float)((Fr5DhTable.Params[1, 2] + Fr5DhTable.Params[3, 2] + Fr5DhTable.Params[2, 2]) /
            Mathf.Sqrt(Mathf.Pow(p05[0], 2) + Mathf.Pow(p05[1], 2))));

        theta[0, 0] = psi + phi + Mathf.PI / 2;
        theta[0, 1] = psi + phi + Mathf.PI / 2;
        theta[0, 2] = psi + phi + Mathf.PI / 2;
        theta[0, 3] = psi + phi + Mathf.PI / 2;
        theta[0, 4] = psi - phi + Mathf.PI / 2;
        theta[0, 5] = psi - phi + Mathf.PI / 2;
        theta[0, 6] = psi - phi + Mathf.PI / 2;
        theta[0, 7] = psi - phi + Mathf.PI / 2;

        for (int i = 0; i < 8; i += 4)
        {
            double t5 = (transformMatrix[0, 3] * Mathf.Sin((float)theta[0, i]) - transformMatrix[1, 3] * Mathf.Cos((float)theta[0, i]) -
                (Fr5DhTable.Params[1, 2] + Fr5DhTable.Params[3, 2] + Fr5DhTable.Params[2, 2])) / Fr5DhTable.Params[5, 2];
            float th5 = (t5 >= -1 && t5 <= 1) ? Mathf.Acos((float)t5) : 0f;

            if (i == 0)
            {
                theta[4, 0] = th5; theta[4, 1] = th5; theta[4, 2] = -th5; theta[4, 3] = -th5;
            }
            else
            {
                theta[4, 4] = th5; theta[4, 5] = th5; theta[4, 6] = -th5; theta[4, 7] = -th5;
            }
        }

        // theta6 (muneca 3): la formula de abajo viene de dividir por sin(theta5) antes
        // del atan2 (sin(theta5) >= 0 porque theta5 = acos(..) en [0,pi] para las columnas
        // 0,1,4,5). Para las columnas con theta5 = -acos(..) (2,3,6,7, sin(theta5) <= 0),
        // esa division cambia de signo y el atan2 resultante queda desfasado +-pi respecto
        // del calculado con theta[0,0]/theta[0,4] solos -- sin este +pi, theta2(elbow)/
        // theta1(hombro)/theta3(muneca1), que se derivan mas abajo a partir de T14 (y T14
        // usa T56, construida con este theta6), quedan todos encadenados mal y la rama
        // entera no alcanza el objetivo (bug heredado del proyecto de referencia: ahi un
        // operador humano elegia a ojo entre las 8 soluciones y nunca lo notaba).
        Matrix4x4 inv = transformMatrix.inverse;
        float th0 = Mathf.Atan2((-inv[1, 0] * Mathf.Sin((float)theta[0, 0]) + inv[1, 1] * Mathf.Cos((float)theta[0, 0])),
            (inv[0, 0] * Mathf.Sin((float)theta[0, 0]) - inv[0, 1] * Mathf.Cos((float)theta[0, 0])));
        float th4 = Mathf.Atan2((-inv[1, 0] * Mathf.Sin((float)theta[0, 4]) + inv[1, 1] * Mathf.Cos((float)theta[0, 4])),
            (inv[0, 0] * Mathf.Sin((float)theta[0, 4]) - inv[0, 1] * Mathf.Cos((float)theta[0, 4])));
        float th0Flipped = NormalizeAngleRad(th0 + Mathf.PI);
        float th4Flipped = NormalizeAngleRad(th4 + Mathf.PI);

        theta[5, 0] = th0; theta[5, 1] = th0; theta[5, 2] = th0Flipped; theta[5, 3] = th0Flipped;
        theta[5, 4] = th4; theta[5, 5] = th4; theta[5, 6] = th4Flipped; theta[5, 7] = th4Flipped;

        for (int i = 0; i <= 7; i += 2)
        {
            double[,] t1 = RowFromColumn(theta, i);
            Matrix4x4 t01 = ComputeTransformMatrix(1, t1);
            Matrix4x4 t45 = ComputeTransformMatrix(5, t1);
            Matrix4x4 t56 = ComputeTransformMatrix(6, t1);
            Matrix4x4 t14 = t01.inverse * transformMatrix * (t45 * t56).inverse;

            Vector4 p13 = t14 * new Vector4(0, (float)-Fr5DhTable.Params[3, 2], 0, 1);
            double t3 = (Mathf.Pow(p13[0], 2) + Mathf.Pow(p13[1], 2) - Mathf.Pow((float)Fr5DhTable.Params[1, 0], 2) -
                Mathf.Pow((float)Fr5DhTable.Params[2, 0], 2)) / (2 * Fr5DhTable.Params[1, 0] * Fr5DhTable.Params[2, 0]);
            double th3 = (t3 >= -1 && t3 <= 1) ? Mathf.Acos((float)t3) : 0f;

            theta[2, i] = th3;
            theta[2, i + 1] = -th3;
        }

        for (int i = 0; i < 8; i++)
        {
            double[,] t1 = RowFromColumn(theta, i);
            Matrix4x4 t01 = ComputeTransformMatrix(1, t1);
            Matrix4x4 t45 = ComputeTransformMatrix(5, t1);
            Matrix4x4 t56 = ComputeTransformMatrix(6, t1);
            Matrix4x4 t14 = t01.inverse * transformMatrix * (t45 * t56).inverse;

            Vector4 p13 = t14 * new Vector4(0, (float)-Fr5DhTable.Params[3, 2], 0, 1);
            theta[1, i] = Mathf.Atan2(-p13[1], -p13[0]) -
                Mathf.Asin((float)(-Fr5DhTable.Params[2, 0] * Mathf.Sin((float)theta[2, i]) / Mathf.Sqrt(Mathf.Pow(p13[0], 2) + Mathf.Pow(p13[1], 2))));

            double[,] t2 = RowFromColumn(theta, i);
            Matrix4x4 t32 = ComputeTransformMatrix(3, t2).inverse;
            Matrix4x4 t21 = ComputeTransformMatrix(2, t2).inverse;
            Matrix4x4 t34 = t32 * t21 * t14;
            theta[3, i] = Mathf.Atan2(t34[1, 0], t34[0, 0]);
        }

        return theta;
    }

    private static double[,] RowFromColumn(double[,] theta, int col)
    {
        double[,] t = new double[1, 6];
        for (int row = 0; row < 6; row++) t[0, row] = theta[row, col];
        return t;
    }

    private static float NormalizeAngleRad(float angle)
    {
        angle = Mathf.Repeat(angle, 2f * Mathf.PI);
        if (angle > Mathf.PI) angle -= 2f * Mathf.PI;
        return angle;
    }
}
