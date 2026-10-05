using System;

// Local (no-ROS-round-trip) forward kinematics for the FR5, mirroring MGD_Node's
// DH-based computation. SecCoordQueueController used to get this same
// translation from MGD_Node via a ROS round trip (publish joints to
// input_joint_position, wait for MGD_Subscriber.OnInverseKinematicsResultReceived),
// but that event is also consumed by ControlArticular.cs with no per-request
// correlation id -- whichever controller's callback runs first steals the
// response, so a saved point could silently end up with another point's (or
// another panel's) cartesian result instead of its own, or its own joint
// values falling through if no MGD result ever arrived. Computing FK locally
// removes the race entirely and works even when ROS/MGD_Node isn't connected.
public static class LocalForwardKinematics
{
    // Tabla compartida con FR5AnalyticIK.cs (ver Fr5DhTable) -- ambas deben operar
    // sobre la misma cadena DH.
    private static readonly double[,] DhParams = Fr5DhTable.Params;

    // jointsDeg: 6 joint angles in degrees. Returns {x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg}.
    public static float[] CartesianFromJointsDeg(float[] jointsDeg)
    {
        double[,] t = Identity4();
        for (int i = 0; i < 6; i++)
        {
            double a = DhParams[i, 0], alpha = DhParams[i, 1], d = DhParams[i, 2];
            double theta = jointsDeg[i] * Math.PI / 180.0 + DhParams[i, 3];
            t = Multiply4(t, DhMatrix(a, alpha, d, theta));
        }

        double px = t[0, 3] * 1000.0, py = t[1, 3] * 1000.0, pz = t[2, 3] * 1000.0;

        double ry = Math.Atan2(-t[2, 0], Math.Sqrt(t[0, 0] * t[0, 0] + t[1, 0] * t[1, 0])) * 180.0 / Math.PI;
        double rx = Math.Atan2(t[2, 1], t[2, 2]) * 180.0 / Math.PI;
        double rz = Math.Atan2(t[1, 0], t[0, 0]) * 180.0 / Math.PI;

        return new float[] { (float)px, (float)py, (float)pz, (float)rx, (float)ry, (float)rz };
    }

    private static double[,] DhMatrix(double a, double alpha, double d, double theta)
    {
        double[,] m = new double[4, 4];
        m[0, 0] = Math.Cos(theta); m[0, 1] = -Math.Sin(theta) * Math.Cos(alpha); m[0, 2] = Math.Sin(theta) * Math.Sin(alpha); m[0, 3] = a * Math.Cos(theta);
        m[1, 0] = Math.Sin(theta); m[1, 1] = Math.Cos(theta) * Math.Cos(alpha); m[1, 2] = -Math.Cos(theta) * Math.Sin(alpha); m[1, 3] = a * Math.Sin(theta);
        m[2, 0] = 0; m[2, 1] = Math.Sin(alpha); m[2, 2] = Math.Cos(alpha); m[2, 3] = d;
        m[3, 0] = 0; m[3, 1] = 0; m[3, 2] = 0; m[3, 3] = 1;
        return m;
    }

    private static double[,] Identity4()
    {
        double[,] id = new double[4, 4];
        for (int i = 0; i < 4; i++) id[i, i] = 1;
        return id;
    }

    private static double[,] Multiply4(double[,] a, double[,] b)
    {
        double[,] r = new double[4, 4];
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
            {
                double sum = 0;
                for (int k = 0; k < 4; k++) sum += a[i, k] * b[k, j];
                r[i, j] = sum;
            }
        return r;
    }
}
