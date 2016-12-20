using System;
using System.Runtime.InteropServices;
using System.Text;
using UnityEngine;

public class HapticNativePlugin
{
    [DllImport("UnityPlugin")]
    public static extern bool prepareHaptics(double hapticScale);

    [DllImport("UnityPlugin")]
    public static extern void startHaptics();

    [DllImport("UnityPlugin")]
    public static extern void stopHaptics();

    [DllImport("UnityPlugin")]
    protected static extern void getProxyPosition(double[] array);

    public static Vector3 GetProxyPosition()
    {
        double[] arrayToUse = new double[3];
        getProxyPosition(arrayToUse);
        return new Vector3((float)arrayToUse[0], (float)arrayToUse[1], (float)arrayToUse[2]);
    }

    [DllImport("UnityPlugin")]
    protected static extern void getDevicePosition(double[] array);

    public static Vector3 GetDevicePosition()
    {
        double[] arrayToUse = new double[3];
        getDevicePosition(arrayToUse);
        return new Vector3((float)arrayToUse[0], (float)arrayToUse[1], (float)arrayToUse[2]);
    }

    [DllImport("UnityPlugin")]
    public static extern bool isButtonPressed(int buttonId);

    [DllImport("UnityPlugin")]
    public static extern bool isTouching(int objectId);

    public static bool IsInContact()
    {
        var pos1 = GetProxyPosition();
        var pos2 = GetDevicePosition();
        return !(pos1.x == pos2.x && pos1.y == pos2.y && pos1.y == pos2.y);
    }

    [DllImport("UnityPlugin")]
    protected static extern void addObject(double[] objectPos, double[] objectScale, double[] objectRotation, double[,] vertPos, double[,] normals, int vertNum, int[,] tris, int triNum);

    private static int objectCount = 0;
    public static int AddObject(Vector3 position, Vector3 scale, Vector3 rotation, Vector3[] vertPos, Vector3[] normals, int vertNum, int[,] tris, int triNum)
    {
        double[] objectPosition = new double[3];
        objectPosition[0] = (double)position.x;
        objectPosition[1] = (double)position.y;
        objectPosition[2] = (double)position.z;

        double[] objectScale = new double[3];
        objectScale[0] = (double)scale.x;
        objectScale[1] = (double)scale.y;
        objectScale[2] = (double)scale.z;

        double[] objectRotation = new double[3];
        objectRotation[0] = (double)rotation.x;
        objectRotation[1] = (double)rotation.y;
        objectRotation[2] = (double)rotation.z;

        double[,] objectVertPos = new double[vertNum, 3];
        for (int i = 0; i < vertNum; i++)
        {
            objectVertPos[i, 0] = (double)vertPos[i].x;
            objectVertPos[i, 1] = (double)vertPos[i].y;
            objectVertPos[i, 2] = (double)vertPos[i].z;
        }

        double[,] objectNormals = new double[vertNum, 3];
        for (int i = 0; i < vertNum; i++)
        {
            objectNormals[i, 0] = (double)normals[i].x;
            objectNormals[i, 1] = (double)normals[i].y;
            objectNormals[i, 2] = (double)normals[i].z;
        }
        addObject(objectPosition, objectScale, objectRotation, objectVertPos, objectNormals, vertNum, tris, triNum);

        return (objectCount++);
    }

    [DllImport("UnityPlugin")]
    protected static extern void translateObjects(double[] translation);
    public static void TranslateObjects(Vector3 translation)
    {
        double[] objectPos = new double[3];
        objectPos[0] = (double)translation.x;
        objectPos[1] = (double)translation.y;
        objectPos[2] = (double)translation.z;
        translateObjects(objectPos);
    }

    [DllImport("UnityPlugin")]
    protected static extern void setHapticPosition(double[] position);
    public static void SetHapticPosition(Vector3 position)
    {
        double[] objectPos = new double[3];
        objectPos[0] = (double)position.x;
        objectPos[1] = (double)position.y;
        objectPos[2] = (double)position.z;
        setHapticPosition(objectPos);
    }

    [DllImport("UnityPlugin")]
    protected static extern void setHapticRotation(double[] rotation);
    public static void SetHapticRotation(Vector3 rotation)
    {
        double[] objectPos = new double[3];
        objectPos[0] = (double)(Mathf.Deg2Rad * rotation.x);
        objectPos[1] = (double)(Mathf.Deg2Rad * rotation.y);
        objectPos[2] = (double)(Mathf.Deg2Rad * rotation.z);
        setHapticRotation(objectPos);
    }

}