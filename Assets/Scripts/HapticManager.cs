using UnityEngine;
using System.Collections;
using System.Runtime.InteropServices;
using System.IO;
using System;
using System.Threading;

public class HapticManager : MonoBehaviour
{
    public bool useHaptic;
    public static bool isHapticAvail;
   
    public GameObject leftHandIndex;
    private Vector3 originalPosition;

    public static bool IsCalibrated = false;

    // Use this for initialization
    private void Awake()
    {
        if (!useHaptic)
            isHapticAvail = false;

        isHapticAvail = HapticNativePlugin.prepareHaptics(0.3d);
    }


    private void Start()
    {
        if(isHapticAvail)
            HapticNativePlugin.startHaptics();
    }

    private void OnDestroy()
    {
        if (isHapticAvail)
            HapticNativePlugin.stopHaptics();
    }

    private void Update()
    {
        if (isHapticAvail)
            this.transform.localPosition = HapticNativePlugin.GetProxyPosition();
    }
    
}