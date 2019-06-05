using UnityEngine;
using System.Collections;

public class TouchableObject : MonoBehaviour {

	public double stiffness;
	public double staticFriction;
	public double dynamicFriction;
	public double damping;
	public double viscosity;
    public int objectId;

	// Use this for initialization
	void Start () {

        var devicePosition = GameObject.Find("Haptic Origin");

        Mesh mesh = this.GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;
        Vector3[] normals = mesh.normals;
        int[,] triangles = new int[(mesh.triangles.Length / 3),3];

        for (int i = 0; i < mesh.triangles.Length / 3; i++)
        {
            triangles[i, 0] = mesh.triangles[3 * i];
            triangles[i, 1] = mesh.triangles[3 * i + 1];
            triangles[i, 2] = mesh.triangles[3 * i + 2];
        }

        objectId = HapticNativePlugin.AddModificableObject(this.transform.localPosition - devicePosition.transform.localPosition, this.transform.localScale, this.transform.localRotation.eulerAngles, vertices, normals, mesh.vertices.Length, triangles, mesh.triangles.Length / 3, stiffness, staticFriction, dynamicFriction, damping, viscosity);
    }
}
