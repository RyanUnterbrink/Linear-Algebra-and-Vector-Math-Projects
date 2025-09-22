using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteAlways]
public class Helix : MonoBehaviour
{
    public Transform initialPosition;
    public Transform directionHandle;
    public Transform projectile;

    public float radius = 1;
    public float pitch = 1;
    public float timeLength = 1;
    [Range(.01f, 1)] public float timeStep = 0.1f;
    public float angularVelocity = 3;
    
    List<Vector3> path = new List<Vector3>();
    Coroutine coroutine;

    public bool refresh = false;
    public bool edit = true;
    public bool start = false;

    [Range(1, 100)] public int directionHandleLength = 10;

    [Range(0, 1)] public float gizmoVelocityAlpha = .1f;
    [Range(0, 1)] public float gizmoLineAlpha = 1;
    [Range(0, 1)] public float gizmoPointAlpha = .5f;
    [Range(0, 1)] public float gizmoPointSize = .5f;
    public Color gizmoColor;

    void Update()
    {
        if (refresh)
        {
            refresh = false;

            if (coroutine != null)
            {
                StopCoroutine(coroutine);
                coroutine = null;
            }

            projectile.position = initialPosition.position;

            GeneratePath();
        }

        if (edit)
            GeneratePath();

        if (start)
        {
            start = false;

            if (path.Count > 0)
            {
                if (coroutine != null)
                {
                    StopCoroutine(coroutine);
                    coroutine = null;
                }

                coroutine = StartCoroutine(Play());
            }
        }
    }
    void OnDrawGizmos()
    {
        Color c = gizmoColor;

        c.a = gizmoPointAlpha;
        Gizmos.color = c;

        Gizmos.DrawSphere(initialPosition.position, gizmoPointSize);
        Gizmos.DrawSphere(directionHandle.position, gizmoPointSize);
        Gizmos.DrawSphere(projectile.position, gizmoPointSize);

        c.a = gizmoLineAlpha;
        Gizmos.color = c;
        Gizmos.DrawLine(initialPosition.position, directionHandle.position);

        c.a = gizmoVelocityAlpha;
        Gizmos.color = c;

        if (path.Count > 1)
        {
            for (int i = 1; i < path.Count; i++)
                Gizmos.DrawLine(path[i - 1], path[i]);
        }
    }

    void GeneratePath()
    {
        Vector3 direction = directionHandle.position - initialPosition.position;
        if (direction.magnitude > directionHandleLength)
            directionHandle.position = initialPosition.position + direction.normalized * directionHandleLength;

        Vector3 position = initialPosition.position;

        path.Clear();

        for (float t = 0; t <= timeLength; t += timeStep)
        {
            float angle = Mathf.Deg2Rad * angularVelocity * t;

            float x = radius * Mathf.Cos(angle);
            float y = radius * Mathf.Sin(angle);
            float z = pitch * t;

            Vector3 point = new Vector3(x, y, z);
            Vector3 rotatedPoint = position + Quaternion.LookRotation(direction, Vector3.up) * point;

            path.Add(rotatedPoint);
        }
    }
    IEnumerator Play()
    {
        int index = 0;
        while (index < path.Count)
        {
            projectile.position = path[index++];

            yield return new WaitForSeconds(timeStep);
        }
    }
}
