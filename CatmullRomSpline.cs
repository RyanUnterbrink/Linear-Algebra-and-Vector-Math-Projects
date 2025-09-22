using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteAlways]
public class CatmullRomSpline : MonoBehaviour
{
    List<Transform> controlPoints = new List<Transform>();
    List<Vector3> spline = new List<Vector3>();

    public bool loop = true;
    [Range(1, 100)] public int detail = 3;
    public float tension = 1;
    public bool refresh = false;
    public bool edit = true;

    [Range(0, 1)] public float gizmoSplineAlpha = 1;
    [Range(0, 1)] public float gizmoLineAlpha = .5f;
    [Range(0, 1)] public float gizmoPointAlpha = .75f;
    [Range(0, 1)] public float gizmoPointSize = .5f;
    public Color gizmoColor;
    
    void Start()
    {
        SetupControlPoints();
    }
    void Update()
    {
#if UNITY_EDITOR
        if (controlPoints.Count > 3 && (edit || refresh))
        {
            refresh = false;
            SetupControlPoints();
            GenerateSpline();
        }
#endif
    }
    void OnDrawGizmos()
    {
        Color c = gizmoColor;

        int numControlPoints = transform.childCount;
        for (int i = 0; i < numControlPoints; i++)
        {
            c.a = gizmoPointAlpha;
            Gizmos.color = c;

            Gizmos.DrawSphere(transform.GetChild(i).position, gizmoPointSize);

            if (i != 0 && i != transform.childCount - 1)
            {
                c.a = gizmoLineAlpha;
                Gizmos.color = c;

                Gizmos.DrawLine(transform.GetChild(i).position, transform.GetChild(0).position);
                Gizmos.DrawLine(transform.GetChild(i).position, transform.GetChild(numControlPoints - 1).position);
            }
        }

        if (spline.Count == 0)
            GenerateSpline();

        c.a = gizmoSplineAlpha;
        Gizmos.color = c;

        for (int i = 0; i < spline.Count - 1; i++)
            Gizmos.DrawLine(spline[i], spline[i + 1]);
    }

    void SetupControlPoints()
    {
        controlPoints.Clear();

        for (int i = 0; i < transform.childCount; i++)
            controlPoints.Add(transform.GetChild(i));
    }
    public void GenerateSpline()
    {
        spline.Clear();

        for (int i = 0; i < controlPoints.Count; i++)
        {
            if (!loop && i == controlPoints.Count - 1)
                continue;

            Vector3 prev = controlPoints[ClampIndex(i - 1)].position;
            Vector3 currStart = controlPoints[i].position;
            Vector3 currEnd = controlPoints[ClampIndex(i + 1)].position;
            Vector3 next = controlPoints[ClampIndex(i + 2)].position;

            for (int step = 0; step <= detail; step++)
            {
                float t = (float)step / detail;
                float tSquared = t * t;
                float tCubed = tSquared * t;

                Vector3 interpolatedPoint =
                    (-.5f * tension * tCubed + tension * tSquared - .5f * tension * t) * prev +
                    (1 + .5f * tSquared * (tension - 6) + .5f * tCubed * (4 - tension)) * currStart +
                    (.5f * tCubed * (tension - 4) + .5f * tension * t - (tension - 3) * tSquared) * currEnd +
                    (-.5f * tension * tSquared + .5f * tension * tCubed) * next;

                spline.Add(interpolatedPoint);
            }
        }
    }
    int ClampIndex(int index)
    {
        if (index < 0)
            index = controlPoints.Count - 1;

        if (index > controlPoints.Count)
            index = 1;
        else if (index > controlPoints.Count - 1)
            index = 0;

        return index;
    }
}
