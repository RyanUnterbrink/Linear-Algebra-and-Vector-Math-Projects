using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ProjectileController : Projectile
{
    // transform handles
    [SerializeField] Transform initialTransform;
    [SerializeField] Transform moveVelocityTransform;
    [SerializeField] Transform targetTransform;
    [SerializeField] Transform targetVelocityTransform;
    [SerializeField] Transform directionTransform;
    [SerializeField] Transform windVelocityTransform;
    [SerializeField] Transform projectileTransform;

    // controls
    // put initVel, ang, etc
    [SerializeField] bool edit = true;
    [SerializeField] bool reset = false;
    [SerializeField] bool shoot = false;

    // background temp vars
    Coroutine coroutine;
    Vector3 iPos;
    Vector3 tPos;

    // gizmos
    [Header("Gizmos")]
    [Range(1, 50)] [SerializeField] int directionTransformLength = 10;
    [Range(1, 180)] [SerializeField] int maxRangeCircleDetail = 90;
    [Range(0, 10)] [SerializeField] float gizmoPointRadius = .5f;
    [Range(0, 1)] [SerializeField] float gizmoLineAlpha = .2f;
    [Range(0, 1)] [SerializeField] float gizmoPathAlpha = 1f;
    [SerializeField] Color gizmoColor = Color.white;

    void Update()
    {
        if (reset)
            ResetProjectile();

        if (edit)
            CalculateProjectile();

        if (shoot)
            Shoot();
    }
    void OnValidate()
    {
        if (edit)
            CalculateProjectile();
    }
    void OnDrawGizmos()
    {
        // initial position
        Gizmos.color = Color.white;
        if (coroutine != null)
            Gizmos.DrawSphere(iPos, gizmoPointRadius);
        else
            Gizmos.DrawSphere(initialPosition, gizmoPointRadius);

        // target position
        Gizmos.color = Color.red;
        if (coroutine != null)
            Gizmos.DrawSphere(tPos, gizmoPointRadius);
        else
            Gizmos.DrawSphere(targetPosition, gizmoPointRadius);

        // move velocity position
        if (initialPosition != moveVelocityTransform.position)
        {
            Gizmos.color = Color.blue;
            Gizmos.DrawSphere(moveVelocityTransform.position, gizmoPointRadius);
        }

        // target velocity position
        if (targetPosition != targetVelocityTransform.position)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawSphere(targetVelocityTransform.position, gizmoPointRadius);
        }

        // collision position
        if (aimMode == AimMode.MovingTargetSpeed)
        {
            Gizmos.color = Color.magenta;

            if (coroutine != null && relativeAim)
            {
                Vector3 diff = iPos - initialPosition;
                Gizmos.DrawSphere(targetCollision + diff, gizmoPointRadius);
            }
            else
                Gizmos.DrawSphere(targetCollision, gizmoPointRadius);
        }

        // projectile position
        if (coroutine != null)
        {
            Gizmos.color = Color.black;
            Gizmos.DrawSphere(projectileTransform.position, gizmoPointRadius);
        }

        // drection position
        if (initialSpeed != 0)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawSphere(directionTransform.position, gizmoPointRadius);
        }

        // wind velocity position
        if (initialPosition != windVelocityTransform.position)
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawSphere(windVelocityTransform.position, gizmoPointRadius);
        }

        // connecting lines
        Color c = gizmoColor;
        c.a = gizmoLineAlpha;
        Gizmos.color = c;
        Gizmos.DrawLine(initialPosition, targetPosition);
        Gizmos.DrawLine(initialPosition, moveVelocityTransform.position);
        Gizmos.DrawLine(targetPosition, targetVelocityTransform.position);
        Gizmos.DrawLine(initialPosition, directionTransform.position);
        Gizmos.DrawLine(initialPosition, windVelocityTransform.position);

        // current velocity
        if (coroutine != null)
            Gizmos.DrawLine(projectileTransform.position, projectileTransform.position + currentVelocity);

        // initPos and targetPos to target collision
        if (aimMode == AimMode.MovingTargetSpeed)
        {
            if (coroutine != null && relativeAim)
            {
                Vector3 diff = iPos - initialPosition;
                Gizmos.DrawLine(initialPosition + diff, targetCollision + diff);
                Gizmos.DrawLine(targetVelocityTransform.position, targetCollision + diff);
            }
            else
            {
                Gizmos.DrawLine(initialPosition, targetCollision);
                Gizmos.DrawLine(targetVelocityTransform.position, targetCollision);
            }
        }

        // path
        c.a = gizmoPathAlpha;
        Gizmos.color = c;
        for (int i = 1; i < path.Count; i++)
        {
            if (coroutine != null && relativeAim)
            {
                Vector3 diff = iPos - initialPosition;
                Gizmos.DrawLine(path[i - 1] + diff, path[i] + diff);
            }
            else
                Gizmos.DrawLine(path[i - 1], path[i]);
        }

        // ground plane x mark
        if (timeLengthMode == TimeLengthMode.Ground && groundCollision != Vector3.positiveInfinity)
        {
            for (int i = 0; i < 2; i++)
            {
                float z = i % 2 == 0 ? 1 : -1;
                Vector3 v = new Vector3(1, 0, z) * 2;
                Gizmos.DrawLine(groundCollision - v, groundCollision + v);
            }
        }

        // max range circle
        Vector3 lastPoint = Vector3.zero;
        for (int i = 0; i <= maxRangeCircleDetail; i++)
        {
            float x = Mathf.Cos(i / (float)maxRangeCircleDetail * 360 * Mathf.Deg2Rad) * maxRange;
            float z = Mathf.Sin(i / (float)maxRangeCircleDetail * 360 * Mathf.Deg2Rad) * maxRange;

            Vector3 pos = initialPosition + new Vector3(x, 0, z);

            if (lastPoint != Vector3.zero)
                Gizmos.DrawLine(lastPoint, pos);

            lastPoint = pos;
        }
    }

    public void ResetProjectile()
    {
        reset = false;

        if (coroutine != null)
        {
            StopCoroutine(coroutine);
            coroutine = null;
        }

        projectileTransform.position = initialPosition;

        CalculateProjectile();

        currentVelocity = Vector3.zero;
        currentSpeed = 0;
        currentAngle = initialAngle;

        iPos = Vector3.zero;
        tPos = Vector3.zero;

        projectileTransform.LookAt(projectileTransform.position + Vector3.forward, Vector3.up);
    }
    IEnumerator Play()
    {
        float time = 0;
        int index = 0;

        while (time <= timeLength)
        {
            float timeInc = Time.fixedDeltaTime;
            if (timeInc <= 0)
                timeInc = 1 / 60f;

            time += timeInc;
            if (timeLength - time < timeInc)
                time = timeLength;

            iPos = initialPosition + moveVelocity * time;
            tPos = targetPosition + targetVelocity * time;

            if (drag)
            {
                if (index < path.Count)
                    projectileTransform.position = path[index++];
            }
            else
            {
                if (relativeAim)
                {
                    Vector3 diff = iPos - initialPosition;
                    projectileTransform.position = CalculatePoint(initialPosition, initialVelocity, gravity, time) + diff;
                }
                else
                    projectileTransform.position = CalculatePoint(initialPosition, initialVelocity, gravity, time);
            }

            currentVelocity = CalculateVelocity(initialVelocity, gravity, time);
            currentSpeed = currentVelocity.magnitude;
            currentAngle = CalculateAngle(currentVelocity);

            if (projectileLookAtVelocity)
                projectileTransform.LookAt(projectileTransform.position + currentVelocity);

            yield return new WaitForSeconds(timeInc);
        }

        Stop();
    }
    public void Shoot()
    {
        CalculateProjectile();

        shoot = false;

        if (path.Count > 0)
        {
            if (coroutine != null)
            {
                StopCoroutine(coroutine);
                coroutine = null;
            }

            iPos = initialPosition;
            tPos = targetPosition;

            coroutine = StartCoroutine(Play());
        }
    }
    void Stop()
    {
        currentVelocity = Vector3.zero;
        currentSpeed = 0;
    }
    bool CalculateTransformValues()
    {
        bool b = aimMode != AimMode.FixedTargetAngle;

        if (!initialTransform || !targetTransform || !directionTransform || !windVelocityTransform || ((!moveVelocityTransform && !targetVelocityTransform) && b))
            return false;

        initialPosition = initialTransform.position;

        targetPosition = targetTransform.position;

        aimDirection = directionTransform.position - initialPosition;

        windVelocity = windVelocityTransform.position - initialPosition;

        if (b)
        {
            moveVelocity = moveVelocityTransform.position - initialPosition;
            targetVelocity = targetVelocityTransform.position - targetPosition;
        }
        else
        {
            moveVelocity = Vector3.zero;
            targetVelocity = Vector3.zero;
        }

        return true;
    }
    protected override void CalculateProjectile()
    {
        if (!CalculateTransformValues())
            return;

        base.CalculateProjectile();

        if (coroutine == null)
            currentVelocity = Vector3.zero;
    }
    protected override void CalculateMovingTargetSpeed()
    {
        base.CalculateMovingTargetSpeed();

        Vector3 direction = initialVelocity.normalized * directionTransformLength;
        directionTransform.position = initialPosition + direction;

        iSpeed = initialSpeed;
    }
    protected override void CalculateFixedTargetAngle()
    {
        base.CalculateFixedTargetAngle();

        directionTransform.position = initialPosition + initialVelocity.normalized * directionTransformLength;
    }
    protected override void CalculateManual()
    {
        moveSpeed = moveVelocity.magnitude;
        targetSpeed = targetVelocity.magnitude;
        relativeVelocity = targetVelocity - moveVelocity;
        relativeSpeed = relativeVelocity.magnitude;

        float initMag = aimDirection.magnitude;

        if (initialAngle != iAng)
        {
            float xz = initialSpeed * Mathf.Cos(initialAngle * Mathf.Deg2Rad);
            float y = initialSpeed * Mathf.Sin(initialAngle * Mathf.Deg2Rad);
            Vector3 dir = new Vector3(aimDirection.x, 0, aimDirection.z).normalized * xz;
            dir.y = y;
            aimDirection = dir;

            iAng = initialAngle;

            initialVelocity = aimDirection.normalized * initialSpeed;
        }
        else
        {
            initialVelocity = aimDirection.normalized * initialSpeed;

            initialAngle = iAng = CalculateAngle(initialVelocity);
        }

        timeToTarget = float.NaN;

        if (aimDirection.magnitude > directionTransformLength)
            directionTransform.position = initialPosition + aimDirection.normalized * directionTransformLength;
        else
            directionTransform.position = initialPosition + aimDirection.normalized * initMag;
    }
}
