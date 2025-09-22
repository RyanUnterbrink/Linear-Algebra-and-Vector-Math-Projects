using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteAlways]
public class Projectile : MonoBehaviour
{
    [Header("Movement")]
    public Vector3 initialPosition;
    public Vector3 moveVelocity;
    public float moveSpeed;
    public Vector3 targetPosition;
    public Vector3 targetVelocity;
    public float targetSpeed;
    public Vector3 relativeVelocity;
    public float relativeSpeed;

    [Header("Projectile Data")]
    public Vector3 aimDirection;
    public Vector3 targetDirection;

    public Vector3 initialVelocity;
    public Vector3 currentVelocity;
    public Vector3 finalVelocity;

    public float initialSpeed = 25;
    public float currentSpeed;
    public float finalSpeed;

    [Range(-90, 90)] public float initialAngle = 45;
    public float currentAngle;
    public float finalAngle;

    public float gravity = -9.81f;

    public bool projectileLookAtVelocity = true;

    public enum AimMode
    {
        MovingTargetSpeed = 0,
        FixedTargetAngle,
        Manual
    }
    public AimMode aimMode = AimMode.MovingTargetSpeed;

    public bool fastShot = true;
    public bool relativeAim = false;

    public enum TimeLengthMode
    {
        Target = 0,
        MaxHeight,
        FullArc,
        Ground,
        Manual
    }
    public TimeLengthMode timeLengthMode = TimeLengthMode.Target;
    [Range(0, maxTimeLength)] public float timeLength = 10;
    const float maxTimeLength = 25;

    public float timeToMaxHeight;
    public float timeToTarget;
    public float timeToGround;

    public float targetHeightDifference;
    public Vector3 targetCollision;

    public float groundY = 0;
    public Vector3 groundCollision;

    public float initialDistance;
    public float finalDistance;

    public float maxRange;
    public float maxHeight;

    public bool wind = true;
    public Vector3 windVelocity;
    public float windSpeed;

    public bool drag = true;
    [Range(0, 10)] public float dragCoefficient = .5f;
    [Range(0, 1)] public float crossSection = .1f;
    float maxAltitude = 100000;
    public AnimationCurve airDensityVsAltitude;

    public bool magnusEffect = true;
    public Vector3 spinAxis;
    /*[Range(0, 10)] */public float spinSpeed = 1.5f;

    [Range(.001f, 1000)] public float mass = .007f;
    float impactForce;

    protected List<Vector3> path = new List<Vector3>();
    [Range(1, 500)] [SerializeField] int pathDetail = 35;
    [SerializeField] bool autoPathDetail = true;
    [Range(.01f, 1)] public float timeStep = .01f;
    const double zeroBuffer = 1e-9;

    protected float iSpeed = float.NaN;
    protected float iAng;

    protected virtual void CalculateProjectile()
    {
        targetDirection = targetPosition - initialPosition;
        targetHeightDifference = targetDirection.y;
        initialDistance = targetDirection.magnitude;

        switch (aimMode)
        {
            case AimMode.MovingTargetSpeed:

                CalculateMovingTargetSpeed();

                break;

            case AimMode.FixedTargetAngle:

                CalculateFixedTargetAngle();

                break;

            case AimMode.Manual:

                CalculateManual();

                break;
        }

        CalculateValues();

        if (wind || drag)
            CalculatePathRungeKutta4(ref path, initialPosition, initialVelocity);
        else
            CalculatePath(ref path, initialVelocity);
    }
    protected virtual void CalculateMovingTargetSpeed()
    {
        if (float.IsNaN(initialSpeed) && !float.IsNaN(iSpeed))
            initialSpeed = iSpeed;

        relativeVelocity = targetVelocity - moveVelocity;

        Vector3 vel = relativeAim ? relativeVelocity : targetVelocity;
        if (vel.magnitude > zeroBuffer)
        {
            moveSpeed = moveVelocity.magnitude;
            targetSpeed = targetVelocity.magnitude;
            relativeSpeed = relativeVelocity.magnitude;

            CalculateInitialVelocityMovingTarget(initialPosition, initialSpeed, targetPosition, vel, gravity, fastShot, ref initialVelocity, ref timeToTarget);

            initialAngle = CalculateAngle(initialVelocity);
        }
        else
        {
            moveSpeed = 0;
            targetSpeed = 0;
            relativeVelocity = Vector3.zero;
            relativeSpeed = 0;

            CalculateInitialVelocityFixedTargetSpeed(initialPosition, initialSpeed, targetPosition, gravity, fastShot, ref initialVelocity, ref initialAngle);

            timeToTarget = CalculateTargetTime(initialPosition, initialVelocity, gravity, targetPosition);
        }
    }
    protected virtual void CalculateFixedTargetAngle()
    {
        moveSpeed = 0;
        targetSpeed = 0;
        relativeVelocity = Vector3.zero;
        relativeSpeed = 0;

        CalculateInitialVelocityFixedTargetAngle(initialPosition, initialAngle, targetPosition, gravity, ref initialVelocity);

        initialSpeed = initialVelocity.magnitude;

        initialAngle = CalculateAngle(initialVelocity);

        timeToTarget = CalculateTargetTime(initialPosition, initialVelocity, gravity, targetPosition);
    }
    protected virtual void CalculateManual()
    {

    }
    void CalculateValues()
    {
        spinAxis = initialVelocity;

        if (!float.IsNaN(timeToTarget))
        {
            targetCollision = CalculatePoint(initialPosition, initialVelocity, gravity, timeToTarget);

            finalVelocity = CalculateVelocity(initialVelocity, gravity, timeToTarget);
        }
        else
        {
            targetCollision = CalculatePoint(initialPosition, initialVelocity, gravity, timeLength);

            finalVelocity = CalculateVelocity(initialVelocity, gravity, timeLength);
        }

        maxRange = CalculateMaxRange(initialVelocity, gravity);

        maxHeight = CalculateMaxHeight(initialPosition.y, initialVelocity.y, gravity);

        timeToMaxHeight = CalculateTimeToMaxHeight(initialVelocity.y, gravity);

        finalDistance = Vector3.Distance(initialPosition, targetCollision);

        finalSpeed = finalVelocity.magnitude;

        finalAngle = CalculateAngle(finalVelocity);

        windSpeed = windVelocity.magnitude;

        CalculatePlaneTime();

        CalculateTimeLength();
    }
    public Vector3 CalculateVelocity(Vector3 initVel, float grav, float time)
    {
        float velX = initVel.x;
        float velY = initVel.y + grav * time;
        float velZ = initVel.z;

        return new Vector3(velX, velY, velZ);
    }
    public float CalculateAngle(Vector3 vel)
    {
        return (float)(Math.Atan2(vel.y, Math.Sqrt(vel.x * vel.x + vel.z * vel.z)) * 180 / Math.PI);
    }
    float CalculateMaxRange(Vector3 initVel, float grav)
    {
        return initVel.magnitude * initVel.magnitude / -grav;
    }
    float CalculateMaxHeight(float initPosY, float initVelY, float grav)
    {
        return initPosY + initVelY * initVelY / (2 * -grav);
    }
    float CalculateTimeToMaxHeight(float initVelY, float grav)
    {
        float timeToMaxHeight = initVelY / -grav;

        if (timeToMaxHeight < 0)
            timeToMaxHeight = 0;
        else if (grav >= 0)
            timeToMaxHeight = float.NaN;

        return timeToMaxHeight;
    }
    void CalculateTimeLength()
    {
        if (aimMode == AimMode.Manual && timeLengthMode == TimeLengthMode.Target)
            timeLengthMode = TimeLengthMode.FullArc;

        switch (timeLengthMode)
        {
            case TimeLengthMode.Target:
                if (float.IsNaN(timeToTarget))
                    timeLength = timeToMaxHeight * 2;
                else
                    timeLength = timeToTarget;
                break;
            case TimeLengthMode.MaxHeight:
                timeLength = timeToMaxHeight;
                break;
            case TimeLengthMode.FullArc:
                timeLength = timeToMaxHeight * 2;
                break;
            case TimeLengthMode.Ground:
                if (float.IsNaN(timeToGround))
                    timeLength = timeToMaxHeight;
                else
                    timeLength = timeToGround;
                break;
        }
    }
    float CalculateTargetTime(Vector3 initPos, Vector3 initVel, float grav, Vector3 targetPos)
    {
        Quadratic(.5f * -grav, -initVel.y, targetPos.y - initPos.y, out double time1, out double time2);

        if (time1 > 0 && time2 < 0)
            return (float)time1;

        float minTimeDist = Vector3.Distance(CalculatePoint(initPos, initVel, grav, (float)time1), targetPos);
        float maxTimeDist = Vector3.Distance(CalculatePoint(initPos, initVel, grav, (float)time2), targetPos);

        if (minTimeDist < maxTimeDist)
            return (float)time1;
        else if (minTimeDist >= maxTimeDist)
            return (float)time2;
        return float.NaN;
    }
    void CalculatePlaneTime()
    {
        int solutions = Quadratic(.5f * -gravity, -initialVelocity.y, groundY - initialPosition.y, out double time1, out double time2);

        Vector3 maxHeightPos = CalculatePoint(initialPosition, initialVelocity, gravity, timeToMaxHeight);

        if (solutions == 2)
        {
            if (time1 > 0 && time2 > 0)
            {
                timeToGround = (float)time1;
                groundCollision = CalculatePoint(initialPosition, initialVelocity, gravity, timeToGround);
            }
            else
            {
                float minTimeDist = Vector3.Distance(CalculatePoint(initialPosition, initialVelocity, gravity, (float)time1), targetCollision);
                float maxTimeDist = Vector3.Distance(CalculatePoint(initialPosition, initialVelocity, gravity, (float)time2), targetCollision);

                if (minTimeDist >= maxTimeDist && time2 > 0)
                {
                    timeToGround = (float)time2;
                    groundCollision = CalculatePoint(initialPosition, initialVelocity, gravity, timeToGround);
                }
                else if (minTimeDist < maxTimeDist || time1 >= 0)
                {
                    timeToGround = (float)time1;
                    groundCollision = CalculatePoint(initialPosition, initialVelocity, gravity, timeToGround);
                }
                else
                {
                    timeToGround = float.NaN;
                    groundCollision = new Vector3(maxHeightPos.x, groundY, maxHeightPos.z);
                }
            }
        }
        else if (solutions == 1)
        {
            if (time1 > 0)
                timeToGround = (float)time1;
            else
                timeToGround = 0;

            groundCollision = CalculatePoint(initialPosition, initialVelocity, gravity, timeToGround);
        }
        else
        {
            timeToGround = float.NaN;
            groundCollision = new Vector3(maxHeightPos.x, groundY, maxHeightPos.z);
        }
    }
    Vector3 CalculateImpactForceVector(Vector3 vel)
    {
        float force =  mass * vel.magnitude / Time.deltaTime;

        return vel.normalized * force;
    }
    Vector3 CalculateDrag(Vector3 vel, float airDensity)
    {
        double dragMagnitude = .5 * dragCoefficient * crossSection * airDensity;

        Vector3 vxz = vel;
        vxz.y = 0;
        double dxz = dragMagnitude * vxz.magnitude * vxz.magnitude;

        Vector3 _drag = Vector3.zero;

        double angle = Math.Atan2(vel.y, vxz.magnitude);

        double dy = dragMagnitude * vel.y * vel.y;
        _drag.y = (float)(dy * Math.Sin(angle) / vel.magnitude);

        double axz = dxz * Math.Cos(angle) / vel.magnitude;
        vxz = vxz.normalized * (float)axz;
        _drag.x = vxz.x;
        _drag.z = vxz.z;

        return _drag;
    }
    Vector3 CalculateMagnusEffect(Vector3 vel, Vector3 _spinAxis, float airDensity)
    {
        Vector3 cross;
        if (spinSpeed >= 0)
            cross = Vector3.Cross(vel, _spinAxis).normalized;
        else
            cross = Vector3.Cross(_spinAxis, vel).normalized;

        float magnitude = spinSpeed * spinSpeed * crossSection / 2f;

        return airDensity * magnitude * cross;
    }
    Vector3 CalculateSpinAxis(Vector3 spinAxis, Vector3 magnusForce, float time)
    {
        Vector3 angVel = Vector3.Cross(spinAxis, magnusForce / spinSpeed);

        return Quaternion.AngleAxis(angVel.magnitude * time, angVel.normalized) * spinAxis.normalized;
    }
    Vector3 CalculateWindDrift(Vector3 vel, Vector3 windVel, float time)
    {
        Vector3 velNor = vel.normalized;
        float velDot = Vector3.Dot(windVel, velNor);

        float x = (windVel.x - velDot * velNor.x) * time / 2f;
        float y = (windVel.y - velDot * velNor.y) * time / 2f;
        float z = (windVel.z - velDot * velNor.z) * time / 2f;

        return new Vector3(x, y, z);
    }
    protected Vector3 CalculatePoint(Vector3 initPos, Vector3 initVel, float grav, float time)
    {
        float x = initPos.x + initVel.x * time;
        float y = initPos.y + initVel.y * time + .5f * grav * time * time;
        float z = initPos.z + initVel.z * time;

        return new Vector3(x, y, z);
    }
    void CalculatePath(ref List<Vector3> points, Vector3 initVel)
    {
        points.Clear();
        points.Add(initialPosition);

        int detail;
        float time;
        if (autoPathDetail)
        {
            detail = (int)(timeLength / timeStep);
            pathDetail = detail;
            time = timeLength / detail;
        }
        else
        {
            detail = pathDetail;
            time = timeLength / detail;
        }

        for (int i = 1; i <= detail; i++)
            points.Add(CalculatePoint(initialPosition, initVel, gravity, time * i));
    }
    public void CalculatePathRungeKutta4(ref List<Vector3> points, Vector3 initPos, Vector3 initVel)
    {
        double x = initPos.x;
        double y = initPos.y;
        double z = initPos.z;

        double vx = initVel.x;
        double vy = initVel.y;
        double vz = initVel.z;

        Vector3 spAx = spinAxis;

        points.Clear();
        points.Add(initialPosition);

        double time = 0;
        double t = Time.fixedDeltaTime;

        bool run = true;
        while (run)
        {
            double diff = timeLength - time;
            if (diff < t)
            {
                run = false;
                t = diff;
            }

            double ax = 0;
            double ay = gravity;
            double az = 0;

            Vector3 v = new Vector3((float)vx, (float)vy, (float)vz);

            if (wind)
            {
                Vector3 wd = CalculateWindDrift(v, windVelocity, (float)t);
                ax += wd.x;
                ay += wd.y;
                az += wd.z;
            }

            if (drag)
            {
                double ad = CalculateAirDensity((float)y);

                Vector3 d = CalculateDrag(v, (float)ad);
                ax -= d.x;
                ay -= d.y;
                az -= d.z;

                if (magnusEffect)
                {
                    Vector3 me = CalculateMagnusEffect(v, spAx, (float)ad);
                    spAx = CalculateSpinAxis(spAx, me, (float)t);

                    ax += me.x;
                    ay += me.y;
                    az += me.z;
                }
            }

            double k1x = t * vx;
            double k1y = t * vy;
            double k1z = t * vz;
            double k1vx = t * ax;
            double k1vy = t * ay;
            double k1vz = t * az;

            double k2x = t * (vx + .5 * k1vx);
            double k2y = t * (vy + .5 * k1vy);
            double k2z = t * (vz + .5 * k1vz);
            double k2vx = t * (ax + .5 * k1vx);
            double k2vy = t * (ay + .5 * k1vy);
            double k2vz = t * (az + .5 * k1vz);

            double k3x = t * (vx + .5 * k2vx);
            double k3y = t * (vy + .5 * k2vy);
            double k3z = t * (vz + .5 * k2vz);
            double k3vx = t * (ax + .5 * k2vx);
            double k3vy = t * (ay + .5 * k2vy);
            double k3vz = t * (az + .5 * k2vz);

            double k4x = t * (vx + k3vx);
            double k4y = t * (vy + k3vy);
            double k4z = t * (vz + k3vz);
            double k4vx = t * (ax + k3vx);
            double k4vy = t * (ay + k3vy);
            double k4vz = t * (az + k3vz);

            x += (k1x + 2 * k2x + 2 * k3x + k4x) / 6;
            y += (k1y + 2 * k2y + 2 * k3y + k4y) / 6;
            z += (k1z + 2 * k2z + 2 * k3z + k4z) / 6;

            vx += (k1vx + 2 * k2vx + 2 * k3vx + k4vx) / 6;
            vy += (k1vy + 2 * k2vy + 2 * k3vy + k4vy) / 6;
            vz += (k1vz + 2 * k2vz + 2 * k3vz + k4vz) / 6;

            points.Add(new Vector3((float)x, (float)y, (float)z));

            time += t;
        }
    }
    float CalculateAirDensity(float altitude)
    {
        return 1.225f * airDensityVsAltitude.Evaluate(altitude / maxAltitude);
    }
    void CalculateInitialVelocityFixedTargetAngle(Vector3 initPos, float initAngle, Vector3 targetPos, float grav, ref Vector3 velocitySolution)
    {
        Vector3 targetXZPos = new Vector3(targetPos.x, 0, targetPos.z);
        Vector3 initXZPos = new Vector3(initPos.x, 0, initPos.z);

        float xzDistance = Vector3.Distance(initXZPos, targetXZPos);

        float height = targetPos.y - initPos.y;

        float tan = Mathf.Tan(initAngle * Mathf.Deg2Rad);

        float zVel = Mathf.Sqrt(grav * xzDistance * xzDistance / (2 * (height - xzDistance * tan)));
        float yVel = tan * zVel;

        velocitySolution = (targetXZPos - initXZPos).normalized * zVel;
        velocitySolution.y = yVel;
    }
    int CalculateInitialVelocityFixedTargetSpeed(Vector3 initPos, float initSpeed, Vector3 targetPos, float grav, bool fastShot, ref Vector3 velocitySolution, ref float angleSolution)
    {
        velocitySolution = Vector3.zero;
        angleSolution = 0;

        Vector3 diff = initPos - targetPos;
        Vector3 diffXZ = new Vector3(diff.x, 0f, diff.z);
        float groundDist = diffXZ.magnitude;

        float speed2 = initSpeed * initSpeed;
        float root = speed2 * speed2 - grav * (grav * groundDist * groundDist + 2 * diff.y * speed2);

        if (root < 0)
            return 0;

        root = Mathf.Sqrt(root);

        float gx = grav * groundDist;
        float lowAng = Mathf.Atan2(speed2 - root, gx);
        float highAng = Mathf.Atan2(speed2 + root, gx);
        int numSolutions = lowAng != highAng ? 2 : 1;

        Vector3 groundDir = diffXZ.normalized;

        if (fastShot)
        {
            velocitySolution = groundDir * Mathf.Cos(lowAng) * initSpeed + Vector3.up * Mathf.Sin(lowAng) * initSpeed;
            angleSolution = lowAng * Mathf.Rad2Deg;
        }
        else if (numSolutions > 1)
        {
            velocitySolution = groundDir * Mathf.Cos(highAng) * initSpeed + Vector3.up * Mathf.Sin(highAng) * initSpeed;
            angleSolution = highAng * Mathf.Rad2Deg;
        }

        return numSolutions;
    }
    int CalculateInitialVelocityMovingTarget(Vector3 initPos, float initSpeed, Vector3 targetPos, Vector3 targetVel, float grav, bool fastShot, ref Vector3 velocitySolution, ref float timeSolution)
    {
        velocitySolution = Vector3.zero;
        timeSolution = float.NaN;

        double tVelX = targetVel.x;
        double tVelY = targetVel.y;
        double tVelZ = targetVel.z;

        double xDiff = targetPos.x - initPos.x;
        double zDiff = targetPos.z - initPos.z;
        double yDiff = targetPos.y - initPos.y;

        double halfGrav = .5f * grav;

        double a = halfGrav * halfGrav;
        double b = -2 * tVelY * halfGrav;
        double c = tVelY * tVelY - 2 * yDiff * halfGrav - initSpeed * initSpeed + tVelX * tVelX + tVelZ * tVelZ;
        double d = 2 * yDiff * tVelY + 2 * xDiff * tVelX + 2 * zDiff * tVelZ;
        double e = yDiff * yDiff + xDiff * xDiff + zDiff * zDiff;

        double[] times = new double[4];
        Quartic(a, b, c, d, e, out times[0], out times[1], out times[2], out times[3]);

        Array.Sort(times);

        Vector3[] solutions = new Vector3[2];
        int numSolutions = 0;
        for (int i = 0; i < times.Length && numSolutions < 2; ++i)
        {
            double t = times[i];
            if (t <= 0 || double.IsNaN(t))
                continue;

            solutions[numSolutions].x = (float)((xDiff + tVelX * t) / t);
            solutions[numSolutions].y = (float)((yDiff + tVelY * t - halfGrav * t * t) / t);
            solutions[numSolutions].z = (float)((zDiff + tVelZ * t) / t);

            ++numSolutions;

            if (numSolutions == 1 && fastShot)
            {
                timeSolution = (float)t;
            }
            else if (!fastShot)
                timeSolution = (float)t;
        }

        if (numSolutions > 0 && fastShot)
            velocitySolution = solutions[0];
        else if (numSolutions > 1)
            velocitySolution = solutions[1];

        return numSolutions;
    }
    int Quartic(double c0, double c1, double c2, double c3, double c4, out double s0, out double s1, out double s2, out double s3)
    {
        s0 = s1 = s2 = s3 = double.NaN;

        double[] coeffs = new double[4];
        double z, u, v, sub;
        double A, B, C, D;
        double sq_A, p, q, r;
        int num;

        A = c1 / c0;
        B = c2 / c0;
        C = c3 / c0;
        D = c4 / c0;

        sq_A = A * A;
        p = -3.0 / 8 * sq_A + B;
        q = 1.0 / 8 * sq_A * A - 1.0 / 2 * A * B + C;
        r = -3.0 / 256 * sq_A * sq_A + 1.0 / 16 * sq_A * B - 1.0 / 4 * A * C + D;

        if (IsZero(r))
        {
            coeffs[3] = q;
            coeffs[2] = p;
            coeffs[1] = 0;
            coeffs[0] = 1;

            num = Cubic(coeffs[0], coeffs[1], coeffs[2], coeffs[3], out s0, out s1, out s2);
        }
        else
        {
            coeffs[3] = 1.0 / 2 * r * p - 1.0 / 8 * q * q;
            coeffs[2] = -r;
            coeffs[1] = -1.0 / 2 * p;
            coeffs[0] = 1;

            Cubic(coeffs[0], coeffs[1], coeffs[2], coeffs[3], out s0, out s1, out s2);

            z = s0;

            u = z * z - r;
            v = 2 * z - p;

            if (IsZero(u))
                u = 0;
            else if (u > 0)
                u = Math.Sqrt(u);
            else
                return 0;

            if (IsZero(v))
                v = 0;
            else if (v > 0)
                v = Math.Sqrt(v);
            else
                return 0;

            coeffs[2] = z - u;
            coeffs[1] = q < 0 ? -v : v;
            coeffs[0] = 1;

            num = Quadratic(coeffs[0], coeffs[1], coeffs[2], out s0, out s1);

            coeffs[2] = z + u;
            coeffs[1] = q < 0 ? v : -v;
            coeffs[0] = 1;

            if (num == 0) num += Quadratic(coeffs[0], coeffs[1], coeffs[2], out s0, out s1);
            else if (num == 1) num += Quadratic(coeffs[0], coeffs[1], coeffs[2], out s1, out s2);
            else if (num == 2) num += Quadratic(coeffs[0], coeffs[1], coeffs[2], out s2, out s3);
        }

        sub = 1.0 / 4 * A;

        if (num > 0) s0 -= sub;
        if (num > 1) s1 -= sub;
        if (num > 2) s2 -= sub;
        if (num > 3) s3 -= sub;

        return num;
    }
    int Cubic(double c0, double c1, double c2, double c3, out double s0, out double s1, out double s2)
    {
        s0 = double.NaN;
        s1 = double.NaN;
        s2 = double.NaN;

        int num;
        double sub;
        double A, B, C;
        double sq_A, p, q;
        double cb_p, D;

        A = c1 / c0;
        B = c2 / c0;
        C = c3 / c0;

        sq_A = A * A;
        p = 1.0 / 3 * (-1.0 / 3 * sq_A + B);
        q = 1.0 / 2 * (2.0 / 27 * A * sq_A - 1.0 / 3 * A * B + C);

        cb_p = p * p * p;
        D = q * q + cb_p;

        if (IsZero(D))
        {
            if (IsZero(q)) // triple solution
            {
                s0 = 0;
                num = 1;
            }
            else // single and double solution
            {
                double u = CubicRoot(-q);
                s0 = 2 * u;
                s1 = -u;
                num = 2;
            }
        }
        else if (D < 0) // 3 solutions
        {
            double phi = 1.0 / 3 * Math.Acos(-q / Math.Sqrt(-cb_p));
            double t = 2 * Math.Sqrt(-p);

            s0 = t * Math.Cos(phi);
            s1 = -t * Math.Cos(phi + Math.PI / 3);
            s2 = -t * Math.Cos(phi - Math.PI / 3);
            num = 3;
        }
        else // 1 solution
        {
            double sqrt_D = Math.Sqrt(D);
            double u = CubicRoot(sqrt_D - q);
            double v = -CubicRoot(sqrt_D + q);

            s0 = u + v;
            num = 1;
        }

        sub = 1.0 / 3 * A;

        if (num > 0) s0 -= sub;
        if (num > 1) s1 -= sub;
        if (num > 2) s2 -= sub;

        return num;
    }
    double CubicRoot(double value)
    {
        if (value > 0.0)
            return Math.Pow(value, 1.0 / 3.0);
        else if (value < 0)
            return -Math.Pow(-value, 1.0 / 3.0);
        else
            return 0.0;
    }
    int Quadratic(double c0, double c1, double c2, out double s0, out double s1)
    {
        s0 = double.NaN;
        s1 = double.NaN;

        double p, q, D;

        p = c1 / (2 * c0);
        q = c2 / c0;

        D = p * p - q;

        if (IsZero(D))
        {
            s0 = -p;
            return 1;
        }
        else if (D < 0)
            return 0;
        else
        {
            double sqrt_D = Math.Sqrt(D);

            s0 = sqrt_D - p;
            s1 = -sqrt_D - p;
            return 2;
        }
    }
    bool IsZero(double d)
    {
        return d > -zeroBuffer && d < zeroBuffer;
    }
}
