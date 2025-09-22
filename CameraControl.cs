using UnityEngine;

[ExecuteInEditMode]
public class CameraControl : MonoBehaviour
{
    [Header("Camera")]
    public Camera cam;
    [SerializeField] [Range(1, 100)] float minCamDistance = 5;
    [SerializeField] float camDistance = 20;
    [SerializeField] [Range(1, 100)] float maxCamDistance = 50;
    [SerializeField] bool sightLine = true;
    [SerializeField] [Range(.1f, 2)] float camWallDistance = .25f;
    [SerializeField] float raycastDistance = 1000;
    Plane plane;
    [SerializeField] Transform camBoundsParent;
    CameraBounds currentBounds;
    [SerializeField] Transform followTarget;
    public bool move = true;
    public bool rotate = true;
    public bool zoom = true;
    [Range(-90, 90)] [SerializeField] float minAngle = -60;
    [SerializeField] float xRotation = 0;
    [Range(-90, 90)] [SerializeField] float maxAngle = 0;
    [SerializeField] bool slide = true;
    public Vector3 slidePosition;
    [SerializeField] float slideLerp = .1f;
    [SerializeField] float slideMultiplier = .1f;
    [SerializeField] bool stayYHeight = true;
    [SerializeField] float yHeight = 0;

    [Header("Input")]
    [Range(0, 2)] [SerializeField] float moveSensitivity = .5f;
    [Range(0, 5)] [SerializeField] float xSensitivity = 1;
    [Range(0, 5)] [SerializeField] float ySensitivity = .2f;
    [Range(0, .5f)] [SerializeField] float zoomSensitivity = .02f;
    [SerializeField] float kbmMoveSensitivity = .5f;
    [SerializeField] float kbmXSensitivity = 3;
    [SerializeField] float kbmYSensitivity = 7;
    [SerializeField] float kbmZoomSensitivity = 5;

    [Header("UI")]
    public bool centerUI = true;
    public bool destinationUI = true;
    public bool fingerUI = true;

    public float CamDistance
    {
        get
        {
            return camDistance;
        }
        private set
        {
            if (value < minCamDistance)
            {
                camDistance = minCamDistance;
            }
            else if (value > maxCamDistance)
            {
                camDistance = maxCamDistance;
            }
            else
            {
                camDistance = value;
            }
        }
    }

    void Awake()
    {
        SetupCamera();
        SetupCameraBounds();

        SetupPlane();

        //Load();
    }
    void Update()
    {
        UpdateMovement();
        UpdateRotationAndZoom();

        UpdateCameraPosition();

        UpdateRaycast();
        CameraForwardRaycast();
    }
    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.blue;
        Gizmos.DrawWireSphere(transform.position, 1);

        if (currentBounds)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawWireCube(currentBounds.Center, currentBounds.size);
        }
        else
            SetupCameraBounds();

        Gizmos.color = Color.cyan;
        Gizmos.DrawWireSphere(transform.position, camDistance);

        Gizmos.color = Color.green;
        for (int i = 0; i < 4; i++)
        {
            float x = i < 2 ? 1 : -1;
            float z = i % 2 == 0 ? 1 : -1;
            Gizmos.DrawLine(transform.position, transform.position + new Vector3(x, 0, z) * 5);
        }
    }

    void SetupCamera()
    {
        if (!cam)
            cam = Camera.main;

        if (!cam.gameObject.activeSelf)
            cam.gameObject.SetActive(true);
    }
    void SetupCameraBounds()
    {
        if (camBoundsParent.childCount > 0)
            currentBounds = camBoundsParent.GetChild(0).GetComponent<CameraBounds>();
    }
    public void Enable(bool active)
    {
        move = rotate = zoom = active;
    }
    void UpdateMovement()
    {
        if (move)
        {
            if (currentBounds && currentBounds.enabled)
            {
                if (yHeight < currentBounds.Min.y)
                    yHeight = currentBounds.Min.y;
                else if (yHeight > currentBounds.Max.y)
                    yHeight = currentBounds.Max.y;
            }

            if (followTarget)
            {
                Vector3 pos = followTarget.position;
                if (stayYHeight)
                    pos.y = yHeight;

                SetPosition(ClampToBoundary(pos));
            }
            else
            {
                // keyboard input
                Vector3 forward = Vector3.Cross(cam.transform.right, Vector3.up).normalized;
                Vector3 direction = forward * Input.GetAxis("Vertical") + cam.transform.right * Input.GetAxis("Horizontal");
                Vector3 newPosition = transform.position + direction * kbmMoveSensitivity;

                if (direction != Vector3.zero)
                {
                    newPosition = ClampToBoundary(newPosition);
                    transform.position = newPosition;
                    slidePosition = newPosition;
                }
                else if (Input.touchCount == 1) // touch input
                {
                    Touch touch = Input.GetTouch(0);
                    Vector2 delta = touch.deltaPosition;

                    Ray now = cam.ScreenPointToRay(touch.position);
                    Ray before = cam.ScreenPointToRay(touch.position - delta);

                    // get direction from touch raycasted positions
                    if (plane.Raycast(before, out float beforeDistance) && plane.Raycast(now, out float nowDistance))
                    {
                        direction = before.GetPoint(beforeDistance) - now.GetPoint(nowDistance);

                        // stay at y height
                        //if (stayYHeight)
                        //    direction.y = yHeight;

                        // get new position and clamp it
                        newPosition = ClampToBoundary(transform.position + direction * moveSensitivity);
                        if (stayYHeight)
                            newPosition.y = yHeight;

                        // move to new position
                        if (Input.GetTouch(0).phase == TouchPhase.Moved)
                            transform.position = newPosition;

                        // calculate slide after touch and clamp values
                        slidePosition = ClampToBoundary(newPosition + direction * delta.magnitude * slideMultiplier);
                    }
                }
            }
            
            if (slide)
            {
                if (stayYHeight)
                    slidePosition.y = yHeight;
                Vector3 pos = Vector3.Lerp(transform.position, slidePosition, slideLerp);
                transform.position = pos;
            }
        }
    }
    void UpdateRotationAndZoom()
    {
        // mouse input
        float xInput = 0;
        float yInput = 0;
        float zoomInput = Input.GetAxis("Mouse ScrollWheel");
        if (zoomInput != 0)
            zoomInput = (zoomInput > 0 ? -1 : 1) * kbmZoomSensitivity;

        if (Input.GetMouseButton(1))
        {
            xInput = Input.GetAxis("Mouse X") * kbmXSensitivity;
            yInput = Input.GetAxis("Mouse Y") * kbmYSensitivity;
        }

        // touch input
        if (Input.touchCount > 1)
        {
            // get deltas and positions
            Vector2 delta1 = Input.GetTouch(0).deltaPosition;
            Vector2 delta2 = Input.GetTouch(1).deltaPosition;

            if (delta1 == Vector2.zero || delta2 == Vector2.zero)
                return;

            Vector2 pos1Now = Input.GetTouch(0).position;
            Vector2 pos2Now = Input.GetTouch(1).position;

            Vector2 now = pos1Now - pos2Now;
            Vector2 before = pos1Now - delta1 - pos2Now + delta2;

            if (rotate)
            {
                // x
                float angleBetweenVectors = AngleBetweenVectors(now, before);

                Vector3 nowRight = Vector3.Cross(now, new Vector3(0, 0, 1));
                float cosNowRightBefore = Vector2.Dot(nowRight, before) / (nowRight.magnitude * before.magnitude);
                float clockwise = 1;
                if (cosNowRightBefore < 0)
                    clockwise = -1;

                xInput = angleBetweenVectors * clockwise * xSensitivity;

                // y
                yInput = (delta1.y + delta2.y) / 2f * ySensitivity;
            }

            if (zoom)
                zoomInput = (before.magnitude - now.magnitude) * zoomSensitivity;
        }

        if (rotate)
            Rotate(xInput, yInput);

        if (zoom)
            Zoom(zoomInput);
    }
    void UpdateCameraPosition()
    {
        // calculate camera position
        Vector3 origin = transform.position;
        Vector3 camDirection = cam.transform.position - transform.position;
        Vector3 camNewPosition = origin + camDirection.normalized * camDistance;

        // check if anything blocks camera direction
        if (sightLine)
        {
            Ray r = new Ray(origin, camDirection);
            r.origin = r.GetPoint(minCamDistance);
            float raycastDistance = Vector3.Distance(r.origin, camNewPosition);

            if (Physics.Linecast(r.origin, r.origin + camDirection.normalized * raycastDistance, out RaycastHit rch))
            {
                float distance = Vector3.Distance(r.origin, rch.point);
                distance -= camWallDistance;
                camNewPosition = r.GetPoint(distance);
            }
        }

        cam.transform.position = camNewPosition;

        Debug.DrawLine(origin, camNewPosition, Color.white);
        Debug.DrawLine(origin, origin + transform.up, Color.red);
    }
    void SetupPlane()
    {
        plane.SetNormalAndPosition(Vector3.up, transform.position);
    }
    void Rotate(float xInput, float yInput)
    {
        // rotate horizontally about y axis
        try
        {
            //transform.Rotate(new Vector3(0, xInput, 0), Space.World);
            transform.RotateAround(transform.position, Vector3.up, xInput);
        }
        catch (System.Exception)
        {
            return;
        }

        // rotate vertically about x axis and keep local euler angles
        xRotation -= yInput;
        xRotation = Mathf.Clamp(xRotation, minAngle, maxAngle);
        transform.localEulerAngles = new Vector3(xRotation, transform.localEulerAngles.y, 0);
    }
    void Zoom(float zoomInput)
    {
        CamDistance += zoomInput;
    }
    void UpdateRaycast()
    {
        if (Input.GetMouseButtonDown(0))
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            Vector3 end = ray.origin + ray.direction * raycastDistance;
            Raycast(ray, raycastDistance, ref end);

            float duration = 3;
            Debug.DrawLine(cam.transform.position, ray.origin, Color.yellow, duration);
            Debug.DrawLine(ray.origin, end, Color.red, duration);
        }
    }
    bool Raycast(Ray ray, float distance, ref Vector3 v)
    {
        // check if raycast hit an object or the plane, or nothing
        bool b = false;

        // check for object
        if (Physics.Raycast(ray, out RaycastHit rch, distance))
        {
            v = rch.point;
            b = true;
        }

        // check for plane
        if (plane.Raycast(ray, out float enter))
        {
            if (enter <= distance)
            {
                v = ray.GetPoint(enter);
                b = true;
            }
        }
        return b;
    }
    Vector3 CameraForwardRaycast()
    {
        Vector3 end = cam.transform.position + cam.transform.forward * raycastDistance;
        Ray cameraFw = new Ray(cam.transform.position, cam.transform.forward);
        Raycast(cameraFw, raycastDistance, ref end);

        Debug.DrawLine(cam.transform.position, end, Color.blue);

        return end;
    }
    float AngleBetweenVectors(Vector3 v1, Vector3 v2)
    {
        return Mathf.Acos(Vector3.Dot(v1, v2) / (v1.magnitude * v2.magnitude)) * 180 / Mathf.PI;
    }
    public void SetPosition(Vector3 position)
    {
        if (slide)
            slidePosition = position;
        else
            transform.position = position;
    }
    public void ChangeBoundary(CameraBounds newBoundary)
    {
        currentBounds = newBoundary;
        SetPosition(new Vector3(currentBounds.Center.x, 0, currentBounds.Center.z));
    }
    public void NextBoundary()
    {
        int index;
        for (int i = 0; i < camBoundsParent.childCount; i++)
        {
            if (camBoundsParent.GetChild(i) == currentBounds.transform)
            {
                if (i == camBoundsParent.childCount - 1)
                    index = 0;
                else
                    index = ++i;
                currentBounds = camBoundsParent.GetChild(index).GetComponent<CameraBounds>();
                SetPosition(new Vector3(currentBounds.Center.x, 0, currentBounds.Center.z));
                return;
            }
        }
    }
    Vector3 ClampToBoundary(Vector3 position)
    {
        //if (stayYHeight)
        //{
        //    if (currentBounds)
        //    {
        //        if (yHeight < currentBounds.Min.y)
        //            yHeight = currentBounds.Min.y;
        //        else if (yHeight > currentBounds.Max.y)
        //            yHeight = currentBounds.Max.y;
        //    }

        //    slidePosition.y = yHeight;
        //}

        if (currentBounds)
            position = currentBounds.ClampToBoundary(position);

        return position;
    }
    //void Save()
    //{
    //    if (GameManager.Instance)
    //    {
    //        Vector3 v = transform.position;
    //        GameManager.Instance.Data.position = new Vector3Token(v.x, v.y, v.z);

    //        v = transform.rotation.eulerAngles;
    //        GameManager.Instance.Data.rotation = new Vector3Token(v.x, v.y, v.z);
    //    }
    //}
    //void Load()
    //{
    //    if (GameManager.Instance)
    //    {
    //        GameManager.Instance.saveLoad.OnSave += Save;

    //        Vector3Token v = GameManager.Instance.Data.position;
    //        SetPosition(new Vector3(v.x, v.y, v.z));

    //        v = GameManager.Instance.Data.rotation;
    //        xRotation = v.x - 360;
    //        transform.eulerAngles = new Vector3(xRotation, v.y, v.z);
    //    }
    //}
}
