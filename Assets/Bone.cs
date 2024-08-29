using System.Runtime.CompilerServices;
using UnityEngine;

public class Bone : MonoBehaviour
{
    public Bone parent;
    public Bone child;

    public Vector3 localDOFBaseX;
    public Vector3 localDOFBaseY;

    public float minDOFYaw;
    public float maxDOFYaw;
    public float minDOFPitch;
    public float maxDOFPitch;
    public float rollLimit;


    private Vector3 _localBaseX;
    private Vector3 _localBaseY;
    private Vector3 _localBaseZ;
    private Vector3 _localPosition;

    private Vector3 _worldBaseX;
    private Vector3 _worldBaseY;
    private Vector3 _worldBaseZ;
    private Vector3 _position;

    private Quaternion _rotation;
    private Transform  _cachedTransform;
    private Quaternion _defaultLocalRotation;

    private void Awake()
    {
        _cachedTransform = transform;
        _defaultLocalRotation = transform.localRotation;
    }

    public void UpdateLocalTransform()
    {
        _localBaseX    = _cachedTransform.localRotation * Vector3.right;
        _localBaseY    = _cachedTransform.localRotation * Vector3.up;
        _localBaseZ    = _cachedTransform.localRotation * Vector3.forward;

        _localPosition = _cachedTransform.localPosition;


        if (child)
        {
            child.UpdateLocalTransform();
        }
    }

    public void UpdateWorldTransform(Vector3 basePostion)
    {
        if (parent)
        {
            _worldBaseX = MatrixMul(parent._worldBaseX, parent._worldBaseY, parent._worldBaseZ, _localBaseX);
            _worldBaseY = MatrixMul(parent._worldBaseX, parent._worldBaseY, parent._worldBaseZ, _localBaseY);
            _worldBaseZ = MatrixMul(parent._worldBaseX, parent._worldBaseY, parent._worldBaseZ, _localBaseZ);

            _rotation   = Quaternion.LookRotation(_worldBaseZ, _worldBaseY);

            _position   = MatrixMul(parent._worldBaseX, parent._worldBaseY, parent._worldBaseZ, _localPosition) + basePostion;
        }
        else
        {
            _worldBaseX = _cachedTransform.rotation * Vector3.right;
            _worldBaseY = _cachedTransform.rotation * Vector3.up;
            _worldBaseZ = _cachedTransform.rotation * Vector3.forward;

            _position   = transform.position;
            _rotation   = transform.rotation;
        }

        if (child)
        {
            child.UpdateWorldTransform(_position);
        }
    }

    public void ApplyTransform()
    {
        _cachedTransform.rotation = _rotation;
        _cachedTransform.position = _position;

        if (child)
        {
            child.ApplyTransform();
        }
    }

    public Bone GetTail()
    {
        if (child)
            return child.GetTail();

        return this;
    }

    public void CyclicCoordinateDescent(Vector3 worldTarget, Vector3 tail)
    {
        if (parent)
        {
            // 计算出世界空间下的DOF基向量
            var worldDOFBaseX      = parent._rotation * localDOFBaseX;
            var worldDOFBaseY      = parent._rotation * localDOFBaseY;
            var worldDOFBaseZ      = Vector3.Cross(worldDOFBaseX, worldDOFBaseY);

            //世界空间下的DOF旋转及其逆
            var worldDOFRotation   = Quaternion.LookRotation(worldDOFBaseZ, worldDOFBaseY);
            var inverseDOFRotation = Quaternion.Inverse(worldDOFRotation);
              
            var baseTarget         = worldTarget - _position;
            var baseTail           = tail - _position;

            //世界空间下的子节点位置
            var basebone           = _rotation * child._localPosition;

            // 将目标位置，末端点，和子节点位置变换到DOF Base
            var targetInDOFBase    = inverseDOFRotation * baseTarget;
            var tailInDOFBase      = inverseDOFRotation * baseTail;
            var boneInDOFBase      = inverseDOFRotation * basebone;

            //理想情况下的水平旋转变换
            var form = new Vector3(tailInDOFBase.x, 0, tailInDOFBase.z);
            var to   = new Vector3(targetInDOFBase.x, 0, targetInDOFBase.z);
            var rot1 = Quaternion.FromToRotation(form, to);

            //理想情况下的垂直旋转变换
            var tempTail    = rot1 * tailInDOFBase;
            var rot2        = Quaternion.FromToRotation(tempTail, targetInDOFBase);

            // 将变换作用到bone上，计算出理想情况下的bone位置
            var perfectRot  = worldDOFRotation * rot2 * rot1 * inverseDOFRotation;
            var perfectBone = perfectRot * basebone;

            // 变换到DOF Base下
            var perfectBoneInDOFBase = inverseDOFRotation * perfectBone;

            var optimalDirInDOFBase = Vector3.zero;
            var optimalRot          = Quaternion.identity;

            // 将perfectBoneInDOFBase分解到水平面
            var perfectBoneInDOFBaseZX = new Vector2(perfectBoneInDOFBase.z, perfectBoneInDOFBase.x);
            if (perfectBoneInDOFBaseZX != Vector2.zero)
            {
                // 计算出水平面约束条件下的最优向量
                var zx   = ConstraintRotation2D(perfectBoneInDOFBaseZX, minDOFYaw, maxDOFYaw);

                // 以水平最优向量和Vector3.up向量确定一个Base
                var az   = new Vector3(zx.y, 0, zx.x).normalized;
                var ay   = Vector3.up;
                var ax   = Vector3.Cross(ay, az);

                //将perfectBoneInDOFBase变换到Base内
                var bone = MatrixInverseMul(ax, ay, az, perfectBoneInDOFBase);
                var v2   = new Vector2(bone.z, bone.y);

                // 计算垂直面约束条件下的
                var zy   = ConstraintRotation2D(v2, minDOFPitch, maxDOFPitch);

                //在DOF Base下最优的方向
                optimalDirInDOFBase = MatrixMul(ax, ay, az, new Vector3(0, zy.y, zy.x));

                // 水平方向上的最优旋转变换
                var optimalRot1 = Quaternion.FromToRotation(new Vector3(boneInDOFBase.x, 0, boneInDOFBase.z), az);
                // 相等表示水平方向没有自由度
                if (minDOFYaw == maxDOFYaw)
                {
                    optimalRot1 = Quaternion.identity;
                }

                // 垂直方向上的最优旋转变换
                var tempBone    = optimalRot1 * boneInDOFBase;
                var optimalRot2 = Quaternion.FromToRotation(tempBone, optimalDirInDOFBase);

                // 相等表示垂直方向没有自由度
                if (minDOFPitch == maxDOFPitch)
                {
                    optimalRot1 = Quaternion.identity;
                }

                // 最终的旋转变换
                optimalRot = optimalRot2 * optimalRot1;
            }
            else
            {
                //如果不能水平分解则只计算垂直旋转
                var ax  = Vector3.Cross(Vector3.up, boneInDOFBase).normalized;
                if (ax != Vector3.zero)
                {
                    var az = Vector3.Cross(ax, Vector3.up).normalized;
                    var zy = ConstraintRotation2D(Vector2.up, minDOFYaw, maxDOFYaw);

                    optimalDirInDOFBase = MatrixMul(ax, Vector3.up, az, new Vector3(0, zy.y, zy.x));
                    optimalRot          = Quaternion.FromToRotation(boneInDOFBase, optimalDirInDOFBase);
                }
            }

            // 先变换到DOF Base，然后作用最终的旋转变化，最后在变换到世界空间
            optimalRot  = worldDOFRotation * optimalRot * inverseDOFRotation;

            _worldBaseX = optimalRot * _worldBaseX;
            _worldBaseY = optimalRot * _worldBaseY;
            _worldBaseZ = optimalRot * _worldBaseZ;

            _localBaseX = MatrixInverseMul(parent._worldBaseX, parent._worldBaseY, parent._worldBaseZ, _worldBaseX);
            _localBaseY = MatrixInverseMul(parent._worldBaseX, parent._worldBaseY, parent._worldBaseZ, _worldBaseY);
            _localBaseZ = MatrixInverseMul(parent._worldBaseX, parent._worldBaseY, parent._worldBaseZ, _worldBaseZ);

            tail = optimalRot * baseTail + _position;

            parent.CyclicCoordinateDescent(worldTarget, tail);

        }
    }

    public void ForwardwardReachingUpdate()
    {
        if (parent)
        {
            if (child)
            {
                _localBaseX = MatrixInverseMul(parent._worldBaseX, parent._worldBaseY, parent._worldBaseZ, _worldBaseX);
                _localBaseY = MatrixInverseMul(parent._worldBaseX, parent._worldBaseY, parent._worldBaseZ, _worldBaseY);
                _localBaseZ = MatrixInverseMul(parent._worldBaseX, parent._worldBaseY, parent._worldBaseZ, _worldBaseZ);
                _rotation = Quaternion.LookRotation(_worldBaseZ, _worldBaseY);
            }
        }

        if (child)
        {
            child.ForwardwardReachingUpdate();
        }
    }

    public void ForwardwardReaching1(Vector3 worldTarget)
    {
        if (parent)
        {
            if (child)
            {
                var worldDOFBaseX = parent._rotation * localDOFBaseX;
                var worldDOFBaseY = parent._rotation * localDOFBaseY;
                var worldDOFBaseZ = Vector3.Cross(worldDOFBaseX, worldDOFBaseY);

                //世界空间下的DOF旋转及其逆
                var worldDOFRotation = Quaternion.LookRotation(worldDOFBaseZ, worldDOFBaseY);
                var inverseDOFRotation = Quaternion.Inverse(worldDOFRotation);

                var baseBone = _rotation * child._localPosition;
                var perfectDir = child._position - worldTarget;

                //Debug.DrawRay(_position, baseBone, Color.red, 10);
                //Debug.DrawRay(_position, perfectDir, Color.black, 10);


                var boneInDOFBase = worldDOFRotation * baseBone;
                var perfectDirInDOFBase = worldDOFRotation * perfectDir;

                var optimalRotInDOFBase = CalculateOptimalRotationInDOFBase(boneInDOFBase, perfectDirInDOFBase);
                var optimalRot = worldDOFRotation * optimalRotInDOFBase * inverseDOFRotation;

                _rotation = optimalRot * _rotation;
                baseBone = _rotation * child._localPosition;
                _position = child._position - baseBone;

            }
            else
            {
                _position = worldTarget;
            }

            worldTarget = _position + (parent._position - _position).normalized * _localPosition.magnitude;
            parent.ForwardwardReaching1(worldTarget);
        }
        else
        {
            _position = worldTarget;
        }
    }

    public Quaternion CalculateRollLimit(Quaternion inverseDOFRotation, Quaternion quaternion, Quaternion defaultLocal)
    {
        var quaternionInDOFBase = inverseDOFRotation * quaternion;
        var defaultLocalInDOFBase = inverseDOFRotation * defaultLocal;

        var axis1 = quaternionInDOFBase * Vector3.forward;
        var axis2 = defaultLocalInDOFBase * Vector3.forward;

        var toLimit = Quaternion.identity;
        if (Vector3.Angle(axis1, axis2) > rollLimit)
        {
            toLimit = Quaternion.FromToRotation(axis1, axis2);
        }

        return Quaternion.Inverse(inverseDOFRotation) * toLimit * quaternionInDOFBase;
    }

    public void BackwardReaching(Vector3 worldTarget)
    {
        //Debug.DrawRay(worldTarget, Vector3.up, Color.red, 10);

        if (parent)
        {
            var worldDOFBaseX = parent._rotation * localDOFBaseX;
            var worldDOFBaseY = parent._rotation * localDOFBaseY;
            var worldDOFBaseZ = Vector3.Cross(worldDOFBaseX, worldDOFBaseY);

            //世界空间下的DOF旋转及其逆
            var worldDOFRotation = Quaternion.LookRotation(worldDOFBaseZ, worldDOFBaseY);
            var inverseDOFRotation = Quaternion.Inverse(worldDOFRotation);


            if (parent.parent == null)
            {
                _rotation = CalculateRollLimit(inverseDOFRotation, _rotation, parent._rotation * _defaultLocalRotation);

                _worldBaseX = _rotation * Vector3.right;
                _worldBaseY = _rotation * Vector3.up;
                _worldBaseZ = _rotation * Vector3.forward;
            }
            else
            {
                _worldBaseX = parent._rotation * _localBaseX;
                _worldBaseY = parent._rotation * _localBaseY;
                _worldBaseZ = parent._rotation * _localBaseZ;
            }


            if (child)
            {
                var length      = child._localPosition.magnitude;
                var perfectBone = (child._position - worldTarget).normalized * length;
                var baseBone    = MatrixMul(_worldBaseX, _worldBaseY, _worldBaseZ, child._localPosition);

                var perfectBoneInDOFBase = inverseDOFRotation * perfectBone;
                var baseBoneInDOFBase = inverseDOFRotation * baseBone;

                //Debug.DrawRay(worldTarget, baseBone * 10, Color.green, 10);
                //Debug.DrawRay(worldTarget, perfectBone * 5, Color.blue, 10);

                var optimalRot = CalculateOptimalRotationInDOFBase(baseBoneInDOFBase, perfectBoneInDOFBase);
                optimalRot = worldDOFRotation * optimalRot * inverseDOFRotation;

                //Debug.DrawRay(worldTarget, optimalRot * baseBone * 10, Color.black, 10);


                _position = worldTarget;
                _worldBaseX = optimalRot * _worldBaseX;
                _worldBaseY = optimalRot * _worldBaseY;
                _worldBaseZ = optimalRot * _worldBaseZ;
                _rotation = Quaternion.LookRotation(_worldBaseZ, _worldBaseY);

                _localBaseX = MatrixInverseMul(parent._worldBaseX, parent._worldBaseY, parent._worldBaseZ, _worldBaseX);
                _localBaseY = MatrixInverseMul(parent._worldBaseX, parent._worldBaseY, parent._worldBaseZ, _worldBaseY);
                _localBaseZ = MatrixInverseMul(parent._worldBaseX, parent._worldBaseY, parent._worldBaseZ, _worldBaseZ);

                worldTarget = _rotation * child._localPosition + _position;
                child.BackwardReaching(worldTarget);
            }
            else
            {
                _rotation = Quaternion.LookRotation(_worldBaseZ, _worldBaseY);
                _position = worldTarget;
            }
        }
        else
        {
            _rotation = _cachedTransform.rotation;
            _localBaseX = _cachedTransform.localRotation * Vector3.right;
            _localBaseY = _cachedTransform.localRotation * Vector3.up;
            _localBaseZ = _cachedTransform.localRotation * Vector3.forward;

            _worldBaseX = _rotation * Vector3.right;
            _worldBaseY = _rotation * Vector3.up;
            _worldBaseZ = _rotation * Vector3.forward;


            //Debug.DrawRay(worldTarget, _worldBaseX, Color.red, 10);
            //Debug.DrawRay(worldTarget, _worldBaseY, Color.green, 10);
            //Debug.DrawRay(worldTarget, _worldBaseZ, Color.black, 10);
            _position = worldTarget;
            worldTarget = _rotation * child._localPosition + _position;
            child.BackwardReaching(worldTarget);
        }
    }

    public void ForwardwardReaching(Vector3 worldTarget, Quaternion optimalRot)
    {
        var lastPosition = _position;

        _position = worldTarget;
        _worldBaseX = optimalRot * _worldBaseX;
        _worldBaseY = optimalRot * _worldBaseY;
        _worldBaseZ = optimalRot * _worldBaseZ;

        if (child)
        {
            if (parent)
            {
                var worldDOFBaseX = parent._rotation * localDOFBaseX;
                var worldDOFBaseY = parent._rotation * localDOFBaseY;
                var worldDOFBaseZ = Vector3.Cross(worldDOFBaseX, worldDOFBaseY);

                // 作用最优旋转后的DOF Base
                worldDOFBaseX = optimalRot * worldDOFBaseX;
                worldDOFBaseY = optimalRot * worldDOFBaseY;
                worldDOFBaseZ = optimalRot * worldDOFBaseZ;

                //世界空间下的DOF旋转及其逆
                var worldDOFRotation = Quaternion.LookRotation(worldDOFBaseZ, worldDOFBaseY);
                var inverseDOFRotation = Quaternion.Inverse(worldDOFRotation);

                var baseBone = _rotation * child._localPosition;
                var parentBone = parent._rotation * -_localPosition;

                var baseBone1 = optimalRot * baseBone;
                var parentBone1 = optimalRot * parentBone;
                var length = _localPosition.magnitude;
                var perfectParentBone = (parent._position - worldTarget).normalized * length;

                //Debug.DrawRay(worldTarget, baseBone1, Color.yellow, 10);
                //Debug.DrawRay(worldTarget, parentBone1, Color.gray, 10);
                //Debug.DrawRay(worldTarget, perfectParentBone, Color.white, 10);

                var perfectParentBoneInDOFBase = inverseDOFRotation * perfectParentBone;
                var baseBone1InDOFBase = inverseDOFRotation * baseBone1;
                var parentBone1InDOFBase = inverseDOFRotation * parentBone1;

                //理想情况下的水平旋转变换
                var form = new Vector3(parentBone1InDOFBase.x, 0, parentBone1InDOFBase.z);
                var to = new Vector3(perfectParentBoneInDOFBase.x, 0, perfectParentBoneInDOFBase.z);
                var rot1 = Quaternion.FromToRotation(form, to);

                //理想情况下的垂直旋转变换
                var tempParentBone1 = rot1 * parentBone1InDOFBase;
                var rot2 = Quaternion.FromToRotation(tempParentBone1, perfectParentBoneInDOFBase);
                var perfectRot = rot2 * rot1;
                var perfectRotInverse = Quaternion.Inverse(perfectRot);

                var perfectBone1InDOFBase = perfectRotInverse * baseBone1InDOFBase;
                //Debug.DrawRay(worldTarget, worldDOFRotation * perfectBone1InDOFBase, Color.black, 10);

                //Debug.DrawRay(worldTarget, worldDOFBaseX, Color.red, 10);
                //Debug.DrawRay(worldTarget, worldDOFBaseY, Color.blue, 10);
                //Debug.DrawRay(worldTarget, worldDOFBaseZ, Color.green, 10);

                var optimalRot1Inverse = CalculateOptimalRotationInDOFBase(baseBone1InDOFBase, perfectBone1InDOFBase);
                var optimalRot1 = Quaternion.Inverse(optimalRot1Inverse);

                optimalRot1 = worldDOFRotation * optimalRot1 * inverseDOFRotation;
                var optimalParentBone1 = optimalRot1 * parentBone1;
                var worldTarget1 = worldTarget + optimalParentBone1;
                optimalRot1 = optimalRot1 * optimalRot;

                parent.ForwardwardReaching(worldTarget1, optimalRot1);
            }
        }
        else
        {
            var from = lastPosition - parent._position;
            var to = worldTarget - parent._position;   
            var length = _localPosition.magnitude;
            worldTarget = worldTarget - to.normalized * length;
            optimalRot = Quaternion.FromToRotation(from, to);

            parent.ForwardwardReaching(worldTarget, optimalRot);
        }
    }

    private Quaternion CalculateOptimalRotationInDOFBase(Vector3 boneInDOFBase, Vector3 perfectBoneInDOFBase)
    {
        var optimalDirInDOFBase = Vector3.zero;
        var optimalRot = Quaternion.identity;

        // 将perfectBoneInDOFBase分解到水平面
        var perfectBoneInDOFBaseZX = new Vector2(perfectBoneInDOFBase.z, perfectBoneInDOFBase.x);
        if (perfectBoneInDOFBaseZX != Vector2.zero)
        {
            // 计算出水平面约束条件下的最优向量
            var zx = ConstraintRotation2D(perfectBoneInDOFBaseZX, minDOFYaw, maxDOFYaw);

            // 以水平最优向量和Vector3.up向量确定一个Base
            var az = new Vector3(zx.y, 0, zx.x).normalized;
            var ay = Vector3.up;
            var ax = Vector3.Cross(ay, az);

            //将perfectBoneInDOFBase变换到Base内
            var bone = MatrixInverseMul(ax, ay, az, perfectBoneInDOFBase);
            var v2   = new Vector2(bone.z, bone.y);

            // 计算垂直面约束条件下的
            var zy = ConstraintRotation2D(v2, minDOFPitch, maxDOFPitch);

            //在DOF Base下最优的方向
            optimalDirInDOFBase = MatrixMul(ax, ay, az, new Vector3(0, zy.y, zy.x));

            // 水平方向上的最优旋转变换
            var optimalRot1 = Quaternion.FromToRotation(new Vector3(boneInDOFBase.x, 0, boneInDOFBase.z), az);
            // 相等表示水平方向没有自由度
            if (minDOFYaw == maxDOFYaw)
            {
                optimalRot1 = Quaternion.identity;
            }

            // 垂直方向上的最优旋转变换
            var tempBone    = optimalRot1 * boneInDOFBase;
            var optimalRot2 = Quaternion.FromToRotation(tempBone, optimalDirInDOFBase);

            // 相等表示垂直方向没有自由度
            if (minDOFPitch == maxDOFPitch)
            {
                optimalRot1 = Quaternion.identity;
            }

            // 最终的旋转变换
            optimalRot = optimalRot2 * optimalRot1;
        }
        else
        {
            //如果不能水平分解则只计算垂直旋转
            var ax  = Vector3.Cross(Vector3.up, boneInDOFBase).normalized;
            if (ax != Vector3.zero)
            {
                var az = Vector3.Cross(ax, Vector3.up).normalized;
                var zy = ConstraintRotation2D(Vector2.up, minDOFYaw, maxDOFYaw);

                optimalDirInDOFBase = MatrixMul(ax, Vector3.up, az, new Vector3(0, zy.y, zy.x));
                optimalRot          = Quaternion.FromToRotation(boneInDOFBase, optimalDirInDOFBase);
            }
        }

        return optimalRot;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private Vector3 MatrixMul(Vector3 x, Vector3 y, Vector3 z, Vector3 p)
    {
        return p.x * x + p.y * y + p.z * z;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private Vector3 MatrixInverseMul(Vector3 x, Vector3 y, Vector3 z, Vector3 p)
    {
        return new Vector3(Vector3.Dot(x, p),
                           Vector3.Dot(y, p),
                           Vector3.Dot(z, p));
    }

    /// <param name="to">目标向量</param>
    /// <param name="angleMin">角度区域开始[-180, 180]</param>
    /// <param name="angleMax">角度区域结束[-180, 180]， 结束角度大于开始</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private Vector2 ConstraintRotation2D(Vector2 to, float angleMin, float angleMax)
    {
        var radMin  = angleMin * Mathf.Deg2Rad;
        var radMax  = angleMax * Mathf.Deg2Rad;
        var minLine = new Vector2(Mathf.Cos(radMin), Mathf.Sin(radMin));
        var maxLine = new Vector2(Mathf.Cos(radMax), Mathf.Sin(radMax));

        var toRad   = Mathf.Atan2(to.y, to.x);
        var diffMin = toRad - radMin;
        var diffMax = radMax - toRad;

        // 要旋转到的目标在限制区域内时直接返回，否则返回最靠近的边界线
        return (diffMin >= 0 && diffMax >= 0 ? to : Vector2.Dot(to, minLine) >= Vector2.Dot(to, maxLine) ? minLine : maxLine).normalized;
    }
}