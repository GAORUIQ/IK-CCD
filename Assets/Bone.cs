using System.Runtime.CompilerServices;
using UnityEngine;

public class Bone : MonoBehaviour
{
    public Bone parent;
    public Bone child;

    public Vector3 localFODBaseX;
    public Vector3 localFODBaseY;

    public float minFODYaw;
    public float maxFODYaw;
    public float minFODPitch;
    public float maxFODPitch;


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

    private void Awake()
    {
        _cachedTransform = transform;
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
        if (child)
        {
            _cachedTransform.rotation = _rotation;
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
            // 计算出世界空间下的FOD基向量
            var worldFODBaseX      = parent._rotation * localFODBaseX;
            var worldFODBaseY      = parent._rotation * localFODBaseY;
            var worldFODBaseZ      = Vector3.Cross(worldFODBaseX, worldFODBaseY);

            //世界空间下的FOD旋转及其逆
            var worldFODRotation   = Quaternion.LookRotation(worldFODBaseZ, worldFODBaseY);
            var inverseFODRotation = Quaternion.Inverse(worldFODRotation);
              
            var baseTarget         = worldTarget - _position;
            var baseTail           = tail - _position;

            //世界空间下的子节点位置
            var basebone           = _rotation * child._localPosition;

            // 将目标位置，末端点，和子节点位置变换到FOD Base
            var targetInFODBase    = inverseFODRotation * baseTarget;
            var tailInFODBase      = inverseFODRotation * baseTail;
            var boneInFODBase      = inverseFODRotation * basebone;

            //理想情况下的水平旋转变换
            var form = new Vector3(tailInFODBase.x, 0, tailInFODBase.z);
            var to   = new Vector3(targetInFODBase.x, 0, targetInFODBase.z);
            var rot1 = Quaternion.FromToRotation(form, to);

            //理想情况下的垂直旋转变换
            var tempTail    = rot1 * tailInFODBase;
            var rot2        = Quaternion.FromToRotation(tempTail, targetInFODBase);

            // 将变换作用到bone上，计算出理想情况下的bone位置
            var perfectRot  = worldFODRotation * rot2 * rot1 * inverseFODRotation;
            var perfectBone = perfectRot * basebone;

            // 变换到FOD Base下
            var perfectBoneInFODBase = inverseFODRotation * perfectBone;

            var optimalDirInFODBase = Vector3.zero;
            var optimalRot = Quaternion.identity;

            // 将perfectBoneInFODBase分解到水平面
            var perfectBoneInFODBaseZX = new Vector2(perfectBoneInFODBase.z, perfectBoneInFODBase.x);
            if (perfectBoneInFODBaseZX != Vector2.zero)
            {
                // 计算出水平面约束条件下的最优向量
                var zx   = ConstraintRotation2D(perfectBoneInFODBaseZX, minFODYaw, maxFODYaw);

                // 以水平最优向量和Vector3.up向量确定一个Base
                var az   = new Vector3(zx.y, 0, zx.x).normalized;
                var ay   = Vector3.up;
                var ax   = Vector3.Cross(ay, az);

                //将perfectBoneInFODBase变换到Base内
                var bone = MatrixInverseMul(ax, ay, az, perfectBoneInFODBase);
                var v2   = new Vector2(bone.z, bone.y);

                // 计算垂直面约束条件下的
                var zy   = ConstraintRotation2D(v2, minFODPitch, maxFODPitch);

                //在FOD Base下最优的方向
                optimalDirInFODBase = MatrixMul(ax, ay, az, new Vector3(0, zy.y, zy.x));

                // 水平方向上的最优旋转变换
                var optimalRot1 = Quaternion.FromToRotation(new Vector3(boneInFODBase.x, 0, boneInFODBase.z), az);
                // 相等表示水平方向没有自由度
                if (minFODYaw == maxFODYaw)
                {
                    optimalRot1 = Quaternion.identity;
                }

                // 垂直方向上的最优旋转变换
                var tempBone = optimalRot1 * boneInFODBase;
                var optimalRot2 = Quaternion.FromToRotation(tempBone, optimalDirInFODBase);

                // 相等表示垂直方向没有自由度
                if (minFODPitch == maxFODPitch)
                {
                    optimalRot1 = Quaternion.identity;
                }

                // 最终的旋转变换
                optimalRot = optimalRot2 * optimalRot1;
            }
            else
            {
                //如果不能水平分解则只计算垂直旋转
                var ax  = Vector3.Cross(Vector3.up, boneInFODBase).normalized;
                if (ax != Vector3.zero)
                {
                    var az = Vector3.Cross(ax, Vector3.up).normalized;
                    var zy = ConstraintRotation2D(Vector2.up, minFODYaw, maxFODYaw);

                    optimalDirInFODBase = MatrixMul(ax, Vector3.up, az, new Vector3(0, zy.y, zy.x));
                    optimalRot          = Quaternion.FromToRotation(boneInFODBase, optimalDirInFODBase);
                }
            }

            // 先变换到FOD Base，然后作用最终的旋转变化，最后在变换到世界空间
            optimalRot  = worldFODRotation * optimalRot * Quaternion.Inverse(worldFODRotation);

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

    public Vector2 ConstraintRotation2D(Vector2 to, float angleMin, float angleMax)
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