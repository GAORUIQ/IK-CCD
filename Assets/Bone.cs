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
            // ���������ռ��µ�FOD������
            var worldFODBaseX      = parent._rotation * localFODBaseX;
            var worldFODBaseY      = parent._rotation * localFODBaseY;
            var worldFODBaseZ      = Vector3.Cross(worldFODBaseX, worldFODBaseY);

            //����ռ��µ�FOD��ת������
            var worldFODRotation   = Quaternion.LookRotation(worldFODBaseZ, worldFODBaseY);
            var inverseFODRotation = Quaternion.Inverse(worldFODRotation);
              
            var baseTarget         = worldTarget - _position;
            var baseTail           = tail - _position;

            //����ռ��µ��ӽڵ�λ��
            var basebone           = _rotation * child._localPosition;

            // ��Ŀ��λ�ã�ĩ�˵㣬���ӽڵ�λ�ñ任��FOD Base
            var targetInFODBase    = inverseFODRotation * baseTarget;
            var tailInFODBase      = inverseFODRotation * baseTail;
            var boneInFODBase      = inverseFODRotation * basebone;

            //��������µ�ˮƽ��ת�任
            var form = new Vector3(tailInFODBase.x, 0, tailInFODBase.z);
            var to   = new Vector3(targetInFODBase.x, 0, targetInFODBase.z);
            var rot1 = Quaternion.FromToRotation(form, to);

            //��������µĴ�ֱ��ת�任
            var tempTail    = rot1 * tailInFODBase;
            var rot2        = Quaternion.FromToRotation(tempTail, targetInFODBase);

            // ���任���õ�bone�ϣ��������������µ�boneλ��
            var perfectRot  = worldFODRotation * rot2 * rot1 * inverseFODRotation;
            var perfectBone = perfectRot * basebone;

            // �任��FOD Base��
            var perfectBoneInFODBase = inverseFODRotation * perfectBone;

            var optimalDirInFODBase = Vector3.zero;
            var optimalRot = Quaternion.identity;

            // ��perfectBoneInFODBase�ֽ⵽ˮƽ��
            var perfectBoneInFODBaseZX = new Vector2(perfectBoneInFODBase.z, perfectBoneInFODBase.x);
            if (perfectBoneInFODBaseZX != Vector2.zero)
            {
                // �����ˮƽ��Լ�������µ���������
                var zx   = ConstraintRotation2D(perfectBoneInFODBaseZX, minFODYaw, maxFODYaw);

                // ��ˮƽ����������Vector3.up����ȷ��һ��Base
                var az   = new Vector3(zx.y, 0, zx.x).normalized;
                var ay   = Vector3.up;
                var ax   = Vector3.Cross(ay, az);

                //��perfectBoneInFODBase�任��Base��
                var bone = MatrixInverseMul(ax, ay, az, perfectBoneInFODBase);
                var v2   = new Vector2(bone.z, bone.y);

                // ���㴹ֱ��Լ�������µ�
                var zy   = ConstraintRotation2D(v2, minFODPitch, maxFODPitch);

                //��FOD Base�����ŵķ���
                optimalDirInFODBase = MatrixMul(ax, ay, az, new Vector3(0, zy.y, zy.x));

                // ˮƽ�����ϵ�������ת�任
                var optimalRot1 = Quaternion.FromToRotation(new Vector3(boneInFODBase.x, 0, boneInFODBase.z), az);
                // ��ȱ�ʾˮƽ����û�����ɶ�
                if (minFODYaw == maxFODYaw)
                {
                    optimalRot1 = Quaternion.identity;
                }

                // ��ֱ�����ϵ�������ת�任
                var tempBone = optimalRot1 * boneInFODBase;
                var optimalRot2 = Quaternion.FromToRotation(tempBone, optimalDirInFODBase);

                // ��ȱ�ʾ��ֱ����û�����ɶ�
                if (minFODPitch == maxFODPitch)
                {
                    optimalRot1 = Quaternion.identity;
                }

                // ���յ���ת�任
                optimalRot = optimalRot2 * optimalRot1;
            }
            else
            {
                //�������ˮƽ�ֽ���ֻ���㴹ֱ��ת
                var ax  = Vector3.Cross(Vector3.up, boneInFODBase).normalized;
                if (ax != Vector3.zero)
                {
                    var az = Vector3.Cross(ax, Vector3.up).normalized;
                    var zy = ConstraintRotation2D(Vector2.up, minFODYaw, maxFODYaw);

                    optimalDirInFODBase = MatrixMul(ax, Vector3.up, az, new Vector3(0, zy.y, zy.x));
                    optimalRot          = Quaternion.FromToRotation(boneInFODBase, optimalDirInFODBase);
                }
            }

            // �ȱ任��FOD Base��Ȼ���������յ���ת�仯������ڱ任������ռ�
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

        // Ҫ��ת����Ŀ��������������ʱֱ�ӷ��أ����򷵻�����ı߽���
        return (diffMin >= 0 && diffMax >= 0 ? to : Vector2.Dot(to, minLine) >= Vector2.Dot(to, maxLine) ? minLine : maxLine).normalized;
    }
}