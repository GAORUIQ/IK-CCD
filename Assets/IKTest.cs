using UnityEngine;

namespace Assets
{
    public class IKTest : MonoBehaviour
    {
        public Bone Root;
        public Transform target;
        public bool autoDescent;

        public void Test()
        {
            var tail = Root.GetTail();

            Root.UpdateLocalTransform();
            Root.UpdateWorldTransform(Root.transform.position);
            tail.parent.CyclicCoordinateDescent(target.position, tail.transform.position);
            Root.UpdateWorldTransform(Root.transform.position);
            Root.ApplyTransform();
        }

        private void Update()
        {
            if (autoDescent)
                Test();
        }
    }
}