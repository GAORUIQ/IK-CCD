using UnityEngine;

namespace Assets
{
    public class IKTest : MonoBehaviour
    {
        public Bone Root;
        public Transform target;
        public bool autoDescent;

        public void CCDTest()
        {
            var tail = Root.GetTail();

            Root.UpdateLocalTransform();
            Root.UpdateWorldTransform(Root.transform.position);
            tail.parent.CyclicCoordinateDescent(target.position, tail.transform.position);
            Root.UpdateWorldTransform(Root.transform.position);
            Root.ApplyTransform();
        }

        public void FABRTest()
        {
            var tail = Root.GetTail();
            Root.UpdateLocalTransform();
            Root.UpdateWorldTransform(Root.transform.position);
            tail.ForwardwardReaching(target.position, Quaternion.identity);
            Root.ForwardwardReachingUpdate();
            Root.BackwardReaching(Root.transform.position);
            Root.ApplyTransform();
        }

        private void Update()
        {
            if (autoDescent)
                FABRTest();
        }

        private void OnGUI()
        {
            if (GUI.Button(new Rect(100, 100, 100, 100), "FABR"))
            {
                FABRTest();
            }
        }
    }
}