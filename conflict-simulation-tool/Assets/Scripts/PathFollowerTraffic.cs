using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace PathCreation.Examples
{
    public class PathFollowerTraffic : MonoBehaviour
    {
        public PathCreator pathCreator;
        public EndOfPathInstruction endOfPathInstruction;
        public float speed = 5;
        public float distanceTravelled;
        // Start is called before the first frame update
        void Start()
        {
            if (pathCreator != null)
            {
                // Subscribed to the pathUpdated event so that we're notified if the path changes during the game
                pathCreator.pathUpdated += OnPathChanged;
            }
        }

        // Update is called once per frame
        void Update()
            {
                //Debug.Log(distanceTravelled);
                if (pathCreator != null)
                {
                    distanceTravelled -= speed * Time.deltaTime;
                    transform.position = pathCreator.path.GetPointAtDistance(distanceTravelled, endOfPathInstruction);
                    transform.rotation = pathCreator.path.GetRotationAtDistance(distanceTravelled, endOfPathInstruction);
                }
            }

            // If the path changes during the game, update the distance travelled so that the follower's position on the new path
            // is as close as possible to its position on the old path
            void OnPathChanged() {
                distanceTravelled = pathCreator.path.GetClosestDistanceAlongPath(transform.position);
            }
    }
}
