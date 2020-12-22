# 0. Pub/Sub to publish and receive name in ROS

1. **AIM**: To print the name via pub/sub

2. **METHOD**: 
  - Create catkin workspace: `mkdir -p catkin_ws/src`
  - Create catkin package: `catkin_create_pkg exp0_postlab rospy roscpp std_msgs`
  - Create scripts: `cd src/exp0_postlab/src && mkdir scripts && cd scripts && touch publisher.py && touch subscriber.py`
  - Add the scripts in this directory
  - Catkin Make: `catkin_make`

3. **CODE**:
  - publisher.py
```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def publisher():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        name = "Aaryamann Challani"
        rospy.loginfo(name)
        pub.publish(name)

        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
```
  - subscriber.py
```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "Name:%s", data.data)

def subscriber():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('chatter', String, callback)
    rospy.spin()

if __name__ == '__main__':
    subscriber()
```

4. **RESULT**: Successfully published and received messages via ROS's pubsub transport

