#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32MultiArray

# Parameters
RATE_HZ  = rospy.get_param('~rate', 50)     # Loop frequency in Hz
BOUNCE_S = rospy.get_param('~bounce', 0.02) # Debounce interval in **seconds**
HYST     = rospy.get_param('~hysteresis', 2) # Hysteresis threshold in degrees
TOPIC    = rospy.get_param('~topic', '/servo_commands')

# Initialize node and publisher
def main():
    rospy.init_node('servo_commands_publisher')
    pub = rospy.Publisher(TOPIC, Int32MultiArray, queue_size=1)
    rate = rospy.Rate(RATE_HZ)
    
    x1, y1 = 95, 94  # initial positions
    last_x = last_y = rospy.Time.now()
    rospy.loginfo("Publisher ready on %s @ %d Hz", TOPIC, RATE_HZ)

    while not rospy.is_shutdown():
        # Input raw angles
        raw = input("Enter raw x y (0–270): ").split()
        x_raw, y_raw = map(int, raw)
        
        # Linear mapping
        xp = (1.078*x_raw) + 237.02
        yp =  (1.07667*y_raw) +  41.30
        xp = max(0, min(270, xp)); yp = max(0, min(270, yp))

        # Map to servo [0–180]
        x2 = int(xp * 180.0 / 270.0)
        y2 = int(yp * 180.0 / 270.0)
        rospy.loginfo("Mapped target: [%d, %d]", x2, y2)

        # Smooth motion
        while not rospy.is_shutdown() and (x1!=x2 or y1!=y2):
            now = rospy.Time.now()
            if (now - last_x).to_sec() >= BOUNCE_S:
                if abs(x2-x1) > HYST: x1 += 1 if x2>x1 else -1
                else: x1 = x2
                last_x = now
            if (now - last_y).to_sec() >= BOUNCE_S:
                if abs(y2-y1) > HYST: y1 += 1 if y2>y1 else -1
                else: y1 = y2
                last_y = now
            # Clamp safe range
            x1 = max(8, min(180, x1))
            y1 = max(7, min(180, y1))
            pub.publish(Int32MultiArray(data=[x1, y1]))
            rate.sleep()
        rospy.loginfo("Reached [%d,%d]", x1, y1)

if __name__=='__main__':
    try: main()
    except rospy.ROSInterruptException: pass
