#!/usr/bin/env python 
import rospy
import tf
import math

HEIGHT_ABOVE_TABLE_TOP = 0.15
NX = 3
NY = 5

DISTANCE_BETWEEN_ROSES_Y = math.sqrt(0.15*0.15+0.15*0.15)
DISTANCE_BETWEEN_ROSES_X = DISTANCE_BETWEEN_ROSES_Y/2
DY_OFFSET_2ND_ROW = DISTANCE_BETWEEN_ROSES_Y/2
BED_X_OFFSET_FROM_CENTER = 0.38/2-DISTANCE_BETWEEN_ROSES_X
BED_Y_OFFSET_FROM_CENTER = 0.5+0.5-2*DISTANCE_BETWEEN_ROSES_Y


def bed_of_roses(prefix, nx, ny, x_offset_from_center, y_offset_from_center):
    roses = []
    for i in range(nx):
        for j in range(ny):
            if (i-1) % 2:
                y_offset = 0
            else:
                y_offset = DY_OFFSET_2ND_ROW
                if j == NY-1:
                    continue
            #print "y_offset = ", i, j, y_offset
            x = i*DISTANCE_BETWEEN_ROSES_X+x_offset_from_center
            y = j*DISTANCE_BETWEEN_ROSES_Y+y_offset+y_offset_from_center
            name = prefix+"_"+str(i)+"-"+str(j)
            rose = [x,y,name]
            roses.append(rose)
    return roses
            #broadcast_tf(i*DISTANCE_BETWEEN_ROSES_X+x_offset_from_center,j*DISTANCE_BETWEEN_ROSES_Y+y_offset+y_offset_from_center,prefix+"_"+str(i)+"-"+str(j))

def broadcast_tf(x,y,name):
    #print "x=",x
    #print "y=",y
    #print "name=",name
    br.sendTransform(
        (x, y, HEIGHT_ABOVE_TABLE_TOP),
         tf.transformations.quaternion_from_euler(0, 0, 0),
         rospy.Time.now(),
         str(name),
         "table_top")


if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10)
    
    roses = []
    roses += bed_of_roses("rose_left", NX, NY, BED_X_OFFSET_FROM_CENTER, BED_Y_OFFSET_FROM_CENTER)
    roses += bed_of_roses("rose_right", NX, NY, BED_X_OFFSET_FROM_CENTER, -BED_Y_OFFSET_FROM_CENTER - (NY-1)*DISTANCE_BETWEEN_ROSES_Y)

    rospy.set_param("/hmi/roses",roses)

    while not rospy.is_shutdown():
        
        br.sendTransform(
            (-0.38, 0, 0.6-0.209), # laser above ground = 0.209
             tf.transformations.quaternion_from_euler(0, 0, 0),
             rospy.Time.now(),
             "table_top",
             "table_reference")
        
        for rose in roses:
            #print "rose=", rose
            broadcast_tf(rose[0],rose[1],rose[2])

        rate.sleep()
