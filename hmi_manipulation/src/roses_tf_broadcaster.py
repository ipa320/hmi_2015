#!/usr/bin/env python 
import rospy
import tf
import math

HEIGHT_ABOVE_TABLE_TOP = 0.12
NX = 4
NY = 5
DX = math.sqrt(0.150*0.150/2)
DY = 2*DX
DY_OFFSET_2ND_ROW = DY/2

DISTANCE_BETWEEN_ROSES = math.sqrt(0.15*0.15+0.15*0.15)
BED_X_OFFSET_FROM_CENTER = 0.38/2-DISTANCE_BETWEEN_ROSES/2
BED_Y_OFFSET_FROM_CENTER = 0.5+0.5-2*DISTANCE_BETWEEN_ROSES

def bed_of_roses(prefix, nx, ny, x_offset_from_center, y_offset_from_center):
    roses = []
    for i in range(nx):
        for j in range(ny):
            if i % 2:
                y_offset = 0
            else:
                y_offset = DY_OFFSET_2ND_ROW
            broadcast_tf(i*DX+x_offset_from_center,j*DY+y_offset+y_offset_from_center,prefix+"_"+str(i)+"-"+str(j))

def broadcast_tf(x,y,name):
    br.sendTransform(
        (x, y, HEIGHT_ABOVE_TABLE_TOP),
         tf.transformations.quaternion_from_euler(0, 0, 0),
         rospy.Time.now(),
         name,
         "table_top")


if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        
        br.sendTransform(
            (-0.38, 0, 0.6-0.209), # laser above ground = 0.209
             tf.transformations.quaternion_from_euler(0, 0, 0),
             rospy.Time.now(),
             "table_top",
             "table_reference")
        
        bed_of_roses("rose_left", NX, NY, BED_X_OFFSET_FROM_CENTER, BED_Y_OFFSET_FROM_CENTER)
        bed_of_roses("rose_right", NX, NY, BED_X_OFFSET_FROM_CENTER, -BED_Y_OFFSET_FROM_CENTER - NY*DY)
        
        
        # testbed e325
        #bed_of_roses("rose_right", NX, NY, -0.3, 0.3-NY*DY)
        

        
        rate.sleep()
