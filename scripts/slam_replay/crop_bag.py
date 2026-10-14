# cropbag.py IN OUT A0 A1 : copy /scan, setting beams with angle in [A0,A1] deg to NaN
import sys, math, rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import LaserScan
src,dst,a0,a1=sys.argv[1],sys.argv[2],float(sys.argv[3]),float(sys.argv[4])
r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=src,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
w=rosbag2_py.SequentialWriter(); w.open(rosbag2_py.StorageOptions(uri=dst,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
for t in r.get_all_topics_and_types():
    if t.name=='/scan': w.create_topic(t)
r.set_filter(rosbag2_py.StorageFilter(topics=['/scan']))
while r.has_next():
    topic,data,ts=r.read_next(); m=deserialize_message(data,LaserScan)
    rr=list(m.ranges); ii=list(m.intensities)
    for k in range(len(rr)):
        a=math.degrees(m.angle_min+k*m.angle_increment)%360
        if a0<=a<=a1: rr[k]=float('nan'); ii[k]=float('nan')
    m.ranges=rr; m.intensities=ii; w.write(topic,serialize_message(m),ts)
