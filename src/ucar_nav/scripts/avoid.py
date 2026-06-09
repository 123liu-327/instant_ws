#!/usr/bin/env python3
import rospy, math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

rospy.init_node('nav')
pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
rate=rospy.Rate(15)
scan=None; px=py=pyaw=0.0
def cb_s(msg):
    global scan; scan=msg
def cb_o(msg):
    global px,py,pyaw
    px=msg.pose.pose.position.x; py=msg.pose.pose.position.y
    q=msg.pose.pose.orientation
    pyaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
rospy.Subscriber('/scan',LaserScan,cb_s)
rospy.Subscriber('/odom',Odometry,cb_o)
gx=rospy.get_param('~goal_x',4.5); gy=rospy.get_param('~goal_y',0.0)
while scan is None and not rospy.is_shutdown():
    rospy.sleep(0.1)
print("=== v5 ===")

# 前向只膨胀车身, 侧面膨胀车身+底座
FRONT_INF = 0.22
SIDE_INF  = 0.38
MAX_V = 0.28
WARN_F = 0.30   # 膨胀后前向预警
DANG_F = 0.18   # 膨胀后前向危险
WARN_S = 0.25   # 膨胀后侧向预警

tdir = 0       # -1左绕, 0无, +1右绕
lock_cnt = 0   # 锁定时长
svx=svz=0.0

while not rospy.is_shutdown():
    raw=list(scan.ranges); N=len(raw)
    # 分角度膨胀
    r=[10]*N
    for i in range(N):
        d=raw[i]
        if 0.05<d<10:
            ang_deg = abs(i-N//2)/(N//2)*180
            inf = SIDE_INF if ang_deg>30 else FRONT_INF
            r[i]=max(0.03, d-inf)

    f30=min(r[N//2-45:N//2+45])
    L=min(r[40:90]); R=min(r[N-90:N-40])
    raw_f=min(raw[N//2-10:N//2+10] or [10])

    dx=gx-px; dy=gy-py
    dist=math.sqrt(dx*dx+dy*dy)
    gdeg=math.degrees(math.atan2(dy,dx)-pyaw)
    gdeg=(gdeg+180)%360-180

    if dist<0.20:
        print("OK"); pub.publish(Twist()); rate.sleep(); continue

    tvx=tvz=0.0

    # 危险: 后退+锁方向
    if f30<DANG_F:
        tvx=-0.10
        if tdir==0: tdir=1 if L>R else -1
        tvz=2.0*tdir
        lock_cnt+=1
    # 预警: 慢速+同向转
    elif f30<WARN_F or (tdir!=0 and (min(L,R)<WARN_S)):
        tvx=0.06
        if tdir==0: tdir=1 if L>R else -1
        tvz=1.5*tdir
        lock_cnt+=1
    # 开阔: 考虑解锁
    elif tdir!=0:
        # 解锁条件: 前向开阔 + (朝向目标 或 锁太久)
        if (raw_f>0.8 and abs(gdeg)<50) or lock_cnt>60:
            tdir=0; lock_cnt=0
        else:
            tvx=0.12; tvz=1.0*tdir
            lock_cnt+=1
    else:
        # 无锁定: 朝目标
        tvx=min(MAX_V, raw_f*0.5)
        tvx=max(0.12, tvx)
        tvz=math.radians(gdeg)*3
        tvz=max(-1.5, min(1.5, tvz))

    svx=0.45*tvx+0.55*svx
    svz=0.45*tvz+0.55*svz
    t=Twist()
    t.linear.x=round(svx,3); t.angular.z=round(svz,3)
    pub.publish(t)

    m="⚡" if f30<DANG_F else ("~" if tdir!=0 else "→")
    print("%s f=%.2f raw=%.2f L=%.2f R=%.2f g=%+d d=%.2f dir=%+d lock=%d vx=%.2f wz=%+.2f"%(
        m,f30,raw_f,L,R,int(gdeg),dist,tdir,lock_cnt,t.linear.x,t.angular.z))
    rate.sleep()
