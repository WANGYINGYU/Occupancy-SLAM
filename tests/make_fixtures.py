"""Small deterministic ROS1 bags and ASCII/binary_compressed PCD sequences."""
from pathlib import Path
import math
import struct
import numpy as np
from scipy.spatial.transform import Rotation


def make_bags(root, topics=None, decoy=False):
    topics = topics or {}
    from rosbags.typesys import get_typestore, Stores, get_types_from_msg
    from rosbags.rosbag1 import Writer
    store=get_typestore(Stores.ROS1_NOETIC)
    store.register(get_types_from_msg('geometry_msgs/TransformStamped[] transforms\n','tf2_msgs/msg/TFMessage'))
    def new(name,*args):return store.types[name](*args)
    def header(t,frame):return new('std_msgs/msg/Header',0,new('builtin_interfaces/msg/Time',int(t),0),frame)
    def vec(x=0.,y=0.,z=0.):return new('geometry_msgs/msg/Vector3',float(x),float(y),float(z))
    def quat(yaw):return new('geometry_msgs/msg/Quaternion',0.,0.,math.sin(yaw/2),math.cos(yaw/2))
    def tf(t,parent,child,x,y,yaw):return new('geometry_msgs/msg/TransformStamped',header(t,parent),child,new('geometry_msgs/msg/Transform',vec(x,y),quat(yaw)))
    for kind in ('laser','cloud'):
        path=root/f'{kind}.bag'
        with Writer(path) as writer:
            scan_type='sensor_msgs/msg/LaserScan' if kind=='laser' else 'sensor_msgs/msg/PointCloud2'
            cs=writer.add_connection(topics.get(kind,'/scan' if kind=='laser' else '/points'),scan_type,typestore=store)
            cd=writer.add_connection('/unused/sensor',scan_type,typestore=store) if decoy else None
            ct=writer.add_connection(topics.get('tf','/tf'),'tf2_msgs/msg/TFMessage',typestore=store)
            cstatic=writer.add_connection(topics.get('tf_static','/tf_static'),'tf2_msgs/msg/TFMessage',typestore=store,latching=1)
            co=writer.add_connection(topics.get('odom','/odom'),'nav_msgs/msg/Odometry',typestore=store)
            def write(c,t,m):writer.write(c,int(t*1e9),store.serialize_ros1(m,c.msgtype))
            write(cstatic,999,new('tf2_msgs/msg/TFMessage',[tf(999,'base','laser',.4,-.1,.3)]))
            for i in range(24):
                t=1000+i
                bx,by,yaw=.03*i,.05*math.sin(i/10),.004*i
                write(ct,t,new('tf2_msgs/msg/TFMessage',[tf(t,'odom','base',bx,by,yaw)]))
                pose=new('geometry_msgs/msg/Pose',new('geometry_msgs/msg/Point',bx,by,0.),quat(yaw))
                covariance=np.zeros(36)
                odom=new('nav_msgs/msg/Odometry',header(t,'odom'),'base',new('geometry_msgs/msg/PoseWithCovariance',pose,covariance),new('geometry_msgs/msg/TwistWithCovariance',new('geometry_msgs/msg/Twist',vec(),vec()),covariance))
                write(co,t,odom)
                lx=bx+math.cos(yaw)*.4+math.sin(yaw)*.1
                ly=by+math.sin(yaw)*.4-math.cos(yaw)*.1
                angles=-math.pi+np.arange(360)*math.pi/180
                world_angles=angles+yaw+.3
                dx,dy=np.cos(world_angles),np.sin(world_angles)
                with np.errstate(divide='ignore',invalid='ignore'):
                    ranges=np.minimum(np.where(dx>0,(5-lx)/dx,(-4-lx)/dx),np.where(dy>0,(4-ly)/dy,(-3-ly)/dy)).astype(np.float32)
                if kind=='laser':
                    msg=new(scan_type,header(t,'laser'),float(angles[0]),float(angles[-1]),math.pi/180,0.,1.,.1,15.,ranges,np.empty(0,dtype=np.float32))
                else:
                    points=np.column_stack([ranges*np.cos(angles),ranges*np.sin(angles),np.zeros(360)]).astype('<f4')
                    # Extra points above the selected slice must not affect the 2D scan.
                    high=points.copy(); high[:,:2]*=.4; high[:,2]=2
                    points=np.concatenate([points,high])
                    fields=[new('sensor_msgs/msg/PointField',name,4*j,7,1) for j,name in enumerate(('x','y','z'))]
                    msg=new(scan_type,header(t,'laser'),1,len(points),fields,False,12,12*len(points),np.frombuffer(points.tobytes(),dtype=np.uint8),True)
                write(cs,t,msg)
                if cd is not None: write(cd,t,msg)
    return root/'laser.bag',root/'cloud.bag'


def make_pcds(root):
    root.mkdir(); (root/'PCD').mkdir()
    rng=np.random.default_rng(91)
    points=[]
    for axis in range(3):
        for side in (-2.,2.):
            p=rng.uniform(-2,2,(100,3));p[:,axis]=side;points.append(p)
    world=np.concatenate(points)
    poses=[]
    for i in range(4):
        t=np.array([.08*i,.025*i,.01*i]); rot=Rotation.from_rotvec([.01*i,-.008*i,.012*i])
        local=((world-t)@rot.as_matrix()).astype('<f4')
        suffix='ascii' if i%2==0 else 'binary_compressed'
        header=f'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(local)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(local)}\nDATA {suffix}\n'
        path=root/'PCD'/f'full{i}.pcd'
        if suffix=='ascii':
            with path.open('w') as f:f.write(header);np.savetxt(f,local,fmt='%.8g')
        else:
            raw=local.T.copy().tobytes() # PCD binary_compressed uses structure of arrays.
            compressed=b''.join(bytes([len(raw[j:j+32])-1])+raw[j:j+32] for j in range(0,len(raw),32))
            path.write_bytes(header.encode()+struct.pack('<II',len(compressed),len(raw))+compressed)
        noisy=t+np.array([.06*i,-.025*i,.02*i])
        q=(rot*Rotation.from_rotvec([.012*i,.009*i,-.01*i])).as_quat()
        poses.append([1000+i,*noisy,*q])
    np.savetxt(root/'odometry.txt',poses,fmt='%.12g')
    (root/'scans.txt').write_text(''.join(f'{1000+i} PCD/full{i}.pcd\n' for i in range(4)))
    return root
