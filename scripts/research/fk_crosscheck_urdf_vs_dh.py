"""Cross-check the official QArm URDF against Quanser's DH forward kinematics.

URDF source : https://github.com/quanser/urdf_representations/blob/main/qarm/urdf/QARM.urdf
DH source   : Quanser_Academic_Resources 0_libraries/python/hal/products/qarm.py (QArmUtilities)

Usage: python3 fk_crosscheck_urdf_vs_dh.py <path/to/QARM.urdf>

Prints, for several joint configurations:
  1. the URDF END-EFFECTOR frame origin vs the DH tip (lambda3 = L4+L5 = 0.40 m) and vs DH with lambda3 = L4;
  2. the DH tip expressed in the URDF END-EFFECTOR frame -> the fixed TCP offset to add to the URDF.
"""
import sys, numpy as np, xml.etree.ElementTree as ET
urdf = sys.argv[1]
def rpy2R(r,p,y):
    cr,sr,cp,sp,cy,sy=np.cos(r),np.sin(r),np.cos(p),np.sin(p),np.cos(y),np.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
def axang(a,t):
    a=np.array(a,float); a/=np.linalg.norm(a); K=np.array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]])
    return np.eye(3)+np.sin(t)*K+(1-np.cos(t))*K@K
r=ET.parse(urdf).getroot(); J={j.get('name'):j for j in r.findall('joint')}
def urdf_fk(q):
    T=np.eye(4)
    for name,qi in zip(['YAW','SHOULDER','ELBOW','WRIST'],q):
        j=J[name]; o=j.find('origin'); xyz=[float(v) for v in o.get('xyz').split()]; rpy=[float(v) for v in o.get('rpy').split()]
        A=np.eye(4); A[:3,:3]=rpy2R(*rpy); A[:3,3]=xyz
        R=np.eye(4); R[:3,:3]=axang([float(v) for v in j.find('axis').get('xyz').split()],qi)
        T=T@A@R
    return T
L1,L2,L3,L4,L5=0.14,0.35,0.05,0.25,0.15; B=np.arctan(L3/L2); l1,l2,l3=L1,np.hypot(L2,L3),L4+L5
def dh(a,al,d,th):
    return np.array([[np.cos(th),-np.sin(th)*np.cos(al),np.sin(th)*np.sin(al),a*np.cos(th)],[np.sin(th),np.cos(th)*np.cos(al),-np.cos(th)*np.sin(al),a*np.sin(th)],[0,np.sin(al),np.cos(al),d],[0,0,0,1]])
def dh_fk(phi, lam3=l3):
    th=[phi[0],phi[1]+B-np.pi/2,phi[2]-B,phi[3]]
    return dh(0,-np.pi/2,l1,th[0])@dh(l2,0,0,th[1])@dh(0,-np.pi/2,0,th[2])@dh(0,0,lam3,th[3])
np.set_printoptions(precision=4,suppress=True)
for q in [[0,0,0,0],[0.5,0.3,-0.4,0.2],[-1.0,-0.6,0.8,1.0]]:
    pu=urdf_fk(q)[:3,3]; pd=dh_fk(q)[:3,3]; pw=dh_fk(q,lam3=L4)[:3,3]
    print(f"q={q}\n  URDF wrist-frame origin : {pu}\n  DH tip (L4+L5=0.40)     : {pd}\n  DH with lam3=L4 (0.25)  : {pw}   |URDF-DH(L4)|={np.linalg.norm(pu-pw)*1000:.1f} mm")

# --- TCP offset check: express (DH tip - URDF wrist origin) in the URDF END-EFFECTOR frame ---
print("\nTCP offset of DH tip expressed in URDF END-EFFECTOR frame (should be constant if a fixed tool frame suffices):")
for q in [[0,0,0,0],[0.5,0.3,-0.4,0.2],[-1.0,-0.6,0.8,1.0],[1.2,0.9,-1.2,-2.0]]:
    T=urdf_fk(q); d=dh_fk(q)[:3,3]-T[:3,3]
    print(f"  q={q}: offset_local = {T[:3,:3].T@d}")
