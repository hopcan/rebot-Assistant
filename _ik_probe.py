import pinocchio as pin
import numpy as np

RS = r"D:/rebotArm-Assistant/rebot-Assistant/urdf/reBot-B601_RS_fixend/urdf/00-arm-rs_asm-v3.urdf"
DM = r"D:/rebotArm-Assistant/rebot-Assistant/urdf/reBot-B601_DM_fixend/urdf/reBot-DevArm_fixend.urdf"

posmax = [2.6, 3.14, 3.14, 1.57, 1.57, 2.6]   # RS joint2/3: [0, 3.14]
posmin = [-2.6, 0.0, 0.0, -1.57, -1.57, -2.6]


def solve(path, nq, target, rpy_args, label):
    m = pin.buildModelFromUrdf(path)
    d = m.createData()
    fid = m.getFrameId("end_link")
    q = np.zeros(nq)
    rot = pin.rpy.rpyToMatrix(rpy_args[0], rpy_args[1], rpy_args[2])
    oMdes = pin.SE3(rot, np.array(target))
    for _ in range(3000):
        pin.forwardKinematics(m, d, q)
        pin.updateFramePlacements(m, d)
        iMd = d.oMf[fid].actInv(oMdes)
        err = pin.log(iMd).vector
        if np.linalg.norm(err) < 0.01:
            break
        J = pin.computeFrameJacobian(m, d, q, fid)
        J = -pin.Jlog6(iMd.inverse()) @ J
        JJT = J @ J.T
        JJT.flat[::7] += 1e-6
        v = -J.T @ np.linalg.solve(JJT, err)
        q = pin.integrate(m, q, v * 0.02)
        for i in range(6):
            q[i] = min(max(q[i], posmin[i]), posmax[i])
    pin.forwardKinematics(m, d, q)
    pin.updateFramePlacements(m, d)
    R = d.oMf[fid].rotation
    resid = np.linalg.norm(pin.log(d.oMf[fid].actInv(oMdes)).vector)
    print(f"{label}: q[deg]={np.round(np.degrees(q[:6]), 1)}")
    print(f"  end t={np.round(d.oMf[fid].translation, 3)}")
    print(f"  end local+Z in world={np.round(R[:, 2], 2)}  resid={resid:.4f}")


# 硬编码目标 target_pos_point=[0.22,0.1,0.3, pitch=0, roll=0.67, yaw=0]
# 代码里 rpyToMatrix(target[3], target[4], target[5]) = rpyToMatrix(0, 0.67, 0)
solve(RS, 8, [0.22, 0.1, 0.3], (0, 0.67, 0), "RS hardcoded (0,0.67,0)")
solve(RS, 8, [0.22, 0.1, 0.3], (0, 0, 0), "RS rpy all zero")
solve(DM, 6, [0.22, 0.1, 0.3], (0, 0.67, 0), "DM same target (0,0.67,0)")