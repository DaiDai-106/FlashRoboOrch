import ampl
import numpy as np
# np.set_printoptions(suppress=True, precision=4)

def main():
    # 设置机械臂
    solver =  ampl.ArmBase("fanuc_crx10ia", ampl.ArmType.CRX6, 6)
    solver.set_base(np.eye(4))
    bounds = solver.joint_limits()
    qt_link = np.zeros(( 7, 7), dtype=np.float64)
    qs_fanuc = np.array([0.001] * 6, dtype=np.float64)
    solver.fk_links( qs_fanuc, qt_link )
    param_ik_iter=np.array([1e-3,1e-2,0,0,5e-4, 1e-4],dtype=np.float64)

    ep = qt_link[-1]
    ept = ampl.qt7_to_tf44(ep)
    print(ept)

    sols = np.zeros((8, 6), dtype=np.float64)
    status = solver.ik_iter(ept, qs_fanuc, sols, param_ik_iter, 20)
    print(sols)
    print(status)

    solver.fk_links( sols[0], qt_link )
    ep = qt_link[-1]
    ept = ampl.qt7_to_tf44(ep)
    print(ept)


if __name__ == "__main__":
    main()