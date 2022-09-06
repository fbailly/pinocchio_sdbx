from utils import *
from time import sleep
from_urdf = False
N = 3
if from_urdf:
    wrapper = loadDoublePendulum()
    model = wrapper.model
    data = wrapper.data
    viz = MeshcatVisualizer(model, wrapper.visual_model, wrapper.visual_model)
    viz.initViewer(open=True)
    viz.loadViewerModel()
    q0 = pin.neutral(model)
    viz.display(q0)
else:
    model, geom_model = create_N_pendulum(N, ['x', 'x', 'x'])
    viz = MeshcatVisualizer(model, geom_model, geom_model)
    viz.initViewer(open=True)
    viz.loadViewerModel()
    q0 = pin.neutral(model)
    q0[0] = -np.pi/2
    viz.display(q0)
    data = model.createData()

Tf = 1
num = int(Tf*200)
dt = Tf/num
t = np.linspace(0, Tf, num=num)
Q = np.vstack((
    # 0*np.pi/2*np.sin(2*np.pi*t),
    # 0*np.pi/2*np.cos(3*np.pi*t),
    np.pi*np.cos(2*np.pi*t),
    np.pi/2*np.sin(3*np.pi*t),
    0*np.pi/2*np.cos(4*np.pi*t),
    ))
v = np.diff(Q)/dt
a = np.diff(v)/dt
AM = np.zeros((3, num-2))
dAM = np.zeros((3, num-2))
V = np.zeros((3, N, num-2))
A = np.zeros((3, N, num-2))
for i in range(num-2):
    qi = Q[:, i]
    vi = v[:, i]
    V[0, :, i] = vi
    A[0, :, i] = a[:, i]
    viz.display(qi)
    pin.computeAllTerms(model, data, qi, vi)
    dAM[:, i] = data.dhg.angular.squeeze()
    AM[:, i] = pin.computeCentroidalMomentum(model, data, qi, vi).angular.squeeze()
    sleep(0.05)
dAM = np.diff(AM)/dt

plt.plot(AM.T, label="angular momentum pinocchio")
plt.legend()
plt.figure()
plt.plot(dAM.T, 'x', label="der angular momentum dhg")
plt.legend()
plt.show()
