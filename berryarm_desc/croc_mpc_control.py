# %load arm_example.py
import crocoddyl
import pinocchio
import numpy as np
import example_robot_data
from math import cos, sin, pi

robot = example_robot_data.load('berry_arm')
robot_model = robot.model

DT = 1e-2
T = 20

wp = 4

runningModels = []
terminalModels = []


def create_circle_waypoints(wp=20, x=0.5, y=0.0, r=0.1):
    waypoints = []

    for i in range(0, 361, int(360 / wp)):
        a = x + r*cos(pi*i/180)
        b = y + r*sin(pi*i/180)
        waypoints.append([a, b, .6])

    for i in range(361, 0, -int(360 / wp)):
        a = x + r*cos(pi*i/180)
        b = y + r*sin(pi*i/180)
        waypoints.append([a, b, .5])

    return waypoints


waypoints = create_circle_waypoints()
last_state = None
h = 0
for waypoint in waypoints:

    target = np.array([waypoint[0], waypoint[1], waypoint[2]])

    display = crocoddyl.GepettoDisplay(robot)
    display.robot.viewer.gui.addSphere(
        'world/point'+str(h), .01, [1., 0., 0., 1.])  # radius = .1, RGBA=1001
    display.robot.viewer.gui.applyConfiguration(
        'world/point'+str(h), target.tolist() + [0., 0., 0., 1.])  # xyz+quaternion
    display.robot.viewer.gui.refresh()

    # Create the cost functions
    state = crocoddyl.StateMultibody(robot.model)
    goalResidual = crocoddyl.ResidualModelFrameTranslation(
        state, robot_model.getFrameId("tcp_joint"), target)

    goalTrackingCost = crocoddyl.CostModelResidual(state, goalResidual)
    xRegCost = crocoddyl.CostModelResidual(
        state, crocoddyl.ResidualModelState(state))
    uRegCost = crocoddyl.CostModelResidual(
        state, crocoddyl.ResidualModelControl(state))

    # Create cost model per each action model
    runningCostModel = crocoddyl.CostModelSum(state)
    terminalCostModel = crocoddyl.CostModelSum(state)

    # Then let's added the running and terminal cost functions
    runningCostModel.addCost("gripperPose", goalTrackingCost, 1e3)
    runningCostModel.addCost("stateReg", xRegCost, 1)
    runningCostModel.addCost("ctrlReg", uRegCost, 1e-2)
    terminalCostModel.addCost("gripperPose", goalTrackingCost, 1e2)
    terminalCostModel.addCost("stateReg", xRegCost, 1)
    terminalCostModel.addCost("ctrlReg", uRegCost, 1e-2)

    # Create the actuation model
    actuationModel = crocoddyl.ActuationModelFull(state)

    # Create the action model
    runningModel = crocoddyl.IntegratedActionModelEuler(
        crocoddyl.DifferentialActionModelFreeFwdDynamics(state, actuationModel, runningCostModel), DT)
    terminalModel = crocoddyl.IntegratedActionModelEuler(
        crocoddyl.DifferentialActionModelFreeFwdDynamics(state, actuationModel, terminalCostModel))

    runningModels.append(runningModel)
    terminalModels.append(terminalModel)

    last_state = state
    h += 1


# Create the problem
q0 = np.array([0.1, - 0.8163960131931421, - 1.0])
x0 = np.concatenate([q0, pinocchio.utils.zero(state.nv)])

seq0 = [runningModels[0]]*T + [terminalModels[0]]
for i in range(1, len(waypoints)-1):
    seqi = [runningModels[i]]*T + [terminalModels[i]]
    seq0 = seq0+seqi

seqlast = [runningModels[-1]]*T
seq0 = seq0+seqlast

problem = crocoddyl.ShootingProblem(x0, seq0, terminalModels[-1])

# Creating the DDP solver for this OC problem, defining a logger
ddp = crocoddyl.SolverDDP(problem)
ddp.setCallbacks([crocoddyl.CallbackVerbose()])

# Solving it with the DDP algorithm
ddp.solve()

# Visualizing the solution in gepetto-viewer
display.displayFromSolver(ddp)

robot_data = robot_model.createData()
xT = ddp.xs[-1]

us = ddp.us
print(*xT)

pinocchio.forwardKinematics(robot_model, robot_data, xT[:state.nq])
pinocchio.updateFramePlacements(robot_model, robot_data)

print('Finally reached = ', robot_data.oMf[robot_model.getFrameId(
    "tcp_joint")].translation.T)
