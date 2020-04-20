import os
import random
import numpy as np
from pydrake.multibody.parsing import Parser
from pydrake.systems.framework import AbstractValue, BasicVector, LeafSystem, DiagramBuilder
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.multibody.plant import VectorExternallyAppliedSpatialForced, ExternallyAppliedSpatialForce
from pydrake.systems.analysis import Simulator
from pydrake.multibody.math import SpatialForce
from pydrake.systems.drawing import plot_system_graphviz
import matplotlib.pyplot as plt
from pydrake.geometry import ConnectDrakeVisualizer
from pydrake.geometry.render import (
    MakeRenderEngineVtk,
    RenderEngineVtkParams,
)
import time
from utils import print_body, get_body_homogeneous_matrix


class CylinderSystem(LeafSystem):

    def __init__(self, plant):
        LeafSystem.__init__(self)

        self.nv = plant.num_velocities()
        self.target_body_index = plant.GetBodyByName("uniformSolidCylinder").index()

        self.DeclareAbstractOutputPort(
            "spatial_forces_vector",
            lambda: AbstractValue.Make(
            VectorExternallyAppliedSpatialForced()), self.DoCalcAbstractOutput)

        self.DeclareVectorOutputPort(
            "generalized_forces",
            BasicVector(self.nv),
            self.DoCalcVectorOutput)

        self.wrench = np.array([0., 0., 8., 0., 0., 0.])

    def DoCalcAbstractOutput(self, context, y_data):
        test_force = ExternallyAppliedSpatialForce()
        test_force.body_index = self.target_body_index
        test_force.p_BoBq_B = np.zeros(3)
        test_force.F_Bq_W = SpatialForce(tau=self.wrench[3:], f=self.wrench[:3])
        y_data.set_value(VectorExternallyAppliedSpatialForced([test_force]))

    def DoCalcVectorOutput(self, context, y_data):
        y_data.SetFromVector(np.zeros(self.nv))


sim_time_step = 0.001
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, sim_time_step)
# pydrake.multibody.tree.ModelInstanceIndex
cylinder_model_instance = Parser(plant).AddModelFromFile('uniform_solid_cylinder.urdf')

cylinder_body_index = plant.GetBodyIndices(cylinder_model_instance).pop()

cylinder_body = plant.get_body(cylinder_body_index)

print_body(cylinder_body)

gravity = np.array([0., 0., -9.81])
cylinder_mass = cylinder_body.default_mass()
h = get_body_homogeneous_matrix(cylinder_body)
position = h[:-1, 3]
target_one = position + np.array([0., 0., 1.])
target_two = position - np.array([0., 0., 1.])
current_target = target_one

plant.Finalize()

print('type(cylinder_body): {}'.format(type(cylinder_body)))

scene_graph.AddRenderer("renderer", MakeRenderEngineVtk(RenderEngineVtkParams()))
ConnectDrakeVisualizer(builder, scene_graph)


cylinder_system = builder.AddSystem(CylinderSystem(plant))


builder.Connect(cylinder_system.get_output_port(0),
                plant.get_applied_spatial_force_input_port())

builder.Connect(cylinder_system.get_output_port(1),
                plant.get_applied_generalized_force_input_port())

diagram = builder.Build()

simulator = Simulator(diagram)

time.sleep(1.0)
time_ = 0
print('Start')
start_force = 10
model_force = -gravity[-1] * cylinder_mass
Kp = 0.1
Kd = 0.3
going_to_target = 1
while True:
    curr_context = diagram.GetMutableSubsystemContext(plant, simulator.get_mutable_context())
    positions = plant.GetPositions(curr_context)
    velocity = plant.GetVelocities(curr_context)

    z_position = positions[-1]
    z_velocity = velocity[-1]

    distance = np.fabs(z_position - current_target[-1])
    if distance < 0.05:
        if going_to_target == 1:
            current_target = target_two
            going_to_target = 2
        else:
            current_target = target_one
            going_to_target = 1

    e_position = (current_target[-1] - z_position)
    force_z = Kp*e_position - Kd*z_velocity
    print('p_e: {} vel: {}'.format(e_position, z_velocity))

    cylinder_system.wrench[2] = model_force + force_z

    time_ += sim_time_step
    simulator.AdvanceTo(time_)
    time.sleep(sim_time_step)
