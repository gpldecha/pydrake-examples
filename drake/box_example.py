import os
import random
import numpy as np
from scipy.spatial.transform import Rotation as R
from pydrake.geometry import ConnectDrakeVisualizer
from pydrake.geometry.render import (
    MakeRenderEngineVtk,
    RenderEngineVtkParams,
)
from pydrake.multibody.math import SpatialForce
from pydrake.multibody.tree import MultibodyForces
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.multibody.tree import BodyIndex
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.sensors import RgbdSensor
from pydrake.systems.primitives import ConstantVectorSource, ConstantValueSource
from pydrake.systems.drawing import plot_system_graphviz
from pydrake.attic.multibody.rigid_body_tree import RigidBodyTree
from pydrake.attic.multibody.rigid_body_plant import RigidBodyPlant
import pydrake.multibody.plant as mbp
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.plant import VectorExternallyAppliedSpatialForced, ExternallyAppliedSpatialForce
import time

from pydrake.systems.framework import (
    AbstractValue,
)



sim_time_step = 0.001
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, sim_time_step)
object_instance = Parser(plant).AddModelFromFile('box.urdf')
scene_graph.AddRenderer("renderer", MakeRenderEngineVtk(RenderEngineVtkParams()))
ConnectDrakeVisualizer(builder, scene_graph)

plant.Finalize()


object_index = plant.GetBodyIndices(object_instance).pop()


force_object = ExternallyAppliedSpatialForce()
force_object.body_index = object_index
force_object.F_Bq_W = SpatialForce(tau=np.zeros(3), f=np.array([0., 0., 10.]))

forces = VectorExternallyAppliedSpatialForced()
forces.append(force_object)


value = AbstractValue.Make(forces)
force_system = builder.AddSystem(ConstantValueSource(value))

builder.Connect(force_system.get_output_port(0), plant.get_applied_spatial_force_input_port())


diagram = builder.Build()
simulator = Simulator(diagram)


context = simulator.get_mutable_context()

print('num_positions: {}'.format(plant.num_positions(object_instance)))
plant.SetPositions(context, object_instance, [0, 0, 0, 1, 0, 0, 0])


plant_context = diagram.GetMutableSubsystemContext(plant, context)
print(dir(plant_context))

time.sleep(1.0)
time_ = 0
print('Start')
start_force = 10
while True:
    curr_context = diagram.GetMutableSubsystemContext(plant, simulator.get_mutable_context())
    print(dir(curr_context))

    #force_object.F_Bq_W = SpatialForce(tau=np.zeros(3), f=np.array([0., 0., start_force]))

    start_force -= sim_time_step

    time_ += sim_time_step
    simulator.AdvanceTo(time_)
    time.sleep(sim_time_step)
