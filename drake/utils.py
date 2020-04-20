

def print_member_functions(object):
    for name in dir(object):
        if '_' != name[0] and '__' != name[:2]:
            print(name)


def get_body_homogeneous_matrix(rigid_body):
    frame = rigid_body.body_frame()
    fixed_position_rigid_transform = frame.GetFixedPoseInBodyFrame()
    homogeneous_matrix = fixed_position_rigid_transform.GetAsMatrix4()
    return homogeneous_matrix


def print_body(rigid_body, public_functions=True):
    """
    :param rigid_body: pydrake.multibody.tree.RigidBody
    :param public_functions: bool
    :return:
    """
    frame = rigid_body.body_frame()
    mass = rigid_body.default_mass()
    # pydrake.multibody.tree.SpatialInertia
    default_spatial_inertia = rigid_body.default_spatial_inertia()
    # pydrake.multibody.tree.UnitInertia
    default_unit_inertia = rigid_body.default_unit_inertia()

    # pydrake.math.RigidTransform
    fixed_position_rigid_transform = frame.GetFixedPoseInBodyFrame()
    homogeneous_matrix = fixed_position_rigid_transform.GetAsMatrix4()

    print('body translation: \n{}'.format(homogeneous_matrix[:-1, 3]))
    print('body orientation: \n{}'.format(homogeneous_matrix[:3, :3]))
    print('body mass:  {}'.format(mass))
