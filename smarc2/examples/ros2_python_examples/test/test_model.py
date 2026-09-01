from ros2_python_examples.pid_model import PIDModel

def test_model():
    # Create the model object, with specific parameters
    # since different params might produce different numbers
    # while testing, use hardcoded values rather than relying on defaults!
    model = PIDModel(init_posi=[0.0, 0.0],
                     init_yaw=0,
                     max_rpm=1000,
                     max_thrust_angle=0.15)

    loop_period = 0.1

    # Test if calling compute without a waypoint works.
    print("Testing no-wp compute")
    yaw_u, thrust_u = model.compute_control_action(loop_period)
    assert yaw_u==0 and thrust_u==0,\
        f"compute_control_action returned non-0 controls w/o a WP: yaw_u:{yaw_u}, thrust_u:{thrust_u}"

    model.set_waypoint([0,10])

    # then use known-good hardcoded values for what you expect
    # in the case that the model is changed, these values should stay
    # the same!
    print("Testing first ever compute")
    yaw_u, thrust_u = model.compute_control_action(loop_period)
    assert yaw_u==0.15 and thrust_u==210.1,\
        f"compute_control_action returned unexpected yaw and thrust value for the first call! yaw_u:{yaw_u} != 0.15 or thrust_u:{thrust_u} != 210.1"
    
    print("Testing second compute")
    yaw_u, thrust_u = model.compute_control_action(loop_period)
    assert yaw_u==0.15 and thrust_u==10.2,\
        f"compute_control_action returned unexpected yaw and thrust value for the first call! yaw_u:{yaw_u} != 0.15 or thrust_u:{thrust_u} != 10.2"
    
    print("All good :D")


if __name__ == "__main__":
    test_model()