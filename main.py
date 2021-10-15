def run(is_simulation):
    if is_simulation:
        import simulation.simple_kinematic_simulator
    else:
        import real.start

run(is_simulation=True)