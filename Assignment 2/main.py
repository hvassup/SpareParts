import sys


def run(is_simulation):
    if is_simulation:
        import simulation.simple_kinematic_simulator
    else:
        import real.start


if len(sys.argv) == 2 and sys.argv[1] == 'Simulation':
    run(is_simulation=True)
else:
    run(is_simulation=False)