# Start everything from here!
import sys

def run(is_simulation):
    if is_simulation:
        import simulation.simulation
    else:
        import real.LED_start


if len(sys.argv) == 2 and sys.argv[1] == 'Simulation':
    run(is_simulation=True)
else:
    run(is_simulation=False)
