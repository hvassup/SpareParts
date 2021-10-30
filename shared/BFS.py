from collections import deque


# BFS which finds the first path that solves the game
def BFS(start_pos, to_pos, get_next_states):
    path = (start_pos,)
    queue = deque([])
    queue.append((start_pos, path))

    explored = set()

    while len(queue) > 0:
        current_robot_pos, path = queue.popleft()

        if current_robot_pos == to_pos:
            print('Done! :)')
            return path

        x, y = current_robot_pos
        next_states = get_next_states(x, y)

        for next_robot_pos in next_states:
            if next_robot_pos in explored:
                continue

            explored.add(next_robot_pos)

            new_path = path + (next_robot_pos,)
            queue.append((next_robot_pos, new_path))

    print('Something went wrong! :(', path)
    return 'Oops'
