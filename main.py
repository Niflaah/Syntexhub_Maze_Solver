
# Maze Solver Using A* Search Algorithm

import heapq

# Maze representation
# 0 = Open path
# 1 = Wall

maze = [
    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    [1, 1, 0, 1, 1, 0, 1, 1, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
    [0, 1, 1, 1, 1, 1, 1, 0, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
    [1, 1, 1, 1, 1, 1, 0, 1, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 1, 1, 1, 1, 1, 1, 1, 0]
]

# Starting and Goal positions
start = (0, 0)
goal = (7, 9)


# Calculate Manhattan Distance
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


# Reconstruct the shortest path
def reconstruct_path(came_from, current):

    path = [current]

    while current in came_from:
        current = came_from[current]
        path.append(current)

    path.reverse()

    return path


# A* Search Algorithm
def astar(maze, start, goal):

    rows = len(maze)
    cols = len(maze[0])

    open_list = []

    heapq.heappush(
        open_list,
        (heuristic(start, goal), start)
    )

    came_from = {}
    g_score = {start: 0}

    directions = [
        (0, 1),   # Right
        (1, 0),   # Down
        (0, -1),  # Left
        (-1, 0)   # Up
    ]

    while open_list:

        current_f, current = heapq.heappop(open_list)

        # Skip outdated entries
        if current_f > g_score[current] + heuristic(current, goal):
            continue

        # Check whether goal is reached
        if current == goal:
            return reconstruct_path(came_from, current)

        # Explore neighboring cells
        for direction in directions:

            neighbor = (
                current[0] + direction[0],
                current[1] + direction[1]
            )

            # Check maze boundaries and walls
            if (
                0 <= neighbor[0] < rows
                and 0 <= neighbor[1] < cols
                and maze[neighbor[0]][neighbor[1]] == 0
            ):

                tentative_g_score = g_score[current] + 1

                if (
                    neighbor not in g_score
                    or tentative_g_score < g_score[neighbor]
                ):

                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score

                    f_score = (
                        tentative_g_score
                        + heuristic(neighbor, goal)
                    )

                    heapq.heappush(
                        open_list,
                        (f_score, neighbor)
                    )

    return None


# Display Maze
def display_maze(maze, path=None):

    path_set = set(path) if path else set()

    for row in range(len(maze)):

        for col in range(len(maze[0])):

            position = (row, col)

            if position == start:
                print("S", end=" ")

            elif position == goal:
                print("G", end=" ")

            elif maze[row][col] == 1:
                print("#", end=" ")

            elif position in path_set:
                print("*", end=" ")

            else:
                print(".", end=" ")

        print()


# Main Program
if __name__ == "__main__":

    print("Original Maze:")
    display_maze(maze)

    path = astar(maze, start, goal)

    if path:

        print("\nMaze Solved Successfully!")

        print("Path:", path)

        print("Total Steps:", len(path) - 1)

        print("\nSolved Maze:")

        display_maze(maze, path)

    else:

        print("\nNo path found!")
