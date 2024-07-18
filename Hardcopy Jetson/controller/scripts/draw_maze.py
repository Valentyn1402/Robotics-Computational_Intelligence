from maze_solver.main import Maze, get_path
from maze_solver.src.a_star import a_star
from pathlib import Path

def main():
    tags_file = Path('/home/jetson/Downloads/tags.yaml')
    start = (0, 0)
    goal = (3, 0)
    
    # manuell example for maze drawing and debugging
    maze = Maze.from_tags(tags_file)
    maze.start_coordinate = start
    maze.goal_coordinate = goal
    
    dist, steps = a_star(maze.connectivity_matrix, maze.start_coordinate, maze.goal_coordinate)
    
    node = dist[maze.goal_coordinate]
    shortest_path = [node]
    while node.coord != maze.start_coordinate:
        node = node.prev
        shortest_path.append(node)
        
    # function which does everything
    path = get_path(tags_file, start, goal)
    print(path)
        
    maze.draw_maze(distances=dist, shortest_path=shortest_path)
    

if __name__ == "__main__":
    main()
