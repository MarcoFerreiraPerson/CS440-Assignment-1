
from src.maze_generator import generate_mazes, save_unsolved_mazes, save_solved_maze, visualize_maze
from src.result import Result


def main():
    print("Initializing mazes...")
    
    mazes = generate_mazes()
    
    print(len(mazes))
    visualize_maze(mazes[0], path=[(0,1), (0,2), (0,3)])  
    # result_repeated_forward_a_star: Result = None
    # result_repeated_backward_a_star: Result = None
    # result_adaptive_a_star: Result = None
        
        
    
if __name__ == "__main__":
    main()