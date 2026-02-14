
from src.maze_generator import generate_mazes, save_unsolved_mazes, save_solved_maze, visualize_maze
from src.adaptive_search import search as adaptive_a_star_search
from src.result import Result


def main():
    print("Initializing mazes...")
    
    mazes = generate_mazes()
    
    print(len(mazes))
    # result_repeated_forward_a_star: Result = None
    # result_repeated_backward_a_star: Result = None
    result_adaptive_a_star: Result = adaptive_a_star_search(mazes[0])
    
    visualize_maze(result_adaptive_a_star.maze, path=result_adaptive_a_star.path)
        
        
    
if __name__ == "__main__":
    main()