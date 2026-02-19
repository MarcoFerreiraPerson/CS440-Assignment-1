
from src.maze_generator import generate_mazes, save_unsolved_mazes, save_solved_maze, visualize_maze
from src.adaptive_search import search as adaptive_a_star_search
from src.repeated_search import search as repeated_a_star_search
from src.result import Result
from src.utils import get_start_and_goal 

def main():
    print("Initializing mazes...")
    
    mazes = generate_mazes()
    
    print(len(mazes))
    result_repeated_forward_a_star_large_g_value: Result = repeated_a_star_search(mazes[0], prefer_smaller_g_tie_break=False)
    result_repeated_forward_a_star_small_g_value: Result = repeated_a_star_search(mazes[0], prefer_smaller_g_tie_break=True)
    
    start, goal = get_start_and_goal(mazes[0])
    
    result_repeated_backward_a_star: Result = repeated_a_star_search(mazes[0], start=goal, goal=start, prefer_smaller_g_tie_break=False)
    result_adaptive_a_star: Result = adaptive_a_star_search(mazes[0])
    
    visualize_maze(result_adaptive_a_star.maze, path=result_adaptive_a_star.path)
    visualize_maze(result_repeated_forward_a_star_large_g_value.maze, path=result_repeated_forward_a_star_large_g_value.path)
    visualize_maze(result_repeated_forward_a_star_small_g_value.maze, path=result_repeated_forward_a_star_small_g_value.path)
    visualize_maze(result_repeated_backward_a_star.maze, path=result_repeated_backward_a_star.path)
    
    # Compare stats
    print("Comparing results...")
    print("Repeated Forward A* (Large g-value tie-break):")
    print(f"Path length: {len(result_repeated_forward_a_star_large_g_value.path)}")
    print(f"Total expanded nodes: {result_repeated_forward_a_star_large_g_value.total_expanded}")
    print(f"Time spent: {result_repeated_forward_a_star_large_g_value.time_spent:.4f} seconds")
    print("\nRepeated Forward A* (Small g-value tie-break):")
    print(f"Path length: {len(result_repeated_forward_a_star_small_g_value.path)}")
    print(f"Total expanded nodes: {result_repeated_forward_a_star_small_g_value.total_expanded}")
    print(f"Time spent: {result_repeated_forward_a_star_small_g_value.time_spent:.4f} seconds")
    print("\nRepeated Backward A*:")
    print(f"Path length: {len(result_repeated_backward_a_star.path)}")
    print(f"Total expanded nodes: {result_repeated_backward_a_star.total_expanded}")
    print(f"Time spent: {result_repeated_backward_a_star.time_spent:.4f} seconds")
    print("\nAdaptive A*:")
    print(f"Path length: {len(result_adaptive_a_star.path)}")
    print(f"Total expanded nodes: {result_adaptive_a_star.total_expanded}")
    print(f"Time spent: {result_adaptive_a_star.time_spent:.4f} seconds")
    
        
        
    
if __name__ == "__main__":
    main()