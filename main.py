
from src.maze_generator import generate_mazes
from src.result import Result


def main():
    print("Initializing mazes...")
    
    mazes = generate_mazes()
    
    
    result_repeated_forward_a_star: Result = None
    result_repeated_backward_a_star: Result = None
    result_adaptive_a_star: Result = None
        
        
    
if __name__ == "__main__":
    main()