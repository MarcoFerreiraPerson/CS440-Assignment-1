# CS440-Assignment-1

## Run Commands

```bash
cd submitting-marco-ferreira-maf407/hw1
```

Generate mazes:

```bash
python create_grid_worlds.py --num_mazes 50 --seed 42 --output mazes.json
```

Run Repeated Forward A* (`q2.py`):

```bash
python q2.py --maze_file mazes.json --tie_braking both --output results_q2.json
```

Run Repeated Backward vs Forward (`q3.py`):

```bash
python q3.py --maze_file mazes.json --output results_q3.json
```

Run Adaptive vs Forward (`q5.py`):

```bash
python q5.py --maze_file mazes.json --output results_q5.json
```

Optional visualization (single maze):

```bash
python q2.py --maze_file mazes.json --tie_braking max_g --show_vis --maze_vis_id 0 --save_vis_path q2-vis-max-g.png
python q3.py --maze_file mazes.json --show_vis --maze_vis_id 0 --save_vis_path q3-vis-max-g.png
python q5.py --maze_file mazes.json --show_vis --maze_vis_id 0 --save_vis_path q5-vis-max-g.png
```
