# Robot Motion Planning
This is the 7th project for the Machine Learning Nanodegree Program. This project is inspired by and based on the annual competition event – the Micromouse, where a small robot mouse is asked to navigate small sized maze (https://en.wikipedia.org/wiki/Micromouse). The maze is in the size of 12×12, 14×14, or 16×16 cells. The robot is given two runs to navigate the maze, where in the first run, it will try to explore the unknown maze as much as possible, and in the second run it aims to reach the predefined goal area as soon as possible.

## Code Usage
- First download the repository to your desktop and unzip it.
- Please then see the 'proposal.pdf' and 'report.pdf' for more details.
- If you want to play around the code yourself, you can use the `showmaze.py` file to visualize any maze first. You can use the command `python showmaze.py test_maze_01.txt` to visualize the fist maze (12×12) for example.
- On the other hand, if you want to ask the robot the solve a spefic maze, you can use the `tester.py` file. You can type 'python tester.py test_maze_01.txt' to see how the robot solves the first maze, and output will be given on performance.
- To clarity, `test_maze_01.txt`, `test_maze_02.txt`, and `test_maze_03.txt` are pre-configured mazes to test my algorithm, the `test_maze_04.txt` is my customized maze to further test the robustness for the robot.
- You are also welcome to test this algorithm using your own maze, as long as it is in the size of 12×12, 14×14, or 16×16 cells.

## License
This project is licensed under the terms of the **MIT** license.