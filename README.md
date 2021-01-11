# ME 495 Final Project: Connect Four!
This is the final project for the course ME 495: Embedded Systems in Robotics.

## Authors:
Juntao He, Peter Sauer, Xiaowei Bi, Yuming Jin

## What is Connect Four:
Connect Four is a two-player connection board game, in which the players choose a color and then take turns dropping colored discs into a seven-column, six-row vertically suspended grid. The pieces fall straight down, occupying the lowest available space within the column. 
The objective of the game is to be the first to form a horizontal, vertical, or diagonal line of four of one's own discs. 
Connect Four is a solved game. The first player can always win by playing the right moves.

## What is this project about:
This project is about utilizing a Sawyer robot arm to play the game connect four against a human player. 

## Demo Videos:
[link](https://drive.google.com/file/d/1XFv08zJQkzPhIdgkCEzb_7ZqwETmZYFJ/view?usp=sharing)
[link](https://drive.google.com/file/d/1JCa3yRlZn9Wqk8FF3n5pmnQAL0TNkeKR/view?usp=sharing)
[link](https://drive.google.com/file/d/1aIOC7iWrF-gh8JKGvElVQHGiTXhGbnQt/view?usp=sharing)

## Key Files:

### Nodes:
`nodes/camSubscribe.py` 
Scans the game board and play connect-4 with a human player by placing checkers into desired slots on the board
- This node will go to a set position above the gameboard to check the camera feed from the arm. 
- It will then use that information to find the pieces and slots to drop the pieces into using a combination of Hough Line transformations and april tags.
- This node will also use the head camera to the side of the gameboard to analyze the state of the game

### Launchfiles:
`launch/JTAS.launch`
 - Launches the `planning_contect.launch` launchfile from the `sawyer_moveit_config` package with pre-defined parameters
 - Launches the `move_group.launch` launchfile from the `sawyer_moveit_config` package
 - Launches the `joint_trajectory_action_server` node from the `intera_interface` package
 - Launches the `camSubscribe.py` node 

## My Focus:
For this project, I mainly focused on the game board scanning and analyzing process. Since the head monitor camera views the game board from an upper angle, the image it records is tilted. In order to restore the image into a horizontal parallel view, four apriltags are placed on four corners of the board, and a perspective transformation is performed.
