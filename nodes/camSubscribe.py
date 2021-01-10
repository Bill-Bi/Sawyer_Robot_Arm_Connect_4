#!/usr/bin/env python3
import os
import sys
import cv2
import math
import copy
import cv_bridge
import cvlib as cv
from cvlib.object_detection import draw_bbox
import geometry_msgs
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge, CvBridgeError
import rospy
import intera_interface
from sensor_msgs.msg import Image
from intera_core_msgs.srv._IOComponentCommandSrv import IOComponentCommandSrv 
from intera_core_msgs.msg._IOComponentCommand import IOComponentCommand 
import numpy as np
import intera_interface
import intera_external_devices
import apriltag
import moveit_commander      
import moveit_msgs.msg   
import copy
from intera_interface import CHECK_VERSION
import random

""" 
Scans the game board and play connect-4 with a human player by placing checkers into desired slots on the board

- this node will go to a set position above the gameboard to check the camera feed from the arm. 
- It will then use that information to find the pieces and slots to drop the pieces into using a combination of Hough Line transformations and april tags.
- this node will also use the head camera to the side of the gameboard to analyze the state of the game

SUBSCRIBES:
+ Image (/io/internal_camera/head_camera/image_rect_color) ~ Receives the video feed from the head camera
+ Image (/io/internal_camera/right_hand_camera/image_raw) ~ Receives the video feed from the arm camera
"""

START_WIDTH = 200
START_HEIGHT = 100
WINDOW_WIDTH = 500
WINDOW_HEIGHT = 550
BOARD_WIDTH = 500
BOARD_HEIGHT = 500
BOARD_IMG_PATH = 'board.png'
PIECE_WIDTH = 46
PIECE_HEIGHT = 46
BLUE_IMG_PATH = 'blue.png'
RED_IMG_PATH = 'red.png'
ARROW_IMG_PATH = 'down_arrow.png'
ROW_COUNT = 6
COLUMN_COUNT = 7
PLAYER_PIECE = 1 #Red
AI_PIECE = 0 #Blue
EMPTY = -1
WINDOW_LENGTH = 4

class getCameraFeed:
    """ Receives camera feed from both the head and hand camera
        Processes the image recorded to determine the state of the game
        Picks up a checker and puts it into a desired slot on the game board
        Checks if the robot has won or lost the game
    """
    def __init__(self):
        rospy.init_node('cam_subscribe')
        rospy.wait_for_service('/io/internal_camera/head_camera/command')
        try: 
            cam_control = rospy.ServiceProxy('/io/internal_camera/head_camera/command', IOComponentCommandSrv) 
            cmd = IOComponentCommand() 
            cmd.time = rospy.Time.now() 
            cmd.op = 'set' 
            cmd.args = '{"signals": {"camera_streaming": {"data": [true], "format": {"type": "bool"}}}}' 
            
            resp = cam_control(cmd, 0) 
    
        except rospy.ServiceException as e: 
            print("Service call failed: %s"%e)

        rospy.wait_for_service('/io/internal_camera/right_hand_camera/command')
        try: 
            cam_control = rospy.ServiceProxy('/io/internal_camera/right_hand_camera/command', IOComponentCommandSrv) 
            cmd = IOComponentCommand() 
            cmd.time = rospy.Time.now() 
            cmd.op = 'set' 
            cmd.args = '{"signals": {"set_exposure": {"data": [10.0], "format": {"role":"output","type": "float"}}, "set_gain": {"data": [5.0], "format": {"role":"output","type": "float"}},"set_brightness": {"data": [12], "format": {"type": "int"}}}}' 
            
            resp = cam_control(cmd, 0) 
    
        except rospy.ServiceException as e: 
            print("Service call failed: %s"%e)

        self.feed = rospy.Subscriber("/io/internal_camera/head_camera/image_rect_color", Image, self.imageRecieved)
        self.hand_feed = rospy.Subscriber("/io/internal_camera/right_hand_camera/image_raw", Image, self.imageRecievedHand)

        moveit_commander.roscpp_initialize(sys.argv)                                                                    

        self.robot = moveit_commander.RobotCommander()                                                                       
        self.scene = moveit_commander.PlanningSceneInterface()                      
        self.group = moveit_commander.MoveGroupCommander("right_arm")
        self.group.set_max_velocity_scaling_factor(1)
        self.home = [1.4436796875,-0.1697880859375,0.2101689453125,-0.9971728515625,-0.5034541015625,1.09334375,3.3138671875]
        self.pixels = []
        
    def imageRecieved(self,img):
        """ Transforms the image message received from the head camera to cv2 image data type

            Args:
                img (Image): Raw img received from the camera
        """
        bridge = CvBridge()
        try:
            cv_img = bridge.imgmsg_to_cv2(img, "passthrough")
        except CvBridgeError as e:
            print(e)
        self.latest_board = cv_img
        # edges = cv2.Canny(cv_img,100,200)
        # cv2.imshow("Edges", edges)
        # self.detectObjects(cv_img)
        cv2.waitKey(3)

    def imageRecievedHand(self,img):
        """ Transforms the image message received from the hand camera to cv2 image data type
            Uses an apriltag detector to detect the apriltag in the cv2 images
            Determines the corners of the apriltag detected for position calibration

            Args:
                img (Image): Raw img received from the camera
        """
        bridge = CvBridge()
        try:
            cv_img = bridge.imgmsg_to_cv2(img, "passthrough")
            self.latest_hand = cv_img
        except CvBridgeError as e:
            print(e)
        # edges = cv2.Canny(cv_img,100,200)
        options = apriltag.DetectorOptions(families="tag36h11")
        detector = apriltag.Detector(options)
        tags = detector.detect(cv_img)
        for i in range(len(tags)):
            corners = tags[i].corners
            # print(corners)
            cv2.rectangle(cv_img,(int(corners[0][0]),int(corners[0][1])),(int(corners[2][0]),int(corners[2][1])),(0,0,0),1)
        cv2.imshow("Tags", cv_img)
        cv2.imwrite("current.png",cv_img)
        

        cv2.waitKey(3)

    # def detectObjects(self, img):
    #     """ Detects the common objects from the img input

    #         Args:
    #             img (cv::Mat): cv img
    #     """
    #     img=cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)
    #     bbox, label, conf = cv.detect_common_objects(img)
    #     out_img = draw_bbox(img,bbox,label,conf)
        
    #     cv2.imshow("Object View", out_img)

    def scan_for_color(self, img,center):
        """ Determines the color of the area around the center of the checker

            Args:
                img (cv::Mat): cv img
                center (array): The center of the checker
        """
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        low_red = np.array([0,89,69])
        high_red = np.array([58, 201, 205])
        low_blue = np.array([72,46,24])
        high_blue = np.array([127, 188, 121])
        red = cv2.inRange(img_hsv, low_red, high_red)
        blue = cv2.inRange(img_hsv, low_blue, high_blue)

        cv2.imshow('output2.jpg', red)
        thresh = 10
        r = 0
        b = 0
        for i in range(center[1]-thresh,center[1]+thresh):
            for j in range(center[0]-thresh,center[0]+thresh):
                r+=red[i][j]
                b+=blue[i][j]
        if r>10000:
            return 1
        if b>10000:
            return 0
        else:
            return -1            

    def getBoardState(self,img):
        """ Performs a perspective transform on the tilted board image to get a straight look at the board.
            Identifies each slot of the board to be red checker (0), blue checker (1) or empty (-1). 
            Returns a 2D array indicating the state of the current game board

            Args:
                img (cv::Mat): cv img of the board
            
            Returns:
                board_state (array) A 2D array that indicates the state of the current game board
        """
        # kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
        # img = cv2.filter2D(img, -1, kernel)
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        options = apriltag.DetectorOptions(families="36h11")
        detector = apriltag.Detector()
        tags = detector.detect(img_gray)
        print(tags)
        canny = cv2.Canny(img,100,200)
        cv2.imshow("test",img_gray)
        corners = [(0,0),(0,0),(0,0),(0,0)]
        for tag in tags:
            cv2.putText(img,str(tag.tag_id), (int(tag.center[0]),int(tag.center[1])), cv2.FONT_HERSHEY_COMPLEX,1,[0,0,255])
            if tag.tag_id == 1:
                corners[0]=(int(tag.center[0]),int(tag.center[1]))
            if tag.tag_id == 2:
                corners[1]=(int(tag.center[0]),int(tag.center[1]))
            if tag.tag_id == 3:
                corners[2]=(int(tag.center[0]),int(tag.center[1]))
            if tag.tag_id == 4:
                corners[3]=(int(tag.center[0]),int(tag.center[1]))
        print(corners)
        pts = np.float32([[corners[0][0],corners[0][1]],[corners[1][0],corners[1][1]],[corners[2][0],corners[2][1]],[corners[3][0],corners[3][1]]])
        new_pts = np.float32([[500,345],[770,345],[500,388],[770,390]])
        matrix = cv2.getPerspectiveTransform(pts, new_pts) 
        result = cv2.warpPerspective(img, matrix, (1024, 600)) 
        board = result[300:(550 + 1), 450:(820 + 1)] 
        size = ((820-450),250)
        for i in range(1,7):
            cv2.line(board,(0,int(size[1]/6*i)),(size[0],int(size[1]/6*i)),(0,0,0),2)

        for i in range(1,8):
            cv2.line(board,(int(size[0]/7*i),0),(int(size[0]/7*i),size[1]),(0,0,0),2)

        board_scan_center = [ [ (0,0) for y in range( 6 ) ] for x in range( 7 ) ]
        for i in range(1,7):
            for j in range(1,8):
                board_scan_center[j-1][i-1]=((int(size[0]/7*j-size[0]/14),int(size[1]/6*i-size[1]/12)))
                cv2.circle(board,(int(size[0]/7*j-size[0]/14),int(size[1]/6*i-size[1]/12)),20,[0,0,0],2)

        cv2.imshow("board",board)
        board_state = [ [ (0,0) for y in range( 7 ) ] for x in range( 6 ) ]

        for i in range(len(board_scan_center)):
            for j in range(len(board_scan_center[i])):
                board_state[j][i]=self.scan_for_color(board,board_scan_center[i][j])

        return board_state

    def checkTurn(self, board_state):
        """ Examines the board state to determine whether it's blue's turn or red's turn (Red always goes first)
            Red's turn: 0, Blue's turn: 1

            Args:
                board_state (2D array): A 2D array that indicates the state of the current game board
            
            Returns:
                int (int): An integer that determines whether it's blue's turn or red's turn
        """
        #red always goes first
        r=0
        b=0
        for i in board_state:
            for j in i:
                if j == 1:
                    r+=1
                if j == 0:
                    b+=1
        if r>b:
            #Blue's turn
            return 0
        else:
            #Red's turn
            return 1
    
    def scanGame(self,player,move):
        """ Moves the arm away from the view path of the head camera so that the head camera can scan the game board

            Args:
                player (int): Indicates which player's turn it is to play
                move (boolean): Indicates whether the robot should make a move
            
            Returns:
                board_state (array): A 2D array that indicates the state of the current game board
        """
        head = intera_interface.Head()
        print(head.pan_mode())
        head.set_pan(-0.536796875)
        limb = intera_interface.Limb()
        joints = limb.joint_names()
        rospy.sleep(1)
        positions = self.home
        pos_cmd=[]
        if move:
            for i in range(len(joints)): 
                joint_pos = {joints[i]: positions[i]}
                print(joint_pos)
                limb.move_to_joint_positions(joint_pos)
            
            limb.move_to_joint_positions({joints[0]:0.4436796875})
            turn = -1
            limb.move_to_joint_positions({joints[1]:-0.8})
        bs = [[]]
        bs = self.getBoardState(self.latest_board)
        turn = self.checkTurn(bs)
        # self.drawCheckers(bs)

        if not turn == player:
            self.scanGame(player,False)
        return bs      
    
    def traverse(self,dx,dy,dz):
        """ Traverse the robot arm to relative position indicated by dx, dy and dz

            Args:
                dx (int): Relative position movement in the x direction
                dy (int): Relative position movement in the y direction
                dz (int): Relative position movement in the z direction
        """
        current_pose = self.group.get_current_pose().pose
        print(current_pose)
        pose_goal = Pose()
        pose_goal.orientation.w = current_pose.orientation.w
        pose_goal.orientation.x = current_pose.orientation.x
        pose_goal.orientation.y = current_pose.orientation.y
        pose_goal.orientation.z = current_pose.orientation.z

        pose_goal.position.x = current_pose.position.x+dx
        pose_goal.position.y = current_pose.position.y+dy
        pose_goal.position.z = current_pose.position.z+dz

        self.group.set_pose_target(pose_goal)
        plan = self.group.go()        

    def moveToTag(self, num):
        ''' Function that moves the robot arm to the correct april tag from the top view of the game board and then places the checker

            Args:
                num (int) Number of the target checker slot
        '''
        #move to default starting position
        limb = intera_interface.Limb("right")
        gripper = intera_interface.Gripper()
        
        joints = limb.joint_names()
        print(joints)
        positions = self.home
        pos_cmd=[]
        for i in range(len(joints)): 
            joint_pos = {joints[i]: positions[i]}
            print(joint_pos)
            limb.move_to_joint_positions(joint_pos)
        limb.move_to_joint_positions({joints[0]:0.4436796875})
        rospy.sleep(1)

        
        #initialize april tag detector
        options = apriltag.DetectorOptions(families="tag36h11")
        detector = apriltag.Detector(options)
        tag_pos = [-1,-1]

        self.traverse(0.1,0,0)

        # move linearly in the y dimension until the desired april tag is in the correct place (ydim) on the camera feed
        while (tag_pos[0]<300 or tag_pos[0]>360) and not rospy.is_shutdown():
            rospy.sleep(0.1)
            tags = detector.detect(self.latest_hand)
            print(tag_pos)
            ind = -1
            for i in range(len(tags)):
                if tags[i].tag_id == num:
                    ind = i
                    tag_pos = tags[i].center
    
            if tag_pos[0]==-1:
                self.traverse(0,-0.08,0)
            else:
                correction=(tag_pos[1]-330)/10000
                self.traverse(0,correction,0)
        self.group.stop()

        # move linearly in the x dimension until the desired april tag is in the correct place (xdim) on the camera feed
        while (tag_pos[1]<190 or tag_pos[1]>240) and not rospy.is_shutdown():
            rospy.sleep(0.1)
            tags = detector.detect(self.latest_hand)
            print(tag_pos)
            ind = -1
            for i in range(len(tags)):
                if tags[i].tag_id == num:
                    ind = i
                    tag_pos = tags[i].center
    
            correction=(tag_pos[1]-215)/10000
            self.traverse(-correction,0,0)
        self.group.stop()

        # move ee joints to position checker for placement
        j_p = limb.joint_angle(joints[5])
        joint_pos = {joints[5]: j_p+np.pi/2}
        limb.move_to_joint_positions(joint_pos)
        rospy.sleep(1)

        # move linearly in the z dimension to deposit the checker
        self.traverse(0,0,-0.04)
        self.traverse(0.012,0,-.04)

        gripper.open()
        self.group.stop()

    def getCheckerSimple(self):
        ''' Picks up the checker
        '''
        limb = intera_interface.Limb("right")
        gripper = intera_interface.Gripper()
        joints = limb.joint_names()
        print(joints)
        positions = self.home
        pos_cmd=[]
        for i in range(len(joints)): 
            joint_pos = {joints[i]: positions[i]}
            print(joint_pos)
            limb.move_to_joint_positions(joint_pos)
        limb.move_to_joint_positions({joints[0]:0.4436796875})
        gripper.open()
        rospy.sleep(5)
        gripper.close()
        rospy.sleep(2)
    
    # def getChecker(self, num):

    #     #move to default starting position
    #     limb = intera_interface.Limb("right")
    #     gripper = intera_interface.Gripper()
    #     gripper.open()

    #     joints = limb.joint_names()
    #     print(joints)
    #     positions = self.home
    #     pos_cmd=[]
    #     for i in range(len(joints)): 
    #         joint_pos = {joints[i]: positions[i]}
    #         print(joint_pos)
    #         limb.move_to_joint_positions(joint_pos)
    #     limb.move_to_joint_positions({joints[0]:0.4436796875})

    #     rospy.sleep(1)
    #     initial_pose = self.group.get_current_pose().pose

    #     self.traverse(-0.1,0.1,0)

    #     #initialize april tag detector
    #     options = apriltag.DetectorOptions(families="tag36h11")
    #     detector = apriltag.Detector(options)
    #     tag_pos = [-1,-1]
    #     self.traverse(0,0,-0.4)

    #     # move linearly in the y dimension until the desired april tag is in the correct place (ydim) on the camera feed
    #     while (tag_pos[0]<300 or tag_pos[0]>380) and not rospy.is_shutdown():
    #         rospy.sleep(0.1)
    #         tags = detector.detect(self.latest_hand)
    #         print(tag_pos)
    #         ind = -1
    #         for i in range(len(tags)):
    #             if tags[i].tag_id == 8:
    #                 ind = i
    #                 tag_pos = tags[i].center
    
    #         if tag_pos[0]==-1:
    #             self.traverse(-0.1,0,0)
    #         else:
    #             correction=(tag_pos[1]-280)/3000
    #             self.traverse(-0.01,0,0)
    #     self.group.stop()
    #     self.traverse(-0.1,0,0)
    #     self.traverse(0,0,-0.43)
    #     self.traverse(0.05,0,0)
    #     rospy.sleep(1)
    #     gripper.close()
    #     rospy.sleep(1)
    #     self.traverse(0,0,0.1)
    #     positions = self.home
    #     pos_cmd=[]
    #     self.traverse(0,0,0.65)
    #     for i in range(len(joints)): 
    #         joint_pos = {joints[i]: positions[i]}
    #         print(joint_pos)
    #         limb.move_to_joint_positions(joint_pos)

    # def drawCheckers(self, arr):
    #     ''' Draw the checkers according to the current game board state onto the head screen

    #         Args:
    #             arr (2D array): A 2D array that indicates the state of the current game board
    #     '''
    #     path = '/home/psauer/rethink_ws/src/connect4/connect4_cv/board_back.png'
    #     print(path)
    #     img = cv2.imread(path)
    #     print(img)
    #     w = 1024
    #     h = 600
    #     w_s = w/7
    #     h_s = h/6
    #     for i in range(len(arr)):
    #         for j in range(len(arr[i])):
    #             if arr[i][j]==1:
    #                 cv2.circle(img,(int(w_s*j+w_s/2),int(h_s*i+h_s/2)),40,(0,0,255),-1)
    #             if arr[i][j]==0:
    #                 cv2.circle(img,(int(w_s*j+w_s/2),int(h_s*i+h_s/2)),40,(255,0,0),-1)
    #     cv2.imwrite("board_show.png",img)
    #     head_disp = intera_interface.HeadDisplay()
    #     head_disp.display_image("board_show.png",False,1)

class connect4_ai:
    ''' This class contains the AI for playing the game connect-4 agains a human player
    '''
    def __init__(self, level):
        self.ai_level = level
        self.board_status = []
        self.has_winner = False
        self.winner = -1
    
    def check_winner(self):
        ''' Check if the current game board has a winner

            Args: 
                None
            
            Return:
                None
        '''
        for y_idx in range(0, 6):
            if not self.has_winner:
                for x_idx in range(0, 7):
                    if self.winning_conditions(x_idx, y_idx, AI_PIECE):
                        self.has_winner = True
                        self.winner = AI_PIECE
                    elif self.winning_conditions(x_idx, y_idx, PLAYER_PIECE):
                        self.has_winner = True
                        self.winner = PLAYER_PIECE

    def update_board_status(self, curr_board_status):
        ''' Updates the board status, and calls for AI's turn

            Args:
                curr_board_status (2D array): A 2D array that indicates the state of the current game board
            
            Return:
                ai_turn(): calls for AI's turn
                -1: if the input board is the same as before
        '''
        # TODO: Conditions for the ai to start its turn need to be updated
        # for now, the ai starts its turn when the board_status changed (new piece placed)
        if curr_board_status != self.board_status:
            # self.board_status = []
            # for row in curr_board_status:
            #     new_row = []
            #     for i in range(0,7):
            #         new_row.append(row[6-i])
            #     self.board_status.append(new_row)
            self.board_status = curr_board_status
            target_col = self.ai_turn()
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            print(target_col)
            return target_col
        return -1
    
    def get_y_idx_available(self, board, x_idx):
        ''' Determine what the corresponding y_idx will be if place the checker in the x_idx slot

            Args:
                x_idx (int): An integer that indicates the x index that the robot is considering putting the checker into
            
            Return:
                y_idx (int): An integer that indicates the y index that the checker is going to be at
        '''
        for y_idx in range(0, 6):
            if board[y_idx][x_idx] == -1:
                return y_idx
        return -1

    def get_x_idx_available(self, board):
        ''' Determine the available x_idx for placing a new checker

            Args:
                board (2D array): A 2D array that indicates the state of a game board
            
            Return:
                available_x_idx (2D array): A 1D array that indicates the available slots for placing a new checker
        '''
        x_idx_available = []
        for x_idx in range(0, 7):
            if self.get_y_idx_available(board, x_idx) != -1:
                x_idx_available.append(x_idx)
        return x_idx_available

    def ai_turn(self):
        ''' AI's decision on where to place the checker
        
            Args: 
                None
            
            Return:
                x_idx (int): the slot where the AI decided to place the checker
        '''
        # Easy AI
        if self.ai_level == "easy":
            # Check if any piece will win the game
            for x_idx in range(0, 7):
                y_idx = self.get_y_idx_available(x_idx)
                if (self.winning_conditions(x_idx, y_idx, AI_PIECE)):
                    return x_idx

            # Check if any piece will let the opponent win the game
            for x_idx in range(0, 7):
                y_idx = self.get_y_idx_available(x_idx)
                if (self.winning_conditions(x_idx, y_idx, PLAYER_PIECE)):
                    return x_idx
            
            # Randomly pick a col
            x_idx = random.randint(0, 6)
            return x_idx
            
        # Hard AI using minimax algorithm
        elif self.ai_level == "hard":
            board_copy = copy.deepcopy(self.board_status)
            return(self.minimax(board_copy, 5, -math.inf, math.inf, True)[0])

    def get_next_open_row(self, board ,col):
        for r in range(ROW_COUNT):
            if board[ROW_COUNT-1-r][col] == EMPTY:
                return ROW_COUNT-1-r
    
    def minimax(self, board, depth, alpha, beta, maximizingPlayer):
        ''' Implementing minimax to find the best move for AI

            Args: 
                board (2D array):  2D array that indicates the state of a game board
                depth (int): depth limit
                alpha (int): initial score for maximizing player
                beta (int): initial score for minimizing player
                maximizingPlayer (boolean): maximizing/minimizing player
            
            Return:
                x_idx (int): the slot where the AI decided to place the checker
        '''
        # if depth == 0 or self.has_winner:
        #     if self.has_winner:
        #         if self.winner == AI_PIECE:
        #         # the winner is AI
        #             return (None, 100000000000000)
        #         elif self.winner == PLAYER_PIECE:
        #         # the winner is player
        #             return (None, -10000000000000)
        #         else: # Game is over, no more valid moves
        #             return (None, 0)
        #     else: # Depth is zero
        #         return (None, self.score_position(board, AI_PIECE))
        
        # x_idx_available = self.get_x_idx_available(board)
        # print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
        # print(x_idx_available)

        # if maximizingPlayer:
        #     curr_score = alpha
        #     selected_x_idx = random.choice(x_idx_available)
        #     for x_idx in x_idx_available:
        #         y_idx = self.get_y_idx_available(board, x_idx)
        #         board_copy = copy.deepcopy(board)
        #         # self.drop_piece(board_copy, y_idx, x_idx, AI_PIECE)
        #         new_score = self.minimax(board_copy, depth-1, alpha, beta, False)[1]
        #         if new_score > curr_score:
        #             curr_score = new_score
        #             selected_x_idx = x_idx
        #         alpha = max(alpha, curr_score)
        #         if alpha >= beta:
        #             break
        #     return selected_x_idx, curr_score
        # else: # Minimizing player
        #     curr_score = beta
        #     selected_x_idx = random.choice(x_idx_available)
        #     for x_idx in x_idx_available:
        #         row = self.get_y_idx_available(board, x_idx)
        #         board_copy = copy.deepcopy(board)
        #         # self.drop_piece(board_copy, row, x_idx, PLAYER_PIECE)
        #         new_score = self.minimax(board_copy, depth-1, alpha, beta, True)[1]
        #         if new_score < curr_score:
        #             curr_score = new_score
        #             selected_x_idx = x_idx
        #         beta = min(beta, curr_score)
        #         if alpha >= beta:
        #             break
        #     return selected_x_idx, curr_score
        
        valid_locations = self.get_valid_locations(board)
        is_terminal = self.is_terminal_node()
        if depth == 0 or is_terminal:
            if is_terminal:
                if self.winning_move(board, AI_PIECE):
                # the winner is AI
                    return (None, 100000000000000)
                elif self.winning_move(board, PLAYER_PIECE):
                # the winner is player
                    return (None, -10000000000000)
                else: # Game is over, no more valid moves
                    return (None, 0)
            else: # Depth is zero
                return (None, self.score_position(board, AI_PIECE))
        
        if maximizingPlayer:
            # print("max")
            value = -math.inf
            column = random.choice(valid_locations)
            for col in valid_locations:
                row = self.get_next_open_row(board, col)
                b_copy = copy.deepcopy(board)
                self.drop_piece(b_copy, row, col, AI_PIECE)
                new_score = self.minimax(b_copy, depth-1, alpha, beta, False)[1]
                if new_score > value:
                    value = new_score
                    column = col
                alpha = max(alpha, value)
                # print(alpha)
                if alpha >= beta:
                    break
            return column, value

        else: # Minimizing player
            # print("mini")
            value = math.inf
            column = random.choice(valid_locations)
            for col in valid_locations:
                row = self.get_next_open_row(board,col)
                b_copy = copy.deepcopy(board)
                self.drop_piece(b_copy, row, col, PLAYER_PIECE)
                new_score = self.minimax(b_copy, depth-1, alpha, beta, True)[1]
                if new_score < value:
                    value = new_score
                    column = col
                beta = min(beta, value)
                if alpha >= beta:
                    break
            return column, value

    def drop_piece(self, board, row, col, piece):
        board[row][col] = piece

    def count_piece(self, array, piece):
        counter = 0
        for i in range(len(array)):
            if array[i] == piece:
                counter += 1
        return counter

    def evaluate_window(self, window, piece):
        score = 0
        opp_piece = PLAYER_PIECE
        if piece == PLAYER_PIECE:
            opp_piece = AI_PIECE
            
        if self.count_piece(window,piece) == 4:
            score += 100
        elif self.count_piece(window,piece) == 3 and self.count_piece(window,EMPTY) == 1:
            score += 5
        elif self.count_piece(window,piece) == 2 and self.count_piece(window,EMPTY) == 2:
            score += 2

        if self.count_piece(window,opp_piece) == 3 and self.count_piece(window,EMPTY) == 1:
            score -= 4
        return score

    def get_board_col(self, board, col):
        col_array = []
        for i in range(ROW_COUNT):
            col_array.append(board[i][col])
        return col_array
        
    def get_board_row(self, board, row):
        row_array = []
        for i in range(COLUMN_COUNT):
            row_array.append(board[row][i])
        return row_array
    
    def score_position(self, board, piece):
        score = 0
       
        ## Score center column
        # center_array = [i for i in list(board[:][COLUMN_COUNT//2])]
        center_array = self.get_board_col(board, COLUMN_COUNT//2)
        center_count = center_array.count(piece)
        score += center_count * 3

        ## Score Horizontal
        for r in range(ROW_COUNT):
            row_array = self.get_board_row(board,r)
            for c in range(COLUMN_COUNT-3):
                window = row_array[c:c+WINDOW_LENGTH]
                score += self.evaluate_window(window, piece)

        ## Score Vertical
        for c in range(COLUMN_COUNT):
            col_array = self.get_board_col(board,c)
            for r in range(ROW_COUNT-3):
                window = col_array[r:r+WINDOW_LENGTH]
                # print(window)
                score += self.evaluate_window(window, piece)

        ## Score posiive sloped diagonal
        for r in range(ROW_COUNT-3):
            for c in range(COLUMN_COUNT-3):
                window = [board[r+i][c+i] for i in range(WINDOW_LENGTH)]
                score += self.evaluate_window(window, piece)

        for r in range(ROW_COUNT-3):
            for c in range(COLUMN_COUNT-3):
                window = [board[r+3-i][c+i] for i in range(WINDOW_LENGTH)]
                score += self.evaluate_window(window, piece)

        return score

    def get_valid_locations(self, board):
        valid_locations = []
        for col in range(COLUMN_COUNT):
            if self.is_valid_location(board,col):
                valid_locations.append(col)
        return valid_locations

    def is_valid_location(self,board, col):
    	return board[0][col] == EMPTY

    def is_terminal_node(self):
	    return self.winning_move(self.board_status, PLAYER_PIECE) or self.winning_move(self.board_status, AI_PIECE) or len(self.get_valid_locations(self.board_status)) == 0

    def winning_move(self,board, piece):
        # Check horizontal locations for win
        for c in range(COLUMN_COUNT-3):
            for r in range(ROW_COUNT):
                if board[r][c] == piece and board[r][c+1] == piece and board[r][c+2] == piece and board[r][c+3] == piece:
                    return True

        # Check vertical locations for win
        for c in range(COLUMN_COUNT):
            for r in range(ROW_COUNT-3):
                if board[r][c] == piece and board[r+1][c] == piece and board[r+2][c] == piece and board[r+3][c] == piece:
                    return True

        # Check positively sloped diaganols
        for c in range(COLUMN_COUNT-3):
            for r in range(ROW_COUNT-3):
                if board[r][c] == piece and board[r+1][c+1] == piece and board[r+2][c+2] == piece and board[r+3][c+3] == piece:
                    return True

        # Check negatively sloped diaganols
        for c in range(COLUMN_COUNT-3):
            for r in range(3, ROW_COUNT):
                if board[r][c] == piece and board[r-1][c+1] == piece and board[r-2][c+2] == piece and board[r-3][c+3] == piece:
                    return True

    def winning_conditions(self, x_idx, y_idx, color):
        ''' Checks whether the robot or the human player has won the game

            Args:
                x_idx (int): the x index of the next checker that is going to be placed
                y_idx (int): the y index of the next checker that is going to be placed
                color (int): the color of the next checker that is going to be placed
            
            Return:
                _ (boolean): A boolean that indicates whether the robot or the human player has won the game
        '''
        return (x_idx <= 3 and self.board_status[y_idx][x_idx+1] == color and self.board_status[y_idx][x_idx+2] == color and self.board_status[y_idx][x_idx+3] == color)\
                or (x_idx >= 3 and self.board_status[y_idx][x_idx-1] == color and self.board_status[y_idx][x_idx-2] == color and self.board_status[y_idx][x_idx-3] == color)\
                or (y_idx <= 2 and self.board_status[y_idx+1][x_idx] == color and self.board_status[y_idx+2][x_idx] == color and self.board_status[y_idx+3][x_idx] == color)\
                or (y_idx >= 2 and self.board_status[y_idx-1][x_idx] == color and self.board_status[y_idx-2][x_idx] == color and self.board_status[y_idx-3][x_idx] == color)\
                or (x_idx <= 3 and y_idx <= 2 and self.board_status[y_idx+1][x_idx+1] == color and self.board_status[y_idx+2][x_idx+2] == color and self.board_status[y_idx+3][x_idx+3] == color)\
                or (x_idx >= 3 and y_idx <= 2 and self.board_status[y_idx+1][x_idx-1] == color and self.board_status[y_idx+2][x_idx-2] == color and self.board_status[y_idx+3][x_idx-3] == color)\
                or (x_idx <= 3 and y_idx >= 3 and self.board_status[y_idx-1][x_idx+1] == color and self.board_status[y_idx-2][x_idx+2] == color and self.board_status[y_idx-3][x_idx+3] == color)\
                or (x_idx >= 3 and y_idx >= 3 and self.board_status[y_idx-1][x_idx-1] == color and self.board_status[y_idx-2][x_idx-2] == color and self.board_status[y_idx-3][x_idx-3] == color)

                    
if __name__ == '__main__':
    ai = connect4_ai(level="hard")
    c = getCameraFeed()
    current_col = 0
    bs = c.scanGame(0, True)
    print(bs)
    current_col = 7 - ai.update_board_status(bs)
    while(not current_col == -1 and not current_col == -2):
        print("COLUMN", current_col)
        c.getCheckerSimple()
        c.moveToTag(current_col)
        rospy.sleep(1)
        bs = c.scanGame(0,True)
        current_col = 7 - ai.update_board_status(bs)
    
    rospy.spin()

