import numpy as np
import random

class Robot(object):
    def __init__(self, maze_dim):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        '''

        self.location = [0, 0]
        self.heading = 'u'
        self.maze_dim = maze_dim

        self.dir_sensors = {'u': ['l', 'u', 'r'],
                            'r': ['u', 'r', 'd'],
                            'd': ['r', 'd', 'l'],
                            'l': ['d', 'l', 'u'],
                            'up': ['l', 'u', 'r'],
                            'right': ['u', 'r', 'd'],
                            'down': ['r', 'd', 'l'],
                            'left': ['d', 'l', 'u']}

        self.dir_move = {'u': [0, 1],
                         'r': [1, 0],
                         'd': [0, -1],
                         'l': [-1, 0],
                         'up': [0, 1],
                         'right': [1, 0],
                         'down': [0, -1],
                         'left': [-1, 0]}

        self.dir_reverse = {'u': 'd',
                            'r': 'l',
                            'd': 'u',
                            'l': 'r',
                            'up': 'd',
                            'right': 'l',
                            'down': 'u',
                            'left': 'r'}

        self.dir_arrow = {'u': '^',
                          'r': '>',
                          'd': 'v',
                          'l': '<',
                          '-': '-'}

        self.findGoal = False

        self.visits = 1

        # This 2D array is used to identify if a cell is visited
        # The value 0 means unvisited and 1 means visited, initially everycell is unvisited  # NOQA
        self.visitedGrid = [[0 for row in range(self.maze_dim)] for col in range(self.maze_dim)]  # NOQA
        self.visitedGrid[0][0] = 1

        # This 2D array is used to record the permissibility of each cell
        # For example, if the value for a cell is 1, that means the cell is only open on the top,  # NOQA
        # and close on the other 3 sides. Initially, value of every cell is -1 to indicate unknown  # NOQA
        self.valueGrid = [[0 for row in range(self.maze_dim)] for col in range(self.maze_dim)]  # NOQA
        self.valueGrid[0][0] = 1

        # This 2D array is used to record the heuristic for each cell
        # The heuristic is a value for shorest movement from this cell to goal
        # Initially, every cell has a heuristic value of -1 to indicate unkonw
        self.heuGrid = [[-1 for row in range(self.maze_dim)] for col in range(self.maze_dim)]  # NOQA

        # This 2D array is used to record the policy for each cell
        # The policy of each cell is a char: 'u', 'r', 'd', 'l',
        # and can be interpreted as an arrow for the robot in the cell
        # to move closer to the goal area. Initially they are set to '-'
        self.policyGrid = [['-' for row in range(self.maze_dim)] for col in range(self.maze_dim)]

        # This 2D array is used to store the arrow of policy for each cell,
        # based on the policyGrid, so that the policy can be more easily
        # understood. Initially each cell in the arrowGrid is set to '-'
        self.arrowGrid = [['-' for row in range(self.maze_dim)] for col in range(self.maze_dim)]

        # This dictionary will be referenced in the updateCellValue() function,
        # If means different numeric values (accounting for permissibility)
        # from different directions that can be added to calcuate the cell value  # NOQA
        self.dir_value = {'u': 1, 'r': 2, 'd': 4, 'l': 8}

        self.step = 1

        self.run_2 = False

    def updateCellValue(self, location, heading, sensors):
        '''
        This function is used in the first (exploratory) run.
        Especially, at the beginning of each step, the robot will call it,
        if and only if it hasn't visited the current cell before.

        Depending on the robot location, heading, and sensors, the robot will
        update the value (permissibility) for this cell, so that we know
        whether or not we can get into (or out of) this cell in a certain
        direction.

        For example, if the cell value is 9, it means this cell can be entered
        from the top or left side, but not from bottom or right side.
        '''

        permitted_dirs = []  # store the permitted travel directions into this cell  # NOQA

        # The cell is always visitable, from the opposite direction of the robot's heading  # NOQA
        permitted_dirs.append(self.dir_reverse[heading])

        # Know the directions of sensors in the global direction system
        global_dirs = self.dir_sensors[heading]

        for i in range(len(sensors)):
            if sensors[i] > 0:  # means that this travel direction is allowed
                permitted_dirs.append(global_dirs[i])

        cellValue = 0
        for direction in permitted_dirs:
            cellValue += self.dir_value[direction]

        # For example, if permitted_dirs == ['u', 'r', 'd'], cellValue is 7

        # Finally update this cellValue to valueGrid
        x, y = location
        self.valueGrid[x][y] = cellValue

    def fixMissingCellValue(self):
        '''
        This function will only be called once, at the end of the 1st run.
        The function will modify the self.valueGrid, in particular for each
        cell our robot never visited in the 1st run, it tries to calculate the
        value for that cell, if all the adjacent cell of that cell is visited.
        '''
        for x in range(self.maze_dim):
            for y in range(self.maze_dim):

                if self.valueGrid[x][y] == 0:
                    # 0 is initial value for every cell in self.valueGrid,
                    # so if the cell value is still 0 now,
                    # it means this cell is never unvisited in the 1st run

                    if y+1 == self.maze_dim:  # if this cell is exactly on the top boundary of maze
                        cellValue_top = 0  # obviously, this cell has wall on the top side
                    elif self.valueGrid[x][y+1] > 0:  # if there is a visited cell on the top side of this cell
                        cellValue_top = (self.valueGrid[x][y+1] & 4 != 0) * 1
                    else:
                        # unfortunately, the cell on the top side of this cell is also unvisited
                        # no need to calculate value for this cell
                        break

                    if x+1 == self.maze_dim:  # if this cell is exactly on the right boundary of maze
                        cellValue_right = 0  # obviously, this cell has wall on the right side
                    elif self.valueGrid[x+1][y] > 0:  # if there is a visited cell on the right side of this cell
                        cellValue_right = (self.valueGrid[x+1][y] & 8 != 0) * 2
                    else:
                        # unfortunately, the cell on the right side is also unvisited
                        break

                    if y-1 == -1:  # if this cell is exactly on the bottom boundary of maze
                        cellValue_bottom = 0  # obviously, this cell has wall on the bottom side
                    elif self.valueGrid[x][y-1] > 0:  # if there is a visited cell on the bottom side of this cell
                        cellValue_bottom = (self.valueGrid[x][y-1] & 1 != 0) * 4
                    else:
                        # unfortunately, the cell on the bottom side is also unvisited
                        break

                    if x-1 == -1:  # if this cell is exactly on the left boundary of maze
                        cellValue_left = 0  # obviously, this cell has wall on the left side
                    elif self.valueGrid[x-1][y] > 0:  # if there is a visited cell on the left side of this cell
                        cellValue_left = (self.valueGrid[x-1][y] & 2 != 0) * 8
                    else:
                        # unfortunately, the cell on the left side is also unvisited
                        break

                    # if we reach this sentence, we have got all valid values
                    # for top, right, bottom and left side for this cell
                    cellValue = cellValue_top + cellValue_right + cellValue_bottom + cellValue_left

                    # assign this value to this unvisited cell
                    self.valueGrid[x][y] = cellValue

    def printGrid(self, grid):

        tempGrid = [[-10 for row in range(self.maze_dim)] for col in range(self.maze_dim)]

        for x in range(self.maze_dim):
            for y in range(self.maze_dim):
                new_x = (self.maze_dim - 1) - y
                new_y = x
                tempGrid[new_x][new_y] = grid[x][y]

        for row in tempGrid:
            print(row)

        print("")

    def calculateArrows(self):
        '''
        This function will modify self.arrowGrid based on the self.policyGrid,
        so that the policy is more easily visualizable
        '''
        for x in range(self.maze_dim):
            for y in range(self.maze_dim):
                self.arrowGrid[x][y] = self.dir_arrow[self.policyGrid[x][y]]

    def calculateAllowedActions(self, location, heading, sensors):
        '''
        This function will calculate allowed actions for the robot,
        at a given location, heading and sensors information.

        In particular, this function will return a list, where each element
        in the list is a 2-elements tuple to indicate the direction and
        movements.

        For example, if this function returns [('u', 1), ('l', 1), ('l','2')],
        that means there're 3 allowed actions for the robot at this stage. One
        is to make 1 movement up, one is to make 1 movemenet to the left, and
        and the other one is to make 2 movements to the left. Note that the
        directions 'u', and 'l' are the global directions, and or the
        absolute directions.
        '''

        # Get the headings of 3 sensors in the global direction system
        sensors_dirs = self.dir_sensors[heading]

        # Use this array to store allowed actions
        allowed_actions = []

        for i in range(len(sensors)):
            if sensors[i] > 0:  # at least one movement on this global direction is allowed  # NOQA
                max_movement = min(3, sensors[i])  # adjuest the allowed movement to 3 at most  # NOQA
                for move in range(1, max_movement + 1):
                    allowed_actions.append((sensors_dirs[i], move))

        return allowed_actions

    def calculatePreferedActions(self, location, allowed_actions):
        '''
        This funtion will be passed currect robot's location and all the
        allowed actions for the robot in the next step.

        This function will calculate, among these actions, which are prefered
        actions. A prefered action is the action that will send the robot to a
        cell it hasn't visited before.

        In other words, this function is a filtering process to keep the
        actions that will send the robot to unvisited cells.
        '''
        prefered_actions = []
        x, y = location
        for action in allowed_actions:
            direction = action[0]  # for example, direction == 'u'
            move = action[1]  # for example, move == 2
            delta = self.dir_move[direction]  # for example, delta == [0, 1]
            # calculate the new coordinate based on this action
            new_x = x + move * delta[0]
            new_y = y + move * delta[1]
            if not self.visitedGrid[new_x][new_y]:  # if cell on the new_coordinate hasn't been visited  # NOQA
                prefered_actions.append(action)

        return prefered_actions

    def calculateRotation(self, heading, chosen_action):
        '''
        This function receives the robot's heading and chosen_action for the
        next step and calculate the rotation it should make.

        Rotation can either be -90 (counter clockwise), 90 (clock wise) or 0
        (no rotation at all).
        '''
        sensors_headings = self.dir_sensors[heading]  # current sensors' headings  # NOQA
        new_heading = chosen_action[0]  # The heading after making this action

        if new_heading == sensors_headings[1]:
            rotation = 0
        elif new_heading == sensors_headings[0]:
            rotation = -90
        else:
            rotation = 90

        return rotation

    def updatePosition(self, chosen_action):
        '''
        According to the project guide, the robot_pos is defined in the
        tester.py file and outside of the Robot class. When calling the
        testrobot.nextmove() function in tester.py, it doesn't pass the robot_pos.

        Moreover, students are not allowed to modify the tester.py, which renders awkward for them to
        access location and heading for the robot object, after robot has started moving.

        Therefore, it is only possible to update the robot position by ourself
        here.

        This function will do 2 things.
        (1) - update the heading of the robot
        (2) - update the location of the robot
        '''
        # Update the heading for our robot object
        self.heading = chosen_action[0]

        # update the position for our robot object
        moves = chosen_action[1]
        delta = self.dir_move[self.heading]
        self.location[0] += delta[0] * moves
        self.location[1] += delta[1] * moves

    def turnClockWise(self):
        '''
        This function is used to update robot position when
        robot is only going to turn 90 degree clockwise
        without anymovement.

        That means we only need to update the heading of the robot
        '''
        sensors_headings = self.dir_sensors[self.heading]
        self.heading = sensors_headings[2]

    def calculateHeuGrid(self):
        '''
        This function is used to generate the heuristic grid
        for the maze. The heuristic grid will be later referened using dynamic
        programming to find a global policy for this maze.

        This function is only called once, when it is the last step for the 1st
        run. This function will calculate the heuristic value for each cell
        and modify the robot.heuGrid variable.
        '''
        # A list used to keep track of the active cells
        current_active_cells = []

        # We initiate the heuristic calculation by assigning 0 to the central
        # 4 cells in the maze, and push them in the list
        for x in range(self.maze_dim/2 - 1, self.maze_dim/2 + 1):
            for y in range(self.maze_dim/2 - 1, self.maze_dim/2 + 1):
                self.heuGrid[x][y] = 0
                current_active_cells.append([(x, y), 0])

        while current_active_cells:  # while the list is not empty
            active_cell = current_active_cells.pop(0)
            x, y = active_cell[0]
            temp_heuristic = active_cell[1]

            if self.valueGrid[x][y] & 1:  # the active_cell is open on the top
                if y+1 <= self.maze_dim -1 and self.valueGrid[x][y+1] > 0:
                    # make sure the adjacent cell exists and does have open on the bottom
                    if self.heuGrid[x][y+1] == -1 or self.heuGrid[x][y+1] > temp_heuristic + 1:
                        # check if need to update the heuristic for the adjacent cell on the top
                        self.heuGrid[x][y+1] = temp_heuristic + 1
                        current_active_cells.append([(x, y+1), temp_heuristic + 1])

            if self.valueGrid[x][y] & 2:  # the active_cell is open on the right
                if x+1 <= self.maze_dim -1 and self.valueGrid[x+1][y] > 0:
                    # make sure the adjacent cell exists and does have open on the left
                    if self.heuGrid[x+1][y] == -1 or self.heuGrid[x+1][y] > temp_heuristic + 1:
                        # check if need to update the heuristic for the adjacent cell on the right
                        self.heuGrid[x+1][y] = temp_heuristic + 1
                        current_active_cells.append([(x+1, y), temp_heuristic + 1])

            if self.valueGrid[x][y] & 4:  # the active_cell is open on the bottom
                if y-1 >= 0 and self.valueGrid[x][y-1] > 0:
                    # make sure the adjacent cell exists and does have open on the left
                    if self.heuGrid[x][y-1] == -1 or self.heuGrid[x][y-1] > temp_heuristic + 1:
                        # check if need to update the heuristic for the adjacent cell on the bottom
                        self.heuGrid[x][y-1] = temp_heuristic + 1
                        current_active_cells.append([(x, y-1), temp_heuristic + 1])

            if self.valueGrid[x][y] & 8:  # the active_cell is open on the left
                if x-1>=0 and self.valueGrid[x-1][y] > 0:
                    # make sure the adjacent cell exists and does have open on the right
                    if self.heuGrid[x-1][y] == -1 or self.heuGrid[x-1][y] > temp_heuristic + 1:
                        # check if need to update the heuristic for the adjacent cell on the left
                        self.heuGrid[x-1][y] = temp_heuristic + 1
                        current_active_cells.append([(x-1, y), temp_heuristic + 1])

    def calculatePolicyGrid(self):
        '''
        This function will be called at the end of 1st run,
        after the robot has calculated the heuristic grid.

        The function will actually modify the self.policyGrid, so that
        it finally contains correct policy for the robot to follow.
        '''
        for x in range(self.maze_dim):
            for y in range(self.maze_dim):
                cell_value = self.valueGrid[x][y]

                # No need to calculate the policy if the cell is already in goal area
                if x not in range(self.maze_dim/2 - 1, self.maze_dim/2 + 1) or y not in range(self.maze_dim/2 - 1, self.maze_dim/2 + 1):
                    # allowed_dirs is used to store the allowed move on this cell, for exmaple ['u','r']
                    allowed_dirs = []

                    # adajacent_heuristics is used to store the heuristic values for the allowed_dirs, for example [16, 14]
                    adjacent_heuristics = []


                    if cell_value & 1:  # cell is open on the top
                        if y+1 <= self.maze_dim - 1:
                            if self.heuGrid[x][y+1] >= 0:
                                allowed_dirs.append('u')
                                adjacent_heuristics.append(self.heuGrid[x][y+1])

                    if cell_value & 2:  # cell is open on the right
                        if x+1 <= self.maze_dim - 1:
                            if self.heuGrid[x+1][y] >= 0:
                                allowed_dirs.append('r')
                                adjacent_heuristics.append(self.heuGrid[x+1][y])

                    if cell_value & 4:  # cell is open on the bottom
                        if y-1 >= 0:
                            if self.heuGrid[x][y-1] >=0:
                                allowed_dirs.append('d')
                                adjacent_heuristics.append(self.heuGrid[x][y-1])

                    if cell_value & 8:  # cell is open on the left
                        if x-1 >= 0:
                            if self.heuGrid[x-1][y] >=0:
                                allowed_dirs.append('l')
                                adjacent_heuristics.append(self.heuGrid[x-1][y])

                    if len(adjacent_heuristics) == 0:
                        # print("x,y", x,y, "cellValue", cell_value)
                        pass
                    else:
                        min_heuristic = min(adjacent_heuristics)
                        indices = [i for i, val in enumerate(adjacent_heuristics) if val == min_heuristic]
                        index = random.choice(indices)
                        self.policyGrid[x][y] = allowed_dirs[index]


    def next_move(self, sensors):
        '''
        Use this function to determine the next move the robot should make,
        based on the input from the sensors after its previous move. Sensor
        inputs are a list of three distances from the robot's left, front, and
        right-facing sensors, in that order.

        Outputs should be a tuple of two values. The first value indicates
        robot rotation (if any), as a number: 0 for no rotation, +90 for a
        90-degree rotation clockwise, and -90 for a 90-degree rotation
        counterclockwise. Other values will result in no rotation. The second
        value indicates robot movement, and the robot will attempt to move the
        number of indicated squares: a positive number indicates forwards
        movement, while a negative number indicates backwards movement. The
        robot may move a maximum of three units per turn. Any excess movement
        is ignored.

        If the robot wants to end a run (e.g. during the first training run in
        the maze) then returing the tuple ('Reset', 'Reset') will indicate to
        the tester to end the run and return the robot to the start.
        '''
        # Get the current location of the robot
        x, y = self.location

        # If it is the 2nd run, follow instruction from policyGrid to guide robot to move
        if self.run_2 == True:

            policy = self.policyGrid[x][y]

            if policy == 'u':  # if policy for this cell is up
                if self.policyGrid[x][y+1] == 'u':  # if policy for the cell upside is also up
                    if self.policyGrid[x][y+2] == 'u':  # if policy for the cell even upside is also up
                        movement = 3
                    else:
                        movement = 2
                else:
                    movement = 1

                rotation = self.calculateRotation(self.heading, ['u', 1])
                self.location[1] += movement
                self.heading = 'u'

            elif policy == 'r':  # if policy for this cell is right
                if self.policyGrid[x+1][y] == 'r':  # if the policy for the cell right side is also right
                    if self.policyGrid[x+2][y] == 'r':  # if the policy for the cell even right side is also right
                        movement = 3
                    else:
                        movement = 2
                else:
                    movement = 1

                rotation = self.calculateRotation(self.heading, ['r', 1])
                self.location[0] += movement
                self.heading = 'r'

            elif policy == 'd':  # if policy for this cell is down
                if self.policyGrid[x][y-1] == 'd':  # if the policy for the cell down side is also down
                    if self.policyGrid[x][y-2] == 'd':  # if the policy for the cell even down side is also down
                        movement = 3
                    else:
                        movement = 2
                else:
                    movement = 1

                rotation = self.calculateRotation(self.heading, ['d', 1])
                self.location[1] -= movement
                self.heading = 'd'

            elif policy == 'l':  # if policy for this cell is left
                if self.policyGrid[x-1][y] == 'l':  # if the policy for the cell left side is also left
                    if self.policyGrid[x-2][y] == 'l':  # if the policy for the cell even left side is also left
                        movement = 3
                    else:
                        movement = 2
                else:
                    movement = 1

                rotation = self.calculateRotation(self.heading, ['l', 1])
                self.location[0] -= movement
                self.heading = 'l'

            print(x,y,policy,movement)

            return rotation, movement


        # If it is the 1st run for the robot
        if self.run_2 == False:
            # Check if the robot already entered the goal
            goal_bounds = [self.maze_dim/2 - 1, self.maze_dim/2]
            if x in goal_bounds and y in goal_bounds:
                self.findGoal = True
            # Now check if robot has explored all the cells, or it has spend 900 steps in run 1 and visited goal
            if (self.visits >= self.maze_dim * self.maze_dim) or (self.step == 900 and self.findGoal):
                print("visitedGrid:")
                self.printGrid(self.visitedGrid)
                print("")
                
                print("valueGrid:")
                self.fixMissingCellValue()
                self.printGrid(self.valueGrid)
                print("")
                
                print("heuGrid:")
                self.calculateHeuGrid()
                self.printGrid(self.heuGrid)
                print("")
                
                print("policyGrid:")
                self.calculatePolicyGrid()
                self.calculateArrows()
                self.printGrid(self.arrowGrid)
                print("")

                self.location = [0, 0]
                self.heading = 'u'
                self.run_2 = True
                return 'Reset', 'Reset'

        print("Step:", self.step)
        print(x, y)
        # print("sensors", sensors)
        print("find goal yet?", self.findGoal)

        # If our robot hasn't visited this cell before
        if not self.visitedGrid[x][y]:
            # Update the value for this cell, depeding on location, robot's heading and sensors  # NOQA
            self.updateCellValue(self.location, self.heading, sensors)  # NOQA

        # Now let's calculate allowed actions for this robot,
        # depending on location, headings and sensors
        allowed_actions = self.calculateAllowedActions(self.location, self.heading, sensors)  # NOQA
        print("allow actions:", allowed_actions)

        # Now let's calculated the robot's prefered actions from the
        # allowed_actions. An action is prefered by the robot, if it will send
        # the robot to a cell it has not visited before
        prefered_actions = self.calculatePreferedActions(self.location, allowed_actions)  # NOQA
        print("prefered actions:", prefered_actions)

        # if robot does have several prefered actions available
        if prefered_actions:
            chosen_action = random.choice(prefered_actions)  # pick up a random prefered action  # NOQA
            rotation = self.calculateRotation(self.heading, chosen_action)  # NOQA
            movement = chosen_action[1]

        # if the robot has allowed actions, but all of them are not prefered
        elif allowed_actions:
            chosen_action = random.choice(allowed_actions)  # pick up a random allowed action  # NOQA
            rotation = self.calculateRotation(self.heading, chosen_action)  # NOQA
            movement = chosen_action[1]

        else:  # that means our robot doesn't even have allowed actions!
            # the only reason for this is that our robot gets stuck at a dead end  # NOQA

            # We tell the robot to turn 90 degrees clockwise without movement
            rotation = 90
            movement = 0

        print("rotation", rotation, "movement", movement)
        print("")

        # Finally update the visit state for this cell, if it's first time visit  # NOQA
        if not self.visitedGrid[x][y]:
            self.visitedGrid[x][y] = 1
            self.visits += 1

        if movement:
            # If the robot does need to move for the next step
            # We have to update the robot position manually here,
            # since we are not allowed to modify tester.py
            self.updatePosition(chosen_action)

        else:  # movement of 0 means robot only turn 90 degree clockwise
            self.turnClockWise()

        self.step += 1

        print("total number of cells visited", self.visits)
        print("")

        return rotation, movement
