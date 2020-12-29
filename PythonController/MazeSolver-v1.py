# Solving the MicroMouse maze
# Thomas W. C. Carlson

# Requests library allows HTTP server requests and data acquisition
# python -m pip install requests
import requests as req
# python -m pip install numpy==1.19.3
import numpy as np
import random as rand
import time as t

# Initial Cell as defined from "top left" of the maze, where y is horizontal and x is vertical
INITIAL_CELL_X = 9
INITIAL_CELL_Y = 9
INITIAL_POS = np.array([(int(INITIAL_CELL_X),int(INITIAL_CELL_Y))])
# Initial Orientation as defined from top left, using West as 0, North as 1, East as 2, South as 3
INITIAL_ORIENT = 1
# Maze dimensions definition
MAZE_X_LENGTH = 10
MAZE_Y_LENGTH = 10
# Maze target cell as defined from "top left" of the maze
TARGET_CELL_X = 0
TARGET_CELL_Y = 0
TARGET_CELL = np.array([(int(TARGET_CELL_X),int(TARGET_CELL_Y))])
# Robot IP for HTTP Server
ROBOT_ADDRESS = "http://192.168.0.185/"


def printArray(ArrayName):
    XMAX, YMAX = ArrayName.shape
    for y in range(0, YMAX):
        for x in range(0, XMAX):
            print('{:<3}'.format(int(ArrayName[x][y])), end='')
        print()


def rotateCW(ROBOT_ADDRESS, ORIENT):
    # print("Orient before rotation:" + str(ORIENT))
    # Ping Robot HTTP Server to command a CW Rotation
    Response = req.get(ROBOT_ADDRESS + "CWRot")
    while Response.text != "ROTATED 90 CW":
        # Incorrect response, wait and try again
        t.sleep(2)
        Response = req.get(ROBOT_ADDRESS + "CWRot")
    ORIENT = (ORIENT + 1) % 4
    # print("Orient after rotation: " + str(ORIENT))
    return ORIENT


def rotateCCW(ROBOT_ADDRESS, ORIENT):
    # print("Orient before rotation:" + str(ORIENT))
    # Ping Robot HTTP Server to command a CW Rotation
    Response = req.get(ROBOT_ADDRESS + "CCWRot")
    while Response.text != "ROTATED 90 CCW":
        # Incorrect response, wait and try again
        t.sleep(2)
        Response = req.get(ROBOT_ADDRESS + "CCWRot")
    ORIENT = (ORIENT - 1) % 4
    # print("Orient after rotation: " + str(ORIENT))
    return ORIENT


def fwdPosChangeDict(ORIENT):
    switch = {
        0: np.array([(-1,0)]),
        1: np.array([(0,-1)]),
        2: np.array([(1,0)]),
        3: np.array([(0,1)]),
    }
    return switch.get(ORIENT, "Invalid ORIENT")


def moveFWD(ROBOT_ADDRESS, CurrentLocation, ORIENT):
    print("Position before movement: " + str(CurrentLocation))
    PosChange = fwdPosChangeDict(ORIENT)
    Response = req.get(ROBOT_ADDRESS + "FWDunk")
    while Response.text != "MOVED FWD 1 CELL":
        # Incorrect response, wait and try again
        t.sleep(2)
        Response = req.get(ROBOT_ADDRESS + "FWDunk")
    CurrentLocation += PosChange
    print("Position after movement: " + str(CurrentLocation))
    return CurrentLocation

def recenter(ROBOT_ADDRESS):
    Response = req.get(ROBOT_ADDRESS + "Recenter")
    while Response.text != "RECENTERED":
        # Incorrect response, wait and try again
        t.sleep(2)
        Response = req.get(ROBOT_ADDRESS + "Recenter")
    print("Recentered using 2 walls")

def readReshapeDict(ORIENT):
    switch = {
        0: 2,
        1: 3,
        2: 0,
        3: 1
    }
    return switch.get(ORIENT, "Invalid ORIENT")


def readAllWalls(ROBOT_ADDRESS, ORIENT):
    # Read the initial position walls
    WallString = readSensors(ROBOT_ADDRESS)
    # Rotate 90 to pick up the other wall, for a CW rotation this is the wall on the right
    ORIENT = rotateCW(ROBOT_ADDRESS, ORIENT)
    # Append last data to old data as it is the only new data
    WallString += readSensors(ROBOT_ADDRESS)[2]
    ORIENT = rotateCCW(ROBOT_ADDRESS, ORIENT)
    # Reshape for different orientations, assuming that 0 is West and the string reads (W,N,E,S) initially
    # Desire an output in terms of (N,E,S,W)
    # Ex. Read in walls known to be to the west and south with desired matrix (0011)
    # If initial orient = 0, 2 shift (ex: 1100 -> 0011)
    # If initial orient = 1, 3 shift (ex: 1001 -> 0011)
    # If initial orient = 2, 0 shift (ex: 0011 -> 0011)
    # If initial orient = 3, 1 shift (ex: 0110 -> 0011)
    # Successive removal and appending of first element
    # Use dict to identify how many times this needs to occur
    Shifts = readReshapeDict(ORIENT)
    for x in range(0, Shifts):
        WallString = WallString[3] + WallString[0] + WallString[1] + WallString[2]
    return WallString


def readSensors(ROBOT_ADDRESS):
    # Ping Robot HTTP Server to read sensor states
    Response = req.get(ROBOT_ADDRESS + "WallRead")
    # Save the response string which indicates wall presence
    WallString = Response.text
    return WallString


def translateWallsToEdges(CurrentLocation, Walls, EdgeDict):
    # Convert the wall indentification obtained from sensor reads in readAllWalls or readSensors non-edges
    # Recall that edges are stored as EdgeDict[(CurrentCell)][(TargetCell)] = 0 or 1
    # If there is a wall between CurrentCell and TargetCell, EdgeDict[(CurrentCell)][(TargetCell)] = 0
    # Remember that while the maze arrays travel right and down, north is up and south is down
    # So when moving south, we add (0,1), and when moving north add (0,-1) to keep array bounds straight
    # Left and right are intuitive
    print("Translating walls to edges.")
    WallWest = int(Walls[3])
    MoveWest = np.array([(-1,0)]) + CurrentLocation
    WallNorth = int(Walls[0])
    MoveNorth = np.array([(0,-1)]) + CurrentLocation
    WallEast = int(Walls[1])
    MoveEast = np.array([1,0]) + CurrentLocation
    WallSouth = int(Walls[2])
    MoveSouth = np.array([0,1]) + CurrentLocation
    # print(EdgeDict[(CurrentLocation[0][0], CurrentLocation[0][1])][(MoveEast[0][0],MoveEast[0][1])])
    if WallWest == 1 and MoveWest[0][0] >= 0 and MoveWest[0][1] >= 0 \
        and MoveWest[0][0] < MAZE_X_LENGTH and MoveWest[0][1] < MAZE_Y_LENGTH:
        # Wall present, remove the edge
        EdgeDict[(CurrentLocation[0][0], CurrentLocation[0][1])][(MoveWest[0][0], MoveWest[0][1])] = 0
        # Make sure the opposite entry exists too
        EdgeDict[(MoveWest[0][0], MoveWest[0][1])][(CurrentLocation[0][0], CurrentLocation[0][1])] = 0
        print("Wall to the west, severing edge.")
    elif WallWest == 0 and MoveWest[0][0] >= 0 and MoveWest[0][1] >= 0 \
        and MoveWest[0][0] < MAZE_X_LENGTH and MoveWest[0][1] < MAZE_Y_LENGTH:
        # Wall not present, add an edge
        EdgeDict[(CurrentLocation[0][0], CurrentLocation[0][1])][(MoveWest[0][0], MoveWest[0][1])] = 1
        # Make sure the opposite entry exists too
        EdgeDict[(MoveWest[0][0], MoveWest[0][1])][(CurrentLocation[0][0], CurrentLocation[0][1])] = 1
        print("No wall to the west, adding edge.")
    else:
         print("West wall not part of maze! No edge.")

    if WallNorth == 1 and MoveNorth[0][0] >= 0 and MoveNorth[0][1] >= 0 \
        and MoveNorth[0][0] < MAZE_X_LENGTH and MoveNorth[0][1] < MAZE_Y_LENGTH:
        # Wall present, remove the edge
        EdgeDict[(CurrentLocation[0][0], CurrentLocation[0][1])][(MoveNorth[0][0], MoveNorth[0][1])] = 0
        # Make sure the opposite entry exists too
        EdgeDict[(MoveNorth[0][0], MoveNorth[0][1])][(CurrentLocation[0][0], CurrentLocation[0][1])] = 0
        print("Wall to the north, severing edge.")
    elif WallNorth == 0 and MoveNorth[0][0] >= 0 and MoveNorth[0][1] >= 0 \
        and MoveNorth[0][0] < MAZE_X_LENGTH and MoveNorth[0][1] < MAZE_Y_LENGTH:
        # Wall not present, add an edge
        EdgeDict[(CurrentLocation[0][0], CurrentLocation[0][1])][(MoveNorth[0][0], MoveNorth[0][1])] = 1
        # Make sure the opposite entry exists too
        EdgeDict[(MoveNorth[0][0], MoveNorth[0][1])][(CurrentLocation[0][0], CurrentLocation[0][1])] = 1
        print("No wall to the north, adding edge.")
    else:
        print("North wall not part of maze! No edge.")

    if WallEast == 1 and MoveEast[0][0] >= 0 and MoveEast[0][1] >= 0 \
        and MoveEast[0][0] < MAZE_X_LENGTH and MoveEast[0][1] < MAZE_Y_LENGTH:
        # Wall present, remove the edge
        EdgeDict[(CurrentLocation[0][0], CurrentLocation[0][1])][(MoveEast[0][0], MoveEast[0][1])] = 0
        # Make sure the opposite entry exists too
        EdgeDict[(MoveEast[0][0], MoveEast[0][1])][(CurrentLocation[0][0], CurrentLocation[0][1])] = 0
        print("Wall to the east, severing edge.")
    elif WallEast == 0 and MoveEast[0][0] >= 0 and MoveEast[0][1] >= 0 \
        and MoveEast[0][0] < MAZE_X_LENGTH and MoveEast[0][1] < MAZE_Y_LENGTH:
        # Wall not present, add an edge
        EdgeDict[(CurrentLocation[0][0], CurrentLocation[0][1])][(MoveEast[0][0], MoveEast[0][1])] = 1
        # Make sure the opposite entry exists too
        EdgeDict[(MoveEast[0][0], MoveEast[0][1])][(CurrentLocation[0][0], CurrentLocation[0][1])] = 1
        print("No wall to the east, adding edge.")
    else:
        print("East wall not part of maze! No edge.")

    if WallSouth == 1 and MoveSouth[0][0] >= 0 and MoveSouth[0][1] >= 0 \
        and MoveSouth[0][0] < MAZE_X_LENGTH and MoveSouth[0][1] < MAZE_Y_LENGTH:
        # Wall present, remove the edge
        EdgeDict[(CurrentLocation[0][0], CurrentLocation[0][1])][(MoveSouth[0][0], MoveSouth[0][1])] = 0
        # Make sure the opposite entry exists too
        EdgeDict[(MoveSouth[0][0], MoveSouth[0][1])][(CurrentLocation[0][0], CurrentLocation[0][1])] = 0
        print("Wall to the south, severing edge.")
    elif WallSouth == 0 and MoveSouth[0][0] >= 0 and MoveSouth[0][1] >= 0 \
        and MoveSouth[0][0] < MAZE_X_LENGTH and MoveSouth[0][1] < MAZE_Y_LENGTH:
        # Wall not present, add an edge
        EdgeDict[(CurrentLocation[0][0], CurrentLocation[0][1])][(MoveSouth[0][0], MoveSouth[0][1])] = 1
        # Make sure the opposite entry exists too
        EdgeDict[(MoveSouth[0][0], MoveSouth[0][1])][(CurrentLocation[0][0], CurrentLocation[0][1])] = 1
        print("No wall to the south, adding edge.")
    else:
        print("South wall not part of maze! No edge.")

    return EdgeDict


def orientToDir(ORIENT):
    switch = {
        0: "West",
        1: "North",
        2: "East",
        3: "South",
    }
    return switch.get(ORIENT,"Invalid Dir")


# Check that robot is online and ready by accessing root directory of webserver
def robotInitCheck(ROBOT_ADDRESS, INITIAL_CELL_X, INITIAL_CELL_Y, MAZE_X_LENGTH, MAZE_Y_LENGTH):
    Response = req.get(ROBOT_ADDRESS + "Init")
    if Response.text == "INITIALIZED":
        print("MICROMOUSE READY. CONFIRM PLACEMENT: (" + str(INITIAL_CELL_X) + "," + str(INITIAL_CELL_Y) + "," + str(INITIAL_ORIENT) + ")")
        print("IN MAZE OF SIZE X: " + str(MAZE_X_LENGTH) + " Y: " + str(MAZE_Y_LENGTH))
        input("Press enter to confirm.")


# If ready, perform an initial cell read


# Initialize a blank flood fill map
# The blank map is a simple map of the maze using x,y coordinates whose initial values should be zero

# Define a function for displaying the flood map
# print("Flood-fill map:")
# printArray(FloodMap)


def initFloodFill(TARGET_CELL_X, TARGET_CELL_Y, MAZE_X_LENGTH, MAZE_Y_LENGTH, FloodSteps):
    FloodMap = np.zeros([int(MAZE_X_LENGTH),int(MAZE_Y_LENGTH)])
    # Floodfill from the destination cell outward
    # Necessary for kickstarting recursive floodfilling
    # Need to do an initial setup to get the first nodes of value 1
    FirstStep = FloodSteps + (int(TARGET_CELL_X),int(TARGET_CELL_Y))
    # For each element in FirstStep, check that it is not out of array bounds
    print((TARGET_CELL[0][0],TARGET_CELL[0][1]))
    for n in range(0, len(FirstStep)):
        x, y = FirstStep[n]
        if x >= 0 and y >= 0 and x < MAZE_X_LENGTH and y < MAZE_Y_LENGTH and EdgeDict[(x,y)][(TARGET_CELL[0][0],TARGET_CELL[0][1])] == 1:
            FloodMap[x][y] = 1
    return FloodMap

# Now floodfill the rest of the maze

def FloodFill(CurrentVal, FloodSteps, FloodMap, EdgeDict, TARGET_CELL,RecursionCount):
    # Find all cells of CurrentVal
    CurrentCells = np.argwhere(FloodMap == CurrentVal)
    # print("Found these cells with value = " + str(CurrentVal) + ": ") 
    # print(CurrentCells)
    # For every cell of CurrentVal, take orthogonal steps by adding the FloodSteps array
    for n in range(0, len(CurrentCells)):
        # print("Step to next cell starting from: " + str(CurrentCells[n]))
        StepCells = FloodSteps + CurrentCells[n]
        # print("Stepped to new cells:")
        # print(StepCells)
        # This creates 4 cells
        for i in range(0, len(StepCells)):
            # print("Checking cell: " + str(StepCells[i]) + " for existence in the maze.")
            x, y = StepCells[i]
            # If the cell is within the maze
            if x < MAZE_X_LENGTH and y < MAZE_Y_LENGTH and x >= 0 and y >= 0:
                # print("Cell exists in the maze: ")
                # print((x,y))
                # And it is of value 0 and not the target cell
                # print("Cell has a flood value of: " + str(FloodMap[x][y]))
                if FloodMap[x][y] == 0 and not np.array_equal([(x, y)],TARGET_CELL):
                    # print("Cell is of value 0 and isn't the target cell: ")
                    # print((x,y))
                    # print("Cell set to current value.")
                    # print("Cell (" + str(x) + ", " + str(y) + ") updated.")
                    # And there is an edge connecting the two cells
                    # print("Cell 1:")
                    # print((x,y))
                    # print("Cell 2:")
                    # print((CurrentCells[n][0],CurrentCells[n][1]))
                    # print("Edge relation:")
                    # print(EdgeDict[(x,y)][(CurrentCells[n][0],CurrentCells[n][1])])
                    if EdgeDict[(x,y)][(CurrentCells[n][0],CurrentCells[n][1])] == 1:
                        # And there is an edge connecting the two cells
                        FloodMap[x][y] = CurrentVal + 1

                    # Need the position of the cell we are stepping from ()
                    # Also need the position of the cell we are stepping to (x,y)
    
    # Check if there are any 0-valued non-target cells left in the flood map
    RemainingCells = np.argwhere(FloodMap == 0)
    # print("There are zero-valued cells at: ")
    # TARGET_CELL = [(TARGET_CELL_X, TARGET_CELL_Y)]
    # print(RemainingCells)
    # print(not np.array_equal(RemainingCells,np.array([TARGET_CELL])))
    # print(TARGET_CELL)
    # Check for 0-valued cells
    if len(RemainingCells != 0):
        # If there are 0-valued cells (there always should be), check if only the target square is one of them
        # print(not np.array_equal(RemainingCells,np.array([TARGET_CELL])))
        if (not np.array_equal(RemainingCells,TARGET_CELL)) and RecursionCount <= 200:
            # printArray(FloodMap)
            CurrentVal += 1
            RecursionCount += 1
            # Recursively call the function until there are no zeros left
            FloodMap = FloodFill(CurrentVal, FloodSteps, FloodMap, EdgeDict, TARGET_CELL, RecursionCount)
    return FloodMap


def findBestPath(CurrentLocation, FloodMap):
    # print(CurrentLocation)
    # Locate the lowest valued cell adjacent to the robot
    # Keep convention of top left, and x being vertical while y is horizontal
    CurrentFloodValue = FloodMap[CurrentLocation[0][0]][CurrentLocation[0][1]]
    # Identify orthogonally adjacent cells, make sure not to exit the maze
    MoveWest = np.array([(-1,0)]) + CurrentLocation
    MoveNorth = np.array([(0,-1)]) + CurrentLocation
    MoveEast = np.array([1,0]) + CurrentLocation
    MoveSouth = np.array([0,1]) + CurrentLocation
    # print(MoveWest)
    # print(MoveNorth)
    # print(MoveEast)
    # print(MoveSouth)
    # Check for actually accessible adjacent cells using maze boundaries
    # Then find the flood value of accessible cells
    if MoveWest[0][0] >= 0 and MoveWest[0][1] >= 0 and MoveWest[0][0] < MAZE_X_LENGTH and MoveWest[0][1] < MAZE_Y_LENGTH:
        # Check for actually accessible adjacent cells using EdgeDict
        if EdgeDict[(CurrentLocation[0][0],CurrentLocation[0][1])][(MoveWest[0][0],MoveWest[0][1])] == 1:
            # The cell is fully accessible
            # Find its value in the flood map
            WestFloodValue = FloodMap[MoveWest[0][0]][MoveWest[0][1]]
            print("Western cell accessible, with flood value: " + str(WestFloodValue))
        else:
            # Cell has no edge
            print("Western cell is blocked by a wall!")
            WestFloodValue = 999999;
    else:
        # Cell is out of bounds
        print("Western cell is out of bounds!")
        WestFloodValue = 999999;

    if MoveNorth[0][0] >= 0 and MoveNorth[0][1] >= 0 and MoveNorth[0][0] < MAZE_X_LENGTH and MoveNorth[0][1] < MAZE_Y_LENGTH:
        # Check for actually accessible adjacent cells using EdgeDict
        if EdgeDict[(CurrentLocation[0][0],CurrentLocation[0][1])][(MoveNorth[0][0],MoveNorth[0][1])] == 1:
            # The cell is fully accessible
            # Find its value in the flood map
            NorthFloodValue = FloodMap[MoveNorth[0][0]][MoveNorth[0][1]]
            print("Northern cell accessible, with flood value: " + str(NorthFloodValue))
        else:
            # Cell has no edge
            print("Northern cell is blocked by a wall!")
            NorthFloodValue = 999999;
    else:
        # Cell is out of bounds
        print("Northern cell is out of bounds!")
        NorthFloodValue = 999999;

    if MoveEast[0][0] >= 0 and MoveEast[0][1] >= 0 and MoveEast[0][0] < MAZE_X_LENGTH and MoveEast[0][1] < MAZE_Y_LENGTH:
        # Check for actually accessible adjacent cells using EdgeDict
        if EdgeDict[(CurrentLocation[0][0],CurrentLocation[0][1])][(MoveEast[0][0],MoveEast[0][1])] == 1:
            # The cell is fully accessible
            # Find its value in the flood map
            EastFloodValue = FloodMap[MoveEast[0][0]][MoveEast[0][1]]
            print("Eastern cell accessible, with flood value: " + str(EastFloodValue))
        else:
            # Cell has no edge
            print("Eastern cell is blocked by a wall!")
            EastFloodValue = 999999
    else:
        # Cell is out of bounds
        print("Eastern cell is out of bounds!")
        EastFloodValue = 999999

    if MoveSouth[0][0] >= 0 and MoveSouth[0][1] >= 0 and MoveSouth[0][0] < MAZE_X_LENGTH and MoveSouth[0][1] < MAZE_Y_LENGTH:
        # Check for actually accessible adjacent cells using EdgeDict
        if EdgeDict[(CurrentLocation[0][0],CurrentLocation[0][1])][(MoveSouth[0][0],MoveSouth[0][1])] == 1:
            # The cell is fully accessible
            # Find its value in the flood map
            SouthFloodValue = FloodMap[MoveSouth[0][0]][MoveSouth[0][1]]
            print("Southern cell accessible, with flood value: " + str(SouthFloodValue))
        else:
            # Cell has no edge
            print("Southern cell is blocked by a wall!")
            SouthFloodValue = 999999
    else:
        # Cell is out of bounds
        print("Southern cell is out of bounds!")
        SouthFloodValue = 999999

    # Collect all the flood values for evaluation
    FloodValues = np.array([WestFloodValue,NorthFloodValue,EastFloodValue,SouthFloodValue])
    print("Found adjacent cell flood values:")
    print(FloodValues)
    # Find the lowest value in the list
    BestValue = min(FloodValues)
    print("Found a lowest value:")
    print(BestValue)
    # Find indices of instances of the lowest value in the flood value list
    # There can be multiple results
    BestDirs = np.argwhere(FloodValues==BestValue)
    print("Lowest value was found at:")
    print(BestDirs)
    # Number of results
    BestDirCount = len(BestDirs)
    print("There were " + str(BestDirCount) + " 'best' directions found.")
    # Randomly select among best directions for now
    DirSelect = rand.randint(0,BestDirCount-1)
    # print("Randomly selected dir " + str(DirSelect))
    # Use the randomly selected index of best directions to find the direction
    BestDir = BestDirs[DirSelect]
    # print("Which had value " + str(BestDir))
    # Similar to ORIENT, 0 = west, 1 = north, 2 = east, 3 = south
    DirName = orientToDir(BestDir[0])
    print("Decided to move " + DirName + ".")
    return BestDir


# Move the robot to that cell
def reorientToTargetCell(CURR_ORIENT, TARG_ORIENT):
    # Find the direction and number of times to rotate to reach desired orientation
    # Ex, for a CURR_ORIENT = 3 and a TARG_ORIENT = 2, (3-2) = 1 and (2-3) = -1
    # 1 mod 4 = 1, while -1 mod 4 = 3
    # So the shortest path is to rotate once with decreasing ORIENT (CCW) from 3 to 2 
    # Ex2, for a CURR_ORIENT = 3 and a TARG_ORIENT = 0, (3-0) = 3 and (0-3) = -3
    # 3 mod 4 = 3, while -3 mod 4 = 1
    # So the shortest path is to rotate once with increasing ORIENT (CW) from 3 to 0
    # Minimize the mod of the distance between CURR_ and TARG_
    # The distance is the result of the modulus operation
    # Choosing the minimum gives the number of rotations
    Moves = min(((CURR_ORIENT - TARG_ORIENT)%4), ((TARG_ORIENT - CURR_ORIENT)%4))
    # Choosing which modulus gives the minimum rotations gives the direction
    if (CURR_ORIENT - TARG_ORIENT)%4 <= (TARG_ORIENT - CURR_ORIENT)%4:
        # Rotate CCW
        print("Reorienting CCW " + str(Moves) + " times")
        # Remember that range is (inclusive, exclusive)
        for i in range(0,Moves[0]):
            CURR_ORIENT = rotateCCW(ROBOT_ADDRESS, CURR_ORIENT)
    elif (CURR_ORIENT - TARG_ORIENT)%4 > (TARG_ORIENT - CURR_ORIENT)%4:
        print("Reorienting CW " + str(Moves) + " times")
        # Rotate CW
        for i in range(0,Moves[0]):
            CURR_ORIENT = rotateCW(ROBOT_ADDRESS, CURR_ORIENT)
    return CURR_ORIENT
    

# Read the cell condition
def readThreeWalls(ROBOT_ADDRESS, ORIENT):
    # Read the L,FWD,R sensors
    WallString = readSensors(ROBOT_ADDRESS)
    # Backward path is always open, so scanning all sensors gives complete information on the current cell
    WallString += str(0)
    Shifts = readReshapeDict(ORIENT)
    for x in range(0, Shifts):
        WallString = WallString[3] + WallString[0] + WallString[1] + WallString[2]
    return WallString

# Edge dictionary, only built once
def buildEdgeDict(MAZE_Y_LENGTH, MAZE_X_LENGTH):
    EdgeDict = {}
    NodeSteps = np.array([(0,1),(1,0),(0,-1),(-1,0)])
    for y in range(0, MAZE_Y_LENGTH):
        for x in range(0, MAZE_X_LENGTH):
            # Using for loops to select the "base" node
            EdgeDict[(x,y)] = {}
            # Stepping along a theoretical orthogonal edge to new nodes
            # Because this is the initialization of the graph, assume no walls so all adjacent cells are connected
            OrthNodes = NodeSteps + (x,y)
            # For each new step node, check that it falls within the graph boundaries
            for n in range(0, len(OrthNodes)):
                # Save the coordinates of the new step node
                i, j = OrthNodes[n]
                if i >= 0 and j >= 0 and i < MAZE_X_LENGTH and j < MAZE_Y_LENGTH:
                    # Check that it lies within the boundaries of the maze
                    print("Edge generated for" + str((x,y)) + " and " + str((i,j)))
                    EdgeDict[(x,y)][(i,j)] = 1
    return EdgeDict


### END FUNCTION DEFINITIONS ###

### START EXPLORING LOOP ###

# Initializations
# Check for robot HTTP server activity
robotInitCheck(ROBOT_ADDRESS, INITIAL_CELL_X, INITIAL_CELL_Y, MAZE_X_LENGTH, MAZE_Y_LENGTH)
# Build the edge dictionary
EdgeDict = buildEdgeDict(MAZE_Y_LENGTH, MAZE_X_LENGTH)
# Build a blank flood map
# Initialize the first values
CurrentVal = 1
RecursionCount = 0
FloodSteps = np.array([(0,1),(1,0),(0,-1),(-1,0)])
FloodMap = initFloodFill(TARGET_CELL_X, TARGET_CELL_Y, MAZE_X_LENGTH, MAZE_Y_LENGTH, FloodSteps)
# Flood the rest recursively
FloodMap = FloodFill(CurrentVal, FloodSteps, FloodMap, EdgeDict, TARGET_CELL, RecursionCount)
printArray(FloodMap)
# Define CurrentPosition as Initial position
CurrentLocation = INITIAL_POS
# Read the starting cell's four walls
Walls = readAllWalls(ROBOT_ADDRESS, INITIAL_ORIENT)
# Modify the edge dictionary based on initial read
EdgeDict = translateWallsToEdges(CurrentLocation, Walls, EdgeDict)
# Calculate the best first direction to take
BestDir = findBestPath(CurrentLocation, FloodMap)
# Face the best first direction
ORIENT = reorientToTargetCell(INITIAL_ORIENT,BestDir)
# Move to the best first cell
CurrentLocation = moveFWD(ROBOT_ADDRESS, CurrentLocation, ORIENT)
# Now jump into the while loop
Turns = 2
while not np.array_equal(CurrentLocation,TARGET_CELL):
    print("BEGINNING STEP #" + str(Turns))
    print("==================")
    # Solution has not been achieved, so continue to iterate along the flood map, updating it after each wall scan
    # Read 3 sensors
    Walls = readThreeWalls(ROBOT_ADDRESS,ORIENT)
    # Modify the edge dictionary based on initial read
    EdgeDict = translateWallsToEdges(CurrentLocation, Walls, EdgeDict)
    # Update the flood
    CurrentVal = 1
    RecursionCount = 0
    FloodSteps = np.array([(0,1),(1,0),(0,-1),(-1,0)])
    FloodMap = initFloodFill(TARGET_CELL_X, TARGET_CELL_Y, MAZE_X_LENGTH, MAZE_Y_LENGTH, FloodSteps)
    # Flood the rest recursively
    FloodMap = FloodFill(CurrentVal, FloodSteps, FloodMap, EdgeDict, TARGET_CELL, RecursionCount)
    printArray(FloodMap)
    # Calculate the best first direction to take
    BestDir = findBestPath(CurrentLocation, FloodMap)
    # Face the best first direction
    ORIENT = reorientToTargetCell(ORIENT,BestDir)
    # Move to the best first cell
    CurrentLocation = moveFWD(ROBOT_ADDRESS, CurrentLocation, ORIENT)
    # Recenter
    recenter(ROBOT_ADDRESS)
    # Increment turns counter
    Turns += 1
print("Maze solved in " + str(Turns) + " steps :)")
# Return to the initial square
# No need to create a new edgedict
TARGET_CELL = np.array([(int(INITIAL_CELL_X),int(INITIAL_CELL_Y))])
print(TARGET_CELL)
TARGET_CELL_X = TARGET_CELL[0][0]
print(TARGET_CELL_X)
TARGET_CELL_Y = TARGET_CELL[0][1]
print(TARGET_CELL_Y)
Turns = 1
while not np.array_equal(CurrentLocation, TARGET_CELL):
    print("BEGINNING RETURN STEP #" + str(Turns))
    print("=========================")
    Walls = readThreeWalls(ROBOT_ADDRESS,ORIENT)
    print(Walls)
    # Modify the edge dictionary based on initial read
    EdgeDict = translateWallsToEdges(CurrentLocation, Walls, EdgeDict)
    # Update the flood
    CurrentVal = 1
    RecursionCount = 0
    FloodSteps = np.array([(0,1),(1,0),(0,-1),(-1,0)])
    FloodMap = initFloodFill(TARGET_CELL_X, TARGET_CELL_Y, MAZE_X_LENGTH, MAZE_Y_LENGTH, FloodSteps)
    # Flood the rest recursively
    FloodMap = FloodFill(CurrentVal, FloodSteps, FloodMap, EdgeDict, TARGET_CELL, RecursionCount)
    printArray(FloodMap)
    # Calculate the best first direction to take
    BestDir = findBestPath(CurrentLocation, FloodMap)
    # Face the best first direction
    ORIENT = reorientToTargetCell(ORIENT,BestDir)
    # Move to the best first cell
    CurrentLocation = moveFWD(ROBOT_ADDRESS, CurrentLocation, ORIENT)
    # Recenter
    recenter(ROBOT_ADDRESS)
    # Increment turns counter
    Turns += 1

# while NOT AT GOAL
# Build the blank orthogonally connected flood map
# Blank flood map, rebuilt after each step
# CurrentVal = 1
# FloodSteps = np.array([(0,1),(1,0),(0,-1),(-1,0)])
# FloodMap = initFloodFill(TARGET_CELL_X, TARGET_CELL_Y, MAZE_X_LENGTH, MAZE_Y_LENGTH, FloodSteps)

# # Perform a flood fill on the blank map


# # Perform floodfill
# CurrentVal = 1

# FloodSteps = np.array([(0,1),(1,0),(0,-1),(-1,0)])

# FloodMap = initFloodFill(TARGET_CELL_X, TARGET_CELL_Y, MAZE_X_LENGTH, MAZE_Y_LENGTH, FloodSteps)

# FloodMap = FloodFill(CurrentVal, FloodSteps, FloodMap, EdgeDict, TARGET_CELL)

# printArray(FloodMap)

# CurrentLocation = INITIAL_POS

# Walls = readAllWalls(ROBOT_ADDRESS, INITIAL_ORIENT)

# EdgeDict = translateWallsToEdges(CurrentLocation, Walls, EdgeDict)


# BestDir = findBestPath(CurrentLocation, FloodMap)

# # print(INITIAL_ORIENT)
# # print(BestDir)
# ORIENT = reorientToTargetCell(INITIAL_ORIENT,BestDir)
# # Advance to the next cell
# CurrentLocation = moveFWD(ROBOT_ADDRESS, CurrentLocation, ORIENT)

# Walls = readThreeWalls(ROBOT_ADDRESS, ORIENT)

# EdgeDict = translateWallsToEdges(CurrentLocation, Walls, EdgeDict)

# FloodMap = initFloodFill(TARGET_CELL_X, TARGET_CELL_Y, MAZE_X_LENGTH, MAZE_Y_LENGTH, FloodSteps)

# CurrentVal = 1
# FloodMap = FloodFill(CurrentVal, FloodSteps, FloodMap, EdgeDict, TARGET_CELL)
# printArray(FloodMap)

# Repeat the flood fill for the whole maze
# This loops until the robot finds a path to the destination
# Explore some additional points? Identify the "least" explored squares?

# Return to the original square using the same floodfill direction calculation with a new destination square

# Solve the graph using A* heuristic search for the shortest path

# Command the robot to execute the path, updating the location of the robot constantly


