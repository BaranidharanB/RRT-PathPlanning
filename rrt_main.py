import random
import math
import pygame

class RRTmap():
    def __init__ (self, startPos, goalPos,MapDim, obsDim,obsNum):
        self.startPos = startPos 
        self.goalPos = goalPos
        self.MapDim = MapDim
        self.MapH, self.MapW = self.MapDim


        # Windows Settings
        self.MapwindowName = "RRT Path Planning"
        pygame.display.set_caption(self.MapwindowName)
        # Setting up the dimention of the window and map
        self.map = pygame.display.set_mode((self.MapW,self.MapH))
        self.map.fill((255,255,255))
        self.nodeRad = 2
        self.nodeThickness = 0
        self.edgeThickness = 1


        # Setting up the obstacle and the color
        self.obstacles = []
        self.obsDim = obsDim
        self.obsNum = obsNum
        
        # Setting up the colors
        self.grey = (70,70,70)
        self.Blue = (0,0,255)
        self.Green = (0,255,0)
        self.Red = (255,0,0)
        self.White = (255,255,255)

        


    def drawMap(self,obstacles):
        pygame.draw.circle(self.map,self.Green,self.startPos,self.nodeRad+5.0)
        pygame.draw.circle(self.map,self.Green,self.startPos,self.nodeRad+20.1)
        self.drawObs(obstacles)
    def drawPath (self):
        pass

    def drawObs(self,obstacles):
        obstaclesList = obstacles.copy()
        while (len(obstaclesList)>0):
            obstacle = obstaclesList.pop(0)
            pygame.draw.rect(self.map,self.grey,obstacle)
        


class RRTGraph:


    def __init__(self, startPos, goalPos,MapDim, obsDim,obsNum):
        (x,y) = startPos
        self.startPos = startPos  
        self.goalPos = goalPos
        self.goalFlag = False
        self.MapDim = MapDim
        self.MapH, self.MapW = self.MapDim

        # Initializing the Tree with node and parent
        self.x = []
        self.y = []
        self.parent = []
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)

        # Obstacles
        self.obstacle = []
        self.obsDim = obsDim
        self.obsNum = obsNum

        # Goal Path
        self.goalstate = None
        self.path = []


    def makeRandomRectangle(self):
        # Setting up random obstacles in the map 
        upperCornerX = int(random.uniform(0,self.MapW-self.obsDim))
        upperCornerY = int(random.uniform(0,self.MapH-self.obsDim))
        
        # This will return the co-ordinates of the obstacles
        return (upperCornerX,upperCornerY)
    def makeobs(self):
        obs = []

        for i in range (0, self.obsNum):
            rectang = None
            # Getting the obstacles temprorily before getting stored

            # This startGoal is a Flag which indicates whether the start and end goal is inside the map    
            startGoalCol = True
            while startGoalCol:
                upper = self.makeRandomRectangle()
                rectang = pygame.Rect(upper,(self.obsDim,self.obsDim))
                if rectang.collidepoint(self.startPos) or rectang.collidepoint(self.goalPos):
                    startGoalCol = True
                else:
                    startGoalCol = False
            obs.append(rectang)
        self.obstacles = obs.copy()
        return obs


    def addNode (self,n,x,y): # n-identification number, x&y co-ordinates
        self.x.insert(n,x)
        self.y.append(y)

    def removeNode(self,n): # This will pop out the n 
        self.x.pop(n)
        self.y.pop(n)


    def addEdge(self,parent,child):
        # Inserting the child of the parent in the parent list
        self.parent.insert(child,parent) # child = index, parent = element

    def removeEdge(self, n):
        # This will pop out the edge child from the parent list
        self.parent.pop(n)
    
    def numberOfNodes(self):
        return len(self.x)

    def distance(self,n1,n2):
        # Calculating distance between the nodes
        (x1,y1) = (self.x[n1],self.y[n1])
        (x2,y2) = (self.x[n2],self.y[n2])
        # Claculating the eculidian distance between the nodes
        px = (float(x1)-float(x2))**2
        py = (float(y1)-float(y2))**2
        return (px+py)**(0.5)

    def sample_env (self):
        x = int(random.uniform(0,self.MapW))
        y = int(random.uniform(0,self.MapH))
        return x,y



    def Nearest(self,n):
        minDis = self.distance(0,n)
        nNear = 0
        for i in range (0,n):
            # Finds the min distance between the nodes
            if self.distance(i,n) <minDis:
                minDis = self.distance(i,n)
                nNear = i
        return nNear

    def isFree(self):
        # need to check the passing of node area is free
        n = self.numberOfNodes()-1
        (x,y) = (self.x[n],self.y[n])
        obs = self.obstacles.copy()
        while len (obs) > 0:
            rectang = obs.pop (0)
            # If the node passing collide with the exisisting obstacle return false
            if rectang.collidepoint(x,y):
                self.removeNode(n)
                return False
        return True

    def crossObstacle(self,x1,x2,y1,y2):
        # Passing every co-ordinates and checking whether it is colliding or not
        obs = self.obstacles.copy()
        while  (len(obs)>0):
            rectang = obs.pop(0)
            # This loop creates a set of checkpoints through the node passing lines and check for collision
            for i in range (0,101):
                u = i/100
                x = x1*u + x2 *(1-u)
                y = y1*u + y2 *(1-u)
                if rectang.collidepoint(x,y):
                    return True
        return False

    def connect(self,n1,n2):
        # This is connecting between the nodes 
        (x1,y1) = (self.x[n1],self.y[n1])
        (x2,y2) = (self.x[n2],self.y[n2])
        if self.crossObstacle(x1,x2,y1,y2):
            self.removeNode(n2)
            return False
        else:
            self.addEdge(n1,n2)
            return True

    def step (self,nNear, nRand, dMax = 35):
        d = self.distance(nNear,nRand)
        if d> dMax:
            u = dMax/d
            (xNear,yNear) = (self.x[nNear],self.y[nNear])
            (xRand,yRand) = (self.x[nRand],self.y[nRand])
            # A--B--C line example.
            # The angle between the line is calculated by providing py and px to arctan2
            (px,py) = (xRand - xNear, yRand - yNear)
            theta = math.atan2(py,px)
            (x,y) = (int(xNear + dMax * math.cos(theta)),
                     int(yNear + dMax * math.sin(theta)))
            self.removeNode(nRand) # This line will remove the node as soon as find the nearest one
            if abs(x-self.goalPos[0])<35 and abs (y-self.goalPos[1])<35:
                self.addNode(nRand,self.goalPos[0],self.goalPos[1])
                self.goalstate = nRand
                self.goalFlag = True 
            else:
                self.addNode(nRand,x,y)
    def pathToGoal(self):
        pass

    def getPathCoords(self):
        pass

    def bias (self, nGoal):
        n = self.numberOfNodes()
        self.addNode(n,nGoal[0],nGoal[1])
        nNear = self.Nearest(n)
        self.step(nNear,n)
        self.connect(nNear,n)
        return self.x, self.y,self.parent


    def expand (self):
        n = self.numberOfNodes()
        x,y = self.sample_env()
        self.addNode(n,x,y)
        if self.isFree():
            xNearest = self.Nearest(n)
            self.step(xNearest,n)
            self.connect(xNearest,n)
        return self.x, self.y,self.parent




    def Cost(self):
        pass






