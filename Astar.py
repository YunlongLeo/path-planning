import math
import numpy as np
from OrderedList import OrderedList,Node
import matplotlib.pyplot as plt
class Astar:
    def __init__(self):
        path = OrderedList()

    def getcenterpoint(node):
        x = node[0].x
        y = node[0].y
        centrex = (node[0].x + node[0].width) / 2
        centrey = (node[0].y + node[0].height) / 2
        return centrex, centrey

    def getdistance(self,x1, y1, x2, y2):
        return math.sqrt((x1 - x2) * (x1-x2) + (y1-y2)*(y1 - y2))

    def Astarprocessing(self,initalnode,goalnode,freeNodes,ax,goalx,goaly,initalx,initaly):

        unvisitedlist = OrderedList()
        stop = False    #check meet goal
        foundpath = False
        visitedlist = OrderedList()

        unvisitedlist.add(initalnode[0])
        k = 1
        while(stop == False  and foundpath == False):
            currnode = unvisitedlist.pop().getData()
            cost = self.findactualcost(currnode)
            visitedlist.add(currnode)
            #check close to goal

            if(currnode.x <= goalnode[0].x <= currnode.x+currnode.width and currnode.y <= goalnode[0].y <= currnode.y+currnode.height):
                foundpath = True
            #find 8 direction's node
            if(stop == False):

                nearnodes = self.findAround(currnode,freeNodes)

                #change actual cost in nearnodes
                for node in nearnodes:
                    found = unvisitedlist.search(node)
                    foundv = visitedlist.search(node)
                    prex = node.x + node.width/2
                    prey = node.y + node.height/2
                    currx = currnode.x +currnode.width/2
                    curry =  currnode.x+currnode.height/2
                    node.gValue =  cost + self.getdistance(prex,prey,currx,curry)


                    if(found != None):
                        if(found.gValue > node.gValue):
                            found.gValue = node.gValue
                            found.father = currnode

                    else:
                        if(foundv == None):
                            node.father = currnode
                            unvisitedlist.add(node)
                        else:
                            if (foundv.gValue > node.gValue):
                                foundv.gValue = node.gValue
                                foundv.father = currnode


            if(unvisitedlist == None):
                stop = True
                print("there is no path")

        pathlength = 0
        if(foundpath == True):

            end = np.array((goalx, goaly))
            pathlength = pathlength + self.getdistance(goalx,goaly,currnode.x,currnode.y)
            pos = end

            while (currnode.father!= None):
                currx = currnode.x + currnode.width / 2
                curry = currnode.y + currnode.height / 2
                pathlength = pathlength + self.getdistance(currx, curry, currnode.father.x, currnode.father.y)
                point = np.array((currx,curry))
                pos = np.vstack((pos,point))
                currnode = currnode.father

            pathlength = pathlength + self.getdistance(currnode.x, currnode.y,initalx,initaly)
            inital = (initalx, initaly)
            pos=np.vstack((pos,inital))
            plt.plot(pos[:,0],pos[:,1],'k-')

        return pathlength

    def findactualcost(self,currnode):
        cost = 0
        fathernode = currnode.father
        while(fathernode != None):
            cost = cost + self.getdistance(currnode.x,currnode.y,fathernode.x,fathernode.y)
            currnode = currnode.father
            fathernode = fathernode.father
        return cost

    def findAround(self,currnode,freeNodes):
        aroundnotes = []
        rectangleX = currnode.x
        rectangleY = currnode.y
        rectangleHeight = currnode.height
        rectangleWidth = currnode.width
        for node in freeNodes:
            nodeX = node[0].x
            nodeY = node[0].y
            nodeHeight = node[0].height
            nodeWidth = node[0].width
            if (rectangleY + rectangleHeight == nodeY and not (nodeX + nodeWidth < rectangleX) and not (
                    nodeX > rectangleX + rectangleWidth)):
                aroundnotes.append(node[0])

            elif (nodeX == rectangleX + rectangleWidth and not (nodeY > rectangleY + rectangleHeight) and not (
                    nodeY + nodeHeight < rectangleY)):
                aroundnotes.append(node[0])

            elif (nodeY + nodeHeight == rectangleY and not (nodeX + nodeWidth < rectangleX) and not (
                    nodeX > rectangleX + rectangleWidth)):
                aroundnotes.append(node[0])
            elif (nodeX + nodeWidth == rectangleX and not (nodeY > rectangleY + rectangleHeight) and not (
                    nodeY + nodeHeight < rectangleY)):
                aroundnotes.append(node[0])

        return aroundnotes
