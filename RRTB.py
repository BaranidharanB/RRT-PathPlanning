import pygame
from rrt_main import RRTGraph
from rrt_main import RRTmap


def main():
    MapDim = (600,1000)
    startPos = (50,50)
    goalPos = (510,510)
    obsDim = 30
    obsNum = 50
    iteration = 0

    pygame.init()
    map = RRTmap(startPos,goalPos,MapDim,obsDim,obsNum)
    graph = RRTGraph(startPos,goalPos,MapDim,obsDim,obsNum)

    obstacles = graph.makeobs()

    map.drawMap(obstacles)

    while (True):
        x,y = graph.sample_env()
        n = graph.numberOfNodes()
        graph.addNode(n,x,y)
        graph.addEdge(n-1,n)
        x1, y1 = graph.x[n],graph.y[n]
        x2,y2 = graph.x[n-1],graph.y[n-1]
        if (graph.isFree()):
            pygame.draw.circle(map.map,map.Red,(graph.x[n],graph.y[n]),map.nodeRad, map.nodeThickness)
            if not graph.crossObstacle(x1,x2,y1,y2):
                pygame.draw.line(map.map,map.Blue,(x1,y1),(x2,y2),map.edgeThickness)

        pygame.display.update()



    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(10)



if __name__ == "__main__":
    main()