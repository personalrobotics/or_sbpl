import numpy

class Control:
    def __init__(self, v, w, t, dist, poses):
        self.v = v
        self.w = w
        self.t = t
        self.dist = dist
        self.poses = poses
        
    def __str__(self):
        s = 'v: %0.3f, w: %0.3f, t:%0.3f, dist:%0.3f' % (self.v, self.w, self.t, self.dist)
        return s

class Coordinate:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
    
    def dist(self, c):
        return numpy.linalg.norm(numpy.array([c.x - self.x, c.y - self.y, c.theta - self.theta]))

    def __str__(self):
        s = '[ %0.3f, %0.3f, %0.3f ]' % (self.x, self.y, self.theta)
        return s

class Map:
    def __init__(self, cellsize, numangles):
        self.cellsize = cellsize
        self.numangles = numangles
        self.anglesize = 2.0*numpy.pi/self.numangles

    def GridCoordinateToWorldCoordinate(self, gc):
        x = gc.x*self.cellsize + 0.5*self.cellsize 
        y = gc.y*self.cellsize + 0.5*self.cellsize 
        theta = gc.theta * self.anglesize

        return Coordinate(x, y, theta)
        

    def WorldCoordinateToGridCoordinate(self, wc):
        x = int(wc.x / self.cellsize)
        y = int(wc.y / self.cellsize)
        
        t = wc.theta
        if t < 0.0:
            t += 2.0*numpy.pi
        if t > 2.0*numpy.pi:
            t -= 2.0*numpy.pi

        theta = int(t / self.anglesize)

        return Coordinate(x, y, theta)

class VehicleModel:
    def __init__(self, start, v, w):
        self.current = start
        self.v = v
        self.w = w

    def snap(self, theta):
        if theta < 0.0:
            theta += 2.0*numpy.pi
        if theta > 2.0*numpy.pi:
            theta -= 2.0*numpy.pi
        return theta
        
    def run(self, dt, maxtime):
        
        poses = []
        t = 0.0

        x = self.current.x
        y = self.current.y
        theta = self.current.theta

        poses.append(Coordinate(x,y,theta))

        while t < maxtime:
            theta = self.snap(theta + self.w*dt)
            x += self.v*numpy.cos(theta)*dt
            y += self.v*numpy.sin(theta)*dt
            poses.append(Coordinate(x,y,theta))
            t += dt
        return poses
