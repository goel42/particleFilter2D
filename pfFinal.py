#@title Default title text
import numpy as np
import random
import matplotlib.pyplot as plt
# from google.colab import files

class Pose:
  def __init__(self,x=0,y=0,theta=0):
    self.x=x
    self.y=y
    self.theta = theta

    
class Noise:
  def __init__(self,linearNoise, angularNoise, senseNoise):
    self.linearNoise = linearNoise
    self.angularNoise = angularNoise
    self.senseNoise = senseNoise


class Utility:
  @staticmethod
  def distanceMetric(a,b):
    return np.linalg.norm(np.array([a.x,a.y]) - np.array([b.x,b.y]) )
  
  
  @staticmethod
  def gaussian(mu, sigma, x):
#     POSSIBILITY OF DIV BY ZERO IF SIGMA = 0.0
    """calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
    :param mu:    distance to the landmark
    :param sigma: standard deviation
    :param x:     distance to the landmark measured by the robot
    :return gaussian value
    """
    # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
    return np.exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / np.sqrt(2.0 * np.pi * (sigma ** 2))
  
  
  @staticmethod
  def resampling(Particles):
    newParticles = []
    numParticles= len(Particles)
    index = int(random.random() * numParticles)
    beta = 0.0
    maxWeight = max(p.weight for p in Particles)
    
    for i in range(numParticles):
      beta += random.random() * 2.0 * maxWeight
      while (Particles[index].weight<beta):
        beta -= Particles[index].weight
        index = (index+1)%numParticles
      newParticles.append(Particles[index].duplicate())
    return newParticles
  drawIndex=0;
  @staticmethod
  def draw(Mule, step, worldSize, Poles, p, pr =[] , evalScore=0, title=""):

    plt.figure("Mule Position Prediction using Particle Filter", figsize=(5., 5.))
    plt.title('Localising Mule using Particle Filter')
 
    # draw coordinate grid for plotting
    grid = [0, worldSize, 0, worldSize]
    plt.axis(grid)
    plt.grid(b=True, which='major', color='0.75', linestyle='--')
    plt.xticks([i for i in range(0, int(worldSize), 5)])
    plt.yticks([i for i in range(0, int(worldSize), 5)])
 
    # draw particles
    for ind in range(len(p)):
 
        # particle
#         print("Pind: ", p[ind].weight)
        circle = plt.Circle((p[ind].Pose.x, p[ind].Pose.y), 1 , facecolor='#ffd700', edgecolor='#994c00', alpha=0.5)
        plt.gca().add_patch(circle)
 
        # particle's orientation
        arrow = plt.Arrow(p[ind].Pose.x, p[ind].Pose.y, 2*np.cos(p[ind].Pose.theta), 2*np.sin(p[ind].Pose.theta), alpha=1., facecolor='#994c00', edgecolor='#994c00')
        plt.gca().add_patch(arrow)
 
    # draw resampled particles
    for ind in range(len(pr)):
 
        # particle
        circle = plt.Circle((pr[ind].Pose.x, pr[ind].Pose.y), 1.0, facecolor='#66ff66', edgecolor='#009900', alpha=0.5)
        plt.gca().add_patch(circle)
 
        # particle's orientation
        arrow = plt.Arrow(pr[ind].Pose.x, pr[ind].Pose.y, 2*np.cos(pr[ind].Pose.theta), 2*np.sin(pr[ind].Pose.theta), alpha=1., facecolor='#006600', edgecolor='#006600')
        plt.gca().add_patch(arrow)
 
    # fixed landmarks of known locations
    for pole in Poles:
        square = plt.Rectangle((pole.Pose.x, pole.Pose.y), 2,2., facecolor='#cc0000', edgecolor='#330000')
        plt.gca().add_patch(square)
 
    # robot's location
    triangle = plt.Circle((Mule.Pose.x, Mule.Pose.y), 1, facecolor='#6666ff', edgecolor='#0000cc')
    plt.gca().add_patch(triangle)
 
    # robot's orientation
    arrow = plt.Arrow(Mule.Pose.x, Mule.Pose.y, 2*np.cos(Mule.Pose.theta), 2*np.sin(Mule.Pose.theta), alpha=0.5, facecolor='#000000', edgecolor='#000000')
    plt.gca().add_patch(arrow)
    evalScore = "N/A" if evalScore==0 else str(evalScore)
    prlen = "N/A" if len(pr)==0 else str(len(pr))
    plt.text(30,20, "Step: "+ str(step)+"\n"+title+"\n")
    plt.text(30,12,"WorldSize: " + str(worldSize)+ " X " + str(worldSize)
             +"\nNum Particles: " + str(len(p)) +"\nNum Resampled Particles: " + 
             str(prlen)+"\nError Score: "+str(evalScore)+"\n(Mean Eucledian Distance of\nSampled Particles from Mule)")
    plt.text(30,5, "Noise Values\nLinear: "+ str(Mule.Noise.linearNoise)+ ", Angular: "
            +str(Mule.Noise.angularNoise)+"\nSensor: "+str(Mule.Noise.senseNoise)
            )
    plt.text(30,1,"Mule: Blue, Landmarks: Red\nParticles: Yellow, Resampled: Green")
    fileName = "figure_" + str(Utility.drawIndex) + ".png"
    Utility.drawIndex+=1
    plt.savefig(fileName)
    plt.show()
#     files.download(fileName)
    plt.close()

    
  @staticmethod
  def eval(Mule, Particles, worldSize):
    sum = 0.0
    for i in range(len(Particles)):
        # the second part is because of world's cyclicity
        dx = (Particles[i].Pose.x - Mule.Pose.x + (worldSize/2.0)) % \
             worldSize - (worldSize/2.0)
        dy = (Particles[i].Pose.y - Mule.Pose.y + (worldSize/2.0)) % \
             worldSize - (worldSize/2.0)
        err = np.sqrt(dx**2 + dy**2)
        sum += err
    return sum / float(len(Particles))
    
class Pole:
  def __init__(self, x, y):
#     Pose for pole does not have theta, kaam chalau.
    self.Pose = Pose(x,y,0)



class Vehicle:
  blah=0
  #ToDo make the instance variables private
  def __init__(self, x, y , theta, stepSize, linearNoise, angularNoise, senseNoise):
    self.id=Vehicle.blah+1
    Vehicle.blah+=1
    self.Pose = Pose(x,y,theta)
    self.Noise = Noise(linearNoise,angularNoise, senseNoise)
    self.SensorData = []
    self.stepSize = stepSize
#     print("poop")
  
  
  def getPose():
#   TODO
    pass
  
  
  def setPose(self, newPose):
#   TODO
    pass
  
  
  def setNoise(self,):
#   TODO
    pass

  def getSensorData(self,):
#   TODO
    pass
  
  def setSensorData(self,data):
    self.sensorData= data


  def move(self,TurnSequence=[]):
    
    turnAngles = {"L": np.pi/2, "R": -1*np.pi/2, "S":0}
    for turn in TurnSequence:
      turnAngle=turnAngles[turn]
      #       print("ID: ", self.id)
      #       print("Pose Theta initial", self.Pose.theta)
      #       print("Pose Pose initial", self.Pose.x, self.Pose.y)
      self.Pose.theta+=float(turnAngle)
      rna = random.gauss(0.0, self.Noise.angularNoise)
#       print("Random Angular Noise", rna)
#       print("RNA", rna)
      self.Pose.theta+= rna
      self.Pose.theta %= 2*np.pi
#       print("Pose Theta final", self.Pose.theta)

      moveDistance = float(self.stepSize) 
      rnd= random.gauss(0.0, self.Noise.linearNoise)
#       print("Random Distance Noise", rnd)
      moveDistance+=rnd
      self.Pose.x += np.cos(self.Pose.theta)*moveDistance
      self.Pose.y += np.sin(self.Pose.theta)*moveDistance    
      self.Pose.x%=worldSize
      self.Pose.y%=worldSize
#       print("Pose Pose final", self.Pose.x, self.Pose.y)

  
#   def move(self,turn):
    
#     turnAngles = {"L": np.pi/2, "R": -1*np.pi/2, "S":0}
#     turnAngle=turnAngles[turn]
#     print("Pose Theta initial", self.Pose.theta)
#     self.Pose.theta+=float(turnAngle)
#     rna = random.gauss(0.0, self.Noise.angularNoise)
# #       print("Random Angular Noise", rna)
#     print("RNA", rna)
#     self.Pose.theta+= rna
#     self.Pose.theta %= 2*np.pi
#     print("Pose Theta final", self.Pose.theta)

#     moveDistance = float(self.stepSize) 
#     rnd= random.gauss(0.0, self.Noise.linearNoise)
# #       print("Random Distance Noise", rnd)
#     moveDistance+=rnd
#     self.Pose.x += np.cos(self.Pose.theta)*moveDistance
#     self.Pose.y += np.sin(self.Pose.theta)*moveDistance    
#     self.Pose.x%=worldSize
#     self.Pose.y%=worldSize
  
  def sense(self, poles=[]):
    data = []
    for pole in poles:
      rns = random.gauss(0.0, self.Noise.senseNoise)
#       print ("Random Sense Noise", rns)
      distance = Utility.distanceMetric(self.Pose, pole.Pose)
      distance += rns
      data.append(distance)
    self.setSensorData(data)
    return data
  
class Particle(Vehicle):
  def __init__(self, x, y , theta, stepSize, linearNoise, 
               angularNoise, senseNoise):
    Vehicle.__init__(self, x, y, theta, stepSize, linearNoise, 
               angularNoise, senseNoise)
    self.weight =0
  
  def duplicate(self):
    p= Particle(self.Pose.x, self.Pose.y, self.Pose.theta, self.stepSize, 
                self.Noise.linearNoise, self.Noise.angularNoise, self.Noise.senseNoise)
    p.weight=self.weight
    return p
  
  def setWeight(self,w):
    self.weight = w
  
  def getWeight(self):
    return self.weight
  
  def calculateWeight(self, muleSensorData):
    prob=1.0
    for i in range(len(self.sensorData)):
      prob *= Utility.gaussian(self.sensorData[i], self.Noise.senseNoise, muleSensorData[i])
    return prob

  
if __name__ == "__main__":
#   blah =0
  stepSize =5
  numParticles = 800
  worldSize = 25
#   turnSequence = ["S", "R", "S","S", "L", "S", "L", "S", "L", "S", "S"]
  turnSequence = ["S","S","S","R","S","S","R","S","S","R","S","S","R","S","S","R","S","S","R","S"]
#   numPoles = 5
  
#   PoleCordinates = [[25,25], [75,25], 
#                     [50,50], [25,75],
# #                     [75,75]]
  
# #   PoleCordinates = [[10.0, 10.0], [10.0, 40.0], [10.0, 25.0],
# #              [25.0, 10.0], [25.0, 40.0], [40.0, 40.0],
# #              [40.0, 10.0], [40.0, 25.0]]
# #   PoleCordinates = [ [10.0, 40.0], [10.0, 25.0],
# #              [25.0, 10.0], [25.0, 40.0], [40.0, 40.0],
# #               [40.0, 25.0]]
#   PoleCordinates = [ [random.randint(10,40), 40.0], [10.0, 25.0],
#              [25.0, 10.0], [25.0, 40.0], [40.0, 40.0],
#               [40.0, 25.0]]
#   PoleCordinates = [[random.randint(10,40),random.randint(10,40)], [random.randint(10,40),random.randint(10,40)], [random.randint(10,40),random.randint(10,40)],
#                    [random.randint(10,40),random.randint(10,40)]]
  PoleCordinates = [[5,23.5],[15,22],[10,1],[0.5,7.5]]
#   for i in range(len(PoleCordinates)):
#     PoleCordinates[i][0]/=2
#     PoleCordinates[i][1]/=2
#   PoleCordinates = [[9,18],[14,7],[16,7],[17,17]]  
  Poles = []
  for i in range(len(PoleCordinates)):
    p = Pole(PoleCordinates[i][0], PoleCordinates[i][1])
    Poles.append(p)
    
  Mule = Vehicle(5,5,np.pi/2,stepSize,0.1,0.02,0.8)
  Particles = []
  for i in range(numParticles):
    p = Particle(random.random()*worldSize, random.random()*worldSize, Mule.Pose.theta,stepSize,0.1,0.02,0.8)
    Particles.append(p)
 

#   Utility.draw(Mule, 0, worldSize, Poles, Particles)
  scores= []
  xvals= []
  for i in range(len(turnSequence)):
    Utility.draw(Mule, i+1, worldSize, Poles, Particles, title="Particles Spawned")
    Mule.move([turnSequence[i]])
    muleSensorData = Mule.sense(Poles)
    for j in range(len(Particles)):
#       introduce probabilistic movement for particles ToDo
      Particles[j].move([turnSequence[i]])
    
    Utility.draw(Mule, i+1, worldSize, Poles, Particles, title="Mule and Particles move")
    
    for j in range(len(Particles)):  
      temp = Particles[j].sense(Poles)
      weight = Particles[j].calculateWeight(muleSensorData)
      Particles[j].setWeight(weight)
    newParticles = Utility.resampling(Particles)
    evalScore = Utility.eval(Mule,Particles,worldSize)
    scores.append(evalScore)
    xvals.append(i)
    Utility.draw(Mule, i+1, worldSize, Poles, Particles, newParticles, evalScore, title="Particles Resampled")
    print("Evaluation Score: ", evalScore)
    print("P: ", len(Particles))
    print("PR: ", len(newParticles))
#     for i in range(len(Particles)):
#       print("P: ", Particles[i].Pose.x, Particles[i].Pose.y, "PR: ", newParticles[i].Pose.x, newParticles[i].Pose.y)
    Particles = newParticles
  plt.plot(xvals, scores, '-o')
  plt.show()
    
    