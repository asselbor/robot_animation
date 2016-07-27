# Choregraphe simplified export in Python.
from naoqi import ALProxy
import animations.showMuscles1_pose

bodyBottom = ["LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll", "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll"]


def increaseSpeed(factor):
    global times
    for i in xrange(len(times)):
        newList = [x / float(factor) for x in times[i]]
        times[i] = newList

def increaseAmplitude(factor):
    for i in xrange(len(names)):
        if names[i] not in bodyBottom:
            keys[i] = [x * float(factor) for x in keys[i]]



def runMotion(a):
    print a.times

try:
  # uncomment the following line and modify the IP if you use this script outside Choregraphe.
  motion = ALProxy("ALMotion", "10.0.0.10", 9559)
  runMotion(animations.showMuscles1_pose)
  # increase speed of movements
  #increaseSpeed(1.5)
  #increaseAmplitude()

  #print .keys
  #animations.showMuscles1_pose.showMuscles1Animation(motion)

except BaseException, err:
  print err
