from numpy import *
import operator


def createDataSet():
    group = array([[1.0,1.1],[1.0,1.0],[0,0],[0,0.1]])
    labels = ['A','A','B','B']
    return group,labels



print "hello world, this is the first python machine learning example \n"

group,labels = createDataSet()

