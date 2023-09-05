#!/usr/bin/python3

import utils
import random

myPriorityQueue = utils.PriorityQueue()

for i in range(10):
    id = random.randint(5, 40)
    myPriorityQueue.push(id, id)
    

for i in range(10):
    print(myPriorityQueue.pop())