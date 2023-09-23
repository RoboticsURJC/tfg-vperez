#!/usr/bin/env python3
import math, time

import turtle

def verticesTriangle(position, sideSize):
    
    height = math.sqrt(3) * sideSize / 2

    vertex1 = (position[0], position[1])
    vertex2 = (position[0] - sideSize / 2, position[1] + height)
    vertex3 = (position[0] + sideSize / 2, position[1] + height)

    return (vertex1, vertex2, vertex3, vertex1)

def verticesSquare(position, sideSize):
    half_side = sideSize / 2

    vertex1 = (position[0] - half_side, position[1] + half_side)
    vertex2 = (position[0] + half_side, position[1] + half_side)
    vertex3 = (position[0] + half_side, position[1] - half_side)
    vertex4 = (position[0] - half_side, position[1] - half_side)

    return (vertex1, vertex2, vertex3, vertex4, vertex1)

def verticesCircle(position, radius, maxVertices=36):

    vertices = []
    for i in range(maxVertices):
        angle = 2 * math.pi * i / maxVertices
        x = position[0] + radius * math.cos(angle)
        y = position[1] + radius * math.sin(angle)
        vertices.append((x, y))
    
    return vertices

def verticesHeart(position, size):
    vertices = []

    for t in range(0, 360):
        radians = math.radians(t)
        x = size * (16 * math.sin(radians)**3)
        y = size * (13 * math.cos(radians) - 5 * math.cos(2*radians) - 2 * math.cos(3*radians) - math.cos(4*radians))
        vertices.append((position[0] + x, position[1] - y))  # Substract "y" to invert it vertically
    
    return vertices

def main():
    window = turtle.Screen()
    window.bgcolor("white")
  
    turtle_1 = turtle.Turtle()
    turtle_1.shape("blank")
    turtle_1.color("red")
    turtle_1.speed(10000)
    
    turtle_1.pendown()
    for v in verticesHeart((1,1), 2):
        turtle_1.goto(v[0], v[1])
 
    turtle_1.penup()
    for v in verticesTriangle((30,30), 200):
        turtle_1.goto(v[0], v[1])
        turtle_1.pendown()

    turtle_1.penup()
    for v in verticesSquare((50, 50), 200):
        turtle_1.goto(v[0], v[1])
        turtle_1.pendown()

    turtle_1.penup()
    for v in verticesCircle((100, 50), 50):
        turtle_1.goto(v[0], v[1])
        turtle_1.pendown()

    window.exitonclick()

main()