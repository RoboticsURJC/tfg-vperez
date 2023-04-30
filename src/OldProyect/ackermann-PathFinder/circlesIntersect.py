import sympy as sym


x = sym.Symbol('x')
y = sym.Symbol('y')


def getCircunferenceExpresion(center_x, center_y, radius):
      
    ec = sym.expand((x - center_x)**2) + sym.expand((y - center_y)**2) - radius**2
    
    return ec

def circunferencesIntersection(circunference1_ec, circunference2_ec):
    
    intersection_line = circunference1_ec - circunference2_ec
    print("Both points are in line:")
    print(intersection_line)
    
    solution = sym.solve((intersection_line, circunference1_ec), (x, y))
    
    point1 = solution[0]
    point2 = solution[1]
    
    print("\nPoint 1 is:")
    print(sym.N(point1[0]))
    print(sym.N(point1[1]))
    print("Point 2 is:")
    print(sym.N(point2[0]))
    print(sym.N(point2[1]))  

    
expr = getCircunferenceExpresion(2,2,1)
print("Circunference ecuation:  center in (2,2) and radius = 1")
print(expr)

expr2 = getCircunferenceExpresion(0,0,3)
print("Circunference ecuation:  center in (0, 0) and radius = 3")
print(expr2)

circunferencesIntersection(expr, expr2)