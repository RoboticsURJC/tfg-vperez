
L1 = float(input("Length of LINK 1 in cm: ")) / 100.0
L2 = float(input("Length of LINK 2 in cm: ")) / 100.0
M1 = float(input("Mass of LINK 1 in g: ")) / 1000
M2 = float(input("Mass of LINK 2 in g: ")) / 1000
Payload = float(input("Desired payload in g: ")) / 1000
g = 9.8

T1 = ( M1 * (L1/2) + M2 * (L1 + L2/2) + Payload * (L1 + L2) ) * g
T2 = ( M2 * (L2/2) + Payload * L2) * g

print("Torque 1 = " + str(T1) + "  " + "Torque 2 = " + str(T2))