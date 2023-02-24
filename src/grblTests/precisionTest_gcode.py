testfile = open("test1.nc", "w")

for i in range(1, 16):
	gcode = "G21G91" + "X" + str(i * 0.125) + "F8"	
	testfile.write(gcode + '\n')
	gcode = "G21G91" + "X" + str(-i * 0.125) + "F8"	
	testfile.write(gcode + '\n')

testfile.close()
