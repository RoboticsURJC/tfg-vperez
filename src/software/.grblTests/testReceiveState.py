import serial, time


serialObj =  None 
port = '/dev/ttyS0'

def grblConnect():
	global serialObj
	try:
		serialObj = serial.Serial(port, 115200)
		print("Connected with grbl")
	except:
		print("Unable to connect with grbl...")
		exit(1)

def grblDisconnect():
	serialObj.close()
	print("Grbl disconnected")		

		
def getAngle():
	
	if serialObj.isOpen():
		serialObj.write('?'.encode())
		
		data = str(serialObj.readline())
		
		if '<' in data:
		
			# Remove unnecesary characters
			msg = data[3:][:-6]
			
			#print(msg)
			
			fields = msg.split('|')
			
			state = fields[0]
			motorPositions = fields[1][5:].split(',')
			motorSpeed = fields[2][3:].split(',')[0]
			
			print("State:" + state)
			print("Position X: " + motorPositions[0] + "   Position Y: " + motorPositions[1] + "   Position Z: " + motorPositions[2]) 
			print("Speed:" + motorSpeed)

def setAngle():
	pass

	
def main():
	
	grblConnect()
	
	getAngle()
	
	grblDisconnect()
	

if __name__ == "__main__":
	main() 
	
