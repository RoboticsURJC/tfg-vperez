import serial
import threading as th  

motorPosition = 0.0
motorSpeed = 0.0
machineState = 'UNDEFINED'

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

	

def updateInfo():

	global motorPosition, motorSpeed, machineState
	
	if serialObj.isOpen():
		
		serialObj.write('?'.encode())
		
		data = str(serialObj.readline())
		
		if '<' in data:
		
			# Remove unnecesary characters
			msg = data[3:][:-6]
			
			fields = msg.split('|')
			
			machineState = fields[0]
			motorPosition = fields[1][5:].split(',')[0]		
			motorSpeed = fields[2][3:].split(',')[0]
			
	else:
		print("Device disconnected manually!")		

	print("updated")

def main():
	
	infoThread = th.Timer(1, u)  
	infoThread.start() 
	
	
	


if __name__ == "__main__":
	main() 
