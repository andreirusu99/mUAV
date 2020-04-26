import socket,time,math
class pyMultiWii:
    
    def __init__(self,TCP_IP, TCP_PORT,debug=False):
        self.TCP_IP=TCP_IP
        self.TCP_PORT=TCP_PORT
        self.BUFFER_SIZE = 1024
        self.debug=debug
        self.mySocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.mySocket.connect((TCP_IP, TCP_PORT))
        headerArray=bytearray([36,77,60])
        self.valueArray=bytearray([])
        roll=1500                    
        pitch=1500                 
        throttle=1500 
        yaw=1500                      
        aux1=1200
        aux2=1000
        aux3=1500
        aux4=1200      
        self.valueArray.extend(headerArray)
        self.valueArray.append(16)
        self.valueArray.append(200)
        self.valueArray.extend([220,5])
        self.valueArray.extend([220,5])
        self.valueArray.extend([220,5])
        self.valueArray.extend([220,5])
        self.valueArray.extend([176,4])
        self.valueArray.extend([232,3])
        self.valueArray.extend([220,5])
        self.valueArray.extend([176,4])
        self.valueArray.append(234)
        self.Array=self.valueArray[:]
        if(self.debug):
            print(self.Array)
        self.isConnected=False
        
    def changeCRC(self):
        self.CRCArray=self.Array[3:-1]
        self.CRCValue=0
        for d in self.CRCArray:
            self.CRCValue= self.CRCValue^d
        return self.CRCValue
    
    def getBytes(self,value): 
        self.LSB=value % 256
        self.MSB=math.floor(value/256)
        return bytearray([self.LSB,self.MSB])

##    def connect(self):
##        self.isConnected=True
##        if(self.debug):        
##            print ("Connected to Drone")            
##        self.sendPacket(self.Array)
        
        

    def arm(self):           
        self.Array[19]=220
        self.Array[20]=5
        Val=self.changeCRC()
        self.Array[21]=Val
        if(self.debug):
            print("Connected")
        self.sendPacket(self.Array)
            
            

##        else:
##            self.Array[21]=0
##            if(self.debug):
##                print("Not Connected")
##            self.sendPacket(self.Array)
            
            

    
    def disarm(self):
        self.Array[19]=176
        self.Array[20]=4
        Val=self.changeCRC()
        self.Array[21]=Val
        self.sendPacket(self.Array)
        
        
    
    def setThrottle(self,value):            
        arr=bytearray([])
        arr.extend(self.getBytes(value))
        self.Array[9]=arr[0]
        self.Array[10]=arr[1]
        Val=self.changeCRC()
        self.Array[21]=Val
        self.sendPacket(self.Array)
        
    
    def setRoll(self,value):                  
        arr=bytearray([])
        arr.extend(self.getBytes(value))
        self.Array[5]=arr[0]
        self.Array[6]=arr[1]
        Val=self.changeCRC()
        self.Array[21]=Val
        self.sendPacket(self.Array)
        
        
    
    def setPitch(self,value):                
        arr=bytearray([])
        arr.extend(self.getBytes(value))
        self.Array[7]=arr[0]
        self.Array[8]=arr[1]
        Val=self.changeCRC()
        self.Array[21]=Val
        self.sendPacket(self.Array)
        
        

    def setYaw(self,value):              
        arr=bytearray([])
        arr.extend(self.getBytes(value))
        self.Array[11]=arr[0]
        self.Array[12]=arr[1]
        Val=self.changeCRC()
        self.Array[21]=Val
        self.sendPacket(self.Array)
        
        

    def sendPacket(self,lValueArray):
        self.mySocket.send(lValueArray)

    def recieveResponse(self):
        return self.mySocket.recv(self.BUFFER_SIZE)
    
    def disconnect(self):
        self.mySocket.close()
