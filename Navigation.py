import matplotlib.pyplot as mb
import smbus
import numpy as np
from numpy import dot
from numpy.linalg import inv
from time import sleep
import math
#Prompt for time interval.
deltaT=0.25
def getState(X,Pk1,a):
	#Initialize the A matrix.
	#This matrix generally deals with the position of the object.
	A=np.identity(6) #gives you an identity matrix of the dimention 6x6
	A[0][3]=deltaT
	A[1][4]=deltaT
	A[2][5]=deltaT

	#Initialize B matrix.
	B=np.zeros((6,3)) #give a null matrix of dimentions 6x3 
	deltaTsq=0.5*deltaT*deltaT
	j=0
	for i in range(3):
		B[i][j]=deltaTsq
		B[i+3][j]=deltaT
		j+=1
	Xk=dot(A,X)+dot(B,a) #gives the matrix multiplication values and adds them together
	#*****************Calculating the covariance matrix****************
	#Multiplying A Pk1 and At.
	#intialize At
	At=np.transpose(A)
	Pk=dot(A,dot(Pk1,At))
	for i in range (len(Pk)):
		for j in range (len(Pk[0])):
			if(i!=j):
				Pk[i][j]=0
	return(Xk,Pk)
def getKalmanGain(Pk,Xk):
	#initialize H
	H=np.identity(6)
	#initialize Ht
	Ht=np.transpose(H)
	R= [	[4,0,0,0,0,0],	[0,1,0,0,0,0],	[0,0,1,0,0,0],	[0,0,0,4,0,0],	[0,0,0,0,1,0],	[0,0,0,0,0,1]]
	K=dot(dot(Pk,Ht),inv((dot(dot(H,Pk),Ht)+R)))
	return K
def getXkf(K,Xk,Ykf):
	#**********************Calculating Xkf*************************
	H=np.identity(6)
	Xkf=Xk+dot(K,(Ykf-dot(H,Xk)))
	return Xkf
def getPkf(K,Pk):
	H=np.identity(6)
	I=np.identity(6)
	Pkf=dot((I-dot(K,H)),Pk)	
	return Pkf

#Powermanagement registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

bus = smbus.SMBus(1)
address = 0x68

bus.write_byte_data(address, power_mgmt_1, 0)

def read_word(adr):
	high = bus.read_byte_data(address, adr)
	low = bus.read_byte_data(address, adr+1)
	val = (high << 8) + low
	return val

def read_word_2c(adr):
	val = read_word(adr)
	if(val >= 0x8000):
		return -((65535 - val) + 1)
	else:
		return val
"""
xp=0
yp=0
zp=0

X1=np.zeros((6,1))
"""
x1=6731000*math.cos(17.466439*math.pi/180)*math.cos(72.489528*math.pi/180)
y1=6731000*math.cos(17.466439*math.pi/180)*math.sin(72.489528*math.pi/180)
z1=6731000*math.sin(17.466439*math.pi/180)
"""
X1[0][0] = x1
X1[1][0] = y1
X1[2][0] = z1


#initialize Pk1
Pk1 = [	[4,0,0,0,0,0],	[0,4,0,0,0,0],	[0,0,4,0,0,0],	[0,0,0,4,0,0],	[0,0,0,0,4,0],	[0,0,0,0,0,4]]

print("Initial latitudes") 
print("latitude is:")
print(math.asin(z1/6731000)*180/math.pi)

print("longitude is:")
print(math.atan(y1/x1)*180/math.pi)
"""
def getLatitude(x,y,z):
	return (math.asin(z/6731000)*180/math.pi)

def getLongitude(x,y,z):
	return (math.atan(y/x)*180/math.pi)

fd = open("latlongdata.txt","w")
gyrox = read_word_2c(0x43)
gyroy = read_word_2c(0x45)
gyroz = read_word_2c(0x47)

gx1 = gyrox*5*0.25 / (16384.0 * 131.0)
gy1 = gyroy*5*0.25 / (16384.0 * 131.0)
gz1 = gyroz*5*0.25 / (16384.0 * 131.0)

fd = open("data1.txt","w")
count=0
i=0
Ax=[]
Ay=[]
Az=[]

gx=[]
gy=[]
gz=[]

t=[] 
k=0
while(count!=5000):

  	accel_xout = read_word_2c(0x3b)
	accel_yout = read_word_2c(0x3d)
	accel_zout = read_word_2c(0x3f)
		
	Ax.append(accel_xout / 16384.0)
	Ay.append(accel_yout / 16384.0)
	Az.append(accel_zout / 16384.0)

	vx = np.trapz(Ax,dx=0.001)
	vy = np.trapz(Ay,dx=0.001)
	vz = np.trapz(Az,dx=0.001)

	gyrox = read_word_2c(0x43)
	gyroy = read_word_2c(0x45)
	gyroz = read_word_2c(0x47)

	gx.append(gyrox*3.3*0.25 / (16384.0 * 131.0))
	gy.append(gyroy*3.3*0.25 / (16384.0 * 131.0))
	gz.append(gyroz*3.3*0.25 / (16384.0 * 131.0))

	t.append(count)
	"""	
	print("\nAt time : ")
	print(count)
	print("Millisecs\n"+"\n Accelerometer")
	print(Ax[i])
	print(Ay[i])
	print(Az[i])
	print("Gyro")
	print(gx[i])
	print(gy[i])
	print(gz[i])
		
	fd.write("\n\n   At Time:\t")
	fd.write(str(count)+"  Millisecs")
	fd.write("\nAccelerometer\n"+str(Ax[i]))
	fd.write("\n"+str(Ay[i]))
	fd.write("\n"+str(Az[i]))
	fd.write("\nGyro:\n"+str(gx[i]))
	fd.write("\n"+str(gy[i]))
	fd.write("\n"+str(gz[i]))
	"""
	count = count+250
	sleep(.25)
	i=i+1
	"""
	#Acceleration along various axes. 
	a=np.zeros((3,1))
	a[0][0] = sum(Ax,0.0)/len(Ax) 
	a[1][0] = sum(Ay,0.0)/len(Ay) 
	a[2][0] = sum(Az,0.0)/len(Az) 

	xp+=vx*deltaT
	yp+=vy*deltaT
	zp+=vz*deltaT

	Yk=[    [xp],	
		[yp],
		[zp],
		[vx],
		[vy],
		[vz]]

	C=np.identity(6)

	Ykf=dot(C,Yk)

	Xk,Pk = getState(X1,Pk1,a)

	K=getKalmanGain(Pk,Xk)

	Xkf=getXkf(K,Xk,Ykf)
	#print (Xkf)

	Pkf=getPkf(K,Pk)
	#print (Pkf)

	X1=Xkf
	Pk1=Pkf

	x2=x1+Xkf[0][0]
	y2=y1+Xkf[1][0]
	z2=z1+Xkf[2][0]	

	print("Latitude:")
	print(getLatitude(x2,y2,z2)+gy-gy1)

	print("Longitude:")
	print(getLongitude(x2,y2,z2)+gx-gx1)

	fd.write("\n\n   At Time:\t")
	fd.write(str(k)+"  Millisecs")
	fd.write("\nLatitude:\n")	
	fd.write(str(getLatitude(x2,y2,z2)+gy-gy1))	
	fd.write("\nLongitude\n")
	fd.write(str(getLongitude(x2,y2,z2)+gx-gx1))	

	k=k+250
	sleep(0.000000001)
	"""
#get a plot using the matplotlib.pyplot library
mb.plot(t,Ax)
mb.axis([0,40000,-2,2])#set the axes
#mb.show()
mb.plot(t,Ay)
mb.axis([0,40000,-2,2])
#mb.show()
mb.plot(t,Az)
mb.axis([0,40000,-2,2])
#mb.show()
mb.plot(t,gx)
mb.axis([0,40000,-1,1])
#mb.show()
mb.plot(t,gy)
mb.axis([0,40000,-1,1])
#mb.show()
mb.plot(t,gz)
mb.axis([0,40000,-1,1])
#mb.show()
#use the polyfit method from numpy to print a plot
zx = np.polyfit(t,Ax,2)
zy = np.polyfit(t,Ay,2)
zz = np.polyfit(t,Az,2)
#get a 1d polynomial
fx = np.poly1d(zx)	    
fy = np.poly1d(zy)	    
fz = np.poly1d(zz)	    
#gives a 2d polynomial
zgx = np.polyfit(t,gx,2)
zgy = np.polyfit(t,gy,2)
zgz = np.polyfit(t,gz,2)

fgx = np.poly1d(zgx)	    
fgy = np.poly1d(zgy)	    
fgz = np.poly1d(zgz)	   

print(fx)
print(fy)
print(fz)

print(fgx)
print(fgy)
print(fgz)
g=0
while(g!=5000):
	kx1 = np.polyval(zx,g)
	ky1 = np.polyval(zy,g)
	kz1 = np.polyval(zz,g)
	kgx = np.polyval(zgx,2000)
	kgy = np.polyval(zgy,2000)
	k1 = kx1+x1
	k2 = ky1+y1
	k3 = kz1+z1
	print("\n")	
	print(getLatitude(k1,k2,k3)+kgy-gy1)
	print(getLongitude(k1,k2,k3)+kgx-gx1)
	print("\n")
	fd.write("\nLatitude:\t")	
	fd.write(str(getLatitude(k1,k2,k3)+kgy-gy1))
	fd.write("\nLongitude:\t")
	fd.write(str(getLongitude(k1,k2,k3)+kgx-gx1))
	fd.write("\n")
	g=g+250
#use the polyval method from numpy to get a polynomial for the variables 
print("Ax[8] : ")
print(Ax[8])
k1 = np.polyval(zx,2000)
print(k1)
print("Ay[8] : ")
print(Ay[8])
k2 = np.polyval(zy,2000)
print(k2)
print("Az[8] : ")
print(Az[8])
k3 = np.polyval(zz,2000)
print(k3)
print("gx[8] : ")
print(gx[8])
k4 = np.polyval(zgx,2000)
print(k4)
print("gy[8] : ")
print(gy[8])
k5 = np.polyval(zgy,2000)
print(k5)
print("gz[8] : ")
print(gz[8])
k6 = np.polyval(zgz,2000)
print(k6)
