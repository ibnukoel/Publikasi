from controller import Robot, Camera, InertialUnit, DistanceSensor, PositionSensor, \
    DistanceSensor, Motor, Gyro, Accelerometer, Display, GPS, Keyboard
import matplotlib.pyplot as plt
import math, numpy, random, cv2
import sys, csv, os
from mode import mode,v, v_r_asli, v_k,init,goal,data_terminal
#testing
from mode import v_r_test,sudutRobot_test,d_rho1,d_rho2,maxRho1,maxRho2
#==========
from mapping import grid_scale,delta,movementRobot,printmap,cartesianRobot,map_posisi
from imager import get_data_from_camera, lineproc
#import calibration

#initialization
robot = Robot()
timestep = int(robot.getBasicTimeStep())
kb = Keyboard()
kb.enable(timestep)

# activation camera
camera = robot.getDevice('camera')
camera.enable(timestep)
#camera.recognitionEnable(timestep)
width = int(camera.getWidth())
height = int(camera.getHeight())

# getting the position sensors
roki = robot.getDevice('left wheel sensor')
roka = robot.getDevice('right wheel sensor')
roki.enable(timestep)
roka.enable(timestep)


# enable imu
imu = robot.getDevice('inertial unit')
imu.enable(timestep)


#enableGPS
gps = robot.getDevice('gps')
gps.enable(timestep)

# get handler to motors and set target position to infinity
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

# values for robot in inch
# wheel_radius = 1.6 / 2.0
# wheel_circ = 2 * 3.14 * wheel_radius
# enc_unit = wheel_circ / 6.28

# variable

RotEnc = [0, 0]
motor_posisi_awal = [0, 0]
baru_sampe_goal = True
hadap_goal = False
#spekRobot
rRoda = 2.05
enc_unit = (2 * 3.18 * rRoda) / 6.28
max_speed = 4
d = 2.28
d_mid = d / 2.0
#Posisi Awal
x_init = init[0]
y_init = init[1]
koreksi = 0
baru_selesaikoreksi = False
baru_selesai_putar = False
pose = [0,0,0]
i = 0 
i_old = -1 #untuk flag print
deltaRho = 0
ks = 0
motor = [0,0]
#jalan = True
baru_putar = True
baru_maju = True
baru_selesai = True
cek_garis = False
pos_rot = 0
xxx=0 #flag
#posisi sekarang
x = x_init
y = y_init
#posisi_sebelumnya
x_mobo_prev = 0
y_mobo_prev = 0
#exportDataVariable
xname = 1
extension = str('.jpg')
v_r = v_r_asli
#testing variable
putaran = 0
kecepatan = 0
timer_putar = 0
putar_gagal_1 = True
putar_gagal_2 = True
update_sudut = False
nama_file = str("data "+str(d_rho1)+"-"+str(maxRho1)+" "+str(d_rho2)+"-"+str(maxRho2)+" "+".csv")
#sudutRobot = sudutRobot_test[putaran]
#v_r = v_r_test[kecepatan] 

def get_direction():
    # maju = 1, mundur=2, kanan=3, kiri=4
    # jelaskan magsud dari bobot :
    imu_val = (imu.getRollPitchYaw()[2] * 180) * (7 / 22)
    bobot = 0
    dir = 0
    """
    if (imu_val <= -135 and imu_val >= -180) or (135 <= imu_val <= 180):
        dir = "North"
        bobot = [[25, 75, 100, 75], [1, 4, 3, 2]]
    elif imu_val <= -45 and imu_val > -135:
        dir = "West"
        bobot = [[75, 25, 75, 100], [3, 1, 4, 2]]
    elif 45 <= imu_val <= 135:
        dir = "East"
        bobot = [[75, 100, 75, 25], [4, 2, 3, 1]]
    elif (-45 < imu_val <= 0) or (0 <= imu_val < 45):
        dir = "South"
        bobot = [[100, 75, 25, 75], [2, 3, 1, 4]]
    """
    
    return imu_val, bobot, dir
def cm(meters):
    return round(meters * 100)
def get_sen_jarak():
    # returns left, front, right sensors in inches
    return cm(depan.getValue()), cm(kanan.getValue()), cm(kiri.getValue())
def get_motor_pos():
    return roki.getValue(), roka.getValue()
def robot_movement(RotEnc, RotEnc_new, enc_unit, pose):
    motor_shift = [0,0]
    jarak = [0,0]
    for i in range(len(RotEnc_new)):  # menghitung jarak pergeseran robot
        motor_shift[i] = RotEnc_new[i] - RotEnc[i]  # menghitung pergeseran rotaty
        jarak[i] = motor_shift[i] * enc_unit  # menghitung jarak pergeseran robot
    v = (jarak[0] + jarak[1]) / 2
    w = (jarak[0] - jarak[1]) / 52
    pose[0] += v * math.cos(pose[2])
    pose[1] += v * math.sin(pose[2])
    pose[2] += w
    return pose

def printdata(saveCSV= 1,saveImage = 1,ImageProc = 1):
#def printdata(saveImage,ImageProc,saveRealPos,saveCSV):
    values=[]
    #print("=====didalam printdata====")
    #print("cek nilai korekse sebelum cetak",koreksi)
    rho = 0
    theta = 0

    values.append(filename)
    

    val_gps = gps.getValues()
    x = val_gps[0]
    y = val_gps[1]
    #x_t = -(map_posisi[movementRobot[i][2]])
    #y_t = map_posisi[movementRobot[i][3]]
    #x_geser = abs((x-x_t)*1000)
    #y_geser = abs((y-y_t)*1000)    
    #e = math.sqrt(x_geser**2+y_geser**2)
    #values.append(e)
    values.append(koreksi)
    values.append(x)
    values.append(y) 
    #values.append(x_t)
    #values.append(y_t)
    
    
    if ImageProc == 1:
        deltaRho_or_values,rho,theta = get_data_from_camera(robot,camera)
        #print(get_data_from_camera(robot,camera))
        #values.append(deltaRho)
        values.append(delta_rho_map)
            
        values.append(deltaRho_or_values)
        values.append(rho)
        values.append(theta)        
        #print(delta_rho_map)
        #print("print value",values)
    
    #values.append(x_geser)
    #values.append(y_geser)
    #values.append(get_direction()[0])
    #values.append(motor)
    
        
    if saveCSV == 1:
        #print("print csv",delta_rho_map)
        file = open(nama_file,'a')
        writer = csv.writer(file)
        writer.writerow(values)
        file.close()
        #print("flieclose")
    
    if saveImage == 1:
        camera.saveImage(loc_save,10)
    
    #print("=======end print data=====")
    return deltaRho_or_values,rho,theta

def putar (sudutRobot,v_r):
    possible_left  = 180 + sudutRobot - arahRobot
    possible_right = 180 - sudutRobot + arahRobot
    
    if sudutRobot == 180:
        if arahRobot < 0:
            axax = possible_left
            possible_left = possible_right
            possible_right = axax
        
    if(possible_right < possible_left): #belokKiri
        motor = [-(v_r),(v_r)]
        sign = "belok kiri"
    if(possible_right > possible_left): #belokKanan
        motor = [v_r,-(v_r)]
        sign = "belok kanan"
            
    """
    if(possible_right < possible_left): #belokKiri
        motor = [minus*(-(v_r)),minus*(v_r)]
        sign = "belok kiri"
    if(possible_right > possible_left): #belokKanan
        motor = [minus*v_r,minus*(-(v_r))]
        sign = "belok kanan"

    #print("    ====> putar", sign, arahRobot)

        #print("L = ",possible_left)
        #print("R = ",possible_right)    
    
    possible_left  = 180 + minus*abs(sudutRobot) - minus*abs(arahRobot)
    possible_right = 180 - minus*abs(sudutRobot) + minus*abs(arahRobot)
    #print(sudutRobot,possible_left, possible_right)
    
    """    

    return motor

def proses_garis():
    global koreksi
    global motor
    value,rho,theta = printdata(0,0)
    koreksi,motor = lineproc(mode,rho,value,delta_rho_map,koreksi)
    print("#### check garis, nilai DP_map =",delta_rho_map,motor)
    value,rho,theta = printdata()

def cek_arah_robot():
    global timer_putar
    global putar_gagal_200
    global putar_gagal_500
    global v_r
    global pos_rot
    if timer_putar == 300 and putar_gagal_1:
        v_r = v_r - (v_r/2)
        putar_gagal_1 = False
    
    if timer_putar == 350 and putar_gagal_2:
        v_r = v_r - (v_r/2)
        putar_gagal_2 = False
                
    if timer_putar == 400:
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        print("gagal putar",v_r,v_r_test[kecepatan],sudutRobot_test[putaran])
        sys.exit(0)
    
    
    print(arahRobot,sudutRobot)
    
    if sudutRobot == 180 or sudutRobot == -90 :
        #print(arahRobot)
        if abs(arahRobot) >= abs(sudutRobot) - 0.5 and abs(arahRobot) <= abs(sudutRobot) + 0.5 :
            robot_lurus = True
            timer_putar = 0
        else :
            robot_lurus = False
            if abs(arahRobot) >= abs(sudutRobot) - 1 and abs(arahRobot) <= abs(sudutRobot) + 1 and pos_rot == 1:
                v_r = v_r - (v_r/2)
                pos_rot = 0

    elif arahRobot >= sudutRobot - 0.5 and arahRobot <= sudutRobot + 0.5:
        robot_lurus = True
        timer_putar = 0
    else : 
        robot_lurus = False
        if arahRobot >= sudutRobot - 1 and arahRobot <= sudutRobot + 1:
            v_r = v_r - (v_r/2)
    timer_putar = timer_putar + 1
    
    if robot_lurus:
        putar_gagal_1 = True
        putar_gagal_2 = True
        pos_rot = 1
        v_r = v_r_asli
        
    return robot_lurus
        
def print_terminal(data_terminal):
    global i_old
    if data_terminal == 0 :
        print("xmobo",pose[0] / grid_scale)
        print("====ceking value dan ambil data====")
        print("ks",ks)
        #print("koreksi",koreksi)
        #delta_rho_map   = movementRobot[i][4+ks]
        print("dp",delta_rho_map)
        print("move",motor)
        print("0. cek garis gak? =",cek_garis)
        
    
        #flag
        if i_old != i:
            print("//ada perubahan grid\\")
            print("delta rho MAP",delta_rho_map,delta_rho_map_2)
            print("sudut",sudutRobot)
            print("nilai i",i)
            i_old = i
    
print(xname)
print(mode)

path = str(str(v)+"-"+str(v_k))
#os.mkdir(str("./"+path))
file = open(nama_file,'a')
writer = csv.writer(file)
writer.writerow(" ")
write1 = ["dP1,dP2"]
write1.append(d_rho1)
write1.append(d_rho2)
write2 = ["P1,P2"]
write2.append(maxRho1)
write2.append(maxRho2)
#headaer semua data
#write3 = ["name","koreksi","x","y","targX","targY","dP_map","dP","P","theta","x_geser","y_geser","sudut"]
#ambil data
write3 = ["name","koreksi","x","y","dP_map","dP","P","theta"]


writer.writerow(write1)
writer.writerow(write2)
writer.writerow(write3)
file.close()


sudutRobot = movementRobot[i][1]
delta_rho_map   = movementRobot[i][4+ks]
delta_rho_map_2 = movementRobot[i][5]

        
while robot.step(timestep) != -1:
    #init values
    print("start")
    arah = get_direction()
    arahRobot = arah[0] 
    #print_terminal(data_terminal)
    
    filename = str(str(v)+"-"+str(v_k)+"-"+str(xname)+extension)
    loc_save = str(path+"/"+filename)
    
    #print(loc_save)
    counter = 0
    #cek MODE
    
    if mode == 5:
        motor = [0.5,0.5]
        printdata()
        leftMotor.setVelocity(motor[0])
        rightMotor.setVelocity(motor[1])
    
  
    elif mode == 4:
        #error hingga 1000 pencarian maka stop tes   
        #print("a",arahRobot,", sb",sudutRobot, ", s =",v_r)
        robot_lurus = cek_arah_robot()
        
        if robot_lurus :
            print("#####",timer_putar,arahRobot,sudutRobot,v_r,putaran,kecepatan)
            putaran = putaran + 1
            timer_putar = 0
            update_sudut = True
        else :
            motor = putar(sudutRobot,v_r)
        
        #selesai percobaan satu kecepatan
        if putaran > (len(sudutRobot_test)-1):
            putaran = 0
            time_putar = 0
            kecepatan = kecepatan + 1
        #selesai tes
        if kecepatan >= len(v_r_test):
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            print("error aneh")
            sys.exit(0)
        
        if update_sudut :
            sudutRobot = sudutRobot_test[putaran]
            v_r = v_r_test[kecepatan]
            update_sudut = False
                    
        leftMotor.setVelocity(motor[0])
        rightMotor.setVelocity(motor[1])
        
        
    elif mode == 0:
        #print("mode 0 ")
        if kb.getKey()==315:
            printdata()
            print("get data")
            print(xname)
            xname=xname+1
            while counter!=10000000:
                counter+=1
    
    #MODE 1,2,3
    else :
        #SAMPAI GOAL
        print("pos x,y",x,y)
        if x == goal[0] and y == goal[1]:
            print("Robot sampai")
            if baru_sampe_goal:
                proses_garis()
            else :
                hadap_goal = True
                robot_lurus = cek_arah_robot()
                if robot_lurus:
                    if ks==1: #cek koreksi setelah putar
                        print("      ====> setelah putar dan cek garis, update ke Dp2")
                        cek_garis = True
                        delta_rho_map   = movementRobot[i][4+ks]
                        ks = 0
                        print("      ====> nilai dp setelah putar", delta_rho_map)
                        print("      ====> koreksi setelah putar",koreksi,delta_rho_map)
                    if baru_selesai :
                        proses_garis()
                    if koreksi == 0 :
                        leftMotor.setVelocity(0)
                        rightMotor.setVelocity(0)
                        #Flag
                        if baru_selesai :
                            #printdata()
                            print("selesai", baru_selesai)
                            sys.exit(0)
                    print("koreksi ",koreksi)
                else :
                    motor = putar (sudutRobot,v_r)
                    ks = 1
            if koreksi == 0 and not hadap_goal :
                print ("sampe dan selesai koreksi 1st")
                baru_sampe_goal = False      
                motor = [0,0]
            
            leftMotor.setVelocity(motor[0])
            rightMotor.setVelocity(motor[1])
            #saveExperimentData()
            
        #Belum Sampai GOAL
        else:           
            #cek Koreksi
            print("1. cek Koreksi =",koreksi)
            if koreksi == 1 or koreksi == 2 or cek_garis:
                #flag
                proses_garis()
                running = False
                print("====> koreksi baru",koreksi,delta_rho_map)
                cek_garis = False
                
            #kondisi tak ada koreksi posisi
            elif koreksi == 0 :
                print("====> tidak ada koreksi aktif")
                #ks = 0
                #reset encoder untuk jalan lurus
                koreksi = 99
                pose = [0, 0, 0]
                x_mobo_prev = 0
                y_mobo_prev = 0
                #===============================
                running = True
            else:
                print("====>robot sedang jalan")
            # kalkulasi pergerakan robot
            RotEnc_new = get_motor_pos()
            pose = robot_movement(RotEnc, RotEnc_new, enc_unit, pose)
            RotEnc = RotEnc_new
            print("2. cek apakah jalan")
            
            if running:
                print("  ====> robot jalan")
                print("  3. cek sudut robot")
                #cek arah robot apakah sesuai, jika sesuai MAJU
                robot_lurus = cek_arah_robot()
                if robot_lurus:    
                    print("    ====> robot lurus",arahRobot,sudutRobot)
                    print("    4. cek kondisi setelah putar (ks)")
                    if ks==1: #cek koreksi setelah putar
                        print("      ====> setelah putar dan cek garis, update ke Dp2")
                        cek_garis = True
                        delta_rho_map   = movementRobot[i][4+ks]
                        ks = 0
                        print("      ====> nilai dp setelah putar", delta_rho_map)
                        proses_garis()
                        print("      ====> koreksi setelah putar",koreksi,delta_rho_map)
                
                    #jika robot pindah grid
                    else :
                        print("    ====> tidak setelah putar")
                        x_mobo = math.floor(abs(pose[0]) / grid_scale) - x_mobo_prev
                        y_mobo = math.floor(abs(pose[1]) / grid_scale) - y_mobo_prev
                        motor = [v,v]
                        if (x_mobo != 0):
                            #Update Robot Posisi Grid
                            ks = 0 #koreks_step untuk load delta_rho 1 
                            x_mobo_prev = x_mobo + x_mobo_prev
                            y_mobo_prev = y_mobo + y_mobo_prev
                            x += (x_mobo * cartesianRobot(sudutRobot)[0]) + (y_mobo * cartesianRobot(sudutRobot)[0])
                            y += (x_mobo * cartesianRobot(sudutRobot)[1]) + (y_mobo * cartesianRobot(sudutRobot)[1])
                            
                            #update everything
                            #map[x_init][y_init] = ' '
                            #map[x][y] = '#'
                            x_init = x
                            y_init = y
                            
                            #grid ceking
                            #if x != movementRobot[i][2] and y != movementRobot[i][3]:
                                #sys.exit(0)
                            #else : print("posisi tepat")
                            
                            i += 1
                            sudutRobot = movementRobot[i][1]
                            delta_rho_map   = movementRobot[i][4+ks]
                                                        
                            #reset pose untuk grid baru
                            pose = [0, 0, 0]
                            x_mobo_prev = 0
                            y_mobo_prev = 0
                            

                            #print('updatemap')
                            #print("update i =",i)
                            #print("pose",pose)
                            #printmap(map)
                            #print(sudutRobot,delta_rho_map,delta_rho_map_2)    
                            proses_garis()
                            if x == goal[0] and y == goal[1]:
                                motor = [0,0]
                            #print("apakah ada koreksi",koreksi)
                #arah tidak sesuai PUTAR sampai arah sesuai
                else :
                    motor = putar (sudutRobot,v_r)
                    ks = 1 #koreksi step untuk load delta rho 2
                   

            
            #print(sudutRobot,delta_rho_map,delta_rho_map_2)    
            print("motor gerak",motor)                
            leftMotor.setVelocity(motor[0])
            rightMotor.setVelocity(motor[1])
                   
        #printmap(map)
        print("====loop====")
    xname=xname+1