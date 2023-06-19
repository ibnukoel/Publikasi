import numpy,cv2
from mode import v_k,v,d_rho2,maxRho1,maxRho2,d_rho1
deltaRho2 = 2

#cv2.startWindowThread()
#cv2.namedWindow("preview")

#modeCam ikut mode simulasi,
#mode 2 = outnya values,rho,theta
#mode 3 = outpunya delta_rho,rho,theta
def get_data_from_camera(robot,camera):
    values=[]
    rho = []
    theta = []
    """
    Take an image from the camera device and prepare it for OpenCV processing:
    - convert data type,
    - convert to RGB format (from BGRA), and
    - rotate & flip to match the actual image.
    """
    img = camera.getImageArray()
    
    img = numpy.asarray(img, dtype=numpy.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
    img2 = cv2.flip(cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE),1)
    #img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #cv2.imwrite("../img1.jpg",img)
    #return cv2.flip(img, 1)

    hsv = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)
    cv2.imshow("preview", img2)
    #cv2.imshow("preview", hsv)
    
    #orange new world with table
    lower = numpy.array([14, 200, 200])
    upper = numpy.array([25 , 255, 255])
    
    #orang world in floor
    #lower = numpy.array([0, 93, 0])
    #upper = numpy.array([225, 255, 255])
    
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.GaussianBlur(mask, (5, 5), 0)
    #cv2.imshow("hsv",mask)
    #cv2.waitKey()
    #output = cv2.bitwise_and(img,img, mask= mask)
    #bluring or not
    edges = cv2.Canny(mask,10,150)
    lines = cv2.HoughLines(edges, 1, numpy.pi / 180, 80, None, 0, 0)
    #print(lines)
    if lines is not None:
        for i in range(0, len(lines)):
            rho.append(lines[i][0][0])
            theta.append(lines[i][0][1])
            values.append([lines[i][0][0],lines[i][0][1]])
    if len(rho)>=1:
        deltaRho = max(rho) - min(rho)
        return deltaRho,rho,theta
    else : 
        deltaRho = 0
        return deltaRho,rho,theta
    

#mode = mode dari simulasi 1,2,3
#mode 0 dan 1 tidak menggunakan koreksi
#mode 2 menggunakan koreksi perhitungan manual delt_rho
#mode 3 menggunakan nilai delta_rho langsung
#rho = nilai rho dari pembacaan garis
#deltaRho = nilai perbandingan rho1 dan rho2
#val = nilai deltaRho dari map
#koreksi = status koreksi yang sedang berjalan
def lineproc(mode,rho,deltaRho,val,koreksi) :
    move = [0,0]
    print("####====function lineproc====")
    #print("banyak garis",len(rho))
    if mode != 0 and mode!=1:
        if val!=0:
            print(len(rho),koreksi)
            if len(rho)>1 and koreksi!=2:
                #dengan perhitungan maxrho
                if val == d_rho2:
                    maxRho = maxRho2
                elif val == d_rho1:
                    maxRho = maxRho1
                else:
                    maxRho = max(rho)
                #+++++++++++++++++++++++++
                
                if mode == 2:
                    deltaRho = max(rho) - min(rho)
                
                print("val",val)
                print("#### rho improc =",deltaRho,"vs MAP =",val)
                print("maxP =",max(rho),"vs",maxRho)
                
                #old 
                """
                if deltaRho==val and max(rho)== maxRho:
                #if deltaRho==val:
                    koreksi = 0
                    move = [v,v]
                    print("#### garis pas")
                    """
                if deltaRho==val:
                    if max(rho)== maxRho:
                        koreksi = 0
                        move = [0,0]
                        print("#### garis pas")
                    elif max(rho)<= maxRho:
                        print("#### koreksi maju")
                        move = [v_k,v_k]
                        koreksi = 1
                    elif max(rho)>= maxRho:
                        print("#### koreksi mundur")
                        move = [-(v_k),-(v_k)]
                        koreksi = 1     
                elif deltaRho<=val:
                    print("#### koreksi maju")
                    move = [v_k,v_k]
                    koreksi = 1
                elif deltaRho>=val:
                    print("#### koreksi mundur")
                    move = [-(v_k),-(v_k)]
                    koreksi = 1
                """
                else:
                    koreksi = 0
                    move = [0,0]
                """    
            else:
                if val == d_rho2:
                    print("#### GARIS TAK TERLIHAT d_rho")
                    move = [(v_k),(v_k)] 
                else :
                    print("#### GARIS TAK TERLIHAT")
                    move = [-(v_k),-(v_k)]
                koreksi = 2
                if len(rho)==2 and deltaRho >= deltaRho2:
                    koreksi = 1
                    print("#### garis ketemu")
                
        else :
            print("#### no koreksi karena val 0")
            koreksi = 0
            move=[v,v]
    else :
        print("mode 1, jalan tanpa koreksi")
        koreksi = 0
        move=[v,v]
    #print("koreksi",koreksi)
    print("####========end line proc=======")
    return koreksi,move
        