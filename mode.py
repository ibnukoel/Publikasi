mode = 3
data_terminal = 0
#mode 0 = ambil data dengan keybo2rd
#mode 1 = run
#mode 2 = run dengan koreksi posisi
#mode 3 = mode2 wit output deltarho
#mode 4 = test kecepatan putar
#mode 5 = test dengan maju terus
simpanData = 0
#1 = simpan data
#0 = tidak simpan

#Adjustable Parameter
d_rho1 = 16
maxRho1 = 116

d_rho2 = 2
maxRho2 = 8

v = 3 #kecepatan lurus
v_r_asli = 0.5  #kecepatan saat putar
#kecepatan Robot ketika koreksi
v_k = 0.5
#testing
v_r_test = [2,2.5,3]
#v_r_test = [0.5,0.6,0.7,0.8,0.9,1,0.9,1,1.5,2,2.5,3]
sudutRobot_test = [-90,90,180,0]





init = [7, 1]
goal = [0,7]
#goal = [5,1]
#goal = [6,1]