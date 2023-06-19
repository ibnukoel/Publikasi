from mode import d_rho1, d_rho2
"""
map = [['#', ' ', ' ', ' ',' ',' ',' '],
       [' ', ' ', ' ', ' ',' ',' ',' '],
       [' ', ' ', ' ', ' ',' ',' ',' '],
       [' ', ' ', ' ', ' ',' ',' ',' ']]

valnya = [[1, 2, 4, 7],
          [99, 3, 6, 11],
          [10, 5, 9, 14],
          [13, 8, 12, 15]]
          
grid = [[0, 0, 99, 0,0,0,0],
        [99, 0, 99, 0,0,0,0],
        [0, 0, 0, 0,0,0,0],
        [0, 0, 0, 0,0,0,0]]
"""        
grid_scale = 12.5

delta = [[-1, 0, 1],  # go up
         [0, -1, 1],  # go left
         [1, 0, 1],  # go down
         [0, 1, 1]]  # go right

#next move,robot facing, x , y ,left,right, rho,rhomax)
movementRobot1 = [[2, 0,    0, 1,  0, 1,0 ,0],
                 [4, 0,    0, 2,  0, 1,3 ,35],
                 [6, -90,  1, 2,  1, 0,15 ,105],
                 [11, 0,   2, 1,  0, 1,15,105],
                 [14, -90, 2, 3,  1, 0,3,35],
                 [15, -90, 3, 3,  1, 0,15 ,105]]
                 
#next move,robot facing, x , y , deltaRho)                 
#update : remove left right sign and move it in the cartesianRobot def
        # remove rhomax, because we not use it
movementRobot2 = [[2, 0,    0, 1,  0],
                 [4, 0,    0, 2,  3],
                 [6, -90,  1, 2,  15],
                 [11, 0,   2, 1,  15],
                 [14, -90, 2, 3,  3],
                 [15, -90, 3, 3,  15]]

#update dengan 2x koreksi sebelum dan sesudah berputar
#next move,robot facing, x , y , deltaRho1, deltaRho2,rhomin)    
            
movementRobot3 = [[2, 0,  0, 0, 0,      0,      0],
                 [2, 0,  0, 1, 0,      0,      0],
                 [4, -90,   0, 2, d_rho2, d_rho2, 35],
                 [6, 0, 1, 2, d_rho1, d_rho2, 105],
                 [11,-90,   2, 2, d_rho1, 0,      105],
                 [14,-90, 2, 3, d_rho2, d_rho2, 35],
                 [15,0, 3, 3, d_rho1, d_rho1,105]]     
                 
"""
--
 --
  -
  ->
"""       

#untuk ambil data kalibrasi deltaP               
movementRobot4 = [[2, -90,   0, 0, 0,      d_rho2, 0],
                 [2, 0,     1, 0, d_rho1, d_rho2, 0],
                 [4, -90,   1, 1, d_rho1, d_rho2, 0],
                 [5, 90,    2, 1, d_rho1, 0, 35],
                 [6, 90,    1, 1, 0,      0, 35],
                 [7, 180,   0, 1, 0,      d_rho2, 35],
                 [2, -90,   0, 0, d_rho1, d_rho2, 0],
                 [2, 0,     1, 0, d_rho1, d_rho2, 0],
                 [4, -90,   1, 1, d_rho1, d_rho2, 0],
                 [5, 90,    2, 1, d_rho1, 0, 35],
                 [6, 90,    1, 1, 0,      0, 35],
                 [7, 180,   0, 1, 0,      d_rho2, 35],
                 [2, -90,   0, 0, d_rho1, d_rho2, 0],
                 [2, 0,     1, 0, d_rho1, d_rho2, 0],
                 [4, -90,   1, 1, d_rho1, d_rho2, 0],
                 [5, 90,    2, 1, d_rho1, 0, 35],
                 [6, 90,    1, 1, 0,      0, 35],
                 [7, 180,   0, 1, 0,      d_rho2, 35],
                 [2, -90,   0, 0, d_rho1, d_rho2, 0],
                 [2, 0,     1, 0, d_rho1, d_rho2, 0],
                 [4, -90,   1, 1, d_rho1, d_rho2, 0],
                 [5, 90,    2, 1, d_rho1, 0, 35],
                 [6, 90,    1, 1, 0,      0, 35],
                 [7, 180,   0, 1, 0,      d_rho2, 35]]
                                  
                    

#untuk ambil data kalibrasi kecepatan                                
movementRobot5  = [[1, -90, 0, 0,      0, d_rho2], 
                  [2,   0, 0, 1, d_rho1, d_rho2], 
                  [3, -90, 1, 1, d_rho1, d_rho2], 
                  [4, 0, 1, 2, d_rho1, d_rho1]]
                

"""
-
--
 ->
"""    

#new world                  
movementRobot6 = [[4, -90, 0, 0, 0, d_rho2], 
                 [5, 0,   1, 0, d_rho1, 0], 
                 [6, 0,   1, 1, 0, 0], 
                 [7, 0,   1, 2, 0, 0], 
                 [8, -90, 1, 3, 0, 0], 
                 [12, -90,2, 3, d_rho2, d_rho2], 
                 [13, 0,  3, 3, d_rho1, 0], 
                 [14, 0,  3, 4, 0, 0], 
                 [15, -90, 3, 5, d_rho2, d_rho1]]
                 
#new world with P_max                  
movementRobot7 = [[4, -90, 0, 0, 0, d_rho2,0,9], 
                 [5, 0,   1, 0, d_rho1, 0,0,0], 
                 [6, 0,   1, 1, 0, 0,0,0], 
                 [7, 0,   1, 2, 0, 0,0,0], 
                 [8, -90, 1, 3, 0, 0,0,0], 
                 [12, -90,2, 3, d_rho2, d_rho2,9,9], 
                 [13, 0,  3, 3, d_rho1, 0,0,0], 
                 [14, 0,  3, 4, 0, 0,0,0], 
                 [15, -90,  3, 5, 0, d_rho1,0,0]]

                  
map_posisi = [0.4375,0.3125,0.1875,0.0625,0.0625,0.1875,0.3125]  

#posisi tes nilai referensi kota 1             
movementRobot9a = [[4, 90, 0, 0, 0, 0],
                 [4, 90, 0, 0, 0, 0], 
                 [5, 90,   1, 0, d_rho1, d_rho1]]

#posisi tes nilai referensi kotak 2             
movementRobot9b = [[4, 90, 0, 0, 0, 0],
                 [4, 90, 0, 0, d_rho2, d_rho2]]

#trial jalan pendek                 
movementRobot9c = [[4, 90, 0, 0, 0, 0],
                 [4, 90, 0, 0, d_rho2, d_rho2], 
                 [5, 90,   1, 0, d_rho1, d_rho1]]
                 

#Meja                 
movementRobot = [[13, 90, 7, 1, 0, 0],
                [12, 90, 6, 1, 0, 0],
                [11, 90, 5, 1, 0, 0],
                [10, 90, 4, 1, 0, 0],
                [9, 90, 3, 1, 0, 0],
                [8, 90, 2, 1, 0, 0],
                [7, 90, 1, 1, d_rho2, d_rho2],
                [6, 0, 0, 1, d_rho1, 0],
                [5, 0, 0, 2, 0, 0],
                [4, 0, 0, 3, 0, 0],
                [3, 0, 0, 4, 0, 0],
                [2, 0, 0, 5, 0, 0],
                [1, 0, 0, 6, d_rho2, d_rho2],
                [0, 0, 0, 7, d_rho1, d_rho1]]
                


                 
def printmap(grid):
    for i in range(len(grid)):
        print(grid[i])

def cartesianRobot (movementRobot):
    if movementRobot == 0:
        pos = [0,1]
    elif movementRobot == 90:
        pos = [-1,0]
    elif movementRobot == 180:
        pos = [0,-1]
    elif movementRobot == -90:
        pos = [1,0]
    return pos