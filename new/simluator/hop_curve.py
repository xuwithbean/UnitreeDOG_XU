# 把这段代码转成python
T1 = 90
T2 = 110
T3 = 300
t1 = [i/T1 for i in range(90+1)]
t2 = [i/T2 for i in range(110+1)]
t3 = [i/T3 for i in range(300+1)]
start_x = 9.41
start_y = -7
start_z = -10
back_y = -12
back_z = -10
hop_y = -16
hop_z = -30
end_y = 10
end_z = -20
bz_y1 = -12
bz_z1 = -20
bz_y2 = -10
bz_z2 = -3
print("\n\n\n\n\n")
'''
#hop back
X=[start_y+t*t*(back_y-start_y) for t in t1]
Z=[start_z+t*t*(back_z-start_z) for t in t1]
'''
'''
#do hop
X=[back_y+t*t*(hop_y-back_y) for t in t2]
Z=[back_z+t*t*(hop_z-back_z) for t in t2]
'''
#'''
#收腿
X = [hop_y * (1 - t) * (1 - t) * (1 - t) + 3 * bz_y1 * t  * (1 - t) * (1 - t) + 3 * bz_y2 * (t) * (t ) * (1 - t ) + end_y * (t) * (t) * (t) for t in t3]
Z = [hop_z * (1 - t) * (1 - t) * (1 - t) + 3 * bz_z1 * t * (1 - t) * (1 - t) + 3 * bz_z2 * (t) * (t) * (1 - t) + end_z * (t) * (t) * (t) for t in t3]
for i in range(len(X)):
    if (X[i]**2 + Z[i]**2)**0.5 > 0.38:
        print("so far")
        print(f"x:{X[i]},z:{Z[i]}")
    if (X[i]**2 + Z[i]**2)**0.5 < 0.15:
        print("so close")
        print(f"x:{X[i]},z:{Z[i]}")
#'''


import matplotlib.pyplot as plt
plt.plot(X,Z)
plt.show()
