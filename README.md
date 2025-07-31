# aircraft_GNC

Assume we have airspeed measurements (from pitot tube or equivalent sensor), which help separate true airspeed from ground-relative velocity.

State Vector
𝑥
=
[
𝑝
𝑥
,
𝑝
𝑦
,
𝑝
𝑧
,
  
𝑣
𝑥
,
𝑣
𝑦
,
𝑣
𝑧
,
  
𝑞
0
,
𝑞
1
,
𝑞
2
,
𝑞
3
,
  
𝑏
𝑎
𝑥
,
𝑏
𝑎
𝑦
,
𝑏
𝑎
𝑧
,
  
𝑏
𝑔
𝑥
,
𝑏
𝑔
𝑦
,
𝑏
𝑔
𝑧
,
  
𝑤
𝑥
,
𝑤
𝑦
]
𝑇
x=[p 
x
​
 ,p 
y
​
 ,p 
z
​
 ,v 
x
​
 ,v 
y
​
 ,v 
z
​
 ,q 
0
​
 ,q 
1
​
 ,q 
2
​
 ,q 
3
​
 ,b 
a
x
​
 ,b 
a
y
​
 ,b 
a
z
​
 ,b 
g
x
​
 ,b 
g
y
​
 ,b 
g
z
​
 ,w 
x
​
 ,w 
y
​
 ] 
T
 
Position (3)

Velocity in NED (3)

Attitude quaternion (4)

Accelerometer bias (3)

Gyro bias (3)

Wind (horizontal components: 
𝑤
𝑥
,
𝑤
𝑦
w 
x
​
 ,w 
y
​
 )

