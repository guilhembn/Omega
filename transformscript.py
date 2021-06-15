import math
from mathutils import Matrix, Quaternion, Vector
import bpy


origin = bpy.data.objects["Origin"]
identity = Matrix()
origin.matrix_world = identity
empty1 = bpy.data.objects["Empty1"]
translate = Matrix.Translation((7.5, 10.7, 0.0))
empty1.matrix_local = translate

rotate = Matrix.Rotation(2.56, 4, "Z")
empty2 = bpy.data.objects["Empty2"]
empty2.matrix_local = rotate

translate2 = Matrix.Translation((2.7, 1.9, 1.79))
empty3 = bpy.data.objects["Empty3"]
empty3.matrix_local = translate2

homogeneous = Matrix([[-0.4727, 0.6627, -0.5809,  1.6660],
                      [0.2508, 0.7331,  0.6322, -3.2043],
                      [0.8448, 0.1532, -0.5127, -0.0320],
                      [0.0000, 0.0000,  0.0000,  1.0000]])


empty4 = bpy.data.objects["Empty4"]
empty4.matrix_local = homogeneous

x = 0.0
y = -0.3851
z = 0.3602
w = math.sqrt(1 - x ** 2 - y ** 2 - z ** 2)
rotate2q = Quaternion((w, x, y, z))
rotate2 = Matrix.Rotation(rotate2q.angle, 4, rotate2q.axis)

empty5 = bpy.data.objects["Empty5"]
empty5.matrix_local = rotate2
#print(empty5.matrix_local)

empty6 = bpy.data.objects["Empty6.002"]
translateX = 5.62
empty6.matrix_local = Matrix.Translation((translateX, 0.0, 0.0))

#loc, rot, scale = empty6.matrix_local.decompose()
#xaxis = Vector((1.0, 0.0, 0.0))

#print(loc)
#loc = loc.normalized()
#print(loc)



#normal = xaxis.cross(loc)
#print(normal)

#print(math.degrees(math.acos(xaxis.dot(loc))))

#rr = Quaternion((math.sqrt(1 - normal.x**2 - normal.y**2 - normal.z**2), normal.x, normal.y, normal.z))
#rrm = Matrix.Rotation(math.acos(xaxis.dot(loc)), 4, normal)
#empty5.matrix_local = rrm
#print(rrm.to_quaternion())

#empty6.matrix_world = wo