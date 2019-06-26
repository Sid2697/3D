import pptk
import numpy
y = pptk.rand(1000,3)
xyz = pptk.rand(10000, 3)
v = pptk.viewer(xyz)
# attr1 = pptk.rand(100)     # 100 random scalars
# attr2 = pptk.rand(100, 3)  # 100 random RGB colors

v2 = pptk.viewer(y)
v.attributes(y+1000, xyz)

