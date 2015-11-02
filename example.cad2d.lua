require('CAD2D')

-- draw a house with these basic dimensions
w = 0.7
h = 0.6

OutputHeader()

OutputRect{0.5*w,0.5*h, center=point(0,0.5*h)}; Stroke()
A = O - 0.5*w*x + h*y
B = O + 0.5*w*x + h*y

roofdir1 = dir(math.rad(30))
roofdir2 = dir(-roofdir1.x, roofdir1.y)

C = xsect(ray(A, roofdir1), ray(B, roofdir2))
OutputPolygon{poly{A,B,C}}; Stroke()

LabelPoint{C, textoffset=vec{-0.1,0.2}}
LabelDimension{O, C, offset = 0.9*x}

LabelPoint{A, textoffset=vec{-1,-2}}

OutputFooter()
