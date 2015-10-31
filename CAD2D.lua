CAD2D = require('CAD2Dkernel');

point = CAD2D.Point;
dir = CAD2D.Direction;
vec = CAD2D.Vector;
dist = CAD2D.Distance;
ray = CAD2D.Ray;
seg = CAD2D.Arcseg;
poly = CAD2D.Poly;
xsect = CAD2D.Intersection;
angle = CAD2D.Angle;
circle = CAD2D.Circle;

function OutputHeader()
	print('%!')
	print('/Times-Roman findfont 2 scalefont setfont')
	print('72 72 scale')
	print('4.25 6.5 translate')
	print('0.005 setlinewidth')

	print('/circle{ % r\n' ..
		'dup 0 0 moveto\n' ..
		'0 exch 0 exch 0 360 arc closepath\n' ..
	'} bind def\n')
	print('/arrowto { %tipx tipy *arrowto* ---\n' ..
		'<< %push mark on stack\n' ..
		'/tipy 3 -1 roll %arrow tip y coordinate\n' ..
		'/tipx 5 -1 roll %arrow tip x coordinate\n' ..
		'/tailx currentpoint %arrow start x coordinate\n' ..
		'/taily exch %arrow start y coordinate\n' ..
		'/tip 22.5 cos 22.5 sin div %arrow tip: 45 degree angle\n' ..
		'/headwidth 5 %head width is 5 line widths\n' ..
		'>> %create dictionary of "local variables"\n' ..
		'begin %push on dictionary stack\n' ..
		'/dx tipx tailx sub def %define dx along arrow\n' ..
		'/dy tipy taily sub def %define dy along arrow\n' ..
		'/angle dy dx atan def %arrow angle relaitve to origin\n' ..
		'/arrowlength\n' ..
		'dx dx mul dy dy mul add sqrt def %compute length of arrow\n' ..
		'/tiplength\n' ..
		'tip currentlinewidth 2 div mul def %compute length of arrow tip\n' ..
		'/base\n' ..
		'arrowlength tiplength sub %Compute where the arrowhead joins the tail.\n' ..
		'def\n' ..
		'/headlength\n' ..
		'tip headwidth mul neg %compute arrowhead length given head width\n' ..
		'def\n' ..
		'gsave %save graphics state\n' ..
		'currentpoint translate %translation to start of arrow\n' ..
		'angle rotate %rotate user space so arrow points right\n' ..
		'base 0 translate %translation to end of arrow base\n' ..
		'0 0 lineto stroke %construct and stroke line\n' ..
		'tiplength 0 translate %translation to tip of arrow\n' ..
		'0 0 moveto %move to tip of arrow\n' ..
		'currentlinewidth 2 div dup scale %scale to 1/2 linewidth\n' ..
		'headlength headwidth lineto %construct side of arrowhead\n' ..
		'headlength tip add 1\n' ..
		'headlength tip add -1\n' ..
		'headlength headwidth neg curveto %construct back of arrowhead\n' ..
		'closepath %construct side of arrowhead\n' ..
		'fill %fill arrowhead\n' ..
		'grestore %restore graphics state\n' ..
		'tipx tipy moveto %move current point to arrow tip\n' ..
		'end %pop dictionary fr dictionary stack\n' ..
		'} def %define the arrowto procedure\n'
	)
end
function OutputFooter()
	print('showpage')
end

function OutputText(arg)
	if CAD2D.IsPoint(arg.at) then
		print(string.format('%f %f moveto (%s) show',
			arg.at.x, arg.at.y,
			string.gsub(tostring(arg[1]), '([%(%)])', '\\%1')
		))
	else
		error('No location specified')
	end
end

function OutputPolygon(arg)
	if not CAD2D.IsPoly(arg[1]) then
		error('OutputPolygon expected a Poly')
	end
	local p = arg[1]
	print(p[1].x, p[1].y, 'moveto')
	for i = 2,p.n do
		print(p[i].x, p[i].y, 'lineto')
	end
	print('closepath')
end

function OutputRay(arg)
	if not CAD2D.IsRay(arg[1]) then
		error('OutputRay expected a Ray')
	end
	local r = arg[1]
	local l = 1
	if arg.length then
		l = arg.length
	end
	print(r.origin.x, r.origin.y, 'moveto')
	local tip = r.origin+l*r.direction
	print(tip.x, tip.y, 'arrowto')
end

function OutputRect(arg)
	local hx = arg[1]
	local hy = arg[2]
	local c = point(0,0)
	if CAD2D.IsPoint(arg.center) then
		c = arg.center
	end
	print(c.x - hx, c.y - hy, 'moveto')
	print(c.x + hx, c.y - hy, 'lineto')
	print(c.x + hx, c.y + hy, 'lineto')
	print(c.x - hx, c.y + hy, 'lineto closepath')
end

function OutputPoly(arg)
	if not CAD2D.IsPoly(arg[1]) then
		error('OutputPolygon expected a Poly')
	end
	local p = arg[1]
	print(p[1].x, p[1].y, 'moveto')
	local n = p.n
	for i = 1,n do
		local s = p:arcseg(i)
		if s.angle == 0 then
			-- do something
		else
			local sc = s.center
			local angle1 = dir(sc, s[0]).angle
			local angle2 = dir(sc, s[1]).angle
			if s.angle >= 0 then
				print(sc.x, sc.y, s.radius, math.deg(angle1), math.deg(angle2), 'arc')
			else
				print(sc.x, sc.y, s.radius, math.deg(angle1), math.deg(angle2), 'arcn')
			end
		end
	end
end

function Stroke()
	print('stroke')
end
function Fill()
	print('fill')
end

O = point(0,0)
x = dir(1,0)
y = dir(0,1)

