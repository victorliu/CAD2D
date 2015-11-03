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
	print('/Times-Roman findfont 0.2 scalefont setfont')
	print('72 72 scale')
	print('4.25 6.5 translate')
	print('0.02 setlinewidth')

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
	print('/ctext { % x y string\n' ..
		'3 dict begin\n' ..
		'/string exch def\n' ..
		'/y exch def\n' ..
		'/x exch def\n' ..
		'0 0 moveto\n' ..
		'string dup stringwidth pop\n' ..
		'2 div\n' ..
		'x exch sub\n' ..
		'y moveto\n' ..
		'show\n' ..
		'end\n' ..
		'} def\n'
	)
	print('/textheight {\n' ..
		'gsave                                  % save graphic context\n' ..
		'{                            \n' ..
		'	100 100 moveto                     % move to some point \n' ..
		'	(HÍpg) true charpath pathbbox      % gets text path bounding box (LLx LLy URx URy)\n' ..
		'	exch pop 3 -1 roll pop             % keeps LLy and URy\n' ..
		'	exch sub                           % URy - LLy\n' ..
		'}\n' ..
		'stopped                                % did the last block fail?\n' ..
		'{\n' ..
		'	pop pop                            % get rid of "stopped" junk\n' ..
		'	currentfont /FontMatrix get 3 get  % gets alternative text height\n' ..
		'}\n' ..
		'if\n' ..
		'grestore                               % restore graphic context\n' ..
		'} bind def\n' ..
		'/textextents{ % str -> width height\n' ..
		'	0 0 moveto\n' ..
		'	true charpath pathbbox % xmin ymin xmax ymax\n' ..
		'	3 -1 roll sub          % xmin xmax ymax-ymin\n' ..
		'	3 1 roll sub neg exch\n' ..
		'} bind def'
	)
end
function OutputFooter()
	print('showpage')
end

function OutputText(arg)
	if CAD2D.IsPoint(arg.at) then
		print('gsave', arg.at.x, arg.at.y, 'translate')
		print(string.format('(%s)',
			string.gsub(tostring(arg[1]), '([%(%)])', '\\%1')
		))
		if arg.textplacement == 'center' then
			print('dup textextents exch -0.5 mul exch -0.5 mul translate')
		elseif arg.textplacement == 'below' then
			print('dup textextents exch -0.5 mul exch neg translate')
		elseif arg.textplacement == 'above' then
			print('dup textextents exch -0.5 mul exch pop 0 translate')
		elseif arg.textplacement == 'belowleft' then
			print('dup textextents exch -1 mul exch neg translate')
		elseif arg.textplacement == 'belowright' then
			print('dup textextents exch pop 0 exch neg translate')
		elseif arg.textplacement == 'aboveleft' then
			print('dup textextents exch -1 mul exch pop 0 translate')
		elseif arg.textplacement == 'left' then
			print('dup textextents exch -1 mul exch -0.5 mul translate')
		elseif arg.textplacement == 'right' then
			print('dup textextents exch pop 0 exch -0.5 mul translate')
		else -- aboveright
			-- do nothing
		end
		print('0 0 moveto show grestore')
	else
		error('No location specified')
	end
end
function PolygonCenter(arg)
	-- future support for other centers
	-- default currently is by vertices
	if not CAD2D.IsPoly(arg[1]) then
		error('PolygonCenter expected a Poly')
	end
	local p = arg[1]
	local cx = 0
	local cy = 0
	for i = 1,p.n do
		cx = cx + p[i].x
		cy = cy + p[i].y
	end
	return point(cx / p.n, cy / p.n)
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
function OutputVector(arg)
	if not CAD2D.IsVector(arg[1]) then
		error('OutputVector expected a Vector')
	end
	local v = arg[1]
	if CAD2D.IsPoint(arg.at) then
		print('gsave', arg.at.x, arg.at.y, 'translate')
		print('0 0 moveto')
		print(v.x, v.y, 'arrowto')
		print('grestore')
	else
		print('0 0 moveto')
		print(v.x, v.y, 'arrowto')
	end
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

function LabelDimension(arg)
	if not CAD2D.IsPoint(arg[1]) then
		error('LabelDimension expected a Point')
	end
	if not CAD2D.IsPoint(arg[2]) then
		error('LabelDimension expected a Point')
	end
	if not CAD2D.IsVector(arg.offset) then
		error('LabelDimension expected a Vector')
	end
	local u = dir(arg.offset.rot)
	local l = math.abs(dist(ray(arg[1], u), arg[2]))
	local d = dir(arg.offset)
	local p1 = arg[1] + arg.offset
	local p2 = arg[2] + arg.offset
	local d12 = dist(ray(arg[1], d), arg[2]);
	if d12 >= 0 then -- p2 ahead of p1
		p1 = p1 + d12*d
	else
		p2 = p2 - d12*d
	end
	local m = seg(p1,p2)[0.5]
	OutputRay{ray(m,  u), length=0.5*l}
	OutputRay{ray(m, -u), length=0.5*l}
	local str = string.format('%g', l)
	local placement = arg.textplacement
	if not placement then
		local a = math.deg(arg.offset.angle)
		if -22.5 <= a and a <= 22.5 then
			placement = 'right'
		elseif 22.5 <= a and a <= 67.5 then
			placement = 'aboveright'
		elseif 67.5 <= a and a <= 112.5 then
			placement = 'above'
		elseif 112.5 <= a and a <= 157.5 then
			placement = 'aboveleft'
		elseif -67.5 <= a and a <= -22.5 then
			placement = 'belowright'
		elseif -112.5 <= a and a <= -67.5 then
			placement = 'below'
		elseif -157.5 <= a and a <= -112.5 then
			placement = 'belowleft'
		else
			placement = 'left'
		end
	end
	if CAD2D.IsVector(arg.textoffset) then
		OutputText{str, at=m+arg.textoffset, textplacement=placement}
	else
		OutputText{str, at=m, textplacement=placement}
	end
	print('gsave currentlinewidth 0.5 mul setlinewidth')
	print(arg[1].x, arg[1].y, 'moveto', p1.x, p1.y, 'lineto stroke')
	print(arg[2].x, arg[2].y, 'moveto', p2.x, p2.y, 'lineto stroke')
	print('grestore')
end

function LabelPoint(arg)
	if not CAD2D.IsPoint(arg[1]) then
		error('LabelPoint expected a Point')
	end
	local p = arg[1]
	local str = string.format('(%g, %g)', p.x, p.y)
	if CAD2D.IsVector(arg.textoffset) then
		local to = p+arg.textoffset
		OutputVector{-arg.textoffset, at=to}
		local a = math.deg(arg.textoffset.angle)
		local placement = nil
		if -22.5 <= a and a <= 22.5 then
			placement = 'right'
		elseif 22.5 <= a and a <= 67.5 then
			placement = 'aboveright'
		elseif 67.5 <= a and a <= 112.5 then
			placement = 'above'
		elseif 112.5 <= a and a <= 157.5 then
			placement = 'aboveleft'
		elseif -67.5 <= a and a <= -22.5 then
			placement = 'belowright'
		elseif -112.5 <= a and a <= -67.5 then
			placement = 'below'
		elseif -157.5 <= a and a <= -112.5 then
			placement = 'belowleft'
		else
			placement = 'left'
		end
		OutputText{str, at=to, textplacement=placement}
	else
		OutputText{str, at=p,centered=true}
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

