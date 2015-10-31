#include <cmath>
#include <limits>
#include <vector>
extern "C" {
#include "Cgeom/geom_la.h"
#include "Cgeom/geom_arc.h"
}

namespace CAD2D{

struct Point{
	double x, y;
private:
	Point(){}
public:
	Point(double x, double y):x(x),y(y){}
	Point(const Point &p):x(p.x),y(p.y){}
	static Point Infinity(){
		return Point(
			std::numeric_limits<double>::quiet_NaN(),
			std::numeric_limits<double>::quiet_NaN()
		);
	}
	Point& operator=(const Point &p){
		x = p.x; y = p.y;
		return *this;
	}
};

double Distance(const Point &p, const Point &q){
	return hypot(p.x - q.x, p.y - q.y);
}
struct Matrix{
	double m[6];
public:
	Matrix(){
		m[0] = 1; m[1] = 0;
		m[2] = 0; m[3] = 1;
		m[4] = 0; m[5] = 0;
	}
	Matrix(double s, double r, double tx, double ty){
		double cs = cos(r);
		double sn = sin(r);
		m[0] = s* cs; m[1] = s*sn;
		m[2] = s*-sn; m[3] = s*cs;
		m[4] = tx;    m[5] = ty;
	}
};

struct Direction{
	double x, y;
private:
	Direction():
		x(std::numeric_limits<double>::quiet_NaN()),
		y(std::numeric_limits<double>::quiet_NaN())
	{
	}
public:
	Direction(double angle):x(cos(angle)),y(sin(angle)){}
	Direction(double x, double y):x(x),y(y){
		double a = hypot(x,y);
		x /= a; y /= a;
	}
	Direction(const Point &p, const Point &q){
		x = q.x-p.x;
		y = q.y-p.y;
		double a = hypot(x,y);
		x /= a; y /= a;
	}
	Direction(const Direction &d):x(d.x), y(d.y){}
	static Direction Infinity(){ return Direction(); }
	Direction& operator=(const Direction &p){
		x = p.x; y = p.y;
		return *this;
	}
	Direction operator-() const{
		return Direction(-x,-y);
	}
	Direction operator!() const{
		return Direction(-y, x);
	}
	double Angle() const{
		return atan2(y, x);
	}
};


double Angle(const Direction &d1, const Direction &d2){
	double cosq = d1.x*d2.x + d1.y*d2.y;
	double sinq = d1.x*d2.y - d1.y*d2.x;
	return atan2(sinq, cosq);
}

struct Vector{
	double x, y;
public:
	Vector(double x, double y):x(x),y(y){}
	Vector(const Point &p, const Point &q):x(q.x-p.x),y(q.y-p.y){}
	Vector(const Direction &d, const double &t = 1):x(d.x*t),y(d.y*t){}
	Vector& operator=(const Vector &v){
		x = v.x; y = v.y; return *this;
	}
	Vector operator-() const{
		return Vector(-x,-y);
	}
	Vector operator!() const{
		return Vector(-y,x);
	}
	double Angle() const{
		return atan2(y,x);
	}
	double Length() const{
		return hypot(x,y);
	}
};
Point operator+(const Point &p, const Vector &v){
	return Point(p.x+v.x, p.y+v.y);
}
Point operator-(const Point &p, const Vector &v){
	return Point(p.x-v.x, p.y-v.y);
}
Vector operator+(const Vector &u, const Vector &v){
	return Vector(u.x+v.x, u.y+v.y);
}
Vector operator-(const Vector &u, const Vector &v){
	return Vector(u.x-v.x, u.y-v.y);
}
Vector operator-(const Point &u, const Point &v){
	return Vector(u.x-v.x, u.y-v.y);
}
Vector operator*(const double &s, const Vector &v){
	return Vector(s*v.x, s*v.y);
}
Vector operator*(const Vector &v, const double &s){
	return Vector(s*v.x, s*v.y);
}
Vector operator/(const Vector &v, const double &s){
	return Vector(v.x / s, v.y / s);
}
double Dot(const Vector &u, const Vector &v){
	return u.x*v.x+u.y*v.y;
}
double Cross(const Vector &u, const Vector &v){
	return u.x*v.y-u.y*v.x;
}

Vector operator*(const double &s, const Direction &v){
	return Vector(s*v.x, s*v.y);
}
Vector operator*(const Direction &v, const double &s){
	return Vector(s*v.x, s*v.y);
}

struct Ray{
	Point p;
	Direction d;
private:
	Ray():p(Point::Infinity()),d(Direction::Infinity()){}
public:
	Ray(const Point &p, const Direction &d):p(p),d(d){}
	Ray(const Point &p, const Vector &v):p(p),d(Direction(v.x,v.y)){}
	Ray(const Ray &r):p(r.p), d(r.d){}
	Ray& operator=(const Ray &r){
		p = r.p; d = r.d;
		return *this;
	}
	Direction GetDirection() const{ return d; }
	Point GetPoint() const{ return p; }
};

struct Arcseg{
	Point p, q;
	double g;
private:
	Arcseg():p(Point::Infinity()),q(Point::Infinity()),g(0){}
public:
	Arcseg(const Point &p, const Point &q, double g = 0):p(p),q(q),g(g){}
	Arcseg(const Point &p, const Point &q, const Point &m):p(p),q(q){
		const double a[2] = {p.x,p.y};
		const double b[2] = {q.x,q.y};
		const double c[2] = {m.x,m.y};
		g = geom_arc_g_from_pt(a, b, c);
	}
	Arcseg(const Arcseg &s):p(s.p), q(s.q), g(s.g){}
	Arcseg& operator=(const Arcseg &s){
		p = s.p; q = s.q; g = s.g;
		return *this;
	}
	Arcseg operator-() const{
		return Arcseg(q, p, -g);
	}
	Ray operator[](const double &s) const{
		const double a[2] = {p.x,p.y};
		const double b[2] = {q.x,q.y};
		double p[2], t[2];
		geom_arc_param(a, b, g, s, p, t);
		return Ray(Point(p[0], p[1]), Direction(t[0], t[1]));
	}
	double Length() const{
		const double a[2] = {p.x,p.y};
		const double b[2] = {q.x,q.y};
		return geom_arc_length(a, b, g);
	}
	Point Center() const{
		Point M(0.5*p.x + 0.5*q.x, 0.5*p.y + 0.5*q.y);
		Direction mN(!Direction(p, q));
		double t = Distance(p, M);
		double d = t*(1-g*g)/(2*g);
		return Point(M.x + d*mN.x, M.y + d*mN.y);
	}
	double Radius() const{
		double t = 0.5 * Distance(p, q);
		return t*(1+g*g)/(2*g);
	}
	double Angle() const{
		return 2*atan(g);
	}
};

struct Poly{
	typedef std::pair<Point,double> PointG;
	std::vector<PointG> v;
private:
	Poly(){}
public:
	Poly(const std::vector<Point> &p){
		for(std::vector<Point>::const_iterator i = p.begin(); i != p.end(); ++i){
			v.push_back(PointG(*i,0));
		}
	}
	Poly(const std::vector<PointG> &v):v(v){}
	Poly(const Poly &p):v(p.v){}
	Poly& operator=(const Poly &p){
		v = p.v;
		return *this;
	}
	int NumVertices() const{ return (int)v.size(); }
	Point operator[](int i) const{
		int n = v.size();
		i = (i%n);
		if(i < 0){ i += n; }
		return v[i].first;
	}
	Arcseg operator()(int i) const{
		int n = v.size();
		i = (i%n);
		if(i < 0){ i += n; }
		int j = (i+1)%n;
		return Arcseg(v[i].first, v[j].first, v[i].second);
	}
};

double Distance(const Ray &r, const Point &p){
	// (p - r.p) . d
	return (p.x - r.p.x) * r.d.x + (p.y - r.p.y) * r.d.y;
}

Point Intersection(const Ray &u, const Ray &v){
	std::vector<Point> pi;

	// Return intersection point, if any
	// up + s * ud == vp + t * vd
	// s * vd x ud == vd x (vp - up)
	// s == (vd x (vp - up)) / (vd x ud)
	double uvx = v.p.x - u.p.x;
	double uvy = v.p.y - u.p.y;
	double num = v.d.x * uvy - v.d.y * uvx;
	double den = v.d.x * u.d.y - v.d.y * u.d.x;
	if(0 == den){ return Point::Infinity(); }
	double s = num/den;
	return Point(u.p.x + s * u.d.x, u.p.y + s * u.d.y);
}

} // namespace CAD2D

extern "C" {
#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>
}

using namespace CAD2D;

const char PointClassName[] = "CAD2D::Point";
const char DirectionClassName[] = "CAD2D::Direction";
const char VectorClassName[] = "CAD2D::Vector";
const char RayClassName[] = "CAD2D::Ray";
const char ArcsegClassName[] = "CAD2D::Arcseg";
const char PolyClassName[] = "CAD2D::Poly";

static int Point_push(lua_State *L, const CAD2D::Point &p){
	CAD2D::Point *P = (CAD2D::Point*)lua_newuserdata(L, sizeof(CAD2D::Point));
	P = new(P) CAD2D::Point(p);
	luaL_getmetatable(L, PointClassName);
	lua_setmetatable(L, -2);
	return 1;
}
static int Point_create(lua_State *L){
	double x = 0, y = 0;
	if(lua_gettop(L) == 2){
		x = luaL_checknumber(L, 1);
		y = luaL_checknumber(L, 2);
	}else if(lua_gettop(L) == 1 && lua_istable(L, 1) && lua_rawlen(L, 1) == 2){
		lua_pushinteger(L, 1);
		lua_gettable(L, 1);
		x = luaL_checknumber(L, -1);
		lua_pop(L, 1);
		lua_pushinteger(L, 2);
		lua_gettable(L, 1);
		y = luaL_checknumber(L, -1);
		lua_pop(L, 1);
	}else{
		luaL_error(L, "Invalid syntax for creating a Point");
	}

	return Point_push(L, CAD2D::Point(x,y));
}
static bool Point_is(lua_State *L, int narg){
	return (lua_type(L, narg) == LUA_TUSERDATA) && (NULL != luaL_testudata(L, narg, PointClassName));
}
static int IsPoint(lua_State *L){
	lua_pushboolean(L, Point_is(L, 1));
	return 1;
}
static CAD2D::Point *Point_check(lua_State *L, int narg){
	luaL_checktype(L, narg, LUA_TUSERDATA);
	void *ud = luaL_checkudata(L, narg, PointClassName);
	if(!ud){
		luaL_argerror(L, narg, "expected Point object");
		return NULL;
	}
	return (CAD2D::Point*)ud;
}
static int Point_gc(lua_State *L) {
	Point_check(L, 1);
	return 0;
}
static int Point_index(lua_State *L) {
	CAD2D::Point *P = Point_check(L, 1);
	if(lua_isnumber(L, 2)){
		if(1 == lua_tointeger(L, 2)){
			lua_pushnumber(L, P->x);
		}else if(2 == lua_tointeger(L, 2)){
			lua_pushnumber(L, P->y);
		}else{
			return luaL_error(L, "Invalid indexing of a Point");
		}
	}else if(lua_isstring(L, 2)){
		if(0 == strcmp("x", lua_tostring(L, 2))){
			lua_pushnumber(L, P->x);
		}else if(0 == strcmp("y", lua_tostring(L, 2))){
			lua_pushnumber(L, P->y);
		}else{
			return luaL_error(L, "Invalid indexing of a Point");
		}
	}else{
		return luaL_error(L, "Invalid indexing of a Point");
	}
	return 1;
}




static int Direction_push(lua_State *L, const CAD2D::Direction &d){
	CAD2D::Direction *D = (CAD2D::Direction*)lua_newuserdata(L, sizeof(CAD2D::Direction));
	D = new(D) CAD2D::Direction(d);
	luaL_getmetatable(L, DirectionClassName);
	lua_setmetatable(L, -2);
	return 1;
}
static int Direction_create(lua_State *L){
	if(lua_gettop(L) == 2){
		if(Point_is(L, 1) && Point_is(L, 2)){
			const CAD2D::Point *p = Point_check(L, 1);
			const CAD2D::Point *q = Point_check(L, 2);
			return Direction_push(L, CAD2D::Direction(*p,*q));
		}else if(lua_isnumber(L, 1) && lua_isnumber(L, 2)){
			return Direction_push(L, CAD2D::Direction(lua_tonumber(L, 1), lua_tonumber(L, 2)));
		}
	}else if(lua_gettop(L) == 1 && lua_istable(L, 1)){
		lua_pushinteger(L, 1);
		lua_gettable(L, 1);
		double x = luaL_checknumber(L, -1);
		lua_pop(L, 1);
		lua_pushinteger(L, 2);
		lua_gettable(L, 1);
		double y = luaL_checknumber(L, -1);
		lua_pop(L, 1);
		return Direction_push(L, CAD2D::Direction(x, y));
	}else if(lua_gettop(L) == 1 && lua_isnumber(L, 1)){
		return Direction_push(L, CAD2D::Direction(lua_tonumber(L, 1)));
	}
	return luaL_error(L, "Invalid syntax for creating a Direction");
}
static bool Direction_is(lua_State *L, int narg){
	return (lua_type(L, narg) == LUA_TUSERDATA) && (NULL != luaL_testudata(L, narg, DirectionClassName));
}
static int IsDirection(lua_State *L){
	lua_pushboolean(L, Direction_is(L, 1));
	return 1;
}
static CAD2D::Direction *Direction_check(lua_State *L, int narg){
	luaL_checktype(L, narg, LUA_TUSERDATA);
	void *ud = luaL_checkudata(L, narg, DirectionClassName);
	if(!ud){
		luaL_argerror(L, narg, "expected Direction object");
		return NULL;
	}
	return (CAD2D::Direction*)ud;
}
static int Direction_gc(lua_State *L) {
	Direction_check(L, 1);
	return 0;
}
static int Direction_index(lua_State *L) {
	CAD2D::Direction *D = Direction_check(L, 1);
	if(lua_isnumber(L, 2)){
		if(1 == lua_tointeger(L, 2)){
			lua_pushnumber(L, D->x);
			return 1;
		}else if(2 == lua_tointeger(L, 2)){
			lua_pushnumber(L, D->y);
			return 1;
		}
	}else if(lua_isstring(L, 2)){
		if(0 == strcmp("x", lua_tostring(L, 2))){
			lua_pushnumber(L, D->x);
			return 1;
		}else if(0 == strcmp("y", lua_tostring(L, 2))){
			lua_pushnumber(L, D->y);
			return 1;
		}else if(0 == strcmp("angle", lua_tostring(L, 2))){
			lua_pushnumber(L, D->Angle());
			return 1;
		}else if(0 == strcmp("rot", lua_tostring(L, 2))){
			return Direction_push(L, !(*D));
		}
	}
	return luaL_error(L, "Invalid indexing of a Direction");
}




static int Vector_push(lua_State *L, const CAD2D::Vector &d){
	CAD2D::Vector *D = (CAD2D::Vector*)lua_newuserdata(L, sizeof(CAD2D::Vector));
	D = new(D) CAD2D::Vector(d);
	luaL_getmetatable(L, VectorClassName);
	lua_setmetatable(L, -2);
	return 1;
}
static int Vector_create(lua_State *L){
	if(lua_gettop(L) == 2){
		if(Point_is(L, 1) && Point_is(L, 2)){
			const CAD2D::Point *p = Point_check(L, 1);
			const CAD2D::Point *q = Point_check(L, 2);
			return Vector_push(L, CAD2D::Vector(*p,*q));
		}else if(lua_isnumber(L, 1) && lua_isnumber(L, 2)){
			return Vector_push(L, CAD2D::Vector(lua_tonumber(L, 1), lua_tonumber(L, 2)));
		}else if(Direction_is(L, 1) && lua_isnumber(L, 2)){
			const CAD2D::Direction *d = Direction_check(L, 1);
			return Vector_push(L, CAD2D::Vector(*d, lua_tonumber(L, 2)));
		}
	}else if(lua_gettop(L) == 1 && lua_istable(L, 1)){
		lua_pushinteger(L, 1);
		lua_gettable(L, 1);
		double x = luaL_checknumber(L, -1);
		lua_pop(L, 1);
		lua_pushinteger(L, 2);
		lua_gettable(L, 1);
		double y = luaL_checknumber(L, -1);
		lua_pop(L, 1);
		return Vector_push(L, CAD2D::Vector(x, y));
	}
	return luaL_error(L, "Invalid syntax for creating a Vector");
}
static bool Vector_is(lua_State *L, int narg){
	return (lua_type(L, narg) == LUA_TUSERDATA) && (NULL != luaL_testudata(L, narg, VectorClassName));
}
static int IsVector(lua_State *L){
	lua_pushboolean(L, Vector_is(L, 1));
	return 1;
}
static CAD2D::Vector *Vector_check(lua_State *L, int narg){
	luaL_checktype(L, narg, LUA_TUSERDATA);
	void *ud = luaL_checkudata(L, narg, VectorClassName);
	if(!ud){
		luaL_argerror(L, narg, "expected Vector object");
		return NULL;
	}
	return (CAD2D::Vector*)ud;
}
static int Vector_gc(lua_State *L) {
	Vector_check(L, 1);
	return 0;
}
static int Vector_index(lua_State *L) {
	CAD2D::Vector *V = Vector_check(L, 1);
	if(lua_isnumber(L, 2)){
		if(1 == lua_tointeger(L, 2)){
			lua_pushnumber(L, V->x);
			return 1;
		}else if(2 == lua_tointeger(L, 2)){
			lua_pushnumber(L, V->y);
			return 1;
		}
	}else if(lua_isstring(L, 2)){
		if(0 == strcmp("x", lua_tostring(L, 2))){
			lua_pushnumber(L, V->x);
			return 1;
		}else if(0 == strcmp("y", lua_tostring(L, 2))){
			lua_pushnumber(L, V->y);
			return 1;
		}else if(0 == strcmp("length", lua_tostring(L, 2))){
			lua_pushnumber(L, V->Length());
			return 1;
		}else if(0 == strcmp("angle", lua_tostring(L, 2))){
			lua_pushnumber(L, V->Angle());
			return 1;
		}else if(0 == strcmp("rot", lua_tostring(L, 2))){
			return Vector_push(L, !(*V));
		}
	}
	return luaL_error(L, "Invalid indexing of a Vector");
}



static int Ray_push(lua_State *L, const CAD2D::Ray &r){
	CAD2D::Ray *R = (CAD2D::Ray*)lua_newuserdata(L, sizeof(CAD2D::Ray));
	R = new(R) CAD2D::Ray(r);
	luaL_getmetatable(L, RayClassName);
	lua_setmetatable(L, -2);
	return 1;
}
static int Ray_create(lua_State *L){
	if(lua_gettop(L) == 2){
		if(Point_is(L, 1) && Direction_is(L, 2)){
			const CAD2D::Point *p = Point_check(L, 1);
			const CAD2D::Direction *d = Direction_check(L, 2);
			return Ray_push(L, CAD2D::Ray(*p,*d));
		}else if(Point_is(L, 1) && Vector_is(L, 2)){
			const CAD2D::Point *p = Point_check(L, 1);
			const CAD2D::Vector *v = Vector_check(L, 2);
			return Ray_push(L, CAD2D::Ray(*p,*v));
		}
	}
	return luaL_error(L, "Invalid syntax for creating a Ray");
}
static bool Ray_is(lua_State *L, int narg){
	return (lua_type(L, narg) == LUA_TUSERDATA) && (NULL != luaL_testudata(L, narg, RayClassName));
}
static int IsRay(lua_State *L){
	lua_pushboolean(L, Ray_is(L, 1));
	return 1;
}
static CAD2D::Ray *Ray_check(lua_State *L, int narg){
	luaL_checktype(L, narg, LUA_TUSERDATA);
	void *ud = luaL_checkudata(L, narg, RayClassName);
	if(!ud){
		luaL_argerror(L, narg, "expected Ray object");
		return NULL;
	}
	return (CAD2D::Ray*)ud;
}
static int Ray_gc(lua_State *L) {
	Ray_check(L, 1);
	return 0;
}
static int Ray_index(lua_State *L) {
	CAD2D::Ray *R = Ray_check(L, 1);
	if(lua_isstring(L, 2)){
		if(0 == strcmp("direction", lua_tostring(L, 2))){
			return Direction_push(L, R->d);
		}else if(0 == strcmp("origin", lua_tostring(L, 2))){
			return Point_push(L, R->p);
		}else{
			return luaL_error(L, "Invalid indexing of a Ray");
		}
	}else{
		return luaL_error(L, "Invalid indexing of a Ray");
	}
	return 1;
}





static int Arcseg_push(lua_State *L, const CAD2D::Arcseg &s){
	CAD2D::Arcseg *S = (CAD2D::Arcseg*)lua_newuserdata(L, sizeof(CAD2D::Arcseg));
	S = new(S) CAD2D::Arcseg(s);
	luaL_getmetatable(L, ArcsegClassName);
	lua_setmetatable(L, -2);
	return 1;
}
static int Arcseg_create(lua_State *L){
	const CAD2D::Point *p = NULL;
	const CAD2D::Point *q = NULL;
	const CAD2D::Point *m = NULL;
	double g;
	if(lua_gettop(L) == 2 && (p = Point_check(L, 1)) && (q = Point_check(L, 2))){
		return Arcseg_push(L, CAD2D::Arcseg(*p,*q));
	}else if(lua_gettop(L) == 3 && (p = Point_check(L, 1)) && (q = Point_check(L, 2)) && lua_isnumber(L, 3)){
		g = lua_tonumber(L, 3);
		return Arcseg_push(L, CAD2D::Arcseg(*p,*q,g));
	}else if(lua_gettop(L) == 3 && (p = Point_check(L, 1)) && (q = Point_check(L, 2)) && (m = Point_check(L, 3))){
		return Arcseg_push(L, CAD2D::Arcseg(*p,*q,*m));
	}
	return luaL_error(L, "Invalid syntax for creating a Arcseg");
}
static bool Arcseg_is(lua_State *L, int narg){
	return (lua_type(L, narg) == LUA_TUSERDATA) && (NULL != luaL_testudata(L, narg, ArcsegClassName));
}
static int IsArcseg(lua_State *L){
	lua_pushboolean(L, Arcseg_is(L, 1));
	return 1;
}
static CAD2D::Arcseg *Arcseg_check(lua_State *L, int narg){
	luaL_checktype(L, narg, LUA_TUSERDATA);
	void *ud = luaL_checkudata(L, narg, ArcsegClassName);
	if(!ud){
		luaL_argerror(L, narg, "expected Arcseg object");
		return NULL;
	}
	return (CAD2D::Arcseg*)ud;
}
static int Arcseg_gc(lua_State *L) {
	Arcseg_check(L, 1);
	return 0;
}
static int Arcseg_index(lua_State *L){
	CAD2D::Arcseg *S = Arcseg_check(L, 1);
	if(lua_isnumber(L, 2)){
		Ray r = (*S)[lua_tonumber(L, 2)];
		int ret = Point_push(L, r.p);
		ret += Direction_push(L, r.d);
		return ret;
	}else if(lua_isstring(L, 2)){
		if(0 == strcmp("length", lua_tostring(L, 2))){
			lua_pushnumber(L, S->Length());
			return 1;
		}else if(0 == strcmp("center", lua_tostring(L, 2))){
			return Point_push(L, S->Center());
		}else if(0 == strcmp("radius", lua_tostring(L, 2))){
			lua_pushnumber(L, S->Radius());
			return 1;
		}else if(0 == strcmp("angle", lua_tostring(L, 2))){
			lua_pushnumber(L, S->Angle());
			return 1;
		}
	}
	return luaL_error(L, "Invalid indexing of a Arcseg");
}
static int geom_arc_ray_intersect(
	const double a1[2], const double b1[2], double g,
	const double a[2], const double ab[2], double pret[4]
){
	// Intesrect circle and line
	double c[2], r, theta[2];
	geom_arc_circle(a1, b1, g, c, &r, theta);



	/* Let the intersection points be parameterized by
	 *   p = a + t*(b-a)
	 * Then we must have
	 *   [ (a-c) + t*(b-a) ]^2 = r^2
	 *   ca^2 - r^2 + 2*t*ca*ab + t^2*ab^2 = 0
	 */
	int ncirc = 0;
	double t[2];
	const double ca[2] = { a[0]-c[0], a[1]-c[1] };
	const double ab2 = ab[0]*ab[0] + ab[1]*ab[1];
	const double p = (ca[0]*ab[0] + ca[1]*ab[1]) / ab2;
	const double q = (ca[0]*ca[0] + ca[1]*ca[1] - r*r) / ab2;
	/* Solve t^2 + 2*t*p + q = 0 */
	double disc = p*p-q;
	if(disc == 0){
		t[0] = -p;
		ncirc = 1;
	}else if(disc < 0){
		return 0;
	}else{
		disc = sqrt(disc);
		t[0] = -p - disc;
		t[1] = -p + disc;
		ncirc = 2;
	}

	int ret = 0;
	for(int i = 0; i < ncirc; ++i){ // filter results to see if they are on segment
		if(g >= 0){
			if(theta[1] < theta[0]){ theta[1] += 2*M_PI; }
			double ttheta = atan2(a[1] + t[i]*ab[1] - c[1], a[0] + t[i]*ab[0] - c[0]);
			if(ttheta < theta[0]){ ttheta += 2*M_PI; }
			if(ttheta < theta[1]){
				pret[2*ret+0] = a[0] + t[i]*ab[0];
				pret[2*ret+1] = a[1] + t[i]*ab[1];
				ret++;
			}
		}else{
			if(theta[1] > theta[0]){ theta[1] -= 2*M_PI; }
			double ttheta = atan2(a[1] + t[i]*ab[1] - c[1], a[0] + t[i]*ab[0] - c[0]);
			if(ttheta > theta[0]){ ttheta -= 2*M_PI; }
			if(ttheta > theta[1]){
				pret[2*ret+0] = a[0] + t[i]*ab[0];
				pret[2*ret+1] = a[1] + t[i]*ab[1];
				ret++;
			}
		}
	}
	return ret;
}
std::vector<Point> Intersection(const Ray &r, const Arcseg &s){
	const double a1[2] = {s.p.x, s.p.y};
	const double b1[2] = {s.q.x, s.q.y};
	const double a2[2] = {r.p.x, r.p.y};
	const double b2[2] = {r.d.x, r.d.y};
	double p[4];
	int n = geom_arc_ray_intersect(a1, b1, s.g, a2, b2, p);
	std::vector<Point> ret;
	for(int i = 0; i < n; ++i){
		ret.push_back(Point(p[2*i+0], p[2*i+1]));
	}
	return ret;
}
/*
std::vector<Point> Intersection(const Arcseg &u, const Arcseg &v){
	const double a1[2] = {u.p.x, u.p.y};
	const double b1[2] = {u.q.x, u.q.y};
	const double a2[2] = {v.p.x, v.p.y};
	const double b2[2] = {v.q.x, v.q.y};
	double p[4];
	int n = geom_arc_arc_intersect(a1, b1, u.g, a2, b2, v.g, p);
	std::vector<Point> ret;
	for(int i = 0; i < n; ++i){
		ret.push_back(Point(p[2*i+0], p[2*i+1]));
	}
	return ret;
}
*/





static int Poly_push(lua_State *L, const CAD2D::Poly &p){
	CAD2D::Poly *P = (CAD2D::Poly*)lua_newuserdata(L, sizeof(CAD2D::Poly));
	P = new(P) CAD2D::Poly(p);
	luaL_getmetatable(L, PolyClassName);
	lua_setmetatable(L, -2);
	return 1;
}
static int Poly_create(lua_State *L){
	std::vector<CAD2D::Point> p;
	const int narg = lua_gettop(L);
	if(narg >= 3){
		if(Arcseg_is(L, 1)){
			/// TODO
		}else{
			for(int i = 1; i <= narg; ++i){
				CAD2D::Point *pp = Point_check(L, i);
				p.push_back(*pp);
			}
		}
	}else if(narg == 1 && lua_istable(L, 1)){
		int ntab = lua_rawlen(L, 1);
		for(int i = 1; i <= ntab; ++i){
			lua_pushinteger(L, i);
			lua_gettable(L, 1);
			CAD2D::Point *pp = Point_check(L, -1);
			p.push_back(*pp);
			lua_pop(L, 1);
		}
	}else{
		luaL_error(L, "Invalid syntax for creating a Poly");
	}
	return Poly_push(L, CAD2D::Poly(p));
}
static bool Poly_is(lua_State *L, int narg){
	return (lua_type(L, narg) == LUA_TUSERDATA) && (NULL != luaL_testudata(L, narg, PolyClassName));
}
static int IsPoly(lua_State *L){
	lua_pushboolean(L, Poly_is(L, 1));
	return 1;
}
static CAD2D::Poly *Poly_check(lua_State *L, int narg){
	luaL_checktype(L, narg, LUA_TUSERDATA);
	void *ud = luaL_checkudata(L, narg, PolyClassName);
	if(!ud){
		luaL_argerror(L, narg, "expected Poly object");
		return NULL;
	}
	return (CAD2D::Poly*)ud;
}
static int Poly_gc(lua_State *L) {
	Poly_check(L, 1);
	return 0;
}
static int Poly_arcseg(lua_State *L){
	CAD2D::Poly *P = Poly_check(L, 1);
	int i = luaL_checkint(L, 2);
	Arcseg_push(L, (*P)(i-1));
	return 1;
}
static int Poly_index(lua_State *L) {
	CAD2D::Poly *P = Poly_check(L, 1);
	if(lua_isnumber(L, 2)){
		int i = lua_tointeger(L, 2);
		return Point_push(L, (*P)[i-1]);
	}else if(lua_isstring(L, 2)){
		if(0 == strcmp("n", lua_tostring(L, 2))){
			lua_pushinteger(L, P->NumVertices());
			return 1;
		}else if(0 == strcmp("arcseg", lua_tostring(L, 2))){
			lua_pushcfunction(L, &Poly_arcseg);
			return 1;
		}
	}
	return luaL_error(L, "Invalid indexing of a Poly");
}




static int Point_add(lua_State *L){
	const CAD2D::Point *p = Point_check(L, 1);
	const CAD2D::Vector *v = Vector_check(L, 2);
	return Point_push(L, (*p)+(*v));
}
static int Point_sub(lua_State *L){
	const CAD2D::Point *p = Point_check(L, 1);
	if(Point_is(L, 2)){
		const CAD2D::Point *q = Point_check(L, 2);
		return Vector_push(L, CAD2D::Vector(*p,*q));
	}else if(Vector_is(L, 2)){
		const CAD2D::Vector *v = Vector_check(L, 2);
		return Point_push(L, (*p)-(*v));
	}
	return luaL_error(L, "Invalid subtraction with Point");
}

static int Direction_mul(lua_State *L){
	double s;
	const CAD2D::Direction *d;
	if(lua_isnumber(L, 1)){
		s = luaL_checknumber(L, 1);
		d = Direction_check(L, 2);
	}else{
		d = Direction_check(L, 1);
		s = luaL_checknumber(L, 2);
	}
	return Vector_push(L, s*(*d));
}



static int Vector_len(lua_State *L){
	const CAD2D::Vector *v = Vector_check(L, 1);
	lua_pushnumber(L, v->Length());
	return 1;
}
static int Vector_unm(lua_State *L){
	const CAD2D::Vector *v = Vector_check(L, 1);
	return Vector_push(L, -(*v));
}
static int Vector_add(lua_State *L){
	const CAD2D::Vector *u = Vector_check(L, 1);
	const CAD2D::Vector *v = Vector_check(L, 2);
	return Vector_push(L, (*u)+(*v));
}
static int Vector_sub(lua_State *L){
	const CAD2D::Vector *u = Vector_check(L, 1);
	const CAD2D::Vector *v = Vector_check(L, 2);
	return Vector_push(L, (*u)-(*v));
}
static int Vector_mul(lua_State *L){
	double s;
	const CAD2D::Vector *v;
	if(lua_isnumber(L, 1)){
		s = luaL_checknumber(L, 1);
		v = Vector_check(L, 2);
	}else{
		v = Vector_check(L, 1);
		s = luaL_checknumber(L, 2);
	}
	return Vector_push(L, s*(*v));
}
static int Vector_div(lua_State *L){
	double s = luaL_checknumber(L, 2);
	const CAD2D::Vector *v = Vector_check(L, 1);
	return Vector_push(L, (*v) / s);
}
static int Vector_pow(lua_State *L){
	const CAD2D::Vector *u = Vector_check(L, 1);
	const CAD2D::Vector *v = Vector_check(L, 2);
	lua_pushnumber(L, Cross(*u,*v));
	return 1;
}
static int Vector_concat(lua_State *L){
	const CAD2D::Vector *u = Vector_check(L, 1);
	const CAD2D::Vector *v = Vector_check(L, 2);
	lua_pushnumber(L, Dot(*u,*v));
	return 1;
}



static int Circle_create(lua_State *L){
	std::vector<CAD2D::Point> p;
	const int narg = lua_gettop(L);
	if(narg == 2){
		CAD2D::Point *c = Point_check(L, 1);
		double radius = luaL_checknumber(L, 2);
		std::vector<CAD2D::Poly::PointG> v;
		v.push_back(CAD2D::Poly::PointG(CAD2D::Point(c->x + radius, c->y), 1));
		v.push_back(CAD2D::Poly::PointG(CAD2D::Point(c->x - radius, c->y), 1));
		return Poly_push(L, CAD2D::Poly(v));
	}else{
		return luaL_error(L, "Invalid syntax for creating a Circle");
	}
}

static int Distance_dispatch(lua_State *L){
	const int narg = lua_gettop(L);
	if(2 == narg){
		CAD2D::Point *p, *q;
		CAD2D::Ray *r;
		if((p = Point_check(L, 1)) && (q = Point_check(L, 2))){
			lua_pushnumber(L, Distance(*p, *q));
			return 1;
		}else if((r = Ray_check(L, 1)) && (p = Point_check(L, 2))){
			lua_pushnumber(L, Distance(*r, *p));
			return 1;
		}
	}
	return luaL_error(L, "Invalid call to Distance");
}
static int Angle_dispatch(lua_State *L){
	const int narg = lua_gettop(L);
	if(2 == narg){
		CAD2D::Direction *u, *v;
		if((u = Direction_check(L, 1)) && (v = Direction_check(L, 2))){
			lua_pushnumber(L, Angle(*u, *v));
			return 1;
		}
	}
	return luaL_error(L, "Invalid call to Angle");
}


static int Intersection_dispatch(lua_State *L){
	const int narg = lua_gettop(L);
	if(2 == narg){
		if(Ray_is(L,1) && Ray_is(L,2)){
			CAD2D::Ray *u = Ray_check(L, 1);
			CAD2D::Ray *v = Ray_check(L, 2);
			Point_push(L, Intersection(*u, *v));
			return 1;
		}else if(Ray_is(L,1) && Arcseg_is(L,2)){
			CAD2D::Ray *u = Ray_check(L, 1);
			CAD2D::Arcseg *s = Arcseg_check(L, 2);
			std::vector<CAD2D::Point> ret = Intersection(*u, *s);
			int n = 0;
			for(int i = 0; i < ret.size(); ++i){
				n += Point_push(L, ret[i]);
			}
			return n;
		}else if(Ray_is(L,2) && Arcseg_is(L,1)){
			CAD2D::Ray *u = Ray_check(L, 2);
			CAD2D::Arcseg *s = Arcseg_check(L, 1);
			std::vector<CAD2D::Point> ret = Intersection(*u, *s);
			int n = 0;
			for(int i = 0; i < ret.size(); ++i){
				n += Point_push(L, ret[i]);
			}
			return n;
		}/*else if(Arcseg_is(L,1) && Arcseg_is(L,2)){
			CAD2D::Arcseg *u = Arcseg_check(L, 1);
			CAD2D::Arcseg *v = Arcseg_check(L, 2);
			std::vector<CAD2D::Point> ret = Intersection(*u, *v);
			int n = 0;
			for(int i = 0; i < ret.size(); ++i){
				n += Point_push(L, ret[i]);
			}
			return n;
		}*/
	}
	return luaL_error(L, "Invalid call to Intersection");
}


void CAD2Dkernel_register(lua_State *L){
	static const luaL_Reg PointLib[] = {
		{"__gc", &Point_gc},
		{"__index", &Point_index},
		{"__add", &Point_add},
		{"__sub", &Point_sub},
		{NULL, NULL}
	};

	luaL_newmetatable(L, PointClassName);
	lua_pushvalue(L, -1);
	lua_setfield(L, -2, "__index");
	luaL_setfuncs(L, PointLib, 0);
	lua_pop(L, 1);  /* pop new metatable */

	static const luaL_Reg DirectionLib[] = {
		{"__gc", &Direction_gc},
		{"__index", &Direction_index},
		{"__mul", &Direction_mul},
		{NULL, NULL}
	};

	luaL_newmetatable(L, DirectionClassName);
	lua_pushvalue(L, -1);
	lua_setfield(L, -2, "__index");
	luaL_setfuncs(L, DirectionLib, 0);
	lua_pop(L, 1);  /* pop new metatable */

	static const luaL_Reg VectorLib[] = {
		{"__gc", &Vector_gc},
		{"__index", &Vector_index},
		{"__len", &Vector_len},
		{"__unm", &Vector_unm},
		{"__add", &Vector_add},
		{"__sub", &Vector_sub},
		{"__mul", &Vector_mul},
		{"__div", &Vector_div},
		{"__pow", &Vector_pow},
		{"__concat", &Vector_concat},
		{NULL, NULL}
	};

	luaL_newmetatable(L, VectorClassName);
	lua_pushvalue(L, -1);
	lua_setfield(L, -2, "__index");
	luaL_setfuncs(L, VectorLib, 0);
	lua_pop(L, 1);  /* pop new metatable */

	static const luaL_Reg RayLib[] = {
		{"__gc", &Ray_gc},
		{"__index", &Ray_index},
		{NULL, NULL}
	};

	luaL_newmetatable(L, RayClassName);
	lua_pushvalue(L, -1);
	lua_setfield(L, -2, "__index");
	luaL_setfuncs(L, RayLib, 0);
	lua_pop(L, 1);  /* pop new metatable */

	static const luaL_Reg ArcsegLib[] = {
		{"__gc", &Arcseg_gc},
		{"__index", &Arcseg_index},
		{NULL, NULL}
	};

	luaL_newmetatable(L, ArcsegClassName);
	lua_pushvalue(L, -1);
	lua_setfield(L, -2, "__index");
	luaL_setfuncs(L, ArcsegLib, 0);
	lua_pop(L, 1);  /* pop new metatable */

	static const luaL_Reg PolyLib[] = {
		{"__gc", &Poly_gc},
		{"__index", &Poly_index},
		{NULL, NULL}
	};

	luaL_newmetatable(L, PolyClassName);
	lua_pushvalue(L, -1);
	lua_setfield(L, -2, "__index");
	luaL_setfuncs(L, PolyLib, 0);
	lua_pop(L, 1);  /* pop new metatable */
}





extern "C" int luaopen_CAD2Dkernel(lua_State *L){
	static const luaL_Reg CAD2Dkernel_lib[] = {
		{"Point", &Point_create},
		{"Direction", &Direction_create},
		{"Vector", &Vector_create},
		{"Ray", &Ray_create},
		{"Arcseg", &Arcseg_create},
		{"Poly", &Poly_create},

		{"IsPoint", &IsPoint},
		{"IsDirection", &IsDirection},
		{"IsVector", &IsVector},
		{"IsRay", &IsRay},
		{"IsArcseg", &IsArcseg},
		{"IsPoly", &IsPoly},

		{"Circle", &Circle_create},

		{"Distance", &Distance_dispatch},
		{"Angle", &Angle_dispatch},
		{"Intersection", &Intersection_dispatch},

		{NULL, NULL}
	};
	luaL_newlib(L, CAD2Dkernel_lib);

	CAD2Dkernel_register(L);
	return 1;
}

