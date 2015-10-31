CXX = c++
CFLAGS = -Wall -O2
LDFLAGS = -bundle -undefined dynamic_lookup -fpic
CGEOM_LIB = Cgeom/libgeom.a

LUA_INCLUDE = -I../S4/lua-5.2.4/install/include

all: CAD2Dkernel.so

$(CGEOM_LIB):
	cd Cgeom; make

CAD2Dkernel.so: $(CGEOM_LIB) CAD2Dkernel.cpp
	$(CXX) $(LDFLAGS) $(CFLAGS) $(LUA_INCLUDE) CAD2Dkernel.cpp $(CGEOM_LIB) -o CAD2Dkernel.so

