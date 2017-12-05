CXX=clang++
CXXFLAGS=-g -std=c++11 -Wall -pedantic -I/usr/local/include -Werror

LDFLAGS=-L/usr/local/lib -lopencv_core -lopencv_imgcodecs -lopencv_video -lopencv_imgproc -lboost_filesystem -lopencv_highgui -lboost_system -lopencv_ximgproc -lboost_regex

BINDIR=build.host
SRC=main.cpp
OBJ=$(BINDIR)/${SRC:.cpp=.o}
BIN=$(BINDIR)/robocupannotator

$(OBJ): $(SRC)
	$(CXX) $(SRC) $(CXXFLAGS) -c -o $@

host: _dummy $(OBJ)
	$(CXX) -o $(BIN) $(OBJ) $(LDFLAGS)

_dummy:
	mkdir -p build.host

clean:
	rm -f $(BINDIR)/*.*
