all: planner

Utils.o: Utils.cpp Utils.hpp
	g++ -c Utils.cpp

Node.o: Node.cpp Node.hpp
	g++ -c Node.cpp

Graph.o: Graph.cpp Graph.hpp
	g++ -c Graph.cpp

PRMSolver.o: PRMSolver.cpp PRMSolver.hpp
	g++ -c PRMSolver.cpp

planner.o: planner.cpp Node.o Graph.o PRMSolver.o
	g++ planner.cpp Utils.o Node.o Graph.o PRMSolver.o -o planner

clean:
	rm -f *.o *~ planner








# OUTPUTDIR := bin/

# CFLAGS := -std=c++14 -fvisibility=hidden -lpthread

# ifeq (,$(CONFIGURATION))
# 	CONFIGURATION := release
# endif

# ifeq (debug,$(CONFIGURATION))
# CFLAGS += -g
# else
# CFLAGS += -O3 -fopenmp 
# endif

# SOURCES := src/*.cpp
# HEADERS := src/*.h

# TARGETBIN := nbody-$(CONFIGURATION)

# .SUFFIXES:
# .PHONY: all clean

# all: $(TARGETBIN)

# $(TARGETBIN): $(SOURCES) $(HEADERS)
# 	$(CXX) -o $@ $(CFLAGS) $(SOURCES) 

# format:
# 	clang-format -i src/*.cpp src/*.h

# clean:
# 	rm -rf ./nbody-$(CONFIGURATION)

# check:	default
# 	./checker.pl

# FILES = src/*.cpp \
# 		src/*.h

# handin.tar: $(FILES)
# 	tar cvf handin.tar $(FILES)