CC = g++
FLAGS = -Wall -g -std=c++11
MAIN = lab1exe
LIBS = 
INCLUDES = 
OBJS = lab1.o
 
$(MAIN):$(OBJS)
	$(CC) $(FLAGS) -g -o $(MAIN) $(OBJS) $(LIBS)
 
%.o: %.cpp $(INCLUDES)
	$(CC) $(FLAGS) -c -o $@ $<
 
clean:
	rm -f $(OBJS) $(MAIN)