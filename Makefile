compile: clean
	@gcc -c -std=c89 astar.c
	@gcc -o main astar.o main.c -lm

clean:
	@rm -f main astar.o
