from collections import deque
import heapq
import State as st

class Agent(object):
	def SearchSolution(self, state):
		return []
		
class AgentSnake(Agent):   

	def is_valid_position(self, state, position, visited):
		return ((0 <= position.X < state.maze.WIDTH) and (0 <= position.Y < state.maze.HEIGHT) and ((position.X, position.Y) not in visited) and (state.maze.MAP[position.Y][position.X] != -1))

	def Astar_cost(self, current_position, food_position, initial_pos):
		# Manhattan distance heuristic
		heuristic = abs(current_position.X - food_position.X) + abs(current_position.Y - food_position.Y)
		# step cost for A*
		sc = abs(current_position.X - initial_pos[0]) + abs(current_position.Y - initial_pos[1])
		return sc + heuristic

	def _heuristic(self, current_position, food_position):
		# Manhattan distance heuristic
		return abs(current_position.X - food_position.X) + abs(current_position.Y - food_position.Y)
	
	def step_cost(self, current_position):
    	# step cost for A*
		return abs(current_position.X - 10) + abs(current_position.Y - 10)

	def SearchSolution(self, state):
		FoodX = state.FoodPosition.X
		FoodY = state.FoodPosition.Y

		HeadX = state.snake.HeadPosition.X #L
		HeadY = state.snake.HeadPosition.Y #T
		
		DR = FoodY - HeadY
		DC = FoodX - HeadX
		
		plan = []
		
		F = -1
		if(DR == 0 and state.snake.HeadDirection.X*DC < 0):
			plan.append(0)
			F = 6
			
		if(state.snake.HeadDirection.Y*DR < 0):
			plan.append(3)
			if(DC == 0):
				F = 9
			else:
				DC = DC - 1
		Di = 6
		if(DR < 0):
			Di = 0
			DR = -DR
		for i in range(0,int(DR)):
			plan.append(Di)
		Di = 3
		if(DC < 0):
			Di = 9
			DC = -DC
		for i in range(0,int(DC)):
			plan.append(Di)
		if(F > 0):
			plan.append(F)
			F = -1
			
		return plan
	
	def showAgent():
		print("A Snake Solver By MB")


	# Implementation of DFS algorithm to find the best move for snake
	def SearchSolutionDFS(self, state):
			# Extract snake and food positions
			snake_head = state.snake.HeadPosition
			food_position = state.FoodPosition

			# Initialize visited set to avoid revisiting cells
			visited = set()

			# Perform DFS starting from the snake's head position
			path = self.dfs(state, snake_head, food_position, visited)

			return path

	# helper DFS function
	def dfs(self, state, current_position, food_position, visited):
			# Check if current position is out of bounds or visited
			if not self.is_valid_position(state, current_position, visited):
				return None

			# Add current position to visited set
			visited.add((current_position.X, current_position.Y))

			# Check if current position is food
			if current_position.X == food_position.X and current_position.Y == food_position.Y:
				return []

			# Define directions: up, down, left, right
			directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]
			actions = [0, 6, 9, 3]

			# Explore neighbors in all directions
			for i, (dx, dy) in enumerate(directions):
				new_position = st.Vector(current_position.X + dx, current_position.Y + dy)
				path = self.dfs(state, new_position, food_position, visited)
				if path is not None:
					return [actions[i]] + path

			return None
	
	#Implementation of Astar Algorithm
	def SearchSolutionAstar(self, state):
		# Extract snake and food positions
		snake_head = state.snake.HeadPosition
		food_position = state.FoodPosition

		# Initialize priority queue (heap) to store nodes to be explored
		initial_sc = 0
		heap = [(initial_sc + self._heuristic(snake_head, food_position),self._heuristic(snake_head, food_position), (snake_head.X, snake_head.Y), [])]

		# Initialize visited set to avoid revisiting cells
		visited = set()

		# Perform A* algorithm
		path = self.astar(state, food_position, heap, visited)

		return path

	# Helper astar algo
	def astar(self, state, food_position, heap, visited):
		snake_head = state.snake.HeadPosition
		initial_pos = (snake_head.X, snake_head.Y)
		while heap:
			# Pop node with minimum cost from heap
			cost, heuristic, current_position, path = heapq.heappop(heap)

			# Check if current position is food
			if current_position[0] == food_position.X and current_position[1] == food_position.Y:
				return path

			# Add current position to visited set
			visited.add((current_position[0], current_position[1]))

			# Define directions: up, down, left, right
			directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]
			actions = [0, 6, 9, 3]

			# Explore neighbors in all directions
			for i, (dx, dy) in enumerate(directions):
					new_position = st.Vector(current_position[0] + dx, current_position[1] + dy)
					if self.is_valid_position(state, new_position, visited):
						new_cost = self.Astar_cost(new_position, food_position, initial_pos)
						new_heuristic = self._heuristic(new_position, food_position)
						# Include a tie-breaker to make vectors comparable
						tie_breaker = (new_position.X, new_position.Y)
						# Check if the new node has lower heuristic cost and total cost
						heapq.heappush(heap, (new_cost, new_heuristic, tie_breaker, path + [actions[i]]))

		return None

	# Implementation of GBFS  Algorithm
	def SearchSolutionGBFS(self, state):
		# Extract snake and food positions
		snake_head = state.snake.HeadPosition
		food_position = state.FoodPosition

		# Initialize priority queue (heap) to store nodes to be explored
		heap = [(self._heuristic(snake_head, food_position), (snake_head.X, snake_head.Y), [])]

		# Initialize visited set to avoid revisiting cells
		visited = set()

		# Perform gbfs algorithm
		path = self.gbfs(state, food_position, heap, visited)

		return path

	# helper gbfs function
	def gbfs(self, state, food_position, heap, visited):
		while heap:
			# Pop node with minimum cost from heap
			cost, current_position, path = heapq.heappop(heap)

			# Check if current position is food
			if current_position[0] == food_position.X and current_position[1] == food_position.Y:
				return path

			# Add current position to visited set
			visited.add((current_position[0], current_position[1]))

			# Define directions: up, down, left, right
			directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]
			actions = [0, 6, 9, 3]

			# Explore neighbors in all directions
			for i, (dx, dy) in enumerate(directions):
					new_position = st.Vector(current_position[0] + dx, current_position[1] + dy)
					if self.is_valid_position(state, new_position, visited):
						new_cost = self._heuristic(new_position, food_position)
						# Include a tie-breaker to make vectors comparable
						tie_breaker = (new_position.X, new_position.Y)
						heapq.heappush(heap, (new_cost, tie_breaker, path + [actions[i]]))

		return None
	
	# Implementation of BFS Algorithm
	def SearchSolutionBFS(self, state):
		# Extract snake and food positions
		snake_head = state.snake.HeadPosition
		food_position = state.FoodPosition

		# Initialize queue to store nodes to be explored
		queue = deque([((snake_head.X,snake_head.Y), [])])

		# Initialize visited set to avoid revisiting cells
		visited = set()

		# Perform BFS algorithm
		path = self.bfs(state, food_position, queue, visited)

		return path

	# Helper BFS function
	def bfs(self, state, food_position, queue, visited):
		if not queue:
			return None  # Base case: if queue is empty, return None

		# Dequeue node from the front of the queue
		current_position, path = queue.popleft()

		# Check if current position is food
		if current_position[0] == food_position.X and current_position[1] == food_position.Y:
			return path

		# Add current position to visited set
		visited.add((current_position[0], current_position[1]))

		# Define directions: up, down, left, right
		directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]
		actions = [0, 6, 9, 3]

		# Explore neighbors in all directions
		for i, (dx, dy) in enumerate(directions):
			new_position = st.Vector(current_position[0] + dx, current_position[1] + dy)
			if self.is_valid_position(state, new_position, visited):
				tie_breaker = (new_position.X, new_position.Y)
				queue.append((tie_breaker, path + [actions[i]]))
				# Add new position to visited set immediately to avoid revisiting
				visited.add((new_position.X, new_position.Y))

		# Recursive call to explore the next node in the queue
		return self.bfs(state, food_position, queue, visited)
