class Heap:
	def __init__(self):
		self.heapList = [Node(-1,[-1,-1],None,-1)]
		self.currentSize = 0
	
	def percUp(self,i):
		while i // 2 > 0:
			if self.heapList[i].path_cost < self.heapList[i // 2].path_cost:
				tmp = self.heapList[i // 2]
				self.heapList[i // 2] = self.heapList[i]
				self.heapList[i] = tmp
			i = i // 2

	def insert(self,k):
		self.heapList.append(k)
		self.currentSize = self.currentSize + 1
		self.percUp(self.currentSize)

	def percDown(self,i):
		while (i * 2) <= self.currentSize:
			mc = self.minChild(i)
			if self.heapList[i].path_cost > self.heapList[mc].path_cost:
				tmp = self.heapList[i]
				self.heapList[i] = self.heapList[mc]
				self.heapList[mc] = tmp
			i = mc

	def minChild(self,i):
		if i * 2 + 1 > self.currentSize:
			return i * 2
		else:
			if self.heapList[i*2].path_cost < self.heapList[i*2+1].path_cost:
				return i * 2
			else:
				return i * 2 + 1

	def delMin(self):
		retval = self.heapList[1]
		self.heapList[1] = self.heapList[self.currentSize]
		self.currentSize = self.currentSize - 1
		self.heapList.pop()
		self.percDown(1)
		return retval

	def delete_node(self,node):
		ind = self.heapList.index(node)
		self.heapList[ind] = self.heapList[self.currentSize]
		self.currentSize = self.currentSize - 1
		self.heapList.pop()
		self.percDown(ind)
			

class mars_rover:
	def read_input_from_file(self,filename):
		file = open(filename, "r")
		inp = []
		i = 0
		for line in file:
			inp.append(line.strip())
			i += 1
		file.close()
		
		self.input = {}

		search_algo = inp[0]
		W = int(inp[1].split(' ')[0])
		H = int(inp[1].split(' ')[1])
		land_X = int(inp[2].split(' ')[0])
		land_Y = int(inp[2].split(' ')[1])
		Z_thresh = int(inp[3])
		N_targets = int(inp[4])
		targets = []
		map_terrain = []

		current_line_number = 5
		for i in range(N_targets):
			X = int(inp[current_line_number + i].split(' ')[0])
			Y = int(inp[current_line_number + i].split(' ')[1])
			targets.append([X,Y])

		current_line_number = 5 + N_targets
		for i in range(H):
			
			curr_row = []
			split_row = inp[current_line_number + i].split(' ')

			for j in range(W):
				elem = int(split_row[j])
				curr_row.append(elem)

			map_terrain.append(curr_row)

		self.input["search_algo"] = search_algo
		self.input["W"] = W
		self.input["H"] = H
		self.input["land_pos"] = [land_X,land_Y]
		self.input["Z_thresh"] = Z_thresh
		self.input["N_targets"] = N_targets
		self.input["targets"] = targets
		self.input["map_terrain"] = map_terrain
		
	def write_output_to_file(self,output,filename):
		f = open(filename, "w")

		for elem in output:
			if type(elem) is str:
				f.write(elem + '\n')
			elif type(elem) is list:
				for i in range(len(elem)-1):
					f.write(str(elem[i].position[0]) + ',' + str(elem[i].position[1]) + ' ')
				f.write(str(elem[len(elem)-1].position[0]) + ',' + str(elem[len(elem)-1].position[1]) + '\n')
		f.close()

	def remove_front(self, queue):
		front = queue[0]
		queue.pop(0)

		return front, queue
	
	def insert_node(self, node, queue):
		queue.append(node)
		return queue

	def check_if_state_exists(self, child,queue):
		child_x = child.position[0]
		child_y = child.position[1]

		for node in queue:
			if node.position[0] == child_x and node.position[1] == child_y:
				return node

		return None

	def delete_node(self,node, queue):
		if node in queue:
			queue.remove(node)

		return queue

	def accepted_children(self,curr_node, children, Z_thresh):
		acc_children = []

		for child in children:
			if abs(curr_node.value - child.value) <= Z_thresh:
				acc_children.append(child)

		return acc_children

	def BFS(self, start_state, search_space, goal_state, Z_thresh):

		rows = len(search_space)
		columns = len(search_space[0])

		root_node = Node(search_space[start_state[1]][start_state[0]], start_state)
		queue = [root_node]
		#closed = []
		closed = set()

		while True:
			if not queue:
				return None

			curr_node, queue = self.remove_front(queue)

			if curr_node.goal_test(goal_state):
				return curr_node

			children = curr_node.get_children(search_space, "BFS", goal_state)
			acceptable_children = self.accepted_children(curr_node,children,Z_thresh)
			
			while acceptable_children:
				child,acceptable_children = self.remove_front(acceptable_children)

				child_state_in_queue = self.check_if_state_exists(child,queue)
				child_state_in_closed = self.check_if_state_exists(child,closed)

				if (not child_state_in_queue) and (not child_state_in_closed):
					queue = self.insert_node(child,queue)

			closed.add(curr_node)

	def UCS(self, start_state, search_space, goal_state, Z_thresh):

		rows = len(search_space)
		columns = len(search_space[0])

		root_node = Node(search_space[start_state[1]][start_state[0]], start_state)

		queue = Heap() 
		queue.insert(root_node)

		closed = set()
		i = 0
		while True:
			if len(queue.heapList) < 2:
				return False
			
			curr_node = queue.delMin()

			if curr_node.goal_test(goal_state):
				return curr_node

			children = curr_node.get_children(search_space, "UCS", goal_state)
			acceptable_children = self.accepted_children(curr_node,children,Z_thresh)

			for child in acceptable_children:
				child_state_in_queue = self.check_if_state_exists(child,queue.heapList)

				child_state_in_closed = self.check_if_state_exists(child,closed)

				if (not child_state_in_queue) and (not child_state_in_closed):
					queue.insert(child) 

				elif child_state_in_queue:
					if child.path_cost < child_state_in_queue.path_cost:
						queue.delete_node(child_state_in_queue)
						queue.insert(child)	

				elif child_state_in_closed:
					if child.path_cost < child_state_in_closed.path_cost:
						closed.delete_node(child_state_in_closed)
						queue.insert(child)	

			closed.add(curr_node)

	def A_star(self, start_state, search_space, goal_state, Z_thresh):

		rows = len(search_space)
		columns = len(search_space[0])

		root_node = Node(search_space[start_state[1]][start_state[0]], start_state)
		queue = Heap() 
		queue.insert(root_node)

		closed = set()

		while True:
			if len(queue.heapList) < 2:
				return False

			curr_node = queue.delMin() 
			
			if curr_node.goal_test(goal_state):
				return curr_node
			
			children = curr_node.get_children(search_space, "A*", goal_state)
			acceptable_children = self.accepted_children(curr_node,children,Z_thresh)
			
			for child in acceptable_children:

				child_state_in_queue = self.check_if_state_exists(child,queue.heapList)
				child_state_in_closed = self.check_if_state_exists(child,closed)

				if (not child_state_in_queue) and (not child_state_in_closed):
					queue.insert(child)

				elif child_state_in_queue:
					if child.path_cost < child_state_in_queue.path_cost:
						queue.delete_node(child_state_in_queue)
						queue.insert(child)	

				elif child_state_in_closed:
					if child.path_cost < child_state_in_closed.path_cost:
						closed.remove(child_state_in_closed)
						queue.insert(child)	

			closed.add(curr_node)

	def sort_by_path_cost(self, array):
		newlist = sorted(array, key=lambda x: x.path_cost)
		return newlist			
			
	def run(self):

		self.read_input_from_file("input.txt")
		results = []

		for target in self.input["targets"]:
			# Call a function that gives the optimal path for one target site
			result = None
			if self.input["search_algo"] == "BFS":
				print("Using BFS")
				result = self.BFS(self.input["land_pos"],self.input["map_terrain"], target, self.input["Z_thresh"])

			elif self.input["search_algo"] == "UCS":
				print("Using UCS")
				result = self.UCS(self.input["land_pos"],self.input["map_terrain"], target, self.input["Z_thresh"])

			elif self.input["search_algo"] == "A*":
				print("Using A*")
				result = self.A_star(self.input["land_pos"],self.input["map_terrain"], target, self.input["Z_thresh"])

			print("OUTPUT: ")
			if result:
				seq = result.path_sequence()
				results.append(seq)
				for node in seq:
					#print(node.position)
					pass
			else:
				results.append("FAIL")
				print("FAIL")

		# Write the ouput to a file in the desired format
		self.write_output_to_file(results,"output.txt")

class Node:
	def __init__(self, value, state_pos, parent=None, path_cost=0):
		self.value = value
		self.position = state_pos
		self.parent = parent
		self.path_cost = path_cost
		self.depth = 0

		if parent:
			self.depth = parent.depth + 1

	def get_children(self,search_space,algo,soln_state):
		rows = len(search_space)
		columns = len(search_space[0])

		i = self.position[0]
		j = self.position[1]

		children = {"N":[i-1,j],"NE":[i-1,j+1],"E":[i,j+1],"SE":[i+1,j+1],"S":[i+1,j],"SW":[i+1,j-1],"W":[i,j-1],"NW":[i-1,j-1]}
		valid_children = []
		cost = {"BFS":{"N":1,"E":1,"S":1,"W":1,"NE":1,"NW":1,"SE":1,"SW":1},
				"UCS":{"N":10,"E":10,"S":10,"W":10,"NE":14,"NW":14,"SE":14,"SW":14},
				"A*":{"N":10,"E":10,"S":10,"W":10,"NE":14,"NW":14,"SE":14,"SW":14}}
		
		for key in children.keys():
			child = children[key]
			if (child[0] >=0 and child[0]< columns) and (child[1]>= 0 and child[1] < rows):
				curr_cost = cost[algo][key]
				
				if algo == "A*":
					Z_diff_cost = abs(search_space[j][i] - search_space[child[1]][child[0]])
					# This is h(n)
					h_cost = max(abs(soln_state[0]-child[0]),abs(soln_state[1]-child[1]))

					curr_cost = curr_cost + h_cost + Z_diff_cost

				valid_children.append(Node(search_space[child[1]][child[0]],child,self,self.path_cost + curr_cost))

		return valid_children

	def goal_test(self,goal_pos):
		if goal_pos:
			if self.position[0] == goal_pos[0] and self.position[1] == goal_pos[1]:
				return True
		return False

	def path_sequence(self):
		node = self
		path = []

		while node:
			path.append(node)
			node = node.parent

		# The path has the sequence from the node till the root.
		# But we need the sequence from the root to the current node.
		return list(reversed(path))

import time
start = time.time()

mars_rover_obj = mars_rover()
mars_rover_obj.run()

print('It took', time.time()-start, 'seconds.')
