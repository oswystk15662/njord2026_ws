class FlagCommandDijkstra:
    def __init__(self, movable_matrix, start_node, end_node):
        self.movable_matrix = movable_matrix
        self.start_node = start_node
        self.end_node = end_node
        
        self.connection_matrix = list(list(int))
        for m in len(movable_matrix):
            raw = []
            for n in len(movable_matrix[m]):
                if movable_matrix[m][n] != 0:
                    raw.append(n)
            self.connection_matrix[m] = raw

        self.path = { 
            f"path{i}" : [[0], self.connection_matrix[0][i]] for i in len(self.connection_matrix[0])
        }
    
    # return index, cost
    def get_nearest_node(self, node_number : int) -> list[int]:
        cost = 1000
        return_node_num = -1

        for n in self.connection_matrix[node_number]:
            if self.connection_matrix[node_number][n] <= cost:
                return_node_num = n
        return [return_node_num, cost]

    def advance_node(self, current_path_key : str) -> dict[list, int]:
        self.path[current_path_key] = {
            current_path_key : self.path[current_path_key][0].append(self.next_node_number),self.path[currrnt_path_key][1]+= self.next_node_cost        ]
        }
        [self.next_node_num, cost] = self.get_nearest_node(self.path[current_path_key][1])
        path_node_line = self.path[current_path_key][0].append(next_node_num)
        # 更新
        self.path[current_path_key] = [path_node_line, self.path[current_path_key][1] + cost]

    def make_disition(self):
        min_total_cost = 1000
        for path_key in self.path.keys:
            if min_total_cost > self.path[path_key][1]:
                min_total_cost = self.path[path_key][1]
                min_key_name = path_key
        
        self.advance_node(min_key_name)

class FlagCommandDijkstraNode:
    def __init__(self):
        # これらのリストはコストが小さい順に入っている。
        self.connected_node_indexes = []
        self.connected_node_costs = []
    def _sort(self):
        a=0
    def save_connection(self, movable_list):
        for i in len(movable_list):
            self.connected_node_indexes.append(i)
            self.connected_node_costs = movable_list[i]
    def on_path(self):

