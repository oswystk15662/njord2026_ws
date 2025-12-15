# qiita記事のグラフを使う
movable_matrix = [
    [0, 10, 16, 12, 0, 0, 0],
    [0, 0, 18, 0, 4, 0, 0],
    [0, 0, 0, 0, 1, 0, 0],
    [0, 0, 3, 0, 5, 0, 0],
    [0, 0, 0, 0, 0, 0, 21],
    [0, 0, 0, 0, 0, 0, 9],
    [0, 0, 0, 0, 0, 0, 0]
]







reached_position = [0]
path_cost = [0]

# これ使って、cost_map並び替えたりすると速くできる？r
# awの長さでfor文の回数を変えて、rawの中身みてやれば早そう 
zero_index = list(int) 

current_pos = 0

while reached_position[-1] != 6:
    min_cost = 1000
    a = 0

    # 次に移動するノードを探す
    for i in range(len(movable_matrix[reached_position[-1]])):
        zero_index_raw = []
        if movable_matrix[reached_position[-1]][i] != 0:
            if movable_matrix[reached_position[-1]][i] < min_cost:
                min_cost = movable_matrix[reached_position[-1]][i]
                a = i
        else:
            zero_index_raw.append(i)
    zero_index[current_pos] = zero_index_raw
    reached_position.append(a)
    current_pos = a
    path_cost[a] += movable_matrix[reached_position[-1]][a]

    for i in 7:
        if movable_matrix[reached_position[-1]][i] != 0:
            if movable_matrix[reached_position[-1]][i] < min_cost:
                min_cost = movable_matrix[reached_position[-1]][i]
                a = i
            elif movable_matrix[reached_position[0]][i] < min_cost:
                min_cost = movable_matrix[reached_position[0]][i]
                a = i
                reached_position
        else:
            zero_index_raw.append(i)
    