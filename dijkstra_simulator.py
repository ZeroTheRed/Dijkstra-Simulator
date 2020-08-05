import matplotlib.pyplot as plt
import pprint
import map_loader


def dijkstra(occ_map, source, destination):
    # Set visual = False if you don't want to view the graphics
    visual = True

    # initialize occupancy grid visual plot
    colormap = (0, 5)
    viz_map = occ_map
    fig = plt.figure(figsize=(5, 5))
    ax = fig.add_subplot(111)
    ax.set_title('Occupancy Grid')
    plt.xticks(visible=False)
    plt.yticks(visible=False)
    plt.imshow(viz_map, origin='upper', interpolation='none', clim=colormap)
    ax.set_aspect('equal')
    plt.pause(1)

    occ_map2 = occ_map.tolist()
    start = source
    goal = destination

    viz_map[start[0]][start[1]] = 3
    viz_map[goal[0]][goal[1]] = 4
    plt.imshow(viz_map, origin='upper', interpolation='none', clim=colormap)
    plt.pause(1)

    node_dists = [[0 for row in range(len(occ_map2[0]))] for col in range(len(occ_map2))]
    pprint.pprint(node_dists)

    cost = 1                        # all connecting edges have a weight of 1
    g_value = 0                     # count of the number of steps taken from the starting node
    row = start[0]
    col = start[1]
    frontier_nodes = [(g_value, row, col)]
    parent_node = {}

    nav = [[-1, 0],  # go up
           [0, -1],  # go left
           [1, 0],  # go down
           [0, 1]]  # go right

    GOAL_FOUND = False

    while len(frontier_nodes) is not 0:
        frontier_nodes.sort(reverse=True)  # sort such as the smallest node is the last element
        current_node = frontier_nodes.pop()  # pop out the smallest node and assign it to current_node
        if current_node[1] == goal[0] and current_node[2] == goal[1]:
            GOAL_FOUND = True
            if visual == True:
                plt.text(2, 10, s='Goal Found!', fontsize=18, ha='center', va='top')
                plt.imshow(viz_map, origin='upper', interpolation='none', clim=colormap)
                plt.pause(1)
            break

        g_value, row, col = current_node

        for i in nav:
            possible_x = row + i[0]
            possible_y = col + i[1]

            if viz_map[possible_x][possible_y] != 1:  # hitting an obstacle
                unsearched_node = node_dists[possible_x][possible_y] == 0 and viz_map[possible_x][possible_y] != 3
                if unsearched_node:
                    node_dists[possible_x][possible_y] = 1
                    scanned_node = (g_value + cost, possible_x, possible_y)
                    frontier_nodes.append(scanned_node)
                    if visual == True:
                        viz_map[possible_x][possible_y] = 2
                        plt.imshow(viz_map, origin='upper', interpolation='none', clim=colormap)
                        plt.pause(0.1)

                    parent_node[scanned_node] = current_node

    if GOAL_FOUND == True:

        route = []
        child_node = current_node
        while child_node in parent_node:
            route.append(parent_node[child_node])
            child_node = parent_node[child_node]
            route.sort()

        pprint.pprint(route)

        if visual == True:
            for i in range(0, len(route)):
                viz_map[route[i][1]][route[i][2]] = 5
                plt.imshow(viz_map, origin='upper', interpolation='none', clim=colormap)
                plt.pause(.01)

            viz_map[goal[0]][goal[1]] = 5
            plt.imshow(viz_map, origin='upper', interpolation='none', clim=colormap)
            plt.pause(1)


img_path = "D:/test_map1.jpg"

test_map1 = map_loader.img_to_array(img_path)
print(test_map1)
start1 = (1, 13)
goal1 = (9, 8)

dijkstra(test_map1, start1, goal1)

'''
test_map1 = np.array([
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
])
start1 = (1,2)
goal1 = (5,8)


test_map2 = np.array([
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 1, 0, 0, 0, 1, 0, 1],
    [1, 0, 0, 0, 0, 1, 0, 0, 0, 1],
    [1, 0, 1, 1, 1, 1, 1, 0, 1, 1],
    [1, 0, 0, 0, 0, 0, 1, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 1, 1, 0, 1],
    [1, 0, 0, 0, 1, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
])
start2 = (6,1)
goal2 = (1,8)

dijkstra(test_map1, start1, goal1)

dijkstra(test_map2, start2, goal2)
'''
