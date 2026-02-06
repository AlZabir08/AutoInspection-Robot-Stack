import heapq
import math

class AStarPlanner:
    def __init__(self, start, goal, grid, blocked):
        self.start = start
        self.goal = goal
        self.grid = set(grid)
        self.blocked = set(blocked)

        # Grid bounds
        xs = [p[0] for p in grid]
        ys = [p[1] for p in grid]
        self.min_x, self.max_x = min(xs), max(xs)
        self.min_y, self.max_y = min(ys), max(ys)

    def heuristic(self, a, b):
        # Euclidean distance as heuristic
        return math.hypot(b[0] - a[0], b[1] - a[1])

    def get_neighbors(self, node):
        neighbors = []
        # 4-connected grid (up, down, left, right)
        directions = [(0.1, 0), (-0.1, 0), (0, 0.1), (0, -0.1)]
        for dx, dy in directions:
            nx, ny = round(node[0] + dx, 1), round(node[1] + dy, 1)
            neighbor = (nx, ny)
            if neighbor in self.grid and neighbor not in self.blocked:
                neighbors.append(neighbor)
        return neighbors

    def plan(self):
        open_set = []
        heapq.heappush(open_set, (0, self.start))

        came_from = {}
        g_score = {self.start: 0}
        f_score = {self.start: self.heuristic(self.start, self.goal)}

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == self.goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                tentative_g = g_score[current] + self.heuristic(current, neighbor)

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, self.goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        # No path found
        return None

    def reconstruct_path(self, came_from, current):
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        total_path.reverse()
        return total_path
