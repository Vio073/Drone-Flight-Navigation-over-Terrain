import heapq  # Untuk priority queue (min-heap)

# Grid 2D berisi nilai bobot tiap sel, 'S' = Start, 'G' = Goal, '#' = Rintangan (tidak bisa dilewati)
grid = [
    ['S', 1,   2,   '#', 3 ],
    [2,   '#', 3,   4,   2 ],
    [3,   2,   '#', 2,   3 ],
    [4,   3,   2,   1,  'G']
]

rows, cols = len(grid), len(grid[0])  # Ukuran grid

# Fungsi untuk mencari posisi karakter tertentu ('S' atau 'G') dalam grid
def find(char):
    for i in range(rows):
        for j in range(cols):
            if grid[i][j] == char:
                return (i, j)
    return None

# Heuristik menggunakan jarak Manhattan (cocok untuk grid 4 arah)
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# Fungsi untuk mendapatkan bobot/biaya sebuah sel di grid
def get_cost(pos):
    i, j = pos
    val = grid[i][j]
    if val == 'S' or val == 'G':
        return 0  # Start dan Goal dianggap tidak memiliki biaya
    return int(val)

# Implementasi algoritma A*
def a_star(start, goal):
    open_set = [(0, start)]  # Priority queue berisi (f_score, posisi)
    came_from = {}           # Untuk menyimpan jalur terbaik
    g_score = {start: 0}     # Biaya dari start ke posisi tertentu
    visited_nodes = 0        # Jumlah node yang dikunjungi

    while open_set:
        _, current = heapq.heappop(open_set)  # Ambil posisi dengan f_score terkecil
        visited_nodes += 1

        if current == goal:
            # Jika sudah sampai tujuan, rekonstruksi jalurnya
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1], visited_nodes  # Kembalikan jalur dari start ke goal

        # Periksa empat arah gerakan: atas, bawah, kiri, kanan
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            ni, nj = current[0]+dx, current[1]+dy  # Posisi tetangga
            # Cek apakah tetangga valid dan bukan rintangan
            if 0 <= ni < rows and 0 <= nj < cols and grid[ni][nj] != '#':
                neighbor = (ni, nj)
                cost = get_cost(neighbor)
                tentative_g = g_score[current] + cost
                # Jika biaya baru ke tetangga lebih kecil, update
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + heuristic(neighbor, goal)  # f(n) = g(n) + h(n)
                    heapq.heappush(open_set, (f, neighbor))

    return None, visited_nodes  # Tidak ditemukan jalur

# Menemukan posisi 'S' (start) dan 'G' (goal) di grid
start = find('S')
goal = find('G')

# Memanggil fungsi a_star
path, visited_nodes = a_star(start, goal)

# Menampilkan hasil jalur dan jumlah node yang dikunjungi
if path:
    print(f"Path from start to goal: {path}")
else:
    print("No path found")

print(f"Visited nodes: {visited_nodes}")
