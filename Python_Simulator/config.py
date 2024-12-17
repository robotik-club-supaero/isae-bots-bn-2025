# Game configuration (Must be non-mutable)

game_name = "Game Name"
version = 1.0

r = 0.4
window_default_size = (int(r * 3000), int(r * 2000)) # 1px = 2 cm
fullscreen = False
fps = 60
fps_treshold = 0.5

global_duration = 60

nx, ny = window_default_size[0]//20, window_default_size[1]//20
cell_x, cell_y = window_default_size[0]/nx, window_default_size[1]/ny
