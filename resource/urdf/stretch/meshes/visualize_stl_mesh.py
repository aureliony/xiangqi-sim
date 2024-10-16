import trimesh

# Load STL mesh
mesh = trimesh.load_mesh('link_lift.STL')

# Visualize the mesh
mesh.show()
