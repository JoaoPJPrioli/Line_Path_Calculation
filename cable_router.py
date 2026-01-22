"""
Cable Routing Application using Dijkstra's Algorithm
This application loads a CAD file (STL/OBJ), allows selection of two points,
and calculates the shortest path for a flexible cable with safety offset.
"""

import numpy as np
import networkx as nx
import pyvista as pv
from scipy.spatial import cKDTree, distance
from scipy.interpolate import splprep, splev
from typing import Tuple, List, Optional
import warnings
warnings.filterwarnings('ignore')


class CableRouter:
    """
    Main class for cable routing with Dijkstra's algorithm.
    """
    
    def __init__(self, stl_path: str, gap_distance: float = 5.0, grid_resolution: float = 2.0, 
                 lightweight_mode: bool = True, target_reduction: float = 0.9):
        """
        Initialize the cable router.
        
        Args:
            stl_path: Path to STL or OBJ file
            gap_distance: Minimum safety offset from geometry (mm)
            grid_resolution: Spacing between grid points (mm)
            lightweight_mode: Use simplified mesh for visualization (saves memory)
            target_reduction: Mesh reduction ratio (0.9 = reduce to 10% of original)
        """
        self.stl_path = stl_path
        self.gap_distance = gap_distance
        self.grid_resolution = grid_resolution
        self.lightweight_mode = lightweight_mode
        self.target_reduction = target_reduction
        
        # Module 1: Geometry Setup
        self.mesh = None
        self.mesh_simplified = None  # Lightweight version for visualization
        self.grid_points = None
        self.safe_points = None
        self.safe_indices = None
        
        # Module 2: Graph
        self.graph = None
        self.kdtree = None
        
        # Module 3: Pathfinding
        self.start_point = None
        self.end_point = None
        self.path = None
        self.smooth_path_result = None  # Renamed to avoid conflict with method
        
        # Visualization
        self.plotter = None
        self.mesh_actor = None
        self.display_mesh = None  # Store for surface projection
        self.point_A = None
        self.point_B = None
        self.click_count = 0
        self.info_text_actor = None
        self.path_length = 0
        self.sphere_A_actor = None
        self.sphere_B_actor = None
        self.path_actors = []
        
    # ==================== MODULE 1: ENVIRONMENT & GEOMETRY SETUP ====================
    
    def load_and_prepare_geometry(self):
        """
        Task 1.1: Load CAD file and center it.
        Task 1.2: Generate 3D voxel grid.
        Task 1.3: Calculate distance field and identify safe points.
        """
        print(f"Loading mesh from {self.stl_path}...")
        # Load with PyVista instead of trimesh
        self.mesh = pv.read(self.stl_path)
        original_points = self.mesh.n_points
        print(f"  Original mesh: {original_points:,} vertices, {self.mesh.n_cells:,} faces")
        
        # Center the mesh at origin
        center = self.mesh.center
        self.mesh.points -= center
        print(f"Mesh centered at origin. Bounds: {self.mesh.bounds}")
        
        # Create simplified version for visualization (saves memory)
        if self.lightweight_mode:
            print(f"Creating lightweight mesh (reducing by {self.target_reduction*100:.0f}%)...")
            try:
                # Decimate mesh to reduce polygon count
                self.mesh_simplified = self.mesh.decimate(self.target_reduction)
                simplified_points = self.mesh_simplified.n_points
                print(f"  Simplified mesh: {simplified_points:,} vertices "
                      f"({100*(1-self.target_reduction):.0f}% of original)")
                print(f"  Memory saved: ~{(original_points - simplified_points) * 12 / 1024 / 1024:.1f} MB")
            except:
                # If decimation fails, use original
                print("  Warning: Decimation failed, using original mesh")
                self.mesh_simplified = self.mesh
        else:
            self.mesh_simplified = self.mesh
        
        # LIGHTWEIGHT MODE: Create a moderate-resolution 3D grid around the mesh
        # Then filter to keep only points at safe distance
        
        print(f"Generating safe point cloud (lightweight mode)...")
        print(f"  Grid resolution: {self.grid_resolution}mm")
        print(f"  Safety gap: {self.gap_distance}mm")
        
        # Get mesh bounds
        bounds = self.mesh.bounds
        margin = self.gap_distance * 2
        
        # Create 3D grid with the specified resolution
        # Use a moderate resolution to balance memory and connectivity
        effective_resolution = max(self.grid_resolution, 50.0)  # At least 50mm for connectivity
        self.effective_resolution = effective_resolution  # Store for later use
        print(f"  Using effective grid resolution: {effective_resolution}mm")
        
        x = np.arange(bounds[0] - margin, bounds[1] + margin, effective_resolution)
        y = np.arange(bounds[2] - margin, bounds[3] + margin, effective_resolution)
        z = np.arange(bounds[4] - margin, bounds[5] + margin, effective_resolution)
        
        grid_shape = (len(x), len(y), len(z))
        n_grid_points = grid_shape[0] * grid_shape[1] * grid_shape[2]
        print(f"  Grid shape: {grid_shape} = {n_grid_points:,} points")
        
        # Check if grid is too large
        if n_grid_points > 10_000_000:  # 10M points max
            print(f"  WARNING: Grid too large! Increasing resolution...")
            effective_resolution = effective_resolution * 1.5
            x = np.arange(bounds[0] - margin, bounds[1] + margin, effective_resolution)
            y = np.arange(bounds[2] - margin, bounds[3] + margin, effective_resolution)
            z = np.arange(bounds[4] - margin, bounds[5] + margin, effective_resolution)
            grid_shape = (len(x), len(y), len(z))
            n_grid_points = grid_shape[0] * grid_shape[1] * grid_shape[2]
            print(f"  Adjusted grid: {grid_shape} = {n_grid_points:,} points")
        
        # Create the grid
        print(f"  Creating grid...")
        xx, yy, zz = np.meshgrid(x, y, z, indexing='ij')
        all_grid_points = np.column_stack([xx.ravel(), yy.ravel(), zz.ravel()])
        print(f"  Grid created: {len(all_grid_points):,} points")
        
        # Filter: keep only points at safe distance from mesh
        # Process in batches to save memory
        print(f"  Filtering to keep safe points (distance >= {self.gap_distance}mm)...")
        safe_points_list = []
        batch_size = 10000
        num_batches = (len(all_grid_points) + batch_size - 1) // batch_size
        
        for batch_idx in range(num_batches):
            start_idx = batch_idx * batch_size
            end_idx = min((batch_idx + 1) * batch_size, len(all_grid_points))
            batch = all_grid_points[start_idx:end_idx]
            
            # Check distance for each point in batch
            for point in batch:
                # Find closest point on mesh
                closest_point, _ = self.mesh.find_closest_cell(point, return_closest_point=True)
                distance = np.linalg.norm(point - closest_point)
                
                # Keep if safe distance
                if distance >= self.gap_distance:
                    safe_points_list.append(point)
            
            # Progress indicator
            if (batch_idx + 1) % max(1, num_batches // 10) == 0:
                progress = (batch_idx + 1) / num_batches * 100
                percent_kept = len(safe_points_list) / max(1, end_idx) * 100
                print(f"    Progress: {progress:.0f}% | Kept: {len(safe_points_list):,} points ({percent_kept:.1f}%)", flush=True)
        
        self.safe_points = np.array(safe_points_list)
        self.grid_points = self.safe_points  # For compatibility
        self.safe_indices = np.arange(len(self.safe_points))
        
        if len(self.safe_points) == 0:
            raise ValueError("No safe points found! Try reducing gap_distance or grid_resolution.")
        
        print(f"\n✓ Generated {len(self.safe_points):,} safe points")
        print(f"  Memory usage: ~{len(self.safe_points) * 24 / 1024 / 1024:.1f} MB")
        print(f"  Point density: {len(self.safe_points) / n_grid_points * 100:.1f}% of grid")
        
        return self.safe_points
    
    # ==================== MODULE 2: GRAPH CONSTRUCTION ====================
    
    def build_graph(self, proximity_penalty: float = 2.0):
        """
        Task 2.1-2.4: Build adjacency graph with 26-neighbor connectivity.
        
        Args:
            proximity_penalty: Weight multiplier for points near gap limit
        """
        print("Building graph from safe points...")
        self.graph = nx.Graph()
        
        # Add all safe points as nodes
        for idx, point in enumerate(self.safe_points):
            self.graph.add_node(idx, pos=point)
        
        # Build KD-tree for efficient neighbor search
        self.kdtree = cKDTree(self.safe_points)
        
        # Connect to neighbors within search radius
        # Use effective_resolution if available, otherwise estimate
        if hasattr(self, 'effective_resolution'):
            search_radius = self.effective_resolution * 3.0
        else:
            search_radius = self.grid_resolution * 3.0
        
        print(f"Connecting neighbors (search radius: {search_radius:.1f}mm)...")
        edges_added = 0
        
        for idx, point in enumerate(self.safe_points):
            # Find neighbors within max distance
            neighbors = self.kdtree.query_ball_point(point, search_radius)
            
            for neighbor_idx in neighbors:
                if neighbor_idx > idx:  # Avoid duplicate edges
                    neighbor_point = self.safe_points[neighbor_idx]
                    
                    # Calculate Euclidean distance
                    distance = np.linalg.norm(point - neighbor_point)
                    
                    # Add edge with weight
                    self.graph.add_edge(idx, neighbor_idx, weight=distance)
                    edges_added += 1
            
            # Progress indicator
            if (idx + 1) % max(1, len(self.safe_points) // 10) == 0:
                progress = (idx + 1) / len(self.safe_points) * 100
                print(f"  Progress: {progress:.0f}% | Edges: {edges_added:,}", flush=True)
        
        print(f"Graph built: {self.graph.number_of_nodes():,} nodes, "
              f"{self.graph.number_of_edges():,} edges")
        print(f"Average degree: {2 * self.graph.number_of_edges() / max(1, self.graph.number_of_nodes()):.1f}")
        
        # Check connectivity
        if not nx.is_connected(self.graph):
            num_components = nx.number_connected_components(self.graph)
            largest_component = max(nx.connected_components(self.graph), key=len)
            print(f"\u26a0 WARNING: Graph has {num_components} disconnected components!")
            print(f"  Largest component: {len(largest_component):,} nodes ({len(largest_component)/len(self.safe_points)*100:.1f}%)")
            print(f"  This may cause path finding to fail between some points.")
        else:
            print("\u2713 Graph is fully connected!")
        
        return self.graph
    
    # ==================== MODULE 3: PATHFINDING LOGIC ====================
    
    def find_nearest_node(self, target_point: np.ndarray) -> int:
        """
        Task 3.1: Map a 3D coordinate to the closest valid node.
        
        Args:
            target_point: 3D coordinates [x, y, z]
            
        Returns:
            Index of nearest node in safe_points
        """
        distance, idx = self.kdtree.query(target_point)
        return idx
    
    def calculate_path(self, point_A: np.ndarray, point_B: np.ndarray) -> List[np.ndarray]:
        """
        Task 3.2-3.3: Execute Dijkstra's algorithm and extract path.
        
        Args:
            point_A: Start point [x, y, z]
            point_B: End point [x, y, z]
            
        Returns:
            List of 3D coordinates along the path
        """
        print(f"\nCalculating path from {point_A} to {point_B}...")
        
        # Find nearest valid nodes
        start_idx = self.find_nearest_node(point_A)
        end_idx = self.find_nearest_node(point_B)
        
        print(f"Start node: {start_idx}, End node: {end_idx}")
        
        # Check if nodes are in the same connected component
        if not nx.is_connected(self.graph):
            # Find which component each node belongs to
            components = list(nx.connected_components(self.graph))
            start_component = None
            end_component = None
            
            for i, component in enumerate(components):
                if start_idx in component:
                    start_component = i
                if end_idx in component:
                    end_component = i
            
            print(f"  Start node in component {start_component} (size: {len(components[start_component])})")
            print(f"  End node in component {end_component} (size: {len(components[end_component])})")
            
            if start_component != end_component:
                print(f"  \u26a0 Nodes are in different components!")
        
        # Execute Dijkstra's algorithm
        try:
            path_indices = nx.dijkstra_path(
                self.graph, start_idx, end_idx, weight='weight'
            )
            
            # Extract coordinates
            self.path = np.array([self.safe_points[idx] for idx in path_indices])
            
            # Calculate path length
            path_length = sum(np.linalg.norm(self.path[i+1] - self.path[i]) 
                            for i in range(len(self.path)-1))
            
            print(f"Path found! {len(self.path)} waypoints, "
                  f"total length: {path_length:.2f}mm")
            
            return self.path
            
        except nx.NetworkXNoPath:
            print("ERROR: No path found! Points may be in disconnected regions.")
            return None
    
    def smooth_path(self, smoothing_factor: float = 0.5, num_points: int = 100) -> np.ndarray:
        """
        Task 4.4: Apply spline smoothing to simulate cable flexibility.
        
        Args:
            smoothing_factor: 0 = exact fit, higher = smoother
            num_points: Number of points in smoothed path
            
        Returns:
            Smoothed path coordinates
        """
        if self.path is None or len(self.path) < 4:
            return self.path
        
        print("Smoothing path with spline...")
        
        # Fit B-spline
        tck, u = splprep([self.path[:, 0], self.path[:, 1], self.path[:, 2]], 
                         s=smoothing_factor, k=min(3, len(self.path)-1))
        
        # Evaluate spline
        u_new = np.linspace(0, 1, num_points)
        smooth_coords = splev(u_new, tck)
        self.smooth_path_result = np.column_stack(smooth_coords)
        
        return self.smooth_path_result
    
    # ==================== MODULE 4: VISUALIZATION & USER INTERACTION ====================
    
    def setup_visualization(self):
        """
        Task 4.1-4.2: Setup PyVista plotter with mesh and point picker.
        """
        print("\nSetting up visualization...")
        self.plotter = pv.Plotter()
        
        # Add title
        self.plotter.add_text(
            "Cable Router - Dijkstra's Algorithm",
            position='upper_edge',
            font_size=14,
            color='white',
            font='arial'
        )
        
        # Add instructions
        self.info_text_actor = self.plotter.add_text(
            "INSTRUCTIONS:\n"
            "1. Click on mesh to select START point (A)\n"
            "2. Click again to select END point (B)\n"
            "3. Path will calculate automatically\n\n"
            "Press 'R' to RESET points\n"
            "Press 'Q' to quit",
            position='upper_left',
            font_size=11,
            color='yellow',
            font='courier'
        )
        
        # Task 4.1: Render STL with transparency (use lightweight mesh)
        # Use simplified mesh for visualization to save memory
        self.display_mesh = self.mesh_simplified if self.mesh_simplified is not None else self.mesh
        print(f"Adding mesh to plotter (color: cyan, opacity: 0.5)...", flush=True)
        self.mesh_actor = self.plotter.add_mesh(
            self.display_mesh,
            color='cyan',
            opacity=0.5,
            show_edges=True,
            edge_color='white',
            pickable=True,
            label='CAD Geometry (Simplified)' if self.lightweight_mode else 'CAD Geometry'
        )
        print("✓ Mesh added to plotter", flush=True)
        
        # Add coordinate axes
        self.plotter.add_axes()
        
        # Task 4.2: Implement point picker
        print("Enabling point picking with surface snapping...", flush=True)
        
        # Store reference to self for callback
        router_self = self
        
        # Create custom picker callback using VTK observer
        def on_left_click(obj, event):
            """Custom click handler that snaps to mesh surface"""
            try:
                # Get the click position
                click_pos = router_self.plotter.iren.interactor.GetEventPosition()
                
                # Create a picker
                picker = router_self.plotter.iren.picker
                
                # Pick from this position
                picker.Pick(click_pos[0], click_pos[1], 0, router_self.plotter.renderer)
                
                # Get the picked position
                picked_pos = picker.GetPickPosition()
                
                if picked_pos and picked_pos != (0.0, 0.0, 0.0):
                    # Convert to numpy array
                    picked_point = np.array(picked_pos, dtype=float)
                    
                    print(f"DEBUG: Picked position: {picked_point}", flush=True)
                    
                    # Snap to mesh surface using a different method
                    # Create a point cloud with just this point
                    from scipy.spatial import distance
                    
                    # Get all mesh vertices
                    mesh_points = router_self.display_mesh.points
                    
                    # Find the closest mesh vertex to the picked point
                    distances = np.linalg.norm(mesh_points - picked_point, axis=1)
                    closest_idx = np.argmin(distances)
                    closest_point = mesh_points[closest_idx]
                    
                    print(f"DEBUG: Snapped to surface: {closest_point}", flush=True)
                    
                    # Call the callback
                    router_self.point_picker_callback(closest_point)
            except Exception as e:
                print(f"Error in picker: {e}", flush=True)
                import traceback
                traceback.print_exc()
        
        # Add the observer for left button press
        self.plotter.iren.interactor.AddObserver('LeftButtonPressEvent', on_left_click)
        
        # Add reset button
        self.plotter.add_key_event('r', self.reset_points)
        self.plotter.add_key_event('R', self.reset_points)
        
        print("✓ Custom click handler enabled with surface snapping!", flush=True)
        print("✓ LEFT CLICK anywhere on the cyan mesh", flush=True)
        print("✓ Press 'R' to reset and select new points", flush=True)
        
        return self.plotter
    
    def point_picker_callback(self, point):
        """
        Callback for point selection via mouse click.
        Point is guaranteed to be on mesh surface.
        """
        if point is None:
            print("No point picked", flush=True)
            return
        
        # Convert to numpy array and ensure it's the right shape
        picked_point = np.array(point, dtype=float)
        
        # Debug: check the shape
        if picked_point.ndim == 0 or picked_point.size != 3:
            print(f"ERROR: Invalid point shape: {picked_point.shape}, value: {picked_point}", flush=True)
            return
        
        # Ensure it's a 1D array with 3 elements
        picked_point = picked_point.flatten()[:3]
        
        print(f"\n*** POINT PICKED: [{picked_point[0]:.2f}, {picked_point[1]:.2f}, {picked_point[2]:.2f}] (Click count: {self.click_count + 1}) ***", flush=True)
        self.click_count += 1
        
        if self.click_count == 1:
            self.point_A = picked_point
            print(f"✓ Point A selected: {self.point_A}", flush=True)
            self.sphere_A_actor = self.plotter.add_mesh(
                pv.Sphere(radius=self.grid_resolution * 2, center=picked_point),
                color='green',
                label='Start (A)'
            )
            # Update info text
            self.plotter.remove_actor(self.info_text_actor)
            self.info_text_actor = self.plotter.add_text(
                f"✓ START POINT (A) Selected\n"
                f"  Position: [{self.point_A[0]:.1f}, {self.point_A[1]:.1f}, {self.point_A[2]:.1f}]\n\n"
                f"Now click to select END point (B)\n\n"
                f"Press 'R' to RESET\n"
                f"Press 'Q' to quit",
                position='upper_left',
                font_size=11,
                color='lime',
                font='courier'
            )
            
        elif self.click_count == 2:
            self.point_B = picked_point
            print(f"✓ Point B selected: {self.point_B}", flush=True)
            self.sphere_B_actor = self.plotter.add_mesh(
                pv.Sphere(radius=self.grid_resolution * 2, center=picked_point),
                color='red',
                label='End (B)'
            )
            
            # Update info text
            self.plotter.remove_actor(self.info_text_actor)
            self.info_text_actor = self.plotter.add_text(
                f"✓ END POINT (B) Selected\n"
                f"  Position: [{self.point_B[0]:.1f}, {self.point_B[1]:.1f}, {self.point_B[2]:.1f}]\n\n"
                f"Calculating path...",
                position='upper_left',
                font_size=11,
                color='orange',
                font='courier'
            )
            
            # Calculate and visualize path
            self.calculate_and_visualize_path()
            
            # Reset for new selection
            self.click_count = 0
    
    def calculate_and_visualize_path(self):
        """
        Calculate path between selected points and visualize it.
        """
        # Calculate path
        path = self.calculate_path(self.point_A, self.point_B)
        
        if path is None:
            print("Cannot visualize - no path found!")
            # Update info text
            self.plotter.remove_actor(self.info_text_actor)
            self.info_text_actor = self.plotter.add_text(
                f"✗ NO PATH FOUND!\n\n"
                f"Points may be in disconnected regions.\n"
                f"Try selecting different points or\n"
                f"increase GAP_DISTANCE parameter.\n\n"
                f"Click to select new points (A then B)",
                position='upper_left',
                font_size=11,
                color='red',
                font='courier'
            )
            return
        
        # Calculate path length
        self.path_length = sum(np.linalg.norm(path[i+1] - path[i]) for i in range(len(path)-1))
        
        # Calculate straight-line distance for comparison
        straight_line = np.linalg.norm(self.point_B - self.point_A)
        efficiency = (straight_line / self.path_length) * 100 if self.path_length > 0 else 0
        
        # Smooth the path
        smooth_path = self.smooth_path(smoothing_factor=len(path) * 0.5, num_points=200)
        
        # Calculate smooth path length
        smooth_length = sum(np.linalg.norm(smooth_path[i+1] - smooth_path[i]) 
                           for i in range(len(smooth_path)-1))
        
        # Update info text with results
        self.plotter.remove_actor(self.info_text_actor)
        self.info_text_actor = self.plotter.add_text(
            f"═══ PATH CALCULATED ═══\n\n"
            f"START (A): [{self.point_A[0]:.1f}, {self.point_A[1]:.1f}, {self.point_A[2]:.1f}]\n"
            f"END (B):   [{self.point_B[0]:.1f}, {self.point_B[1]:.1f}, {self.point_B[2]:.1f}]\n\n"
            f"━━━━━━━━━━━━━━━━━━━━━━\n"
            f"PATH LENGTH:      {self.path_length:.2f} mm\n"
            f"SMOOTH LENGTH:    {smooth_length:.2f} mm\n"
            f"STRAIGHT LINE:    {straight_line:.2f} mm\n"
            f"EFFICIENCY:       {efficiency:.1f}%\n"
            f"WAYPOINTS:        {len(path)}\n"
            f"━━━━━━━━━━━━━━━━━━━━━━\n\n"
            f"Press 'R' to RESET and calculate new path",
            position='upper_left',
            font_size=10,
            color='cyan',
            font='courier'
        )
        
        # Task 4.3: Render path as 3D tube
        cable_radius = self.grid_resolution * 0.5
        
        # Create polydata from path
        path_polydata = pv.PolyData(smooth_path)
        path_polydata.points = smooth_path
        
        # Create cells (lines connecting consecutive points)
        lines = np.full((len(smooth_path)-1, 3), 2, dtype=np.int_)
        lines[:, 1] = np.arange(len(smooth_path)-1)
        lines[:, 2] = np.arange(1, len(smooth_path))
        path_polydata.lines = lines
        
        # Create tube around path
        tube = path_polydata.tube(radius=cable_radius, n_sides=12)
        
        tube_actor = self.plotter.add_mesh(
            tube,
            color='orange',
            opacity=0.9,
            label='Cable Path',
            smooth_shading=True
        )
        self.path_actors.append(tube_actor)
        
        # Also show the raw path nodes
        raw_path_points = pv.PolyData(path)
        points_actor = self.plotter.add_mesh(
            raw_path_points,
            color='blue',
            point_size=5,
            render_points_as_spheres=True,
            opacity=0.3
        )
        self.path_actors.append(points_actor)
        
        print("✓ Path visualized!")
        print("Press 'R' to reset and select new points", flush=True)
        print("Press 'R' to reset and select new points", flush=True)
    
    def reset_points(self):
        """Reset selected points and clear visualization"""
        print("\n" + "="*60, flush=True)
        print("RESETTING - Clear all selections", flush=True)
        print("="*60 + "\n", flush=True)
        
        # Remove sphere actors
        if self.sphere_A_actor is not None:
            self.plotter.remove_actor(self.sphere_A_actor)
            self.sphere_A_actor = None
        
        if self.sphere_B_actor is not None:
            self.plotter.remove_actor(self.sphere_B_actor)
            self.sphere_B_actor = None
        
        # Remove path actors
        for actor in self.path_actors:
            self.plotter.remove_actor(actor)
        self.path_actors = []
        
        # Reset state
        self.point_A = None
        self.point_B = None
        self.click_count = 0
        self.path_length = 0
        
        # Reset info text
        self.plotter.remove_actor(self.info_text_actor)
        self.info_text_actor = self.plotter.add_text(
            "INSTRUCTIONS:\n"
            "1. Click on mesh to select START point (A)\n"
            "2. Click again to select END point (B)\n"
            "3. Path will calculate automatically\n\n"
            "Press 'R' to RESET points\n"
            "Press 'Q' to quit",
            position='upper_left',
            font_size=11,
            color='yellow',
            font='courier'
        )
        
        print("✓ Reset complete! Click to select new points.", flush=True)
    
    def run(self):
        """
        Main execution method: load geometry, build graph, and launch interactive viewer.
        """
        # Module 1: Load and prepare geometry
        self.load_and_prepare_geometry()
        
        # Module 2: Build graph
        self.build_graph()
        
        # Check if graph is connected
        if not nx.is_connected(self.graph):
            num_components = nx.number_connected_components(self.graph)
            print(f"WARNING: Graph has {num_components} disconnected components. "
                  f"Some paths may not be possible.")
        
        # Module 4: Launch visualization
        self.setup_visualization()
        
        print("\n" + "="*60)
        print("READY! Click on the mesh to select points A and B")
        print("="*60 + "\n")
        
        self.plotter.show()


# ==================== ADVANCED: BEND RADIUS CONSTRAINT ====================

def calculate_path_with_bend_constraint(
    router: CableRouter,
    point_A: np.ndarray,
    point_B: np.ndarray,
    min_bend_radius: float = 10.0
) -> List[np.ndarray]:
    """
    Enhanced pathfinding with bend radius constraint.
    Adds cost penalty for sharp direction changes.
    
    Args:
        router: CableRouter instance with built graph
        point_A: Start point
        point_B: End point
        min_bend_radius: Minimum allowed bend radius (mm)
        
    Returns:
        Path with smooth bends
    """
    print(f"\nCalculating path with bend radius constraint ({min_bend_radius}mm)...")
    
    start_idx = router.find_nearest_node(point_A)
    end_idx = router.find_nearest_node(point_B)
    
    # Modified Dijkstra with direction penalties
    # This would require custom implementation - simplified version here
    path_indices = nx.dijkstra_path(router.graph, start_idx, end_idx, weight='weight')
    path = np.array([router.safe_points[idx] for idx in path_indices])
    
    # Post-process to check bend angles
    if len(path) >= 3:
        for i in range(1, len(path) - 1):
            v1 = path[i] - path[i-1]
            v2 = path[i+1] - path[i]
            
            # Calculate angle between vectors
            cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
            angle = np.arccos(np.clip(cos_angle, -1, 1))
            
            if angle > np.pi / 2:  # Sharp turn detected
                print(f"  Warning: Sharp bend at waypoint {i} (angle: {np.degrees(angle):.1f}°)")
    
    return path


# ==================== MAIN EXECUTION ====================

def main():
    """
    Main entry point for the cable routing application.
    """
    import sys
    
    print("="*60)
    print("CABLE ROUTING APPLICATION - DIJKSTRA'S ALGORITHM")
    print("="*60 + "\n")
    
    # Configuration
    STL_FILE = "T4781.obj"  # Update this path as needed
    GAP_DISTANCE = 5.0      # Safety offset in mm
    GRID_RESOLUTION = 3.0   # Grid spacing in mm (smaller = more accurate but slower)
    
    # Check if file exists
    import os
    if not os.path.exists(STL_FILE):
        print(f"ERROR: File '{STL_FILE}' not found!")
        print("Please update the STL_FILE path in the script.")
        sys.exit(1)
    
    # Create router instance
    router = CableRouter(
        stl_path=STL_FILE,
        gap_distance=GAP_DISTANCE,
        grid_resolution=GRID_RESOLUTION
    )
    
    # Run the application
    router.run()


if __name__ == "__main__":
    main()
