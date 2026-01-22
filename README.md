# Cable Routing Application

A Python application that calculates optimal cable paths in 3D space using Dijkstra's algorithm while maintaining safety clearance from CAD geometry.

## Features

### Interactive GUI Setup Window
- Browse and select CAD files (OBJ/STL)
- Configure parameters before loading
  - Safety Gap (1-500mm)
  - Grid Resolution (10-500mm)
  - Lightweight Mode (mesh simplification)
  - Mesh Reduction Ratio (0.5-0.99)
- Real-time processing with progress tracking
- Separate "Load & Process" and "Start Visualization" workflow

### Module 1: Environment & Geometry Setup
- Loads STL/OBJ CAD files using `PyVista`
- Centers geometry at origin
- **Mesh Simplification**: Reduces polygon count by 90% for visualization (saves memory)
- Generates 3D grid in safe space around geometry
- Filters points to maintain safety clearance
- Memory-efficient batch processing
- Configurable safety gap (GAP_DISTANCE)

### Module 2: Graph Construction
- Converts safe points into a searchable network using `networkx`
- Adaptive neighbor connectivity (3x grid resolution search radius)
- Euclidean distance-based edge weights
- KD-tree optimization for efficient neighbor search
- Connectivity diagnostics (component detection)

### Module 3: Pathfinding Logic
- Dijkstra's algorithm for shortest path calculation
- Automatic node mapping from arbitrary 3D coordinates
- Path length calculation and statistics
- Component checking to detect disconnected regions

### Module 4: Visualization & User Interaction
- Interactive 3D visualization with `PyVista`
- **Surface Snapping**: Click anywhere, point snaps to mesh surface
- **Reset Feature**: Press 'R' to clear and select new points
- Real-time path calculation and display
- Smooth cable representation with B-spline interpolation
- Transparent cyan mesh rendering with white edges
- Color-coded markers:
  - Green sphere = Start point (A)
  - Red sphere = End point (B)
  - Orange tube = Calculated cable path
  - Blue dots = Raw waypoints
- Comprehensive statistics display:
  - Path length (mm)
  - Smooth path length (mm)
  - Straight-line distance (mm)
  - Path efficiency (%)
  - Number of waypoints

### Advanced Features
- VTK-based click handling for reliable point selection
- Spline smoothing for realistic cable flexibility
- Disconnected component detection and warnings
- Memory-efficient mesh decimation (90% reduction)
- Progress indicators during processing

## Installation

```bash
pip install -r requirements.txt
```

Required packages:
- numpy >= 1.21.0
- networkx >= 2.6.0
- scipy >= 1.7.0
- pyvista >= 0.34.0

## Usage

### Quick Start with GUI

```bash
python cable_router_gui.py
```

**Workflow:**
1. **Setup Window Opens**
   - Click "Browse..." to select your OBJ/STL file
   - Adjust parameters:
     - Safety Gap: 120mm (default) - minimum clearance from geometry
     - Grid Resolution: 100mm (default) - spacing between points
     - Lightweight Mode: ✓ Enabled (recommended)
     - Mesh Reduction: 0.9 (keeps 10% of polygons)
   - Click "Load & Process File"
   
2. **Processing**
   - Watch console for progress
   - Mesh simplification
   - Grid generation and filtering
   - Graph construction with connectivity check
   
3. **Visualization**
   - Click "Start Visualization" when ready
   - 3D viewer opens with cyan mesh
   
4. **Point Selection**
   - **LEFT CLICK** on mesh to select START point (green sphere)
   - **LEFT CLICK** again to select END point (red sphere)
   - Path calculates automatically and displays as orange tube
   - Statistics shown in upper-left corner
   
5. **Reset and Repeat**
   - **Press 'R' key** to clear points and path
   - Select new points to calculate different paths
   - **Press 'Q' key** to quit

### Parameters Guide

#### Safety Gap (GAP_DISTANCE)
- Minimum clearance between cable and geometry
- **Default: 120mm** (conservative for large models)
- Smaller values: 50-80mm for tight spaces
- Larger values: 150-300mm for extra safety
- Affects: Available routing space

#### Grid Resolution
- Spacing between grid points in 3D space
- **Default: 100mm** (good balance)
- Smaller (50-80mm): More precise paths, slower processing
- Larger (150-300mm): Faster processing, less precise
- Minimum enforced: 50mm for connectivity
- **Trade-off:** Precision vs. computation time and memory

#### Lightweight Mode
- **Enabled**: Uses simplified mesh (10% of polygons) for visualization
- Keeps full mesh for accurate distance calculations
- **Recommended**: Always on for large models (>100k vertices)
- Saves 80-95% of display memory

#### Mesh Reduction Ratio
- 0.9 = Keep 10% of original polygons (default)
- 0.95 = Keep 5% (more aggressive)
- 0.85 = Keep 15% (less reduction)
- Only affects visualization, not path accuracy

## Performance Tips

### Memory Optimization
1. **Enable Lightweight Mode** - reduces display memory by 90%
2. **Increase Grid Resolution** for very large models (150-200mm)
3. **Increase Mesh Reduction** to 0.95 for models with >1M vertices

### Speed Optimization
1. **Grid Resolution**: Use 100-150mm for faster processing
2. **Safety Gap**: Larger gaps = more safe points = faster filtering
3. **Batch Size**: Automatically optimized (10k points per batch)

### Quality Optimization
1. **Grid Resolution**: Use 50-80mm for precise paths
2. **Safety Gap**: Tune based on actual clearance needs
3. **Smoothing**: Automatic (0.5 × path length)

### Typical Processing Times
- **Small model** (<100k faces): 30-60 seconds
- **Medium model** (100k-500k faces): 1-3 minutes  
- **Large model** (>500k faces): 3-10 minutes

## Technical Details

### Algorithm
- **Pathfinding:** Dijkstra's shortest path algorithm
- **Complexity:** O((V + E) log V) where V = safe grid points, E = edges
- **Graph Type:** Undirected weighted graph with adaptive connectivity

### Grid Generation
- Creates uniform 3D grid with specified resolution
- Filters points by distance from mesh surface
- Minimum effective resolution: 50mm (enforced for connectivity)
- Batch processing (10k points) for memory efficiency

### Distance Field Calculation
- Uses PyVista's `find_closest_cell` for efficient distance queries
- Identifies safe zones where distance ≥ GAP_DISTANCE
- Processes in batches to minimize memory usage

### Graph Construction
- Adaptive neighbor search (3× grid resolution radius)
- KD-tree for O(log n) neighbor queries
- Connectivity validation and component detection
- Typical degree: 10-30 neighbors per node

### Point Selection
- VTK observer pattern for click detection
- Automatic surface snapping to nearest mesh vertex
- Manual projection ensures points always on geometry shell

### Path Smoothing
- B-spline interpolation using `scipy.interpolate.splprep`
- Converts grid-based path to smooth cable representation
- Default: 200 interpolated points
- Smoothing factor: 0.5 × waypoint count

## Keyboard Controls

- **Left Click**: Select points (A then B)
- **R**: Reset points and path
- **Q**: Quit application
- **Mouse Drag**: Rotate/pan view

## Troubleshooting

### "No path found! Points may be in disconnected regions"
**Causes:**
- Selected points are in separate graph components
- Grid resolution too large relative to geometry gaps
- Safety gap too large, blocking connections

**Solutions:**
1. Reduce Safety Gap (try 80-100mm)
2. Reduce Grid Resolution (try 50-80mm for better connectivity)
3. Select points closer together
4. Check console for component diagnostics

### Application hangs during loading
**Causes:**
- Grid too dense (too many points)
- Very large mesh file

**Solutions:**
1. Increase Grid Resolution (100-150mm)
2. Enable Lightweight Mode
3. Check console for grid size warnings

### Memory errors
**Causes:**
- Grid resolution too fine for model size
- Large mesh with lightweight mode disabled

**Solutions:**
1. Increase Grid Resolution to 150-200mm
2. Enable Lightweight Mode
3. Increase Mesh Reduction to 0.95

### Click not working / points not selecting
**Causes:**
- Graphics driver issue
- Mesh not properly loaded

**Solutions:**
1. Update graphics drivers
2. Try rotating view first, then clicking
3. Check console for error messages
4. Ensure clicking directly on cyan mesh

## File Structure

```
test CAD line djistra/
├── cable_router.py          # Main application (routing engine)
├── cable_router_gui.py      # GUI setup window
├── requirements.txt         # Python dependencies
├── README.md               # This file
└── T4781.obj              # Example CAD file
```

## Example Console Output

```
======================================================================
               CABLE ROUTING APPLICATION
                    Interactive Setup
======================================================================

Creating GUI widgets...
✓ All widgets created successfully

[User selects file and clicks "Load & Process File"]

======================================================================
CABLE ROUTER - PROCESSING
======================================================================
File: T4781.obj
Gap Distance: 120.0 mm
Grid Resolution: 100.0 mm
Lightweight Mode: True
Mesh Reduction: 0.9
======================================================================

Loading mesh from T4781.obj...
  Original mesh: 1,635,288 vertices, 545,096 faces
Mesh centered at origin. Bounds: [-1545, 1545, -2144, 2144, -8250, 8250]
Creating lightweight mesh (reducing by 90%)...
  Simplified mesh: 1,123,392 vertices (10% of original)
  Memory saved: ~5.9 MB

Generating safe point cloud (lightweight mode)...
  Grid resolution: 100.0mm
  Using effective grid resolution: 100.0mm
  Grid shape: (32, 44, 166) = 233,728 points
  Creating grid...
  Grid created: 233,728 points
  Filtering to keep safe points (distance >= 120.0mm)...
    Progress: 10% | Kept: 15,234 points (65.2%)
    Progress: 20% | Kept: 30,891 points (66.1%)
    ...
    Progress: 100% | Kept: 152,447 points (65.2%)

✓ Generated 152,447 safe points
  Memory usage: ~3.5 MB
  Point density: 65.2% of grid

Building graph from safe points...
Connecting neighbors (search radius: 300.0mm)...
  Progress: 10% | Edges: 234,521
  Progress: 20% | Edges: 468,234
  ...
  Progress: 100% | Edges: 2,341,234

Graph built: 152,447 nodes, 2,341,234 edges
Average degree: 30.7
✓ Graph is fully connected!

======================================================================
✓ FILE LOADED AND PROCESSED SUCCESSFULLY
======================================================================

[User clicks "Start Visualization"]

Setting up visualization...
✓ Mesh added to plotter
✓ Custom click handler enabled with surface snapping!

======================================================================
3D VIEWER READY - CLICK ON THE MESH TO SELECT POINTS
======================================================================

[User clicks on mesh]

*** POINT PICKED: [47.22, -12.70, -15.56] (Click count: 1) ***
✓ Point A selected

[User clicks again]

*** POINT PICKED: [-49.71, -12.70, -0.91] (Click count: 2) ***
✓ Point B selected

Calculating path from [47.22, -12.70, -15.56] to [-49.71, -12.70, -0.91]...
Start node: 45234, End node: 98721
Path found! 127 waypoints, total length: 2845.32mm

Smoothing path with spline...
✓ Path visualized!

[Path displayed with statistics:]
═══ PATH CALCULATED ═══

START (A): [47.2, -12.7, -15.6]
END (B):   [-49.7, -12.7, -0.9]

━━━━━━━━━━━━━━━━━━━━━━
PATH LENGTH:      2845.32 mm
SMOOTH LENGTH:    2798.45 mm
STRAIGHT LINE:    97.13 mm
EFFICIENCY:       3.4%
WAYPOINTS:        127
━━━━━━━━━━━━━━━━━━━━━━

Press 'R' to RESET and calculate new path
```

## Manufacturing Considerations

### Bend Radius Constraint
For manufacturing validation:
- Monitor path waypoint angles (check console warnings)
- Sharp turns (>90°) are flagged automatically
- Consider post-processing to enforce minimum bend radius

### Cable Properties
- Cable radius visualization: Grid Resolution × 0.5
- Adjust based on actual cable diameter
- Path clearance validated at each grid point

### Export Path Data
Path coordinates available in `router.path` (numpy array)
```python
# Access after path calculation
path_coords = router.path  # Nx3 array
smooth_coords = router.smooth_path_result  # Mx3 array
```

## License

This project is provided as-is for educational and commercial use.

## Support

For issues or questions:
1. Check console output for diagnostic messages
2. Review troubleshooting section above
3. Verify all dependencies are installed correctly
4. Test with smaller grid resolution settings
Connecting neighbors...
Graph built: 87500 nodes, 524000 edges

READY! Click on the mesh to select points A and B

✓ Point A selected: [10.5, 20.3, 15.7]
✓ Point B selected: [-15.2, -18.9, 22.1]

Calculating path from [10.5, 20.3, 15.7] to [-15.2, -18.9, 22.1]...
Path found! 156 waypoints, total length: 467.23mm
Smoothing path with spline...
✓ Path visualized!
```

## Troubleshooting

### "No path found"
- Points may be in disconnected safe zones
- Try increasing GAP_DISTANCE to create more safe space
- Check if geometry has enclosed cavities

### Slow performance
- Increase GRID_RESOLUTION
- Reduce margin around geometry
- Use smaller CAD models for testing

### Memory errors
- Reduce grid size by increasing GRID_RESOLUTION
- Process smaller sections of large assemblies

## File Structure

```
cable_router.py          # Main application
requirements.txt         # Python dependencies
T4781.obj               # Example CAD model
README.md               # This file
```

## API Reference

### CableRouter Class

```python
router = CableRouter(stl_path, gap_distance=5.0, grid_resolution=2.0)
```

**Methods:**
- `load_and_prepare_geometry()` - Module 1 implementation
- `build_graph()` - Module 2 implementation
- `calculate_path(point_A, point_B)` - Module 3 implementation
- `smooth_path()` - Spline smoothing
- `setup_visualization()` - Module 4 implementation
- `run()` - Complete workflow execution

## Future Enhancements

- [ ] Multi-cable routing with collision avoidance
- [ ] Export path as STEP/IGES for CAD integration
- [ ] Automated mounting point suggestion
- [ ] Real-time path cost display
- [ ] Support for different cable diameters
- [ ] Path optimization for manufacturing constraints

## License

Open source - modify as needed for your application.
