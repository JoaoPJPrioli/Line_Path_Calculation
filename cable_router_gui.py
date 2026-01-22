"""
Cable Router GUI - Interactive Setup Window
Run this script to launch the interactive cable routing application with parameter selection
"""

import sys
import os
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import threading

# Add current directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


class CableRouterSetupGUI:
    """Setup window for cable router parameters"""
    
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Cable Router - Setup")
        
        # Variables
        self.file_path = tk.StringVar()
        self.gap_distance = tk.DoubleVar(value=2.0)
        self.grid_resolution = tk.DoubleVar(value=0.01)
        self.lightweight_mode = tk.BooleanVar(value=True)
        self.reduction_ratio = tk.DoubleVar(value=0.9)
        
        # Create widgets first
        self.create_widgets()
        
        # Set geometry after widgets are created
        self.root.geometry("650x500")
        self.root.resizable(True, True)
        self.root.update_idletasks()
        self.root.minsize(600, 450)
        
    def create_widgets(self):
        """Create all GUI widgets"""
        print("Creating GUI widgets...", flush=True)
        
        # Title
        title_frame = tk.Frame(self.root, bg="#2c3e50", height=80)
        title_frame.pack(fill=tk.X, pady=0)
        title_frame.pack_propagate(False)
        
        title_label = tk.Label(
            title_frame, 
            text="Cable Router - 3D Path Planning",
            font=("Arial", 18, "bold"),
            bg="#2c3e50",
            fg="white"
        )
        title_label.pack(pady=10)
        
        subtitle_label = tk.Label(
            title_frame,
            text="Dijkstra's Algorithm with CAD Geometry",
            font=("Arial", 10),
            bg="#2c3e50",
            fg="white"
        )
        subtitle_label.pack()
        
        # Main content frame
        content_frame = tk.Frame(self.root, padx=20, pady=20)
        content_frame.pack(fill=tk.BOTH, expand=True)
        
        # File selection section
        file_frame = tk.LabelFrame(content_frame, text="1. Select CAD File", font=("Arial", 10, "bold"), padx=10, pady=10)
        file_frame.pack(fill=tk.X, pady=(0, 15))
        
        file_entry_frame = tk.Frame(file_frame)
        file_entry_frame.pack(fill=tk.X)
        
        file_entry = tk.Entry(file_entry_frame, textvariable=self.file_path, font=("Arial", 9), state="readonly")
        file_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 10))
        
        browse_btn = tk.Button(
            file_entry_frame,
            text="Browse...",
            command=self.browse_file,
            font=("Arial", 9),
            bg="#3498db",
            fg="white",
            cursor="hand2",
            width=10
        )
        browse_btn.pack(side=tk.RIGHT)
        
        file_hint = tk.Label(file_frame, text="Supported: .obj, .stl files", font=("Arial", 8), fg="black")
        file_hint.pack(anchor=tk.W, pady=(5, 0))
        
        # Parameters section
        param_frame = tk.LabelFrame(content_frame, text="2. Configure Parameters", font=("Arial", 10, "bold"), padx=10, pady=10)
        param_frame.pack(fill=tk.X, pady=(0, 15))
        
        # Gap distance
        gap_frame = tk.Frame(param_frame)
        gap_frame.pack(fill=tk.X, pady=5)
        tk.Label(gap_frame, text="Safety Gap (mm):", font=("Arial", 9), width=20, anchor=tk.W).pack(side=tk.LEFT)
        gap_spinbox = tk.Spinbox(gap_frame, from_=1.0, to=500.0, increment=5.0, textvariable=self.gap_distance, font=("Arial", 9), width=10)
        gap_spinbox.pack(side=tk.LEFT, padx=5)
        tk.Label(gap_frame, text="Minimum clearance from geometry", font=("Arial", 8), fg="black").pack(side=tk.LEFT, padx=5)
        
        # Grid resolution
        grid_frame = tk.Frame(param_frame)
        grid_frame.pack(fill=tk.X, pady=5)
        tk.Label(grid_frame, text="Grid Resolution (mm):", font=("Arial", 9), width=20, anchor=tk.W).pack(side=tk.LEFT)
        grid_spinbox = tk.Spinbox(grid_frame, from_=10.0, to=500.0, increment=10.0, textvariable=self.grid_resolution, font=("Arial", 9), width=10)
        grid_spinbox.pack(side=tk.LEFT, padx=5)
        tk.Label(grid_frame, text="Larger = faster but less accurate", font=("Arial", 8), fg="black").pack(side=tk.LEFT, padx=5)
        
        # Lightweight mode
        light_frame = tk.Frame(param_frame)
        light_frame.pack(fill=tk.X, pady=5)
        tk.Checkbutton(
            light_frame,
            text="Lightweight Mode (recommended for large files)",
            variable=self.lightweight_mode,
            font=("Arial", 9)
        ).pack(side=tk.LEFT)
        
        # Reduction ratio
        reduction_frame = tk.Frame(param_frame)
        reduction_frame.pack(fill=tk.X, pady=5)
        tk.Label(reduction_frame, text="Mesh Reduction:", font=("Arial", 9), width=20, anchor=tk.W).pack(side=tk.LEFT)
        reduction_spinbox = tk.Spinbox(reduction_frame, from_=0.5, to=0.99, increment=0.05, textvariable=self.reduction_ratio, font=("Arial", 9), width=10, format="%.2f")
        reduction_spinbox.pack(side=tk.LEFT, padx=5)
        tk.Label(reduction_frame, text="0.9 = keep 10% of polygons", font=("Arial", 8), fg="black").pack(side=tk.LEFT, padx=5)
        
        # Status section
        self.status_label = tk.Label(content_frame, text="Ready to load file", font=("Arial", 9), fg="black")
        self.status_label.pack(pady=10)
        
        # Progress bar (hidden initially)
        self.progress_bar = ttk.Progressbar(content_frame, mode='indeterminate', length=400)
        
        # Store router instance for visualization
        self.router = None
        
        # Action buttons
        button_frame = tk.Frame(content_frame)
        button_frame.pack(pady=10)
        
        self.load_btn = tk.Button(
            button_frame,
            text="Load & Process File",
            command=self.load_and_process,
            font=("Arial", 11, "bold"),
            bg="#27ae60",
            fg="white",
            cursor="hand2",
            width=20,
            height=2
        )
        self.load_btn.pack(side=tk.LEFT, padx=5)
        
        self.visualize_btn = tk.Button(
            button_frame,
            text="Start Visualization",
            command=self.start_visualization,
            font=("Arial", 11, "bold"),
            bg="#3498db",
            fg="white",
            cursor="hand2",
            width=20,
            height=2,
            state=tk.DISABLED
        )
        self.visualize_btn.pack(side=tk.LEFT, padx=5)
        
        cancel_btn = tk.Button(
            button_frame,
            text="Exit",
            command=self.root.quit,
            font=("Arial", 11),
            bg="#e74c3c",
            fg="white",
            cursor="hand2",
            width=10,
            height=2
        )
        cancel_btn.pack(side=tk.LEFT, padx=5)
        
        print("✓ All widgets created successfully", flush=True)
        print(f"  - Load button state: {self.load_btn['state']}", flush=True)
        print(f"  - Visualize button state: {self.visualize_btn['state']}", flush=True)
        
    def browse_file(self):
        """Open file browser to select CAD file"""
        filename = filedialog.askopenfilename(
            title="Select CAD File",
            filetypes=[
                ("CAD Files", "*.obj *.stl *.OBJ *.STL"),
                ("OBJ Files", "*.obj *.OBJ"),
                ("STL Files", "*.stl *.STL"),
                ("All Files", "*.*")
            ],
            initialdir=os.getcwd()
        )
        if filename:
            self.file_path.set(filename)
            self.status_label.config(text=f"Selected: {os.path.basename(filename)}", fg="black")
    
    def load_and_process(self):
        """Load and process the selected file"""
        print("\n" + "="*70, flush=True)
        print("LOAD & PROCESS BUTTON CLICKED", flush=True)
        print("="*70, flush=True)
        
        file = self.file_path.get()
        print(f"Selected file: {file}", flush=True)
        
        # Validation
        if not file:
            print("ERROR: No file selected", flush=True)
            messagebox.showerror("Error", "Please select a CAD file first!")
            return
        
        if not os.path.exists(file):
            print(f"ERROR: File not found: {file}", flush=True)
            messagebox.showerror("Error", f"File not found: {file}")
            return
        
        print(f"File exists: {file}", flush=True)
        print("Starting background processing thread...", flush=True)
        
        # Disable buttons and show progress
        self.load_btn.config(state=tk.DISABLED)
        self.visualize_btn.config(state=tk.DISABLED)
        self.status_label.config(text="Loading and processing... Please wait...", fg="black")
        self.progress_bar.pack(pady=5)
        self.progress_bar.start(10)
        
        # Run in separate thread to keep GUI responsive
        thread = threading.Thread(target=self.process_in_background, daemon=True)
        thread.start()
        print(f"Background thread started: {thread.name}", flush=True)
        print("Waiting for processing to complete...", flush=True)
        print("="*70 + "\n", flush=True)
    
    def process_in_background(self):
        """Process the file in background thread"""
        try:
            # Import here to avoid loading delays on startup
            from cable_router import CableRouter
            
            # Get parameters
            file = self.file_path.get()
            gap = self.gap_distance.get()
            resolution = self.grid_resolution.get()
            lightweight = self.lightweight_mode.get()
            reduction = self.reduction_ratio.get()
            
            print("\n" + "="*70, flush=True)
            print("CABLE ROUTER - PROCESSING", flush=True)
            print("="*70, flush=True)
            print(f"File: {file}", flush=True)
            print(f"Gap Distance: {gap} mm", flush=True)
            print(f"Grid Resolution: {resolution} mm", flush=True)
            print(f"Lightweight Mode: {lightweight}", flush=True)
            print(f"Mesh Reduction: {reduction}", flush=True)
            print("="*70 + "\n", flush=True)
            
            # Create router instance
            print("Creating router instance...", flush=True)
            router = CableRouter(
                stl_path=file,
                gap_distance=gap,
                grid_resolution=resolution,
                lightweight_mode=lightweight,
                target_reduction=reduction
            )
            print("✓ Router created\n", flush=True)
            
            # Load and prepare geometry
            print("Loading geometry...", flush=True)
            router.load_and_prepare_geometry()
            print("✓ Geometry loaded\n", flush=True)
            
            # Build graph
            print("Building graph...", flush=True)
            router.build_graph()
            print("✓ Graph built\n", flush=True)
            
            # Store router and enable visualization button
            self.router = router
            print("✓ Processing complete! Enabling visualization button...\n", flush=True)
            self.root.after(0, self.loading_complete)
            
        except Exception as e:
            # Show error in GUI thread with full traceback
            import traceback
            error_details = f"{str(e)}\n\nTraceback:\n{traceback.format_exc()}"
            print(f"\n✗ ERROR in background processing:\n{error_details}", flush=True)
            self.root.after(0, self.show_error, error_details)
    
    def loading_complete(self):
        """Called when file loading is complete"""
        print("\n" + "="*70, flush=True)
        print("✓ ENABLING VISUALIZATION BUTTON", flush=True)
        print("="*70 + "\n", flush=True)
        
        self.progress_bar.stop()
        self.progress_bar.pack_forget()
        self.load_btn.config(state=tk.NORMAL)
        self.visualize_btn.config(state=tk.NORMAL)
        self.status_label.config(text="✓ File loaded successfully! Click 'Start Visualization' to begin.", fg="black")
        
        print("="*70, flush=True)
        print("✓ FILE LOADED AND PROCESSED SUCCESSFULLY", flush=True)
        print("="*70, flush=True)
        print("The 'Start Visualization' button is now enabled.", flush=True)
        print("Click it to open the 3D viewer.", flush=True)
        print("="*70 + "\n", flush=True)
    
    def start_visualization(self):
        """Start the 3D visualization"""
        if self.router is None:
            messagebox.showerror("Error", "No file loaded. Please load a file first!")
            return
        
        try:
            self.root.withdraw()  # Hide setup window
            print("\n" + "="*70)
            print("Launching 3D Viewer...")
            print("Instructions:")
            print("  1. Click on the mesh to select START point (red sphere)")
            print("  2. Click again to select END point (green sphere)")
            print("  3. Path will be calculated automatically")
            print("  4. Close the 3D window when done")
            print("="*70 + "\n")
            
            # Start visualization (setup_visualization and enable_picker are called in run)
            print("Setting up visualization...", flush=True)
            self.router.setup_visualization()
            print("\n" + "="*70, flush=True)
            print("3D VIEWER READY - CLICK ON THE MESH TO SELECT POINTS", flush=True)
            print("="*70, flush=True)
            print("If clicking doesn't work, try:", flush=True)
            print("  - Clicking directly on the blue mesh surface", flush=True)
            print("  - Using left mouse button", flush=True)
            print("  - Rotating the view first (drag with mouse)", flush=True)
            print("="*70 + "\n", flush=True)
            self.router.plotter.show()
            
            print("\n✓ Application closed normally.\n")
            self.root.quit()
        except Exception as e:
            self.show_error(f"Viewer error: {e}")
            self.root.deiconify()  # Show setup window again
    
    def show_error(self, error_msg):
        """Show error message"""
        self.progress_bar.stop()
        self.progress_bar.pack_forget()
        self.load_btn.config(state=tk.NORMAL)
        self.visualize_btn.config(state=tk.DISABLED)
        self.status_label.config(text="Error occurred! Check console for details.", fg="black")
        
        # Show truncated error in dialog, full error in console
        short_error = error_msg.split('\n')[0] if '\n' in error_msg else error_msg
        if len(short_error) > 200:
            short_error = short_error[:200] + "..."
        
        messagebox.showerror("Error", f"Failed to process file:\n\n{short_error}\n\nCheck console for full details.")
        print(f"\n✗ ERROR: {error_msg}\n", flush=True)
    
    def run(self):
        """Start the GUI"""
        print("Starting GUI mainloop...", flush=True)
        print(f"Final window geometry: {self.root.geometry()}", flush=True)
        self.root.mainloop()
        print("GUI closed.", flush=True)


def main():
    """Launch the setup GUI"""
    print("="*70)
    print(" "*15 + "CABLE ROUTING APPLICATION")
    print(" "*20 + "Interactive Setup")
    print("="*70)
    print("\nLaunching setup window...\n")
    
    try:
        app = CableRouterSetupGUI()
        app.run()
    except KeyboardInterrupt:
        print("\n✓ Application closed by user (Ctrl+C).")
    except Exception as e:
        print(f"\n✗ ERROR: {e}")
        import traceback
        traceback.print_exc()
        input("\nPress Enter to exit...")
        sys.exit(1)

if __name__ == "__main__":
    main()
