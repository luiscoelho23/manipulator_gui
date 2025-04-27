import os
import subprocess
import tkinter as tk
from PIL import Image, ImageTk
from tkinter import filedialog, messagebox, ttk
from ament_index_python import get_package_prefix, get_package_share_directory
import glob
import signal
import importlib

def get_package_directory(package_name):
    """Get the package directory for a given package name."""
    return get_package_prefix(package_name)
    

def select_file(label, file_types):
    file_path = filedialog.askopenfilename(filetypes=file_types)
    if file_path:
        label.config(text=os.path.basename(file_path), foreground="dark green")
        return file_path
    return None

def select_files(label, file_types):
    file_paths = filedialog.askopenfilenames(filetypes=file_types)
    if file_paths:
        filenames = ", ".join(os.path.basename(path) for path in file_paths)
        label.config(text=filenames, foreground="dark green", wraplength=400, justify="left")
        return file_paths
    return []

def select_output_file(extension):
    file_path = filedialog.asksaveasfilename(defaultextension=extension, filetypes=[(f"{extension.upper()} Files", f"*{extension}")])
    return file_path

def run_script(script_name, args):
    if not os.path.exists(script_name):
        messagebox.showerror("Error", f"Script {script_name} not found!")
        return
    try:
        subprocess.Popen(["python3", script_name] + args)
    except Exception as e:
        messagebox.showerror("Error", f"Failed to execute script:\n{e}")

def setup_gui():
    root = tk.Tk()
    root.title("Setup Menu")
    
    # Get screen dimensions
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    
    # Calculate position to center the window
    window_width = 700
    window_height = 800
    x = (screen_width - window_width) // 2
    y = (screen_height - window_height) // 2
    
    # Set window size and position
    root.geometry(f"{window_width}x{window_height}+{x}+{y}")
    root.configure(bg="#F0F0F0")
    
    # Configure styles
    style = ttk.Style()
    style.configure("TButton", padding=8, relief="flat", background="#4CAF50", foreground="white", font=("Arial", 10))
    style.configure("TLabel", foreground="#333333", font=("Arial", 10))
    style.configure("TNotebook", background="#F0F0F0")
    style.configure("TNotebook.Tab", padding=[10, 5], font=("Arial", 10, "bold"))
    style.configure("TEntry", padding=5, font=("Arial", 10))
    style.map("TButton",
        background=[("active", "#45a049")],
        foreground=[("active", "white")])
    
    # Configure return button style
    style.configure("Return.TButton", padding=8, relief="flat", background="#2196F3", foreground="white", font=("Arial", 10))
    style.map("Return.TButton",
        background=[("active", "#0b7dda")],
        foreground=[("active", "white")])
    
    notebook = ttk.Notebook(root)
    notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
    
    frame_extract = ttk.Frame(notebook, padding=20)
    notebook.add(frame_extract, text="Process Data")

    frame_ee_position = ttk.Frame(notebook, padding=20)
    notebook.add(frame_ee_position, text="Get End-Effector Position")

    frame_trajectory = ttk.Frame(notebook, padding=20)
    notebook.add(frame_trajectory, text="Generate Robot Trajectory")
    
    frame_dmp = ttk.Frame(notebook, padding=20)
    notebook.add(frame_dmp, text="Generate DMP")
    
    # Add buttons to each tab
    def add_exit_button(frame):
        button_frame = ttk.Frame(frame)
        button_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=10)
        
        # Safe return function to ensure clean closing
        def safe_return():
            # In the SetupMenu, we don't typically have long-running processes
            # But we'll terminate any running script processes if needed
            for proc in subprocess.run(["ps", "-ef"], capture_output=True, text=True).stdout.splitlines():
                if "SetupMenu" in proc and "python" in proc:
                    try:
                        pid = int(proc.split()[1])
                        os.kill(pid, signal.SIGTERM)
                    except (ValueError, ProcessLookupError, PermissionError):
                        pass
            root.destroy()
            
            # Import and launch the main GUI menu - use delayed import to avoid circular dependency
            try:
                # Dynamically import the GUI module
                gui_module = importlib.import_module('manipulator_gui.GUI')
                # Call the launch_gui function
                gui_module.launch_gui()
            except Exception as e:
                print(f"Error returning to main menu: {e}")
                # If import fails, exit cleanly
                os._exit(0)
        
        # Return to Main Menu button
        return_btn = ttk.Button(
            button_frame, 
            text="Return to Main Menu", 
            command=safe_return,
            style="Return.TButton"
        )
        return_btn.pack(side=tk.LEFT, pady=10, padx=5)
        
        # Exit button
        exit_btn = ttk.Button(button_frame, text="Exit", command=lambda: os._exit(0))
        exit_btn.pack(side=tk.RIGHT, pady=10, padx=5)
    
    add_exit_button(frame_extract)
    add_exit_button(frame_ee_position)
    add_exit_button(frame_trajectory)
    add_exit_button(frame_dmp)
    
    files = {}
    labels = {}
    ee_values = {}

    def validate_joint_values():
        for label, entry in ee_values.items():
            value = entry.get()
            if not value.strip():
                messagebox.showerror("Error", f"{label} cannot be empty.")
                return False
            try:
                float(value)
            except ValueError:
                messagebox.showerror("Error", f"{label} must be a number.")
                return False
        return True
    
    def create_file_selector(frame, text, key, file_type):
        btn = ttk.Button(frame, text=f"Select {text}", command=lambda: files.update({key: select_file(labels[key], file_type)}))
        btn.pack(pady=5)
        labels[key] = ttk.Label(frame, text="No file selected", foreground="red", wraplength=400)
        labels[key].pack(pady=5)

    def create_files_selector(frame, text, key, file_type):
        btn = ttk.Button(frame, text=f"Select {text}", command=lambda: files.update({key: select_files(labels[key], file_type)}))
        btn.pack(pady=5)
        labels[key] = ttk.Label(frame, text="No file selected", foreground="red", wraplength=400, justify="left")
        labels[key].pack(pady=5)
    
    # Process Data
    create_files_selector(frame_extract, "Raw data", "xlsx_raw", [("Excel Files", "*.xlsx")])
    
    # Create frame for resampling size
    resample_frame = ttk.Frame(frame_extract)
    resample_frame.pack(fill=tk.X, padx=10, pady=5)
    
    ttk.Label(resample_frame, text="Resample Size:").pack(side=tk.LEFT, padx=5)
    resample_size = tk.StringVar(value="149")
    resample_entry = ttk.Entry(resample_frame, textvariable=resample_size, width=10)
    resample_entry.pack(side=tk.LEFT, padx=5)
    
    # Create frame for column selection
    column_frame = ttk.LabelFrame(frame_extract, text="Select Columns to Extract")
    column_frame.pack(fill=tk.X, padx=10, pady=10)
    
    # Create variables to store column selections
    column_vars = {
        "Joint Angles ZXY": {
            "Right Shoulder Flexion/Extension": tk.BooleanVar(value=True),
            "Right Elbow Flexion/Extension": tk.BooleanVar(value=True)
        },
        "Ergonomic Joint Angles ZXY": {
            "Vertical_Pelvis Flexion/Extension": tk.BooleanVar(value=True)
        }
    }
    
    # Create checkboxes for each column
    for sheet, columns in column_vars.items():
        sheet_frame = ttk.LabelFrame(column_frame, text=sheet)
        sheet_frame.pack(fill=tk.X, padx=5, pady=5)
        
        for column, var in columns.items():
            cb = ttk.Checkbutton(sheet_frame, text=column, variable=var)
            cb.pack(anchor=tk.W, padx=5)
    
    # Add button to help users add custom columns
    custom_column_frame = ttk.Frame(column_frame)
    custom_column_frame.pack(fill=tk.X, padx=5, pady=5)
    
    custom_sheet = tk.StringVar()
    custom_column = tk.StringVar()
    
    ttk.Label(custom_column_frame, text="Sheet:").pack(side=tk.LEFT, padx=2)
    sheet_entry = ttk.Entry(custom_column_frame, textvariable=custom_sheet, width=20)
    sheet_entry.pack(side=tk.LEFT, padx=2)
    
    ttk.Label(custom_column_frame, text="Column:").pack(side=tk.LEFT, padx=2)
    column_entry = ttk.Entry(custom_column_frame, textvariable=custom_column, width=20)
    column_entry.pack(side=tk.LEFT, padx=2)
    
    def add_custom_column():
        sheet = custom_sheet.get().strip()
        column = custom_column.get().strip()
        
        if not sheet or not column:
            messagebox.showerror("Error", "Both Sheet name and Column name are required.")
            return
            
        # Add to the column_vars dictionary
        if sheet not in column_vars:
            column_vars[sheet] = {}
            
            # Create a new frame for this sheet
            sheet_frame = ttk.LabelFrame(column_frame, text=sheet)
            sheet_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Check if column already exists
        if column in column_vars[sheet]:
            messagebox.showinfo("Info", f"Column '{column}' already exists in sheet '{sheet}'.")
            return
            
        # Add new column with checkbox
        column_vars[sheet][column] = tk.BooleanVar(value=True)
        
        # Find the frame for this sheet
        for child in column_frame.winfo_children():
            if isinstance(child, ttk.LabelFrame) and child.cget("text") == sheet:
                cb = ttk.Checkbutton(child, text=column, variable=column_vars[sheet][column])
                cb.pack(anchor=tk.W, padx=5)
                break
        
        # Clear the entry fields
        custom_sheet.set("")
        custom_column.set("")
    
    add_btn = ttk.Button(custom_column_frame, text="Add Column", command=add_custom_column)
    add_btn.pack(side=tk.LEFT, padx=5)
    
    def execute_extraction():
        if any(key not in files or files[key] is None for key in ["xlsx_raw"]):
            messagebox.showerror("Error", "Please select all required files before proceeding.")
            return
        
        # Get selected raw files
        raw_files = files["xlsx_raw"]
        if len(raw_files) < 1:
            messagebox.showerror("Error", "Please select at least one raw data file.")
            return
            
        # Ask for output file name just once
        output_file = select_output_file(".xlsx")
        if not output_file:
            # User cancelled
            return
            
        # Get selected columns
        selected_columns = []
        for sheet, columns in column_vars.items():
            for column, var in columns.items():
                if var.get():
                    selected_columns.append(f"{sheet}:{column}")
        
        if not selected_columns:
            messagebox.showerror("Error", "Please select at least one column to extract.")
            return
            
        # Validate resample size
        try:
            samples = int(resample_size.get())
            if samples <= 0:
                messagebox.showerror("Error", "Resample size must be a positive integer.")
                return
        except ValueError:
            messagebox.showerror("Error", "Resample size must be a valid integer.")
            return
            
        package_path = get_package_directory('manipulator_skill_acquisition')
        script_path = os.path.join(package_path, 'lib', 'manipulator_skill_acquisition', 
                                'SetupMenu', 'excel_extract.py')
        
        # Process each file with one base output name
        for i, raw_file in enumerate(raw_files):
            # Create output filename - add suffix for multiple files
            actual_output_file = output_file
            if len(raw_files) > 1:
                base, ext = os.path.splitext(output_file)
                actual_output_file = f"{base}_{i+1}{ext}"
                
            # Build command arguments
            args = [raw_file, actual_output_file]
            if selected_columns:
                args.append("--columns")
                args.extend(selected_columns)
            
            # Add samples parameter
            args.extend(["--samples", str(samples)])
                
            run_script(script_path, args)
            
        # Show a message when all files are processed
        if len(raw_files) > 1:
            messagebox.showinfo("Success", f"Processed {len(raw_files)} files.")

    btn_run_extract = ttk.Button(frame_extract, text="Extract Selected Files", command=execute_extraction)
    btn_run_extract.pack(pady=15)

    create_files_selector(frame_extract, "Extracted Data", "xlsx", [("Excel Files", "*.xlsx")])
    
    def execute_avg():
        if any(key not in files or files[key] is None for key in ["xlsx"]):
            messagebox.showerror("Error", "Please select all required files before proceeding.")
            return
        output_file = select_output_file(".xlsx")
        if not output_file:
            messagebox.showerror("Error", "Output file name is required!")
            return
        
        selected_files = files["xlsx"]
        if len(selected_files) < 1:
            messagebox.showerror("Error", "Please select at least one file to average.")
            return
            
        package_path = get_package_directory('manipulator_skill_acquisition')
        script_path = os.path.join(package_path, 'lib', 'manipulator_skill_acquisition', 
                                'SetupMenu', 'excel_avg.py')
        
        # Pass all selected files as arguments, followed by the output file
        args = list(selected_files) + [output_file]
        run_script(script_path, args)

    btn_run_avg = ttk.Button(frame_extract, text="Average Selected Files", command=execute_avg)
    btn_run_avg.pack(pady=15)

    # Get End-Effector Position
    package_share_dir = get_package_share_directory('manipulator_gui')
    image_path = os.path.join(package_share_dir, 'resources', 'human_kin_model.png')
    if os.path.exists(image_path):
        original_image = Image.open(image_path)
        resized_image = original_image.resize((250, 250), Image.Resampling.LANCZOS) 
        ee_image = ImageTk.PhotoImage(resized_image)
        image_label = tk.Label(frame_ee_position, image=ee_image)
        image_label.image = ee_image 
        image_label.pack(side="left", padx=20, anchor="ne")

    ee_labels = ["Back (L1 m)", "Shoulder (L2 m)", "Arm (L3 m)", "Forearm (L4 m)"]
    for label in ee_labels:
        ttk.Label(frame_ee_position, text=f"{label}:").pack()
        entry = ttk.Entry(frame_ee_position)
        entry.pack()
        ee_values[label] = entry
    
    create_file_selector(frame_ee_position, "Human Joint Angle Trajectory File", "ee_xlsx", [("Excel Files", "*.xlsx")])

    def execute_ee_position():
        if "ee_xlsx" not in files or files["ee_xlsx"] is None:
            messagebox.showerror("Error", "Please select the required Excel file.")
            return
        if not validate_joint_values():
            return
        output_file = select_output_file(".csv")
        if not output_file:
            messagebox.showerror("Error", "Output file name is required!")
            return
        ee_args = [files["ee_xlsx"], output_file] + [ee_values[label].get() for label in ee_labels]
        
        package_path = get_package_directory('manipulator_skill_acquisition')
        script_path = os.path.join(package_path, 'lib', 'manipulator_skill_acquisition', 
                                'SetupMenu', 'fk_human_traj.py')
        run_script(script_path, ee_args)

    btn_run_ee_position = ttk.Button(frame_ee_position, text="Get EE Position", command=execute_ee_position)
    btn_run_ee_position.pack(pady=20)

    # Generate Robot Trajectory
    create_file_selector(frame_trajectory, "EE Trajectory File", "ee_traj", [("CSV Files", "*.csv")])
    create_file_selector(frame_trajectory, "Human Joint Angle Trajectory File", "human_angle_traj", [("Excel Files", "*.xlsx")])

    def execute_trajectory():
        if any(key not in files or files[key] is None for key in ["ee_traj", "human_angle_traj"]):
            messagebox.showerror("Error", "Please select all required files before proceeding.")
            return
        output_file = select_output_file(".csv")
        if not output_file:
            messagebox.showerror("Error", "Output file name is required!")
            return
        package_path = get_package_directory('manipulator_skill_acquisition')
        script_path = os.path.join(package_path, 'lib', 'manipulator_skill_acquisition', 
                                'SetupMenu', 'trajectory_mapping.py')
        run_script(script_path, [files["ee_traj"], files["human_angle_traj"], output_file])
    
    btn_run_trajectory = ttk.Button(frame_trajectory, text="Generate Robot Trajectory", command=execute_trajectory)
    btn_run_trajectory.pack(pady=20)
    
    # Generate DMP
    def get_dmp_config_files():
        pkg_share = get_package_share_directory('manipulator_skill_acquisition')
        config_dir = os.path.join(pkg_share, 'config_dmp')
        if not os.path.exists(config_dir):
            return [], []
        config_files = glob.glob(os.path.join(config_dir, '*.yaml'))
        # Return both full paths and just filenames
        return config_files, [os.path.basename(f) for f in config_files]

    def create_dmp_config_selector(frame):
        config_paths, config_names = get_dmp_config_files()
        if not config_paths:
            messagebox.showwarning("Warning", "No DMP config files found in package config directory!")
            return None
        
        config_var = tk.StringVar()
        # Store full path in a separate variable
        config_full_path = tk.StringVar()
        config_full_path.set(config_paths[0])
        config_var.set(config_names[0])
        
        frame_config = ttk.Frame(frame)
        frame_config.pack(fill=tk.X, pady=5)
        
        ttk.Label(frame_config, text="DMP Config File:").pack(side=tk.LEFT)
        config_combo = ttk.Combobox(frame_config, textvariable=config_var, values=config_names, state="readonly")
        config_combo.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        # Update full path when selection changes
        def on_config_select(event):
            idx = config_names.index(config_var.get())
            config_full_path.set(config_paths[idx])
        
        config_combo.bind('<<ComboboxSelected>>', on_config_select)
        
        return config_full_path

    config_var = create_dmp_config_selector(frame_dmp)
    create_file_selector(frame_dmp, "Robot Joint Angle Trajectory File", "robot_angle_traj", [("CSV Files", "*.csv")]) 

    def execute_dmp():
        if not config_var or not config_var.get():
            messagebox.showerror("Error", "Please select a DMP config file.")
            return
        if "robot_angle_traj" not in files or files["robot_angle_traj"] is None:
            messagebox.showerror("Error", "Please select the robot joint angle trajectory file.")
            return
        output_file = select_output_file(".mpx")
        if not output_file:
            messagebox.showerror("Error", "Output file name is required!")
            return
        
        config_name = os.path.basename(config_var.get())
        package_path = get_package_directory('manipulator_skill_acquisition')
        script_path = os.path.join(package_path, 'lib', 'manipulator_skill_acquisition', 
                                'SetupMenu', 'dmp_generator.py')
        run_script(script_path, [config_name, files["robot_angle_traj"], output_file])

    btn_run_dmp = ttk.Button(frame_dmp, text="Generate DMP", command=execute_dmp)
    btn_run_dmp.pack(pady=20)
    
    root.mainloop()

if __name__ == "__main__":
    setup_gui()