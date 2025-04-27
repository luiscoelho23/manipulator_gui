import os
import signal
import subprocess
import threading
import select
import tkinter as tk
from tkinter.scrolledtext import ScrolledText
from PIL import Image, ImageTk
from tkinter import filedialog, messagebox, ttk
import time
import fcntl
from ament_index_python import get_package_prefix, get_package_share_directory
import importlib

ROS_SETUP_SCRIPT = "/opt/ros/humble/setup.bash" 
current_file = os.path.abspath(__file__)
ws_path = current_file[:current_file.find('/src/')]
WORKSPACE_SETUP = os.path.join(ws_path, 'install/setup.bash')
GAZEBO_SETUP = "/usr/share/gazebo/setup.sh"

ros_launch_process = None
ros_run_process = None

files = {}
labels = {}

def get_package_directory(package_name):
    """Get the package directory for a given package name."""
    return get_package_prefix(package_name)

def read_output(process, output):
    # Set streams to non-blocking mode
    def set_non_blocking(fd):
        flags = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
    
    try:
        set_non_blocking(process.stdout.fileno())
        set_non_blocking(process.stderr.fileno())
    except Exception as e:
        print(f"Error setting non-blocking mode: {e}")
        return
    
    while True:
        # Check if process is still running
        if process.poll() is not None:
            break
            
        # Read from stdout
        try:
            if process.stdout and not process.stdout.closed:
                stdout_line = process.stdout.readline()
                if stdout_line:  # Only process if we got actual output
                    try:
                        decoded_line = stdout_line.decode('utf-8', errors='replace')
                        output.after_idle(append_output, decoded_line, output)
                    except Exception as e:
                        print(f"Error decoding stdout: {e}")
        except (IOError, AttributeError) as e:
            pass
        
        # Read from stderr
        try:
            if process.stderr and not process.stderr.closed:
                stderr_line = process.stderr.readline()
                if stderr_line:  # Only process if we got actual output
                    try:
                        decoded_line = stderr_line.decode('utf-8', errors='replace')
                        output.after_idle(append_output, decoded_line, output)
                    except Exception as e:
                        print(f"Error decoding stderr: {e}")
        except (IOError, AttributeError) as e:
            pass
        
        # Small sleep to prevent CPU spinning
        time.sleep(0.01)
    
    # Read any remaining output after process termination
    try:
        if process.stdout and not process.stdout.closed:
            remaining_stdout = process.stdout.read()
            if remaining_stdout:
                try:
                    decoded_output = remaining_stdout.decode('utf-8', errors='replace')
                    output.after_idle(append_output, decoded_output, output)
                except Exception as e:
                    print(f"Error decoding remaining stdout: {e}")
    except Exception as e:
        print(f"Error reading remaining stdout: {e}")
    
    try:
        if process.stderr and not process.stderr.closed:
            remaining_stderr = process.stderr.read()
            if remaining_stderr:
                try:
                    decoded_output = remaining_stderr.decode('utf-8', errors='replace')
                    output.after_idle(append_output, decoded_output, output)
                except Exception as e:
                    print(f"Error decoding remaining stderr: {e}")
    except Exception as e:
        print(f"Error reading remaining stderr: {e}")

def append_output(text, output):
    try:
        output.configure(state="normal") 
        output.insert(tk.END, text)      
        output.see(tk.END)
        output.configure(state="disabled")
    except Exception as e:
        print(f"Error updating output: {e}")

def is_wsl():
    with open("/proc/version", "r") as f:
        return "microsoft" in f.read().lower()

def create_files_selector(frame, text, key, file_type):
        btn = ttk.Button(frame, text=f"Select {text}", command=lambda: files.update({key: select_files(labels[key], file_type)}))
        btn.pack(pady=5)
        labels[key] = ttk.Label(frame, text="No file selected", foreground="red", wraplength=400)
        labels[key].pack(pady=5)

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
    root.title("Eval & Run Menu")
    
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
    
    # Create frames for each tab
    frame_eval_ang = ttk.Frame(notebook, padding=20)
    frame_eval_ee = ttk.Frame(notebook, padding=20)
    frame_launch = ttk.Frame(notebook, padding=20)
    frame_run = ttk.Frame(notebook, padding=20)
    
    # Add tabs
    notebook.add(frame_eval_ang, text="Evaluate Ang traj")
    notebook.add(frame_eval_ee, text="Evaluate EE traj")
    notebook.add(frame_launch, text="Launch")
    notebook.add(frame_run, text="Run")
    
   
    
    # Add exit buttons to each tab
    def add_exit_button(frame):
        def safe_exit():
            # Stop both processes using existing functions
            ros_launch_stop()
            ros_run_stop()
            # Close the window
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
        
        button_frame = ttk.Frame(frame)
        button_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=10)
        
        # Return to Main Menu button
        return_btn = ttk.Button(
            button_frame, 
            text="Return to Main Menu", 
            command=safe_exit,
            style="Return.TButton"
        )
        return_btn.pack(side=tk.LEFT, pady=10, padx=5)
        
        # Exit button
        exit_btn = ttk.Button(button_frame, text="Exit", command=lambda: os._exit(0))
        exit_btn.pack(side=tk.RIGHT, pady=10, padx=5)
    
    add_exit_button(frame_eval_ang)
    add_exit_button(frame_eval_ee)
    add_exit_button(frame_launch)
    add_exit_button(frame_run)
    
    #########################
    
    # Eval ANG

    text_box = tk.Text(frame_eval_ang, height=6, width=45)
    text_box.pack(padx=10, pady=5)
    text_box.insert(tk.END,"Add all files and select a stadard name for output file 'ang_error' will become "  
                    "'ang_error_x' depending in the number of pairs of files to be processed, so calculate all files at once" )
    text_box.config(state=tk.DISABLED)
    
    create_files_selector(frame_eval_ang, "Robot Angles", "csv_robot_ang", [("CSV Files", "*.csv")])

    create_files_selector(frame_eval_ang, "Human Angles", "xlsx_human_ang", [("CSV Files", "*.xlsx")])
    
    def execute_calc_ang():
        if any(key not in files or files[key] is None for key in ["xlsx_human_ang"]):
            if any(key not in files or files[key] is None for key in["csv_robot_ang"]):
                messagebox.showerror("Error", "Please select all required files before proceeding.")
                return
        output_file = select_output_file(".xlsx")
        if not output_file:
            messagebox.showerror("Error", "Output file name is required!")
            return
        
        ii = 1
        for i,m in zip(files["xlsx_human_ang"] , files["csv_robot_ang"]):       
            output_file_temp = output_file.replace(".xlsx", "_" + str(ii) + ".xlsx")
            package_path = get_package_directory('manipulator_skill_acquisition')
            script_path = os.path.join(package_path, 'lib', 'manipulator_skill_acquisition', 
                                    'EvalnRunMenu', 'error_ang.py')
            run_script(script_path, [i, m, output_file_temp])
            ii = ii + 1
        ii = 1

    btn_eval = ttk.Button(frame_eval_ang, text="Calculate Angle error", command=execute_calc_ang)
    btn_eval.pack(pady=15)

    ###

    # Eval EE

    text_box = tk.Text(frame_eval_ee, height=6, width=45)
    text_box.pack(padx=10, pady=5)
    text_box.insert(tk.END,"Add all files and select a stadard name for output file 'ee_error' will become "
                    "'ee_error_x' depending in the number of pairs of files to be processed, so calculate all files at once" )
    text_box.config(state=tk.DISABLED)
    
    create_files_selector(frame_eval_ee, "Robot EE", "csv_robot_ee", [("CSV Files", "*.csv")])

    create_files_selector(frame_eval_ee, "Human EE", "csv_human_ee", [("CSV Files", "*.csv")])
    
    def execute_calc_ee():
        if any(key not in files or files[key] is None for key in ["csv_robot_ee"]):
            if any(key not in files or files[key] is None for key in["csv_human_ee"]):
                messagebox.showerror("Error", "Please select all required files before proceeding.")
                return
        output_file = select_output_file(".xlsx")
        if not output_file:
            messagebox.showerror("Error", "Output file name is required!")
            return
        
        ii = 1
        for i,m in zip(files["csv_human_ee"] , files["csv_robot_ee"]):       
            output_file_temp = output_file.replace(".xlsx", "_" + str(ii) + ".xlsx")
            package_path = get_package_directory('manipulator_skill_acquisition')
            script_path = os.path.join(package_path, 'lib', 'manipulator_skill_acquisition', 
                                    'EvalnRunMenu', 'error_ee.py')
            run_script(script_path, [i, m, output_file_temp])
            ii = ii + 1
        ii = 1

    btn_run_dmp = ttk.Button(frame_eval_ee, text="Calculate Angle error", command=execute_calc_ee)
    btn_run_dmp.pack(pady=15)

    ###

    # Launch
     # Add launch options
    launch_options_frame = ttk.LabelFrame(frame_launch, text="Launch Options", padding=10)
    launch_options_frame.pack(fill=tk.X, pady=10)
    
    # Controller selection
    controller_var = tk.StringVar(value="forward_position_controller")
    ttk.Radiobutton(launch_options_frame, text="Forward Position Controller", 
                   variable=controller_var, value="forward_position_controller").pack(anchor=tk.W)
    ttk.Radiobutton(launch_options_frame, text="Joint Trajectory Controller", 
                   variable=controller_var, value="joint_trajectory_controller").pack(anchor=tk.W)
    
    # Additional launch options
    rviz_var = tk.BooleanVar(value=True)
    joint_state_var = tk.BooleanVar(value=True)
    gazebo_var = tk.BooleanVar(value=True)
    
    ttk.Checkbutton(launch_options_frame, text="Launch RViz", 
                   variable=rviz_var).pack(anchor=tk.W)
    ttk.Checkbutton(launch_options_frame, text="Launch Joint State Publisher GUI", 
                   variable=joint_state_var).pack(anchor=tk.W)
    ttk.Checkbutton(launch_options_frame, text="Launch Gazebo", 
                   variable=gazebo_var).pack(anchor=tk.W)
    
    # World file selection
    world_frame = ttk.LabelFrame(launch_options_frame, text="World File", padding=5)
    world_frame.pack(fill=tk.X, pady=5)
    
    world_var = tk.StringVar(value="world.xml")
    world_entry = ttk.Entry(world_frame, textvariable=world_var, width=30)
    world_entry.pack(side=tk.LEFT, padx=5)
    
    def select_world_file():
        manipulator_pkg_path = get_package_share_directory('manipulator')
        initial_dir = os.path.join(manipulator_pkg_path, 'resources/worlds')
        file_path = filedialog.askopenfilename(
            initialdir=initial_dir,
            title="Select World File",
            filetypes=[("XML Files", "*.xml"), ("World Files", "*.world"), ("All Files", "*.*")]
        )
        if file_path:
            world_var.set(os.path.basename(file_path))
    
    world_btn = ttk.Button(world_frame, text="Browse", command=select_world_file)
    world_btn.pack(side=tk.LEFT, padx=5)

    def toggle_world_selection():
        if gazebo_var.get():
            world_frame.pack(fill=tk.X, pady=5)
        else:
            world_frame.pack_forget()

    # Add callback to toggle world selection when Gazebo checkbox changes
    gazebo_var.trace_add('write', lambda *args: toggle_world_selection())
    # Initial state
    toggle_world_selection()

    def ros_launch():
        global ros_launch_process
        env_setup = f"source {ROS_SETUP_SCRIPT}"
        if os.path.exists(WORKSPACE_SETUP):
            env_setup += f" && source {WORKSPACE_SETUP}"
        if os.path.exists(GAZEBO_SETUP):
            env_setup += f" && source {GAZEBO_SETUP}"

        # Build launch command with options
        command = 'ros2 launch manipulator launch.py'
        command += f' controller_type:={controller_var.get()}'
        if not rviz_var.get():
            command += ' rviz:=false'
        if not joint_state_var.get():
            command += ' joint_state_publisher:=false'
        if not gazebo_var.get():
            command += ' gazebo:=false'
        if gazebo_var.get() and world_var.get():
            command += f' world:={world_var.get()}'
        
        command = f"{env_setup} && {command}"
        if ros_launch_process is None or ros_launch_process.poll() is not None:
            if is_wsl():
                ros_launch_process = subprocess.Popen(["wsl.exe", "bash", "-c", command],
                stdout=subprocess.PIPE, # log_file
                stderr=subprocess.PIPE,
                )             
            else:
                ros_launch_process = subprocess.Popen(["gnome-terminal", "--", "bash", "-c", command],
                stdout=subprocess.PIPE, # log_file
                stderr=subprocess.PIPE,
                )

            threading.Thread(target=read_output, args=(ros_launch_process,output_text_launch), daemon=True).start()
            print("ROS launch started.")
        else:
            print("ROS already running.")

    def ros_launch_stop():

        global ros_launch_process

        if ros_launch_process and ros_launch_process.poll() is None:
            print("Stopping ROS launch...")             
            try:
                ros_launch_process.terminate() 
                ros_launch_process.wait(timeout=10)  
            except subprocess.TimeoutExpired:
                print("Force killing ROS process...")
                ros_launch_process.kill()
            finally:
                ros_launch_process = None
                print("ROS stopped.")
        else:
            print("No running ROS process to stop.")
    
    btn_run_trajectory = ttk.Button(frame_launch, text="Launch", command=ros_launch)
    btn_run_trajectory.pack(pady=20)

    btn_run_trajectory = ttk.Button(frame_launch, text="Stop", command=ros_launch_stop)
    btn_run_trajectory.pack(pady=20)

    output_text_launch = ScrolledText(frame_launch, height=30, width=150, state="disabled", wrap="word")
    output_text_launch.pack(padx=10, pady=10)
    
    ###

    # Run
    run_options_frame = ttk.LabelFrame(frame_run, text="Run Options", padding=10)
    run_options_frame.pack(fill=tk.X, pady=10)

    # MPX file selection
    mpx_frame = ttk.LabelFrame(run_options_frame, text="MPX File", padding=5)
    mpx_frame.pack(fill=tk.X, pady=5)
    
    mpx_var = tk.StringVar(value="")
    mpx_path_var = tk.StringVar(value=os.path.join(os.path.expanduser("~"), ""))
    mpx_entry = ttk.Entry(mpx_frame, textvariable=mpx_var, width=30)
    mpx_entry.pack(side=tk.LEFT, padx=5)
    
    def select_mpx_file():
        package_path = get_package_share_directory('manipulator_skill_acquisition')
        file_path = filedialog.askopenfilename(
            initialdir=os.path.join(package_path, 'resources'),
            title="Select MPX File",
            filetypes=[("MPX Files", "*.mpx"), ("All Files", "*.*")]
        )
        if file_path:
            mpx_path_var.set(file_path)
            mpx_var.set(os.path.basename(file_path))
    
    mpx_btn = ttk.Button(mpx_frame, text="Browse", command=select_mpx_file)
    mpx_btn.pack(side=tk.LEFT, padx=5)

    # RL Actor file selection
    rl_frame = ttk.LabelFrame(run_options_frame, text="RL Actor File", padding=5)
    rl_frame.pack(fill=tk.X, pady=5)
    
    rl_var = tk.StringVar(value="")
    rl_path_var = tk.StringVar(value=os.path.join(os.path.expanduser("~"), ""))
    rl_entry = ttk.Entry(rl_frame, textvariable=rl_var, width=30)
    rl_entry.pack(side=tk.LEFT, padx=5)
    
    def select_rl_file():
        package_path = get_package_share_directory('manipulator_skill_acquisition')
        file_path = filedialog.askopenfilename(
            initialdir=os.path.join(package_path, 'resources/rl'),
            title="Select RL Actor File",
            filetypes=[("PyTorch Files", "*.pt"), ("All Files", "*.*")]
        )
        if file_path:
            rl_path_var.set(file_path)
            rl_var.set(os.path.basename(file_path))
    
    rl_btn = ttk.Button(rl_frame, text="Browse", command=select_rl_file)
    rl_btn.pack(side=tk.LEFT, padx=5)

    def ros_run():
        global ros_run_process
        env_setup = f"source {ROS_SETUP_SCRIPT}"
        if os.path.exists(WORKSPACE_SETUP):
            env_setup += f" && source {WORKSPACE_SETUP}"
        if os.path.exists(GAZEBO_SETUP):
            env_setup += f" && source {GAZEBO_SETUP}"

        # Check if files exist
        mpx_path = mpx_path_var.get()
        rl_path = rl_path_var.get()
        
        if not os.path.exists(mpx_path):
            messagebox.showerror("Error", f"MPX file not found: {mpx_var.get()}")
            return
            
        if not os.path.exists(rl_path):
            messagebox.showerror("Error", f"RL Actor file not found: {rl_var.get()}")
            return

        # Use PYTHONUNBUFFERED to ensure unbuffered output
        command = f'PYTHONUNBUFFERED=1 ros2 run manipulator_control_strategies load_dmps.py {mpx_path} {rl_path}'
        
        command = f"{env_setup} && {command}"
        if ros_run_process is None or ros_run_process.poll() is not None:
            if is_wsl():
                ros_run_process = subprocess.Popen(
                    ["wsl.exe", "bash", "-c", command],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=False,  # Use bytes mode
                    bufsize=0,  # Unbuffered
                )
            else:
                ros_run_process = subprocess.Popen(
                    ["gnome-terminal", "--", "bash", "-c", command],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=False,  # Use bytes mode
                    bufsize=0,  # Unbuffered
                )
            
            # Set up non-blocking mode for the output streams
            def set_non_blocking(fd):
                flags = fcntl.fcntl(fd, fcntl.F_GETFL)
                fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
            
            try:
                set_non_blocking(ros_run_process.stdout.fileno())
                set_non_blocking(ros_run_process.stderr.fileno())
            except Exception as e:
                print(f"Error setting non-blocking mode: {e}")
            
            threading.Thread(target=read_output, args=(ros_run_process, output_text_run), daemon=True).start()
            print("ROS2 run started.")
        else:
            print("ROS2 run already running.")

    def ros_run_stop():
        global ros_run_process
        if ros_run_process and ros_run_process.poll() is None:
            print("Stopping ROS launch...")             
            try:
                ros_run_process.terminate() 
                ros_run_process.wait(timeout=10)  
            except subprocess.TimeoutExpired:
                print("Force killing ROS process...")
                ros_run_process.kill()
            finally:
                ros_run_process = None
                print("ROS stopped.")
        else:
            print("No running ROS process to stop.")
    
    btn_run_trajectory = ttk.Button(frame_run, text="Run", command=ros_run)
    btn_run_trajectory.pack(pady=20)

    btn_run_trajectory = ttk.Button(frame_run, text="Stop", command=ros_run_stop)
    btn_run_trajectory.pack(pady=20)

    output_text_run = ScrolledText(frame_run, height=30, width=150, state="disabled", wrap="word")
    output_text_run.pack(padx=10, pady=10)
    
    ###
    
    root.mainloop()

if __name__ == "__main__":
    setup_gui()