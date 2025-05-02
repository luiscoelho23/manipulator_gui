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
import yaml  # Add yaml module for parsing controllers.yaml

ROS_SETUP_SCRIPT = "/opt/ros/humble/setup.bash" 
current_file = os.path.abspath(__file__)

if '/install/' in current_file:
    ws_path = current_file[:current_file.find('/install/')]
elif '/src/' in current_file:
    ws_path = current_file[:current_file.find('/src/')]
else:
    print("Error: Could not determine workspace path. Script must be run from install or src directory.")
    sys.exit(1)

WORKSPACE_SETUP = os.path.join(ws_path, 'install/setup.bash')
GAZEBO_SETUP = "/usr/share/gazebo/setup.sh"

ros_launch_process = None
ros_run_process = None

files = {}
labels = {}

def get_package_directory(package_name):
    """Get the package directory for a given package name."""
    return get_package_prefix(package_name)

def load_controllers_from_yaml():
    """Load controller names and types from controllers.yaml"""
    try:
        # Get path to controllers.yaml
        manipulator_pkg_path = get_package_share_directory('manipulator')
        controllers_yaml_path = os.path.join(manipulator_pkg_path, 'config', 'controllers.yaml')
        
        # Check if file exists in share directory, if not try from source
        if not os.path.exists(controllers_yaml_path):
            # Fallback to the source directory
            controllers_yaml_path = os.path.join(ws_path, 'src', 'manipulator', 'config', 'controllers.yaml')
            
        if not os.path.exists(controllers_yaml_path):
            print(f"Could not find controllers.yaml in {controllers_yaml_path}")
            return {}, {}
            
        # Load YAML file
        with open(controllers_yaml_path, 'r') as file:
            controllers_config = yaml.safe_load(file)
            
        # Extract controller names and types
        controller_types = {}
        friendly_names = {}
        
        if 'controller_manager' in controllers_config and 'ros__parameters' in controllers_config['controller_manager']:
            controllers = controllers_config['controller_manager']['ros__parameters']
            # Skip update_rate key
            for controller_name, controller_info in controllers.items():
                if isinstance(controller_info, dict) and 'type' in controller_info:
                    controller_type = controller_info['type']
                    controller_types[controller_name] = controller_type
                    
                    # Create friendly name using the convert_to_friendly_name function
                    friendly_names[controller_name] = convert_to_friendly_name(controller_name, controller_type)
        
        return controller_types, friendly_names
    except Exception as e:
        print(f"Error loading controllers from yaml: {e}")
        return {}, {}

def convert_to_friendly_name(controller_name, controller_type):
    """Convert controller name to a human-readable friendly name.
    
    Args:
        controller_name: The raw controller name from ROS
        controller_type: The controller type from ROS
        
    Returns:
        A human-readable friendly name
    """
    # Handle common controller types
    if 'joint_state_broadcaster' in controller_name:
        return "Joint State Broadcaster"
        
    if 'joint_trajectory_controller' in controller_name:
        if 'position' in controller_name:
            return "Joint Trajectory Controller Position"
        elif 'effort' in controller_name:
            return "Joint Trajectory Controller Effort"
        else:
            return "Joint Trajectory Controller"
            
    if 'forward_position_controller' in controller_name:
        return "Forward Position Controller"
        
    # For other controllers, create a title-cased name from the base name
    words = []
    for word in controller_name.split('_'):
        words.append(word.capitalize())
    
    return " ".join(words)

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
        # Try to create a pop-up warning if GUI output fails
        try:
            messagebox.showerror("Output Error", f"Failed to update output: {e}")
        except:
            pass  # Even messagebox failed, nothing more we can do

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
    
    # Function to restart the ROS2 daemon
    def restart_ros2_daemon():
        try:
            env_setup = f"source {ROS_SETUP_SCRIPT}"
            if os.path.exists(WORKSPACE_SETUP):
                env_setup += f" && source {WORKSPACE_SETUP}"
            
            # First stop the daemon
            stop_cmd = f"{env_setup} && ros2 daemon stop"
            print("Stopping ROS2 daemon...")
            
            try:
                proc = subprocess.Popen(["bash", "-c", stop_cmd], 
                                      stdout=subprocess.PIPE, 
                                      stderr=subprocess.PIPE, 
                                      text=True)
                stdout, stderr = proc.communicate(timeout=5)
                
                if stderr and "Error" in stderr:
                    print(f"Warning when stopping daemon: {stderr}")
                else:
                    print("ROS2 daemon stopped.")
                
                # Small delay to ensure daemon is fully stopped
                time.sleep(1)
                
                # Then start the daemon
                start_cmd = f"{env_setup} && ros2 daemon start"
                print("Starting ROS2 daemon...")
                
                proc = subprocess.Popen(["bash", "-c", start_cmd], 
                                      stdout=subprocess.PIPE, 
                                      stderr=subprocess.PIPE, 
                                      text=True)
                stdout, stderr = proc.communicate(timeout=5)
                
                if stderr and "Error" in stderr:
                    print(f"Error starting daemon: {stderr}")
                    messagebox.showerror("Error", f"Failed to start ROS2 daemon: {stderr}")
                else:
                    print("ROS2 daemon started successfully.")
                    messagebox.showinfo("Success", "ROS2 daemon restarted successfully")
                
            except subprocess.TimeoutExpired:
                print("Command timed out. ROS2 may be unresponsive.")
                messagebox.showerror("Timeout", "Command timed out. ROS2 may be unresponsive.")
            except Exception as e:
                print(f"Error during daemon restart: {str(e)}")
                messagebox.showerror("Error", f"Error during daemon restart: {str(e)}")
        except Exception as e:
            print(f"Failed to restart ROS2 daemon: {str(e)}")
            messagebox.showerror("Error", f"Failed to restart ROS2 daemon: {str(e)}")

    # Add ROS2 daemon restart button at the top of the window
    daemon_frame = ttk.Frame(root)
    daemon_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=5, before=notebook)
    
    daemon_label = ttk.Label(daemon_frame, text="ROS2 Daemon Control:")
    daemon_label.pack(side=tk.LEFT, padx=5)
    
    restart_daemon_btn = ttk.Button(
        daemon_frame, 
        text="Restart ROS2 Daemon", 
        command=restart_ros2_daemon,
        style="Return.TButton"
    )
    restart_daemon_btn.pack(side=tk.LEFT, padx=5)
    
    # Create frames for each tab
    frame_eval_ang = ttk.Frame(notebook, padding=20)
    frame_eval_ee = ttk.Frame(notebook, padding=20)
    frame_launch = ttk.Frame(notebook, padding=20)
    frame_run = ttk.Frame(notebook, padding=20)
    frame_controller = ttk.Frame(notebook, padding=20)
    
    # Add tabs
    notebook.add(frame_eval_ang, text="Evaluate Ang traj")
    notebook.add(frame_eval_ee, text="Evaluate EE traj")
    notebook.add(frame_launch, text="Launch")
    notebook.add(frame_run, text="Run")
    
    # We'll add the controller tab last to avoid issues
    
    # Add exit buttons to each tab
    def add_exit_button(frame):
        def return_to_main():
            try:
                # Stop ROS launch processes
                if ros_launch_process is not None:
                    ros_launch_process.terminate()
                    ros_launch_process.wait(timeout=5)
                
                # Stop ROS run processes
                if ros_run_process is not None:
                    ros_run_process.terminate()
                    ros_run_process.wait(timeout=5)
                
                # Additional cleanup for any remaining processes
                try:
                    if is_wsl():
                        subprocess.run(["wsl.exe", "bash", "-c", "pkill -f ros2"], 
                                     stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    else:
                        subprocess.run(["pkill", "-f", "ros2"], 
                                     stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                except Exception as e:
                    print(f"Error during cleanup: {e}")
                
                # Close the window
                root.destroy()
                
                # Import and launch the main GUI menu
                try:
                    gui_module = importlib.import_module('manipulator_gui.GUI')
                    gui_module.launch_gui()
                except Exception as e:
                    print(f"Error returning to main menu: {e}")
                    # If import fails, just destroy the window
                    root.destroy()
                
            except Exception as e:
                print(f"Error during return to main: {e}")
                # On error, just destroy the window
                root.destroy()

        def exit_application():
            try:
                # Stop ROS launch processes
                if ros_launch_process is not None:
                    ros_launch_process.terminate()
                    ros_launch_process.wait(timeout=5)
                
                # Stop ROS run processes
                if ros_run_process is not None:
                    ros_run_process.terminate()
                    ros_run_process.wait(timeout=5)
                
                # Additional cleanup for any remaining processes
                try:
                    if is_wsl():
                        subprocess.run(["wsl.exe", "bash", "-c", "pkill -f ros2"], 
                                     stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    else:
                        subprocess.run(["pkill", "-f", "ros2"], 
                                     stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                except Exception as e:
                    print(f"Error during cleanup: {e}")
                
                # Close the window and exit completely
                root.destroy()
                os._exit(0)
                
            except Exception as e:
                print(f"Error during exit: {e}")
                os._exit(1)

        button_frame = ttk.Frame(frame)
        button_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=10)
        
        # Return to Main Menu button
        return_btn = ttk.Button(
            button_frame, 
            text="Return to Main Menu", 
            command=return_to_main,
            style="Return.TButton"
        )
        return_btn.pack(side=tk.LEFT, pady=10, padx=5)
        
        # Exit button
        exit_btn = ttk.Button(button_frame, text="Exit", command=exit_application)
        exit_btn.pack(side=tk.RIGHT, pady=10, padx=5)
    
    add_exit_button(frame_eval_ang)
    add_exit_button(frame_eval_ee)
    add_exit_button(frame_launch)
    add_exit_button(frame_run)
    add_exit_button(frame_controller)
    
    # Now add the controller tab after everything else is set up
    notebook.add(frame_controller, text="Switch Controller")
    
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
    
    # Controller type selection
    controller_frame = ttk.LabelFrame(launch_options_frame, text="Controller Type", padding=5)
    controller_frame.pack(fill=tk.X, pady=5)
    
    controller_var = tk.StringVar(value="forward_position_controller")
    ttk.Radiobutton(controller_frame, text="Forward Position Controller", 
                   variable=controller_var, value="forward_position_controller").pack(anchor=tk.W)
    ttk.Radiobutton(controller_frame, text="Joint Trajectory Controller", 
                   variable=controller_var, value="joint_trajectory_controller").pack(anchor=tk.W)
    
    # Control mode selection (only visible for joint trajectory controller)
    control_mode_frame = ttk.LabelFrame(launch_options_frame, text="Control Mode", padding=5)
    
    control_mode = tk.StringVar(value="position")
    ttk.Radiobutton(control_mode_frame, text="Position Control", 
                   variable=control_mode, value="position").pack(anchor=tk.W)
    ttk.Radiobutton(control_mode_frame, text="Effort Control", 
                   variable=control_mode, value="effort").pack(anchor=tk.W)
    
    def toggle_control_mode(*args):
        if controller_var.get() == "joint_trajectory_controller":
            control_mode_frame.pack(fill=tk.X, pady=5)
        else:
            control_mode_frame.pack_forget()
    
    # Add callback to toggle control mode visibility
    controller_var.trace_add('write', toggle_control_mode)
    # Initial state
    toggle_control_mode()
    
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
        
        # Set the correct controller name based on selection
        if controller_var.get() == "joint_trajectory_controller":
            controller_name = f"joint_trajectory_controller_{control_mode.get()}"
        else:
            controller_name = controller_var.get()
            
        command += f' controller_type:={controller_name}'
            
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
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                )             
            else:
                ros_launch_process = subprocess.Popen(["gnome-terminal", "--", "bash", "-c", command],
                stdout=subprocess.PIPE,
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
                # First try to terminate gracefully
                ros_launch_process.terminate() 
                ros_launch_process.wait(timeout=5)  
            except subprocess.TimeoutExpired:
                print("Force killing ROS process...")
                try:
                    # If graceful termination fails, force kill
                    ros_launch_process.kill()
                    ros_launch_process.wait(timeout=5)
                except Exception as e:
                    print(f"Error killing process: {e}")
            finally:
                ros_launch_process = None
                print("ROS stopped.")
        else:
            print("No running ROS process to stop.")
    
    btn_run_trajectory = ttk.Button(frame_launch, text="Launch", command=ros_launch)
    btn_run_trajectory.pack(pady=20)

    btn_run_trajectory = ttk.Button(frame_launch, text="Stop", command=ros_launch_stop)
    btn_run_trajectory.pack(pady=20)

    output_text_launch = ScrolledText(frame_launch, height=25, width=150, state="disabled", wrap="word")
    output_text_launch.pack(padx=10, pady=10, fill=tk.BOTH, expand=True)
    
    ###

    # Run
    run_options_frame = ttk.LabelFrame(frame_run, text="Run Options", padding=10)
    run_options_frame.pack(fill=tk.X, pady=10)

    # Script selection
    script_frame = ttk.LabelFrame(run_options_frame, text="Script Selection", padding=5)
    script_frame.pack(fill=tk.X, pady=5)
    
    script_var = tk.StringVar(value="load_dmps")
    ttk.Radiobutton(script_frame, text="Run DMP (load_dmps.py)", 
                   variable=script_var, value="load_dmps").pack(anchor=tk.W)
    ttk.Radiobutton(script_frame, text="Send to Position (send_to_pos.py)", 
                   variable=script_var, value="send_to_pos").pack(anchor=tk.W)
    
    # Parameters frame that will change based on script selection
    params_frame = ttk.Frame(run_options_frame)
    params_frame.pack(fill=tk.X, pady=5)
    
    # Frames for different script parameters
    dmp_params_frame = ttk.LabelFrame(params_frame, text="DMP Parameters", padding=5)
    pos_params_frame = ttk.LabelFrame(params_frame, text="Position Parameters", padding=5)
    
    # Show the appropriate parameter frame based on script selection
    def update_params_frame(*args):
        # Hide all parameter frames
        dmp_params_frame.pack_forget()
        pos_params_frame.pack_forget()
        
        # Show the appropriate frame
        if script_var.get() == "load_dmps":
            dmp_params_frame.pack(fill=tk.X, pady=5)
        else:
            pos_params_frame.pack(fill=tk.X, pady=5)
    
    # Add callback to update parameter frame when script selection changes
    script_var.trace_add('write', update_params_frame)
    
    # ---- DMP Parameters ----
    # MPX file selection
    mpx_frame = ttk.LabelFrame(dmp_params_frame, text="MPX File", padding=5)
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
    rl_frame = ttk.LabelFrame(dmp_params_frame, text="RL Actor File", padding=5)
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
    
    # ---- Position Parameters ----
    # Joint position input
    joint_frame = ttk.Frame(pos_params_frame)
    joint_frame.pack(fill=tk.X, pady=5)
    
    joint_values = []
    for i in range(7):
        joint_frame_row = ttk.Frame(joint_frame)
        joint_frame_row.pack(fill=tk.X, pady=2)
        
        ttk.Label(joint_frame_row, text=f"Joint {i+1}:").pack(side=tk.LEFT, padx=5)
        joint_var = tk.DoubleVar(value=0.0)
        joint_values.append(joint_var)
        joint_entry = ttk.Entry(joint_frame_row, textvariable=joint_var, width=10)
        joint_entry.pack(side=tk.LEFT, padx=5)
        
        if i == 0:  # Add units label on first row
            ttk.Label(joint_frame_row, text="(radians)").pack(side=tk.LEFT, padx=5)
    
    # Add preset buttons for common positions
    preset_frame = ttk.LabelFrame(pos_params_frame, text="Position Presets", padding=5)
    preset_frame.pack(fill=tk.X, pady=5)
    
    def set_home_position():
        values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        for i, val in enumerate(values):
            joint_values[i].set(val)
    
    def set_ready_position():
        values = [0.0, -0.3, 0.0, -1.8, 0.0, 1.5, 0.0]
        for i, val in enumerate(values):
            joint_values[i].set(val)
    
    preset_buttons_frame = ttk.Frame(preset_frame)
    preset_buttons_frame.pack(fill=tk.X, pady=2)
    
    ttk.Button(preset_buttons_frame, text="Home Position", 
              command=set_home_position).pack(side=tk.LEFT, padx=5, pady=5)
    ttk.Button(preset_buttons_frame, text="Ready Position", 
              command=set_ready_position).pack(side=tk.LEFT, padx=5, pady=5)

    # Debug options (shared between both scripts)
    debug_frame = ttk.LabelFrame(run_options_frame, text="Debug Options", padding=5)
    debug_frame.pack(fill=tk.X, pady=5)
    
    debug_var = tk.BooleanVar(value=False)
    debug_check = ttk.Checkbutton(debug_frame, text="Enable Debug Mode (run trajectory test & verbose logging)", 
                                 variable=debug_var)
    debug_check.pack(anchor=tk.W, padx=5, pady=2)

    def ros_run():
        global ros_run_process
        env_setup = f"source {ROS_SETUP_SCRIPT}"
        if os.path.exists(WORKSPACE_SETUP):
            env_setup += f" && source {WORKSPACE_SETUP}"
        if os.path.exists(GAZEBO_SETUP):
            env_setup += f" && source {GAZEBO_SETUP}"

        # Add debug flag
        debug_flag = "--debug" if debug_var.get() else ""
        
        # Determine which script to run
        script_type = script_var.get()
        
        if script_type == "load_dmps":
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
            command = f'PYTHONUNBUFFERED=1 ros2 run manipulator_control_strategies load_dmps.py {mpx_path} {rl_path} {debug_flag}'
        else:  # send_to_pos
            # Get joint values
            joint_positions = [str(var.get()) for var in joint_values]
            
            # Use PYTHONUNBUFFERED to ensure unbuffered output
            command = f'PYTHONUNBUFFERED=1 ros2 run manipulator_control_strategies send_to_pos.py {" ".join(joint_positions)} {debug_flag}'
        
        # Log the command
        append_output(f"Running command: {command}\n", output_text_run)
        
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
            print("Stopping ROS run...")             
            try:
                # First try to terminate gracefully
                ros_run_process.terminate() 
                ros_run_process.wait(timeout=5)  
            except subprocess.TimeoutExpired:
                print("Force killing ROS process...")
                try:
                    # If graceful termination fails, force kill
                    ros_run_process.kill()
                    ros_run_process.wait(timeout=5)
                except Exception as e:
                    print(f"Error killing process: {e}")
            finally:
                ros_run_process = None
                print("ROS stopped.")
        else:
            print("No running ROS process to stop.")
    
    # Add run and stop buttons
    button_frame = ttk.Frame(frame_run)
    button_frame.pack(pady=10)
    
    btn_run = ttk.Button(button_frame, text="Run", command=ros_run)
    btn_run.pack(side=tk.LEFT, padx=10, pady=10)
    
    btn_stop = ttk.Button(button_frame, text="Stop", command=ros_run_stop)
    btn_stop.pack(side=tk.LEFT, padx=10, pady=10)
    
    # Add output text area
    output_text_run = ScrolledText(frame_run, height=25, width=150, state="disabled", wrap="word")
    output_text_run.pack(padx=10, pady=10, fill=tk.BOTH, expand=True)
    
    # Show initial parameter frame
    update_params_frame()
    
    ###
    
    # Controller Switching Tab
    controller_options_frame = ttk.LabelFrame(frame_controller, text="Controller Switching", padding=10)
    controller_options_frame.pack(fill=tk.X, pady=10)
    
    # Available Controllers
    available_controllers = ttk.LabelFrame(controller_options_frame, text="Available Controllers", padding=5)
    available_controllers.pack(fill=tk.X, pady=5)
    
    controller_listbox = tk.Listbox(available_controllers, height=6, width=40)
    controller_listbox.pack(padx=5, pady=5, fill=tk.X)
    
    # Active Controller
    active_controller_frame = ttk.LabelFrame(controller_options_frame, text="Active Controller", padding=5)
    active_controller_frame.pack(fill=tk.X, pady=5)
    
    active_controller_label = ttk.Label(active_controller_frame, text="None")
    active_controller_label.pack(padx=5, pady=5)
    
    # Controller selection options
    selection_frame = ttk.LabelFrame(controller_options_frame, text="Controller Selection", padding=5)
    selection_frame.pack(fill=tk.X, pady=5)
    
    # Create a notebook for different selection methods
    selection_notebook = ttk.Notebook(selection_frame)
    selection_notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
    
    # Tab for selecting from list
    list_selection_frame = ttk.Frame(selection_notebook, padding=5)
    selection_notebook.add(list_selection_frame, text="From List")
    
    list_instruction = ttk.Label(list_selection_frame, text="Select a controller from the list above")
    list_instruction.pack(pady=5)
    
    # Tab for preset controllers
    preset_selection_frame = ttk.Frame(selection_notebook, padding=5)
    selection_notebook.add(preset_selection_frame, text="From Presets")
    
    # Get controller info from YAML
    controller_types, friendly_names = load_controllers_from_yaml()
    
    # Filter out joint_state_broadcaster
    preset_controllers = [name for name in controller_types.keys() 
                         if name != 'joint_state_broadcaster' and name != 'update_rate']
    
    if not preset_controllers:
        # Fallback to default controllers if YAML loading failed
        preset_controllers = [
            "forward_position_controller",
            "joint_trajectory_controller_position",
            "joint_trajectory_controller_effort"
        ]
    
    preset_var = tk.StringVar(value=preset_controllers[0] if preset_controllers else "")
    for controller in preset_controllers:
        friendly_name = friendly_names.get(controller, controller)
        ttk.Radiobutton(preset_selection_frame, text=friendly_name, 
                       variable=preset_var, value=controller).pack(anchor=tk.W)
    
    # Controller switching functions
    def list_controllers():
        try:
            global ros_controller_process
            env_setup = f"source {ROS_SETUP_SCRIPT}"
            if os.path.exists(WORKSPACE_SETUP):
                env_setup += f" && source {WORKSPACE_SETUP}"
            
            command = f"{env_setup} && ros2 control list_controllers"
            append_output(f"Running command: {command}\n", output_text_controller)
            
            # Load controller info from YAML
            controller_types, friendly_names = load_controllers_from_yaml()
            
            try:
                proc = subprocess.Popen(
                    ["bash", "-c", command], 
                    stdout=subprocess.PIPE, 
                    stderr=subprocess.PIPE, 
                    text=True
                )
                stdout, stderr = proc.communicate(timeout=5)
                
                # Debug output
                append_output(f"Command output:\n{stdout}\n", output_text_controller)
                if stderr:
                    append_output(f"Command errors:\n{stderr}\n", output_text_controller)
                
                if proc.returncode == 0:
                    # Clear the listbox
                    controller_listbox.delete(0, tk.END)
                    
                    # Parse the output
                    active_controllers = []
                    available_controllers = []
                    
                    # Skip header line if present
                    lines = stdout.splitlines()
                    start_idx = 0
                    for i, line in enumerate(lines):
                        if "controller_name" in line and "state" in line:
                            start_idx = i + 1
                            break
                    
                    # Process controller lines
                    for line in lines[start_idx:]:
                        line = line.strip()
                        if not line:
                            continue
                        
                        try:
                            # Extract controller name, type and state from the line
                            # Example format:
                            # "joint_trajectory_controller_position joint_trajectory_controller/JointTrajectoryController  inactive"
                            parts = line.split()
                            
                            if len(parts) >= 3:
                                # First part is controller name
                                controller_name = parts[0].strip()
                                
                                # Last part is state (active/inactive)
                                controller_state = parts[-1].strip()
                                
                                # All parts in between are the controller type
                                controller_type = ' '.join(parts[1:-1]).strip()
                                
                                # Get or create friendly name
                                display_name = friendly_names.get(controller_name, 
                                                              convert_to_friendly_name(controller_name, controller_type))
                                
                                # Debug raw state information
                                append_output(f"Debug - Controller: {controller_name}, Raw state: '{controller_state}'\n", 
                                           output_text_controller)
                                
                                # Skip broadcasters (they're not real controllers)
                                if 'broadcaster' in controller_name.lower():
                                    append_output(f"Skipping broadcaster: {display_name}\n", output_text_controller)
                                    continue
                                
                                # Log the discovered controller
                                append_output(f"Found controller: {display_name} ({controller_state})\n", 
                                           output_text_controller)
                                
                                # Check if controller is active - more lenient matching for active state
                                is_active = False
                                
                                # Try multiple approaches to detect active state
                                state_lower = controller_state.lower().strip()
                                append_output(f"Checking state: '{state_lower}'\n", output_text_controller)
                                
                                # Consider a controller active if it doesn't have "inactive" in the state
                                if "inactive" not in state_lower:
                                    is_active = True
                                    append_output(f"Controller is ACTIVE (state doesn't contain 'inactive')\n", 
                                               output_text_controller)
                                else:
                                    append_output(f"Controller is INACTIVE\n", 
                                               output_text_controller)
                                
                                # Add to the appropriate lists
                                if is_active:
                                    active_controllers.append((controller_name, display_name))
                                    append_output(f"Found ACTIVE controller: {display_name}\n", output_text_controller)
                                else:
                                    append_output(f"Controller is INACTIVE: {display_name}\n", output_text_controller)
                                
                                # Add all controllers (not broadcasters) to the available list
                                available_controllers.append((controller_name, display_name))
                            else:
                                append_output(f"Skipping line with invalid format: {line}\n", output_text_controller)
                        except Exception as e:
                            append_output(f"Error parsing line '{line}': {str(e)}\n", output_text_controller)
                    
                    # Add all available controllers to the listbox
                    for _, display_name in available_controllers:
                        controller_listbox.insert(tk.END, display_name)
                    
                    # Update active controller display
                    if active_controllers:
                        # Use the first active controller
                        main_active = active_controllers[0]
                        
                        # Update the active controller label
                        active_controller_label.config(text=main_active[1])
                        append_output(f"Active controller: {main_active[1]}\n", output_text_controller)
                    else:
                        active_controller_label.config(text="None")
                        append_output("No active controllers found.\n", output_text_controller)
                    
                    append_output("Controller list updated.\n", output_text_controller)
                else:
                    append_output(f"Error listing controllers: {stderr}\n", output_text_controller)
                    append_output("Make sure ROS2 is running and properly configured.\n", output_text_controller)
            except subprocess.TimeoutExpired:
                append_output("Command timed out. ROS2 may be unresponsive.\n", output_text_controller)
            except Exception as e:
                append_output(f"Error executing command: {str(e)}\n", output_text_controller)
        except Exception as e:
            print(f"Critical error in list_controllers: {str(e)}")
            try:
                append_output(f"An error occurred: {str(e)}. Please restart the application if issues persist.\n", 
                             output_text_controller)
            except:
                # If we can't even update the output, just print to console
                print("Cannot update GUI - critical error")
                pass
            
    # Controller selection options
    def switch_to_selected_controller():
        # Load controller info from YAML
        controller_types, friendly_names = load_controllers_from_yaml()
        
        # Build reverse mapping
        reverse_friendly_names = {v: k for k, v in friendly_names.items()}
        
        # Determine which tab is active and get the selected controller
        selected_display_name = None
        
        if selection_notebook.index(selection_notebook.select()) == 0:
            # "From List" tab is active
            selected_indices = controller_listbox.curselection()
            if not selected_indices:
                messagebox.showerror("Error", "Please select a controller from the list.")
                return
            
            # Get the selected controller name
            selected_display_name = controller_listbox.get(selected_indices[0])
            
            # Debug selected controller
            append_output(f"Selected controller from list: {selected_display_name}\n", output_text_controller)
            
            # Check if it's already active
            current_active_display = active_controller_label.cget("text")
            if selected_display_name == current_active_display:
                messagebox.showinfo("Info", "The selected controller is already active.")
                return
        else:
            # "From Presets" tab is active
            selected_controller = preset_var.get()
            selected_display_name = friendly_names.get(selected_controller, selected_controller)
            
            # Debug selected controller
            append_output(f"Selected controller from preset: {selected_display_name} (internal name: {selected_controller})\n", 
                         output_text_controller)
            
            # Check if it's already active
            current_active_display = active_controller_label.cget("text")
            if selected_display_name == current_active_display:
                messagebox.showinfo("Info", "The selected controller is already active.")
                return
        
        if not selected_display_name:
            messagebox.showerror("Error", "No controller selected.")
            return
        
        # Convert friendly name back to actual controller name
        selected_controller = reverse_friendly_names.get(selected_display_name, selected_display_name)
        append_output(f"Using internal controller name: {selected_controller}\n", output_text_controller)
        
        # Proceed with controller switching
        env_setup = f"source {ROS_SETUP_SCRIPT}"
        if os.path.exists(WORKSPACE_SETUP):
            env_setup += f" && source {WORKSPACE_SETUP}"
        
        # Get current active controller
        current_active_display = active_controller_label.cget("text")
        current_active = reverse_friendly_names.get(current_active_display, current_active_display)
        append_output(f"Current active controller: {current_active_display} (internal name: {current_active})\n", 
                     output_text_controller)
        
        try:
            # Display the command being executed
            if current_active and current_active != "None":
                deactivate_cmd = f"{env_setup} && ros2 control switch_controllers --deactivate {current_active}"
                append_output(f"Executing: {deactivate_cmd}\n", output_text_controller)
                
                # First deactivate the current controller if one is active
                process = subprocess.Popen(
                    ["bash", "-c", deactivate_cmd],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True
                )
                stdout, stderr = process.communicate(timeout=10)
                
                if process.returncode != 0:
                    append_output(f"Error deactivating controller: {stderr}\n", output_text_controller)
                    messagebox.showerror("Error", f"Failed to deactivate controller:\n{stderr}")
                    return
                
                append_output(f"Successfully deactivated: {current_active}\n", output_text_controller)
            
            # Then activate the new controller
            activate_cmd = f"{env_setup} && ros2 control switch_controllers --activate {selected_controller}"
            append_output(f"Executing: {activate_cmd}\n", output_text_controller)
            
            process = subprocess.Popen(
                ["bash", "-c", activate_cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            stdout, stderr = process.communicate(timeout=10)
            
            if process.returncode != 0:
                append_output(f"Error activating controller: {stderr}\n", output_text_controller)
                messagebox.showerror("Error", f"Failed to activate controller:\n{stderr}")
                return
            
            append_output(f"Successfully activated: {selected_controller}\n", output_text_controller)
            active_controller_label.config(text=selected_display_name)
            
            # Refresh controller list to show updated status
            list_controllers()
            
        except subprocess.TimeoutExpired:
            append_output("Command timed out after 10 seconds\n", output_text_controller)
            messagebox.showerror("Error", "The command timed out. Check if ROS2 is running properly.")
        except Exception as e:
            append_output(f"Error: {str(e)}\n", output_text_controller)
            messagebox.showerror("Error", f"An unexpected error occurred:\n{str(e)}")
        
        append_output("Controller switch operation completed.\n", output_text_controller)
    
    # Single switch button for both methods
    button_frame = ttk.Frame(controller_options_frame)
    button_frame.pack(pady=5, padx=10, anchor=tk.CENTER)
    
    switch_button = ttk.Button(button_frame, text="Switch to Selected Controller", 
                             command=switch_to_selected_controller, width=25)
    switch_button.pack(pady=5, padx=5, side=tk.LEFT)
    
    # Add a refresh button
    refresh_btn = ttk.Button(button_frame, text="Refresh Controller List", 
                          command=list_controllers, width=20)
    refresh_btn.pack(pady=5, padx=5, side=tk.LEFT)
    
    # Function to activate a preset controller
    def activate_preset_controller(controller_name, display_name):
        env_setup = f"source {ROS_SETUP_SCRIPT}"
        if os.path.exists(WORKSPACE_SETUP):
            env_setup += f" && source {WORKSPACE_SETUP}"
        
        append_output(f"Activating preset controller: {display_name} ({controller_name})\n", output_text_controller)
        
        try:
            # Find current active controllers and deactivate them
            list_cmd = f"{env_setup} && ros2 control list_controllers"
            proc = subprocess.Popen(
                ["bash", "-c", list_cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            stdout, stderr = proc.communicate(timeout=10)
            
            if proc.returncode != 0:
                append_output(f"Error listing controllers: {stderr}\n", output_text_controller)
                messagebox.showerror("Error", f"Failed to list controllers:\n{stderr}")
                return
            
            active_controllers = []
            for line in stdout.splitlines():
                append_output(f"Checking controller line: {line}\n", output_text_controller)
                
                parts = line.split()
                if len(parts) < 2:
                    continue
                    
                controller_name = parts[0]
                controller_state = parts[-1].lower()
                
                # Skip broadcasters
                if 'broadcaster' in controller_name.lower():
                    continue
                
                append_output(f"Controller: {controller_name}, State: '{controller_state}'\n", output_text_controller)
                
                # Consider a controller active if it doesn't have "inactive" in the state
                if "inactive" not in controller_state:
                    active_controllers.append(controller_name)
                    append_output(f"Found active controller: {controller_name}\n", output_text_controller)
                else:
                    append_output(f"Controller is inactive: {controller_name}\n", output_text_controller)
            
            append_output(f"Found active controllers: {active_controllers}\n", output_text_controller)
            
            # Deactivate active controllers
            if active_controllers:
                deactivate_cmd = f"{env_setup} && ros2 control switch_controllers --deactivate {' '.join(active_controllers)}"
                append_output(f"Executing: {deactivate_cmd}\n", output_text_controller)
                
                proc = subprocess.Popen(
                    ["bash", "-c", deactivate_cmd],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True
                )
                stdout, stderr = proc.communicate(timeout=10)
                
                if proc.returncode != 0:
                    append_output(f"Error deactivating controllers: {stderr}\n", output_text_controller)
                    messagebox.showerror("Error", f"Failed to deactivate controllers:\n{stderr}")
                    return
                
                append_output("Successfully deactivated current controllers\n", output_text_controller)
            
            # Activate the new controller
            activate_cmd = f"{env_setup} && ros2 control switch_controllers --activate {controller_name}"
            append_output(f"Executing: {activate_cmd}\n", output_text_controller)
            
            proc = subprocess.Popen(
                ["bash", "-c", activate_cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            stdout, stderr = proc.communicate(timeout=10)
            
            if proc.returncode != 0:
                append_output(f"Error activating controller: {stderr}\n", output_text_controller)
                messagebox.showerror("Error", f"Failed to activate controller:\n{stderr}")
                return
            
            append_output(f"Successfully activated: {controller_name}\n", output_text_controller)
            active_controller_label.config(text=display_name)
            
            # Refresh controller list
            list_controllers()
            
        except subprocess.TimeoutExpired:
            append_output("Command timed out after 10 seconds\n", output_text_controller)
            messagebox.showerror("Error", "The command timed out. Check if ROS2 is running properly.")
        except Exception as e:
            append_output(f"Error: {str(e)}\n", output_text_controller)
            messagebox.showerror("Error", f"An unexpected error occurred:\n{str(e)}")
        
        append_output("Controller activation completed.\n", output_text_controller)
    
    # Output window for controller operations
    output_text_controller = ScrolledText(frame_controller, height=25, width=150, state="disabled", wrap="word")
    output_text_controller.pack(padx=10, pady=10, fill=tk.BOTH, expand=True)
    
    # Do an initial refresh when the tab is shown, but with error handling
    def on_tab_selected(event):
        try:
            selected_tab = event.widget.select()
            tab_text = event.widget.tab(selected_tab, "text")
            if tab_text == "Switch Controller":
                # Don't auto-refresh on tab selection to prevent crashes
                append_output("Switched to Controller tab. Click 'Refresh Controller List' to view available controllers.\n", 
                             output_text_controller)
        except Exception as e:
            print(f"Error when switching tabs: {e}")
    
    # Bind the tab selection event
    notebook.bind("<<NotebookTabChanged>>", on_tab_selected)
    
    # Make sure we initialize the output_text_controller before using it
    try:
        # Do an initial population of controllers list when the GUI is fully loaded
        root.after(1000, lambda: append_output("Controller tab ready. Click 'Refresh Controller List' to view available controllers.\n", 
                                             output_text_controller))
    except Exception as e:
        print(f"Error in initial controller tab setup: {e}")
    
    root.mainloop()

if __name__ == "__main__":
    setup_gui()