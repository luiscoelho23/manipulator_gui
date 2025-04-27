import os
import subprocess
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from ament_index_python import get_package_prefix, get_package_share_directory
import yaml
from pathlib import Path
import signal
import psutil
import time
import importlib

# Global list to keep track of running training processes
running_processes = []

def get_rl_resource_path():
    """Get the path to the RL resources directory using ROS package utilities."""
    try:
        # Use ROS package utilities to find the correct path
        pkg_share = get_package_share_directory('manipulator_skill_acquisition')
        rl_path = os.path.join(pkg_share, 'resources', 'rl')
        
        # Create parent directories if they don't exist
        os.makedirs(rl_path, exist_ok=True)
        
        return Path(rl_path)
    except Exception as e:
        print(f"Error finding RL resources path: {e}")
        # Fallback to a directory in the user's home that should be writable
        home_dir = os.path.expanduser("~")
        fallback_path = os.path.join(home_dir, '.manipulator_rl')
        os.makedirs(fallback_path, exist_ok=True)
        return Path(fallback_path)

def get_default_output_dir():
    """Get the default output directory for RL training."""
    try:
        # Get workspace root by walking up from current file
        # Start from the current file's directory
        current_file = os.path.abspath(__file__)
        if '/install/' in current_file:
            ws_path = current_file[:current_file.find('/install/')]
        elif '/src/' in current_file:
            ws_path = current_file[:current_file.find('/src/')]
        else:
            print("Error: Could not determine workspace path. Script must be run from install or src directory.")
            sys.exit(1)
        
        # Construct the path to the desired output directory
        output_dir = os.path.join(
            ws_path, 
            "src", 
            "manipulator_skill_acquisition", 
            "resources", 
            "rl", 
            "outputs"
        )     
        os.makedirs(output_dir, exist_ok=True)
        
        return output_dir
    except Exception as e:
        print(f"Error creating output directory: {e}")
        # Fall back to home directory
        home_dir = os.path.expanduser("~")
        fallback_path = os.path.join(home_dir, '.manipulator_rl', 'outputs')
        os.makedirs(fallback_path, exist_ok=True)
        print(f"Falling back to home directory: {fallback_path}")
        return fallback_path

def get_package_directory(package_name):
    """Get the package directory for a given package name."""
    return get_package_prefix(package_name)

def kill_running_processes():
    """Kill all running training processes and their children."""
    global running_processes
    
    killed_count = 0
    for proc in running_processes[:]:
        try:
            # Get the process object using psutil
            p = psutil.Process(proc.pid)
            if p.is_running():
                # Get all child processes before killing the parent
                children = p.children(recursive=True)
                
                # Terminate all child processes
                for child in children:
                    try:
                        print(f"Terminating child process {child.pid}")
                        child.terminate()
                    except:
                        pass
                
                # Terminate the main process
                print(f"Terminating main process {proc.pid}")
                p.terminate()
                
                # Give processes time to terminate gracefully
                time.sleep(0.5)
                
                # Check if processes are still alive and kill them forcefully
                try:
                    p.wait(timeout=1)
                except psutil.TimeoutExpired:
                    print(f"Force killing process {proc.pid}")
                    p.kill()
                
                # Force kill any remaining children
                for child in children:
                    try:
                        if child.is_running():
                            print(f"Force killing child process {child.pid}")
                            child.kill()
                    except:
                        pass
                
                killed_count += 1
            
            running_processes.remove(proc)
        except (psutil.NoSuchProcess, psutil.AccessDenied, ProcessLookupError) as e:
            print(f"Error handling process {proc.pid}: {e}")
            running_processes.remove(proc)
        except Exception as e:
            print(f"Unexpected error: {e}")
            running_processes.remove(proc)
    
    # Try killing any Python processes containing 'train.py' or 'train_ddpg.py'
    try:
        for p in psutil.process_iter(["pid", "name", "cmdline"]):
            try:
                cmdline = " ".join(p.cmdline())
                if "python" in p.name().lower() and ("train.py" in cmdline or "train_ddpg.py" in cmdline):
                    print(f"Found training process: {p.pid} - {cmdline}")
                    p.terminate()
                    time.sleep(0.5)
                    if p.is_running():
                        p.kill()
                    killed_count += 1
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
    except Exception as e:
        print(f"Error searching for other training processes: {e}")
    
    running_processes = []
    return killed_count > 0

def run_script(script_name, args):
    if not os.path.exists(script_name):
        messagebox.showerror("Error", f"Script {script_name} not found!")
        return
    try:
        subprocess.Popen(["python3", script_name] + args)
    except Exception as e:
        messagebox.showerror("Error", f"Failed to execute script:\n{e}")

def load_default_config():
    """Load default configuration from rl_config.yaml in share directory."""
    pkg_share = get_package_share_directory('manipulator_skill_acquisition')
    config_path = os.path.join(pkg_share, 'config_rl', 'rl_config.yaml')
    if not os.path.exists(config_path):
        print(f"Default config file not found at: {config_path}")
        # Try alternative location
        alt_path = os.path.join(pkg_share, '..', '..', 'config_rl', 'rl_config.yaml')
        if os.path.exists(alt_path):
            config_path = alt_path
            print(f"Found config at alternative location: {config_path}")
        else:
            messagebox.showwarning("Warning", f"Default config file not found: {config_path}")
            return None
            
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    # Keep existing output settings if available, or create defaults
    if 'output' not in config:
        config['output'] = {}
    
    # Update with required output settings - always override with valid paths
    output_dir = get_default_output_dir()
    output_defaults = {
        'directory': output_dir,
        'model_name': 'model',
        'save_frequency': 100,
        'save_checkpoints': True,
        'save_best': True
    }
    
    # Apply defaults while preserving any existing settings except directory
    # Always set a valid directory
    config['output']['directory'] = output_dir
    
    for key, value in output_defaults.items():
        if key != 'directory' and key not in config['output']:
            config['output'][key] = value
    
    # Make sure dmp section exists
    if 'dmp' not in config:
        config['dmp'] = {'file': '', 'parameters': {
            'n_basis': 25,
            'n_primitives': 3,
            'phase_bounds': [0.0, 1.0],
            'integration_timestep': 0.001
        }}
    
    return config

def setup_gui():
    global running_processes
    running_processes = []
    
    root = tk.Tk()
    root.title("RL Training Menu")
    
    # Get screen dimensions
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    
    # Calculate position to center the window
    window_width = 600
    window_height = 700
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
    
    # Configure stop button style
    style.configure("Stop.TButton", padding=8, relief="flat", background="#F44336", foreground="white", font=("Arial", 10))
    style.map("Stop.TButton",
        background=[("active", "#D32F2F")],
        foreground=[("active", "white")])
    
    # Configure return button style
    style.configure("Return.TButton", padding=8, relief="flat", background="#2196F3", foreground="white", font=("Arial", 10))
    style.map("Return.TButton",
        background=[("active", "#0b7dda")],
        foreground=[("active", "white")])
    
    # Create main frame
    main_frame = ttk.Frame(root, padding=20)
    main_frame.pack(fill=tk.BOTH, expand=True)
    
    # Load default configuration
    default_config = load_default_config()
    if default_config is None:
        # If loading fails, create a minimal default configuration
        default_config = {
            "environment": "env_dmp_obstacle",
            "hyperparameters": {
                "learning_rate": 0.0003,
                "gamma": 0.99,
                "batch_size": 256,
                "tau": 0.01,
                "buffer_limit": 1000000,
                "init_alpha": 0.1,
                "exploration_noise": 0.1
            },
            "dmp": {
                "file": "",
                "parameters": {
                    "n_basis": 25,
                    "n_primitives": 3,
                    "phase_bounds": [0.0, 1.0],
                    "integration_timestep": 0.001
                }
            },
            "output": {
                "directory": get_default_output_dir(),
                "model_name": "rl_actor",
                "save_frequency": 100,
                "save_checkpoints": True,
                "save_best": True
            },
            "logging": {
                "enabled": True,
                "directory": "logs",
                "frequency": 10,
                "tensorboard": True,
                "console": True
            }
        }
        messagebox.showinfo("Notice", "Using default configuration values.")
    
    # Create notebook for tabs
    notebook = ttk.Notebook(main_frame)
    notebook.pack(fill=tk.BOTH, expand=True)
    
    # Training Configuration Tab
    training_tab = ttk.Frame(notebook, padding=10)
    notebook.add(training_tab, text="Training Configuration")
    
    # Environment Selection
    env_frame = ttk.LabelFrame(training_tab, text="Environment Configuration", padding=10)
    env_frame.pack(fill=tk.X, pady=10)
    
    env_var = tk.StringVar(value=default_config['environment'])
    env_options = ["env_dmp_obstacle", "env_dmp_obstacle_via_points"]
    
    # Add algorithm selection
    algo_var = tk.StringVar(value="SAC")  # Default to SAC
    algo_options = ["SAC", "DDPG", "TD3"]
    
    frame_env = ttk.Frame(env_frame)
    frame_env.pack(fill=tk.X, pady=5)
    ttk.Label(frame_env, text="Environment:").pack(side=tk.LEFT)
    env_combo = ttk.Combobox(frame_env, textvariable=env_var, values=env_options, state="readonly")
    env_combo.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
    
    frame_algo = ttk.Frame(env_frame)
    frame_algo.pack(fill=tk.X, pady=5)
    ttk.Label(frame_algo, text="Algorithm:").pack(side=tk.LEFT)
    algo_combo = ttk.Combobox(frame_algo, textvariable=algo_var, values=algo_options, state="readonly")
    algo_combo.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
    
    # Common hyperparameters
    common_hyperparams = {
        "learning_rate": tk.StringVar(value=str(default_config['hyperparameters'].get('learning_rate', 0.0003))),
        "gamma": tk.StringVar(value=str(default_config['hyperparameters'].get('gamma', 0.99))),
        "batch_size": tk.StringVar(value=str(default_config['hyperparameters'].get('batch_size', 256))),
        "tau": tk.StringVar(value=str(default_config['hyperparameters'].get('tau', 0.01))),
        "buffer_limit": tk.StringVar(value=str(default_config['hyperparameters'].get('buffer_limit', 1000000)))
    }
    
    # SAC specific hyperparameters
    sac_hyperparams = {
        "init_alpha": tk.StringVar(value=str(default_config['hyperparameters'].get('init_alpha', 0.1)))
    }
    
    # DDPG specific hyperparameters
    ddpg_hyperparams = {
        "exploration_noise": tk.StringVar(value=str(default_config['hyperparameters'].get('exploration_noise', 0.1)))
    }
    
    # TD3 specific hyperparameters
    td3_hyperparams = {
        "exploration_noise": tk.StringVar(value=str(default_config['hyperparameters'].get('exploration_noise', 0.1))),
        "policy_noise": tk.StringVar(value=str(default_config['hyperparameters'].get('policy_noise', 0.2))),
        "noise_clip": tk.StringVar(value=str(default_config['hyperparameters'].get('noise_clip', 0.5))),
        "policy_delay": tk.StringVar(value=str(default_config['hyperparameters'].get('policy_delay', 2)))
    }
    
    # Create common hyperparameter frame
    common_params_frame = ttk.LabelFrame(training_tab, text="Common Parameters", padding=10)
    common_params_frame.pack(fill=tk.X, pady=10)
    
    for param, var in common_hyperparams.items():
        frame = ttk.Frame(common_params_frame)
        frame.pack(fill=tk.X, pady=2)
        ttk.Label(frame, text=f"{param.replace('_', ' ').title()}:", width=20).pack(side=tk.LEFT)
        ttk.Entry(frame, textvariable=var).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
    
    # Create SAC-specific hyperparameter frame
    sac_frame = ttk.LabelFrame(training_tab, text="SAC-specific Parameters", padding=10)
    sac_frame.pack(fill=tk.X, pady=10)
    
    for param, var in sac_hyperparams.items():
        frame = ttk.Frame(sac_frame)
        frame.pack(fill=tk.X, pady=2)
        ttk.Label(frame, text=f"{param.replace('_', ' ').title()}:", width=20).pack(side=tk.LEFT)
        ttk.Entry(frame, textvariable=var).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
    
    # Create DDPG-specific hyperparameter frame
    ddpg_frame = ttk.LabelFrame(training_tab, text="DDPG-specific Parameters", padding=10)
    ddpg_frame.pack(fill=tk.X, pady=10)
    
    for param, var in ddpg_hyperparams.items():
        frame = ttk.Frame(ddpg_frame)
        frame.pack(fill=tk.X, pady=2)
        ttk.Label(frame, text=f"{param.replace('_', ' ').title()}:", width=20).pack(side=tk.LEFT)
        ttk.Entry(frame, textvariable=var).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
    
    # Create TD3-specific hyperparameter frame
    td3_frame = ttk.LabelFrame(training_tab, text="TD3-specific Parameters", padding=10)
    
    for param, var in td3_hyperparams.items():
        frame = ttk.Frame(td3_frame)
        frame.pack(fill=tk.X, pady=2)
        ttk.Label(frame, text=f"{param.replace('_', ' ').title()}:", width=20).pack(side=tk.LEFT)
        ttk.Entry(frame, textvariable=var).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
    
    # System Configuration Tab
    system_tab = ttk.Frame(notebook, padding=10)
    notebook.add(system_tab, text="System Configuration")
    
    # DMP Selection
    dmp_frame = ttk.LabelFrame(system_tab, text="DMP Configuration", padding=10)
    dmp_frame.pack(fill=tk.X, pady=10)
    
    def get_dmp_files():
        """Get DMP files from share directory."""
        pkg_share = get_package_share_directory('manipulator_skill_acquisition')
        dmp_dir = os.path.join(pkg_share, 'resources', 'dmp')
        if not os.path.exists(dmp_dir):
            # Try to create the directory
            try:
                os.makedirs(dmp_dir, exist_ok=True)
                print(f"Created DMP directory: {dmp_dir}")
            except Exception as e:
                print(f"Could not create DMP directory: {e}")
            return [], []
        
        dmp_files = [f for f in os.listdir(dmp_dir) if f.endswith('.mpx')]
        if not dmp_files:
            print(f"No MPX files found in {dmp_dir}")
            # Try to find files in parent directory
            parent_dir = os.path.dirname(dmp_dir)
            if os.path.exists(parent_dir):
                alt_files = [f for f in os.listdir(parent_dir) if f.endswith('.mpx')]
                if alt_files:
                    print(f"Found MPX files in parent directory: {parent_dir}")
                    return [os.path.join(parent_dir, f) for f in alt_files], alt_files
        
        return [os.path.join(dmp_dir, f) for f in dmp_files], dmp_files
    
    dmp_paths, dmp_names = get_dmp_files()
    dmp_var = tk.StringVar()
    dmp_full_path = tk.StringVar()
    
    if dmp_paths:
        dmp_full_path.set(dmp_paths[0])
        dmp_var.set(dmp_names[0])
    
    frame_dmp = ttk.Frame(dmp_frame)
    frame_dmp.pack(fill=tk.X, pady=5)
    ttk.Label(frame_dmp, text="DMP File:").pack(side=tk.LEFT)
    dmp_combo = ttk.Combobox(frame_dmp, textvariable=dmp_var, values=dmp_names, state="readonly")
    dmp_combo.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
    
    def browse_dmp_file():
        """Let user select a DMP file manually"""
        file_path = filedialog.askopenfilename(
            title="Select DMP File",
            filetypes=[("MPX Files", "*.mpx"), ("All Files", "*.*")]
        )
        if file_path:
            dmp_full_path.set(file_path)
            # Extract filename for display
            dmp_var.set(os.path.basename(file_path))
            # Update combobox values if needed
            if os.path.basename(file_path) not in dmp_names:
                new_names = dmp_names + [os.path.basename(file_path)]
                dmp_combo['values'] = new_names
    
    def on_dmp_select(event):
        idx = dmp_names.index(dmp_var.get())
        if idx >= 0 and idx < len(dmp_paths):
            dmp_full_path.set(dmp_paths[idx])
    
    dmp_combo.bind('<<ComboboxSelected>>', on_dmp_select)
    
    ttk.Button(frame_dmp, text="Browse", command=browse_dmp_file).pack(side=tk.LEFT, padx=5)
    
    # Output Configuration
    output_frame = ttk.LabelFrame(system_tab, text="Output Configuration", padding=10)
    output_frame.pack(fill=tk.X, pady=10)
    
    output_dir_var = tk.StringVar(value=get_default_output_dir())
    output_name_var = tk.StringVar(value=default_config['output'].get('model_name', 'rl_actor'))
    
    frame_output = ttk.Frame(output_frame)
    frame_output.pack(fill=tk.X, pady=5)
    ttk.Label(frame_output, text="Output Directory:").pack(side=tk.LEFT)
    ttk.Entry(frame_output, textvariable=output_dir_var).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
    ttk.Button(frame_output, text="Browse", 
              command=lambda: output_dir_var.set(filedialog.askdirectory(initialdir=get_default_output_dir()))).pack(side=tk.LEFT, padx=5)
    
    frame_name = ttk.Frame(output_frame)
    frame_name.pack(fill=tk.X, pady=5)
    ttk.Label(frame_name, text="Model Name:").pack(side=tk.LEFT)
    ttk.Entry(frame_name, textvariable=output_name_var).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
    
    # Show/hide algorithm specific parameters based on algorithm selection
    def on_algo_select(event):
        if algo_var.get() == "DDPG":
            ddpg_frame.pack(fill=tk.X, pady=10)
            sac_frame.pack_forget()
            td3_frame.pack_forget()
        elif algo_var.get() == "TD3":
            td3_frame.pack(fill=tk.X, pady=10)
            sac_frame.pack_forget()
            ddpg_frame.pack_forget()
        else:  # SAC is selected
            sac_frame.pack(fill=tk.X, pady=10)
            ddpg_frame.pack_forget()
            td3_frame.pack_forget()
    
    algo_combo.bind('<<ComboboxSelected>>', on_algo_select)
    
    # Initially show the appropriate parameter frame based on selected algorithm
    if algo_var.get() == "DDPG":
        ddpg_frame.pack(fill=tk.X, pady=10)
        sac_frame.pack_forget()
        td3_frame.pack_forget()
    elif algo_var.get() == "TD3":
        td3_frame.pack(fill=tk.X, pady=10)
        sac_frame.pack_forget()
        ddpg_frame.pack_forget()
    else:  # SAC is selected
        sac_frame.pack(fill=tk.X, pady=10)
        ddpg_frame.pack_forget()
        td3_frame.pack_forget()
    
    # Training Button
    def start_training():
        if not dmp_var.get():
            messagebox.showerror("Error", "Please select a DMP file")
            return
        
        if not output_dir_var.get():
            messagebox.showerror("Error", "Please select an output directory")
            return
        
        # Create output directory if it doesn't exist
        try:
            os.makedirs(output_dir_var.get(), exist_ok=True)
            print(f"Ensuring output directory exists: {output_dir_var.get()}")
        except Exception as e:
            messagebox.showerror("Error", f"Cannot create output directory: {e}")
            return
        
        # Make sure output directory is writable
        if not os.access(output_dir_var.get(), os.W_OK):
            messagebox.showerror("Error", f"Output directory is not writable: {output_dir_var.get()}")
            return
        
        # Create config dictionary with common parameters
        config = {
            "environment": env_var.get(),
            "hyperparameters": {k: float(v.get()) for k, v in common_hyperparams.items()},
            "dmp": {
                "file": dmp_full_path.get(),
                "parameters": default_config.get('dmp', {}).get('parameters', {})
            },
            "output": {
                "directory": output_dir_var.get(),
                "model_name": output_name_var.get(),
                "save_frequency": default_config.get('output', {}).get('save_frequency', 100),
                "save_checkpoints": default_config.get('output', {}).get('save_checkpoints', True),
                "save_best": default_config.get('output', {}).get('save_best', True)
            },
            "logging": default_config.get('logging', {"enabled": True, "directory": "logs", "frequency": 10, "tensorboard": True, "console": True})
        }
        
        # Add SAC specific hyperparameters if SAC is selected
        if algo_var.get() == "SAC":
            for k, v in sac_hyperparams.items():
                config["hyperparameters"][k] = float(v.get())
        
        # Add DDPG specific hyperparameters if DDPG is selected
        if algo_var.get() == "DDPG":
            for k, v in ddpg_hyperparams.items():
                config["hyperparameters"][k] = float(v.get())
        
        # Add TD3 specific hyperparameters if TD3 is selected
        if algo_var.get() == "TD3":
            for k, v in td3_hyperparams.items():
                if k == "policy_delay":  # Convert policy_delay to int
                    config["hyperparameters"][k] = int(float(v.get()))
                else:
                    config["hyperparameters"][k] = float(v.get())
        
        # Save config to output directory
        config_path = os.path.join(output_dir_var.get(), "training_config.yaml")
        
        # Ensure the DMP file path is valid
        if not os.path.exists(config["dmp"]["file"]):
            messagebox.showwarning("Warning", f"DMP file not found: {config['dmp']['file']}\nTraining may fail!")
        
        # Ensure the output directory is valid
        if not os.path.isdir(config["output"]["directory"]):
            messagebox.showwarning("Warning", f"Output directory not valid: {config['output']['directory']}")
            return
        
        # Save the config to the output directory
        try:
            with open(config_path, 'w') as f:
                yaml.dump(config, f)
            print(f"Saved config to: {config_path}")
        except Exception as e:
            messagebox.showerror("Error", f"Could not save config file: {e}")
            return
        
        # Find script directory
        # First try direct import to figure out script location
        try:
            import manipulator_skill_acquisition.rl
            package_dir = os.path.dirname(os.path.dirname(manipulator_skill_acquisition.rl.__file__))
            script_dir = os.path.join(package_dir, 'rl')
        except ImportError:
            # Fall back to searching for the script
            pkg_share = get_package_share_directory('manipulator_skill_acquisition')
            script_dir = os.path.join(pkg_share, '..', '..', 'src', 'manipulator_skill_acquisition', 'manipulator_skill_acquisition', 'rl')
            if not os.path.exists(script_dir):
                # Try the install path
                script_dir = os.path.join(pkg_share, 'lib', 'manipulator_skill_acquisition', 'rl')
        
        # Determine script name based on algorithm
        if algo_var.get() == "DDPG":
            script_name = "train_ddpg.py"
        elif algo_var.get() == "TD3":
            script_name = "train_td3.py"
        elif algo_var.get() == "SAC":
            script_name = "train_sac.py"
        else:
            script_name = "train.py"
        
        # Find the script path
        script_path = os.path.join(script_dir, script_name)
        
        # If script not found, try to find it in other locations
        if not os.path.exists(script_path):
            # Try to find the script in the workspace
            workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            possible_paths = [
                os.path.join(workspace_root, 'manipulator_skill_acquisition', 'manipulator_skill_acquisition', 'rl', script_name),
                os.path.join(workspace_root, 'src', 'manipulator_skill_acquisition', 'manipulator_skill_acquisition', 'rl', script_name),
                os.path.join(pkg_share, 'lib', 'manipulator_skill_acquisition', 'rl', script_name),
                os.path.join(pkg_share, 'lib', 'python3', 'dist-packages', 'manipulator_skill_acquisition', 'rl', script_name)
            ]
            
            for path in possible_paths:
                if os.path.exists(path):
                    script_path = path
                    break
            
            if not os.path.exists(script_path):
                messagebox.showerror("Error", f"Training script not found: {script_name}")
                return
        
        print(f"Using script at: {script_path}")
        
        try:
            print(f"Running: python3 {script_path} {config_path}")
            proc = subprocess.Popen(["python3", script_path, config_path])
            running_processes.append(proc)
            messagebox.showinfo("Success", f"Training started successfully with {algo_var.get()} algorithm!\nThe window will remain open for additional training runs.\nOutputs will be saved to: {output_dir_var.get()}")
            
            # Enable the stop button now that we have a running process
            stop_btn["state"] = "normal"
        except Exception as e:
            messagebox.showerror("Error", f"Failed to start training:\n{e}")

    # Function to stop all training processes
    def stop_training():
        if kill_running_processes():
            messagebox.showinfo("Success", "All training processes have been stopped.")
            # Disable the stop button since no processes are running
            stop_btn["state"] = "disabled"
        else:
            messagebox.showerror("Error", "Failed to stop all training processes.")
            
    # Set up proper cleanup when window is closed
    def on_closing():
        if running_processes:
            if messagebox.askokcancel("Exit", "Do you want to stop all training processes and exit?"):
                kill_running_processes()
                root.destroy()
        else:
            root.destroy()
            
    root.protocol("WM_DELETE_WINDOW", on_closing)
    
    # Create button frame at the bottom
    button_frame = ttk.Frame(main_frame)
    button_frame.pack(fill=tk.X, pady=10)
    
    train_btn = ttk.Button(button_frame, text="Start Training", command=start_training)
    train_btn.pack(side=tk.LEFT, padx=5)
    
    stop_btn = ttk.Button(button_frame, text="Stop Training", command=stop_training, style="Stop.TButton")
    stop_btn.pack(side=tk.LEFT, padx=5)
    
    # Initially disable the stop button until a process is started
    stop_btn["state"] = "disabled"
    
    # Return to Main Menu button with proper cleanup
    def return_to_main():
        if running_processes:
            if messagebox.askokcancel("Return to Main Menu", "Do you want to stop all training processes and return to main menu?"):
                kill_running_processes()
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
        else:
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
        command=return_to_main,
        style="Return.TButton"
    )
    return_btn.pack(side=tk.LEFT, padx=5)
    
    # Exit button
    exit_btn = ttk.Button(button_frame, text="Exit", command=lambda: os._exit(0))
    exit_btn.pack(side=tk.RIGHT, padx=5)
    
    root.mainloop()

if __name__ == "__main__":
    setup_gui() 