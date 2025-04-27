import os
import sys
import tkinter as tk
from tkinter import ttk, messagebox
import importlib.util
from PIL import Image, ImageTk
from ament_index_python import get_package_share_directory

# Import the GUI modules
from manipulator_gui.EvalnRunMenu import setup_gui as setup_evalnrun
from manipulator_gui.SetupMenu import setup_gui as setup_setup
from manipulator_gui.TrainRLMenu import setup_gui as setup_trainrl

def load_logo():
    """Try to load the manipulator logo if available"""
    try:
        package_share_dir = get_package_share_directory('manipulator_gui')
        image_path = os.path.join(package_share_dir, 'resources', 'manipulator_logo.png')
        if os.path.exists(image_path):
            original_image = Image.open(image_path)
            resized_image = original_image.resize((200, 200), Image.Resampling.LANCZOS)
            return ImageTk.PhotoImage(resized_image)
        else:
            print(f"Logo file not found at {image_path}")
    except Exception as e:
        print(f"Error loading logo: {e}")
    return None

def launch_gui():
    root = tk.Tk()
    root.title("Manipulator GUI Launcher")
    
    # Get screen dimensions
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    
    # Calculate position to center the window
    window_width = 500
    window_height = 750
    x = (screen_width - window_width) // 2
    y = (screen_height - window_height) // 2
    
    # Set window size and position
    root.geometry(f"{window_width}x{window_height}+{x}+{y}")
    root.configure(bg="#F0F0F0")
    
    # Configure styles
    style = ttk.Style()
    style.configure("TButton", padding=8, relief="flat", background="#4CAF50", foreground="white", font=("Arial", 12))
    style.configure("TLabel", foreground="#333333", font=("Arial", 12))
    style.configure("Title.TLabel", foreground="#333333", font=("Arial", 16, "bold"))
    style.map("TButton",
        background=[("active", "#45a049")],
        foreground=[("active", "white")])
    
    # Main frame
    main_frame = ttk.Frame(root, padding=20)
    main_frame.pack(fill=tk.BOTH, expand=True)
    
    # Try to load the logo
    logo = load_logo()
    if logo:
        try:
            logo_label = tk.Label(main_frame, image=logo, bg="#F0F0F0")
            logo_label.image = logo  # Keep a reference to prevent garbage collection
            logo_label.pack(pady=10)
        except Exception as e:
            print(f"Failed to display logo: {e}")
    
    # Title
    title_label = ttk.Label(main_frame, text="Manipulator GUI Launcher", style="Title.TLabel")
    title_label.pack(pady=20)
    
    # Description
    desc_label = ttk.Label(main_frame, text="Select which GUI you want to launch:", wraplength=450)
    desc_label.pack(pady=10)
    
    # Button frame
    button_frame = ttk.Frame(main_frame)
    button_frame.pack(fill=tk.BOTH, expand=True, pady=20)
    
    # GUI launch buttons
    def launch_evalnrun():
        root.destroy()
        setup_evalnrun()
    
    def launch_setup():
        root.destroy()
        setup_setup()
    
    def launch_trainrl():
        root.destroy()
        setup_trainrl()
    
    # Setup button
    setup_btn = ttk.Button(
        button_frame, 
        text="Setup Menu", 
        command=launch_setup,
        width=30
    )
    setup_btn.pack(pady=10)
    
    # Training button
    training_btn = ttk.Button(
        button_frame, 
        text="Training RL Menu", 
        command=launch_trainrl,
        width=30
    )
    training_btn.pack(pady=10)
    
    # Eval button
    eval_btn = ttk.Button(
        button_frame, 
        text="Evaluation and Run Menu", 
        command=launch_evalnrun,
        width=30
    )
    eval_btn.pack(pady=10)
    
    # Exit button
    exit_btn = ttk.Button(
        main_frame, 
        text="Exit", 
        command=lambda: os._exit(0),  # Force exit the process
        width=20
    )
    exit_btn.pack(side=tk.BOTTOM, pady=10)
    
    # Add description labels for each button
    ttk.Label(main_frame, text="Setup Menu: Data processing and preparation", 
              wraplength=400, foreground="#666666").pack(side=tk.BOTTOM, pady=2)
    ttk.Label(main_frame, text="Training RL Menu: RL model training", 
              wraplength=400, foreground="#666666").pack(side=tk.BOTTOM, pady=2)
    ttk.Label(main_frame, text="Evaluation and Run Menu: Run and evaluate models", 
              wraplength=400, foreground="#666666").pack(side=tk.BOTTOM, pady=2)
    
    # Set handler for window close button (X)
    root.protocol("WM_DELETE_WINDOW", lambda: os._exit(0))
    
    root.mainloop()

if __name__ == "__main__":
    launch_gui() 