import tkinter as tk
import time

class RobotUI:
    def __init__(self, master):
        self.master = master
        master.title("Robot Metrics Visualization")

        # Set the background color
        master.configure(bg='red')

        # Set the window size
        window_width = 400
        window_height = 250

        # Get the screen dimensions
        screen_width = master.winfo_screenwidth()
        screen_height = master.winfo_screenheight()

        # Calculate position to center the window
        position_top = int(screen_height/2 - window_height/2)
        position_right = int(screen_width/2 - window_width/2)

        # Set the window geometry (size and position)
        master.geometry(f"{window_width}x{window_height}+{position_right}+{position_top}")

        # Create a frame to hold the table
        table_frame = tk.Frame(master, bg='white', bd=2, relief='solid')
        table_frame.grid(row=0, column=0, padx=20, pady=20)

        # Labels for displaying the metrics
        self.current_pose_label = tk.Label(table_frame, text="Current Pose (x, y, z, orientation): ", bg='white')
        self.current_pose_label.grid(row=0, column=0, sticky='w', padx=10, pady=5)

        self.target_pose_label = tk.Label(table_frame, text="Target Pose (x, y, z, orientation): ", bg='white')
        self.target_pose_label.grid(row=1, column=0, sticky='w', padx=10, pady=5)

        self.path_length_label = tk.Label(table_frame, text="Path Length: ", bg='white')
        self.path_length_label.grid(row=2, column=0, sticky='w', padx=10, pady=5)

        self.execution_time_label = tk.Label(table_frame, text="Execution Time: ", bg='white')
        self.execution_time_label.grid(row=3, column=0, sticky='w', padx=10, pady=5)

        self.success_rate_label = tk.Label(table_frame, text="Success Rate: ", bg='white')
        self.success_rate_label.grid(row=4, column=0, sticky='w', padx=10, pady=5)

        # Values for the metrics
        self.current_pose_value = tk.Label(table_frame, text="(0.0, 0.0, 0.0, 1.0)", bg='white')
        self.current_pose_value.grid(row=0, column=1, sticky='w', padx=10, pady=5)

        self.target_pose_value = tk.Label(table_frame, text="(0.1, 0.2, 0.3, 1.0)", bg='white')
        self.target_pose_value.grid(row=1, column=1, sticky='w', padx=10, pady=5)

        self.path_length_value = tk.Label(table_frame, text="0.5 meters", bg='white')
        self.path_length_value.grid(row=2, column=1, sticky='w', padx=10, pady=5)

        self.execution_time_value = tk.Label(table_frame, text="2.3 seconds", bg='white')
        self.execution_time_value.grid(row=3, column=1, sticky='w', padx=10, pady=5)

        self.success_rate_value = tk.Label(table_frame, text="80%", bg='white')
        self.success_rate_value.grid(row=4, column=1, sticky='w', padx=10, pady=5)

        # Start the update loop
        self.update_ui()

    def update_metrics(self, current_pose, target_pose, path_length, execution_time, success_rate):
        self.current_pose_value.config(text=str(current_pose))
        self.target_pose_value.config(text=str(target_pose))
        self.path_length_value.config(text=f"{path_length} meters")
        self.execution_time_value.config(text=f"{execution_time} seconds")
        self.success_rate_value.config(text=f"{success_rate*100}%")

    def update_ui(self):
        # Simulate updating the UI with metrics
        for i in range(10):
            current_pose = (i * 0.1, i * 0.1, i * 0.1, 1.0)
            target_pose = ((i+1) * 0.1, (i+1) * 0.2, (i+1) * 0.3, 1.0)
            path_length = i * 0.5
            execution_time = i * 0.2
            success_rate = 0.8

            # Update the UI with the latest metrics
            self.update_metrics(current_pose, target_pose, path_length, execution_time, success_rate)

            # Sleep for 2 seconds to simulate real-time updates
            self.master.update()
            time.sleep(2)

if __name__ == "__main__":
    root = tk.Tk()
    robot_ui = RobotUI(root)
    root.mainloop()
