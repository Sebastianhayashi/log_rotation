import tkinter as tk
from tkinter import messagebox
import yaml

class LogRotationGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Log Rotation Configuration")

        # 创建输入框和标签
        tk.Label(root, text="Log File Path:").grid(row=0, column=0, padx=10, pady=5, sticky="e")
        self.log_file_path_entry = tk.Entry(root, width=40)
        self.log_file_path_entry.grid(row=0, column=1, padx=10, pady=5)

        tk.Label(root, text="Max Log File Size (bytes):").grid(row=1, column=0, padx=10, pady=5, sticky="e")
        self.max_size_entry = tk.Entry(root, width=20)
        self.max_size_entry.grid(row=1, column=1, padx=10, pady=5)

        tk.Label(root, text="Max Number of Log Files:").grid(row=2, column=0, padx=10, pady=5, sticky="e")
        self.max_files_entry = tk.Entry(root, width=20)
        self.max_files_entry.grid(row=2, column=1, padx=10, pady=5)

        # 创建保存按钮
        tk.Button(root, text="Save Configuration", command=self.save_config).grid(row=3, column=0, columnspan=2, pady=20)

    def save_config(self):
        # 获取用户输入
        log_file_path = self.log_file_path_entry.get()
        max_size = self.max_size_entry.get()
        max_files = self.max_files_entry.get()

        if not log_file_path or not max_size or not max_files:
            messagebox.showerror("Error", "All fields are required!")
            return

        try:
            config = {
                "log_file_path": log_file_path,
                "max_size": int(max_size),
                "max_files": int(max_files)
            }

            # 保存配置到 YAML 文件
            with open("log_config.yaml", "w") as config_file:
                yaml.dump(config, config_file)

            messagebox.showinfo("Success", "Configuration saved successfully!")

        except ValueError:
            messagebox.showerror("Error", "Max size and max files must be integers!")

if __name__ == "__main__":
    root = tk.Tk()
    gui = LogRotationGUI(root)
    root.mainloop()
