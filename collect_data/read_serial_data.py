import threading
import tkinter as tk
from tkinter import scrolledtext, messagebox, font as tkfont, ttk
import serial
import serial.tools.list_ports
import time
import os
import csv
from datetime import datetime
import re # Added for filename sanitization
import random # Added for random label sequence

import cv2
from PIL import Image, ImageTk, ImageFont, ImageDraw # Added ImageFont, ImageDraw
import numpy as np # For converting PIL image back to OpenCV

# ─── CONFIGURATION ──────────────────────────────────────────────────────────
BAUDRATE                = 9600
FONT_FAMILY             = 'Consolas' # For Tkinter UI
FONT_SIZE_TEXT          = 11         # For Tkinter UI
FONT_SIZE_ENTRY         = 12         # For Tkinter UI

OUTPUT_DIR              = 'out_data'
VIDEO_FPS               = 20.0
SERIAL_TIMEOUT          = 1
BOARD_RESET_DELAY_MS    = 2000

# --- OpenCV Text & Font Configuration ---
# !!! IMPORTANT: YOU MUST CHANGE THIS PATH to a valid .ttf or .otf font file on your system that supports Thai.
THAI_FONT_PATH          = "tahoma.ttf" # <<< CHANGE THIS IF NEEDED! Or provide full path.
OPENCV_TEXT_FONT_SIZE_SMALL = 18
OPENCV_TEXT_FONT_SIZE_LARGE = 22 # For the main label text

DEFAULT_UNLABELED_VALUE = "notthing" # Note: "nothing" is misspelled, kept as is from original
SELECTABLE_LABELS_LIST = [
    "หนึ่ง", "สอง", "สาม", "สี่", "ห้า", "หก", "เจ็ด", "แปด", "เก้า", "สิบ"
]

column_names = [
    "timestamp_ms", "motion_detected",
    "roll_deg", "pitch_deg", "yaw_deg",
    "filtered_accel_x_(m/s²)", "filtered_accel_y_(m/s²)", "filtered_accel_z_(m/s²)",
    "filtered_gyro_x_(deg/s)", "filtered_gyro_y_(deg/s)", "filtered_gyro_z_(deg/s)",
    "accel_magnitude", "gyro_magnitude",
    "adc0", "adc1", "adc2", "adc3", "adc4", "Label"
]
os.makedirs(OUTPUT_DIR, exist_ok=True)

class SerialGUI:
    def __init__(self, master: tk.Tk):
        self.master = master
        self.master.title("Serial Monitor (Toggle Labeling, Commands, Sensor Data & Image)")

        self.ser: serial.Serial | None = None
        self.alive: bool = False
        self.reader_thread: threading.Thread | None = None
        self.sensor_paused: bool = False
        self.record: bool = False
        self.sensor_data: list[list] = []
        self.text_record: str = "Not Recording"
        self.video_writer: cv2.VideoWriter | None = None
        self.output_filename_base: str | None = None
        self.current_time_ms: str = "0"

        self.current_selected_label_var = tk.StringVar()
        self.labeling_is_active: bool = False
        self.current_continuous_label: str = DEFAULT_UNLABELED_VALUE

        # New attributes for random labeling mode
        self.random_labeling_mode_active = tk.BooleanVar(value=False)
        self.random_label_sequence: list[str] = []
        self.current_random_label_index: int = 0
        self.total_random_labels_in_sequence: int = 0 

        self.cap: cv2.VideoCapture = cv2.VideoCapture(0)
        self.image_label: tk.Label | None = None

        self.cv_thai_font_small = None
        self.cv_thai_font_large = None
        if THAI_FONT_PATH and THAI_FONT_PATH.strip():
            try:
                self.cv_thai_font_small = ImageFont.truetype(THAI_FONT_PATH, OPENCV_TEXT_FONT_SIZE_SMALL)
                self.cv_thai_font_large = ImageFont.truetype(THAI_FONT_PATH, OPENCV_TEXT_FONT_SIZE_LARGE)
                print(f"Successfully loaded Thai font: {THAI_FONT_PATH}")
            except IOError:
                print(f"Error: Could not load Thai font from '{THAI_FONT_PATH}'. Thai text in OpenCV window may not display correctly.")
                print("Please ensure the THAI_FONT_PATH is correct and the font file exists.")
                self.cv_thai_font_small = None
                self.cv_thai_font_large = None
        else:
            print("Warning: THAI_FONT_PATH is not set. Thai text in OpenCV window may not display correctly.")

        # Updated row configuration
        self.master.rowconfigure(0, weight=0) # port_frame
        self.master.rowconfigure(1, weight=0) # label_frame
        self.master.rowconfigure(2, weight=0) # random_mode_control_frame (NEW)
        self.master.rowconfigure(3, weight=1) # outer_pane
        self.master.rowconfigure(4, weight=0) # entry
        self.master.rowconfigure(5, weight=0) # close_button
        self.master.columnconfigure(0, weight=1)
        self.master.columnconfigure(1, weight=1) # Adjusted for entry/send button

        self.text_font = tkfont.Font(family=FONT_FAMILY, size=FONT_SIZE_TEXT)
        self.entry_font = tkfont.Font(family=FONT_FAMILY, size=FONT_SIZE_ENTRY)

        port_frame = tk.Frame(master)
        port_frame.grid(row=0, column=0, columnspan=2, sticky="we", padx=10, pady=(10, 5))
        port_frame.columnconfigure(0, weight=0); port_frame.columnconfigure(1, weight=1)
        port_frame.columnconfigure(2, weight=0); port_frame.columnconfigure(3, weight=0)
        port_frame.columnconfigure(4, weight=0)
        tk.Label(port_frame, text="Port:", font=self.entry_font).grid(row=0, column=0, sticky="w")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(port_frame, textvariable=self.port_var, state='readonly', font=self.entry_font)
        self.port_combo.grid(row=0, column=1, sticky="we", padx=(5, 5))
        self.refresh_button = tk.Button(port_frame, text="Refresh", font=self.entry_font, command=self.refresh_ports)
        self.refresh_button.grid(row=0, column=2, sticky="e", padx=(5, 5))
        self.connect_button = tk.Button(port_frame, text="Connect", font=self.entry_font, command=self.connect_serial)
        self.connect_button.grid(row=0, column=3, sticky="e", padx=(5, 5))
        self.disconnect_button = tk.Button(port_frame, text="Disconnect", font=self.entry_font, command=self.disconnect_serial, state=tk.DISABLED)
        self.disconnect_button.grid(row=0, column=4, sticky="e")
        self.refresh_ports()

        label_frame = tk.Frame(master)
        label_frame.grid(row=1, column=0, columnspan=2, sticky="we", padx=10, pady=(0,5))
        label_frame.columnconfigure(0, weight=0); label_frame.columnconfigure(1, weight=1)
        tk.Label(label_frame, text="Toggle Label (Spacebar):", font=self.entry_font).grid(row=0, column=0, sticky="w", padx=(0,5))
        self.label_combo = ttk.Combobox(label_frame, textvariable=self.current_selected_label_var, values=SELECTABLE_LABELS_LIST, state='readonly', font=self.entry_font)
        if SELECTABLE_LABELS_LIST: self.current_selected_label_var.set(SELECTABLE_LABELS_LIST[0])
        self.label_combo.grid(row=0, column=1, sticky="we")
        
        # Spacebar binding moved to master, specific widgets will be checked inside on_spacebar_press
        self.master.bind("<space>", self.on_spacebar_press)
        # self.label_combo.bind("<space>", self.on_spacebar_press) # Covered by master bind + check

        # --- Random Labeling Mode Controls ---
        random_mode_control_frame = tk.Frame(master)
        random_mode_control_frame.grid(row=2, column=0, columnspan=2, sticky="we", padx=10, pady=(0, 5))
        random_mode_control_frame.columnconfigure(0, weight=0) 
        random_mode_control_frame.columnconfigure(1, weight=0) 
        random_mode_control_frame.columnconfigure(2, weight=0) # Changed weight for button
        random_mode_control_frame.columnconfigure(3, weight=1) # Changed weight for status label

        tk.Label(random_mode_control_frame, text="Count per Label (Random Mode):", font=self.entry_font).grid(row=0, column=0, sticky="w", padx=(0,5))
        self.random_label_count_var = tk.IntVar(value=5) 
        self.random_label_count_entry = tk.Entry(random_mode_control_frame, textvariable=self.random_label_count_var, font=self.entry_font, width=5)
        self.random_label_count_entry.grid(row=0, column=1, sticky="w", padx=(0,10))

        self.random_mode_button_text_var = tk.StringVar(value="Start Random Session")
        self.random_mode_toggle_button = tk.Button(random_mode_control_frame, textvariable=self.random_mode_button_text_var, font=self.entry_font, command=self.toggle_random_labeling_mode, state=tk.DISABLED)
        self.random_mode_toggle_button.grid(row=0, column=2, sticky="ew", padx=(0,10))

        self.random_mode_status_var = tk.StringVar(value="Random mode: INACTIVE")
        tk.Label(random_mode_control_frame, textvariable=self.random_mode_status_var, font=self.text_font, anchor="w", justify=tk.LEFT).grid(row=0, column=3, sticky="we", padx=(0,5))

        outer_pane = tk.PanedWindow(master, orient=tk.HORIZONTAL)
        outer_pane.grid(row=3, column=0, columnspan=2, sticky="nsew", padx=10, pady=5) # Adjusted row
        
        left_pane = tk.PanedWindow(outer_pane, orient=tk.VERTICAL)
        left_pane.rowconfigure(0, weight=1); left_pane.columnconfigure(0, weight=1) 

        top_frame = tk.Frame(left_pane)
        top_frame.rowconfigure(0, weight=0); top_frame.rowconfigure(1, weight=1); top_frame.columnconfigure(0, weight=1)
        tk.Label(top_frame, text="Commands / Status", font=(FONT_FAMILY, FONT_SIZE_TEXT + 2, "bold")).grid(row=0, column=0, sticky="w", pady=(0, 4))
        self.top_text = scrolledtext.ScrolledText(top_frame, wrap=tk.WORD, font=self.text_font, state=tk.DISABLED, bg='black', fg='lightyellow')
        self.top_text.grid(row=1, column=0, sticky="nsew")
        left_pane.add(top_frame, minsize=180) 

        bottom_frame = tk.Frame(left_pane)
        bottom_frame.rowconfigure(0, weight=0) 
        bottom_frame.rowconfigure(1, weight=0) 
        bottom_frame.rowconfigure(2, weight=0) 
        bottom_frame.rowconfigure(3, weight=1) 
        bottom_frame.columnconfigure(0, weight=1) 
        bottom_frame.columnconfigure(1, weight=1) 

        tk.Label(bottom_frame, text="Filename Base:", font=self.entry_font).grid(row=0, column=0, sticky="w", pady=(5, 3), padx=(0,5)) 
        self.filename_entry_var = tk.StringVar()
        self.filename_entry = tk.Entry(bottom_frame, textvariable=self.filename_entry_var, font=self.entry_font)
        self.filename_entry.grid(row=0, column=1, sticky="we", pady=(5,3)) 

        self.pause_button = tk.Button(bottom_frame, text="Pause Sensor", font=self.entry_font, command=self.toggle_pause_sensor)
        self.pause_button.grid(row=1, column=0, sticky="ew", pady=(0, 6), padx=(0, 2))
        self.record_button = tk.Button(bottom_frame, text="Record", font=self.entry_font, command=self.toggle_record)
        self.record_button.grid(row=1, column=1, sticky="ew", pady=(0, 6), padx=(2, 0))

        tk.Label(bottom_frame, text="Sensor Data", font=(FONT_FAMILY, FONT_SIZE_TEXT + 2, "bold")).grid(row=2, column=0, columnspan=2, sticky="w", pady=(0, 4))
        self.bottom_text = scrolledtext.ScrolledText(bottom_frame, wrap=tk.WORD, font=self.text_font, state=tk.DISABLED, bg='black', fg='cyan')
        self.bottom_text.grid(row=3, column=0, columnspan=2, sticky="nsew")
        left_pane.add(bottom_frame, minsize=150) 

        outer_pane.add(left_pane, stretch="always", minsize=300) 

        image_frame = tk.LabelFrame(master, text="Camera Feed", padx=5, pady=5)
        image_frame.rowconfigure(0, weight=1); image_frame.columnconfigure(0, weight=1)
        self.image_label = tk.Label(image_frame)
        self.image_label.grid(row=0, column=0, sticky="nsew")
        outer_pane.add(image_frame, stretch="always", minsize=300) 

        self.entry = tk.Entry(master, font=self.entry_font, state=tk.DISABLED)
        self.entry.grid(row=4, column=0, padx=(10, 5), pady=(0, 10), sticky="we") # Adjusted row
        self.entry.bind("<Return>", lambda event: self.send_data())
        # self.entry.bind("<space>", self.on_spacebar_press) # Covered by master bind + check

        self.send_button = tk.Button(master, text="Send", font=self.entry_font, command=self.send_data, state=tk.DISABLED, width=10)
        self.send_button.grid(row=4, column=1, padx=(0, 10), pady=(0, 10), sticky="e") # Adjusted row
        self.close_button = tk.Button(master, text="Close", font=self.entry_font, command=self.close)
        self.close_button.grid(row=5, column=0, columnspan=2, pady=(0, 10)) # Adjusted row

        self.update_image()
        self.master.protocol("WM_DELETE_WINDOW", self.close)

    def _update_random_mode_status_display(self):
        if not self.random_labeling_mode_active.get():
            self.random_mode_status_var.set("Random mode: INACTIVE")
            return

        if self.current_random_label_index >= self.total_random_labels_in_sequence:
            self.random_mode_status_var.set(f"Random: COMPLETE! ({self.total_random_labels_in_sequence}/{self.total_random_labels_in_sequence} items).")
            return

        current_target_label = self.random_label_sequence[self.current_random_label_index]
        progress = f"({self.current_random_label_index + 1}/{self.total_random_labels_in_sequence})"

        if self.labeling_is_active:
            self.random_mode_status_var.set(f"Random: LABELING '{current_target_label}' {progress}. Space to stop.")
        else:
            self.random_mode_status_var.set(f"Random: NEXT '{current_target_label}' {progress}. Space to start.")

    def generate_random_label_sequence(self):
        if not SELECTABLE_LABELS_LIST:
            messagebox.showerror("Error", "No labels defined in SELECTABLE_LABELS_LIST for random mode.")
            return False
        
        try:
            count_per_label = self.random_label_count_var.get()
            if count_per_label <= 0:
                messagebox.showerror("Error", "Count per label must be a positive number.")
                return False
        except tk.TclError: # Handle cases where entry might not be a valid int
            messagebox.showerror("Error", "Invalid count per label. Please enter a number.")
            return False


        self.random_label_sequence.clear()
        for label in SELECTABLE_LABELS_LIST:
            self.random_label_sequence.extend([label] * count_per_label)
        
        random.shuffle(self.random_label_sequence)
        self.current_random_label_index = 0
        self.total_random_labels_in_sequence = len(self.random_label_sequence)
        
        if not self.random_label_sequence: # Should only happen if count_per_label was 0 or SELECTABLE_LABELS_LIST was empty
             messagebox.showinfo("Info", "Generated empty random sequence (0 count or no labels).")
             return False
        return True

    def toggle_random_labeling_mode(self):
        if self.random_labeling_mode_active.get(): # ---- STOPPING RANDOM MODE ----
            self.random_labeling_mode_active.set(False)
            self.random_mode_button_text_var.set("Start Random Session")
            if self.record : self.random_mode_toggle_button.config(state=tk.NORMAL) # Can start again if recording
            else: self.random_mode_toggle_button.config(state=tk.DISABLED)
            
            self.label_combo.config(state='readonly')
            self.random_label_count_entry.config(state=tk.NORMAL)
            
            if self.labeling_is_active: 
                stopped_label = self.current_continuous_label
                self.labeling_is_active = False
                self.current_continuous_label = DEFAULT_UNLABELED_VALUE 
                self._append_to_top(f"[Labeling STOPPED] Random mode ended. Was applying '{stopped_label}'.\n")
            self._append_to_top("[Info] Random labeling session STOPPED.\n")
        
        else: # ---- STARTING RANDOM MODE ----
            if not self.record: # Should not happen due to button state, but as safeguard
                messagebox.showwarning("Not Recording", "Please start recording before starting a random labeling session.")
                return
            if not SELECTABLE_LABELS_LIST:
                messagebox.showerror("Setup Error", "No labels available in SELECTABLE_LABELS_LIST to choose from.")
                return

            if not self.generate_random_label_sequence(): 
                self._update_random_mode_status_display() 
                return

            self.random_labeling_mode_active.set(True)
            self.random_mode_button_text_var.set("Stop Random Session")
            self.random_mode_toggle_button.config(state=tk.NORMAL) # Can always stop
            self.label_combo.config(state=tk.DISABLED)
            self.random_label_count_entry.config(state=tk.DISABLED)
            
            if self.random_label_sequence: # Prime the combobox var for consistency
                 self.current_selected_label_var.set(self.random_label_sequence[0])

            self._append_to_top(f"[Info] Random labeling session STARTED. {self.total_random_labels_in_sequence} items in sequence.\n")
        
        self._update_random_mode_status_display()

    def on_spacebar_press(self, event=None):
        focused_widget = self.master.focus_get()

        if focused_widget == self.entry: # Allow Entry widget to use space for typing
            return 

        if not self.record:
            if self.ser and self.ser.is_open:
                 self._append_to_top("[Warning] Press 'Record' to enable labeling.\n")
            return "break" 

        # --- Handle Random Labeling Mode ---
        if self.random_labeling_mode_active.get():
            if self.current_random_label_index >= self.total_random_labels_in_sequence:
                self._append_to_top("[Info] Random labeling sequence complete. Stop session or start a new one.\n")
            else:
                current_target_label = self.random_label_sequence[self.current_random_label_index]
                if not self.labeling_is_active: # Was not labeling, now START
                    self.current_continuous_label = current_target_label
                    self.labeling_is_active = True
                    self._append_to_top(f"[Labeling STARTED] Random: Applying '{self.current_continuous_label}' ({self.current_random_label_index + 1}/{self.total_random_labels_in_sequence}).\n")
                else: # Was labeling, now STOP
                    stopped_label = self.current_continuous_label
                    self.labeling_is_active = False
                    self._append_to_top(f"[Labeling STOPPED] Random: Finished '{stopped_label}' ({self.current_random_label_index + 1}/{self.total_random_labels_in_sequence}).\n")
                    
                    self.current_random_label_index += 1 
                    if self.current_random_label_index < self.total_random_labels_in_sequence:
                        next_label_in_sequence = self.random_label_sequence[self.current_random_label_index]
                        self.current_selected_label_var.set(next_label_in_sequence) 
                        self._append_to_top(f"[Info] Random: Next is '{next_label_in_sequence}'.\n")
                    else:
                        self._append_to_top("[Info] Random labeling sequence complete!\n")
            self._update_random_mode_status_display()

        # --- Handle Normal Labeling Mode ---
        else:
            if not SELECTABLE_LABELS_LIST:
                self._append_to_top("[Warning] No labels defined to select from.\n")
            elif not self.labeling_is_active: # Was not labeling, now START
                self.current_continuous_label = self.current_selected_label_var.get()
                if not self.current_continuous_label: 
                    self.current_continuous_label = DEFAULT_UNLABELED_VALUE
                self.labeling_is_active = True
                self._append_to_top(f"[Labeling STARTED] Applying '{self.current_continuous_label}' continuously.\n")
            else: # Was labeling, now STOP
                stopped_label = self.current_continuous_label
                self.labeling_is_active = False
                self._append_to_top(f"[Labeling STOPPED] Was applying '{stopped_label}'.\n")
                
                try:
                    current_idx = SELECTABLE_LABELS_LIST.index(self.current_selected_label_var.get())
                    next_idx = (current_idx + 1) % len(SELECTABLE_LABELS_LIST)
                    next_label_val = SELECTABLE_LABELS_LIST[next_idx]
                    self.current_selected_label_var.set(next_label_val)
                    self._append_to_top(f"[Info] Next label selected: '{next_label_val}'.\n")
                except (ValueError, IndexError): 
                    if SELECTABLE_LABELS_LIST:
                         self.current_selected_label_var.set(SELECTABLE_LABELS_LIST[0])

        # Prevent default spacebar action if it was used for labeling (e.g. in Combobox)
        if focused_widget == self.label_combo:
            return "break"
        return None


    def update_image(self):
        if not self.cap.isOpened(): 
            self.master.after(100, self.update_image)
            return
        
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.flip(frame, 1) # Mirror
            current_date_str = datetime.now().strftime("%Y-%m-%d") 

            use_pillow = self.cv_thai_font_small is not None and self.cv_thai_font_large is not None
            y_offset = 20 
            line_spacing = 30 
            text_x_pos = 15

            if use_pillow:
                pil_image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                draw = ImageDraw.Draw(pil_image)
                text_color_pil_green = (0, 255, 0)
                text_color_pil_yellow = (255, 255, 0)

                draw.text((text_x_pos, y_offset), f"T: {self.current_time_ms} ms", font=self.cv_thai_font_small, fill=text_color_pil_green)
                y_offset += line_spacing
                draw.text((text_x_pos, y_offset), self.text_record, font=self.cv_thai_font_small, fill=text_color_pil_green)
                y_offset += line_spacing
                
                if self.output_filename_base and self.record:
                    draw.text((text_x_pos, y_offset), f"File: {self.output_filename_base}", font=self.cv_thai_font_small, fill=text_color_pil_green)
                    y_offset += line_spacing
                
                draw.text((text_x_pos, y_offset), f"Date: {current_date_str}", font=self.cv_thai_font_small, fill=text_color_pil_green)
                y_offset += line_spacing

                if self.record:
                    active_label_str = self.current_continuous_label if self.labeling_is_active else DEFAULT_UNLABELED_VALUE
                    label_status_str = " (ACTIVE)" if self.labeling_is_active else " (INACTIVE)"
                    # For random mode, show the target, for normal mode show selected
                    display_label_source = self.current_selected_label_var.get()
                    if self.random_labeling_mode_active.get() and self.current_random_label_index < self.total_random_labels_in_sequence:
                         display_label_source = self.random_label_sequence[self.current_random_label_index]

                    full_label_display = f"Label: {display_label_source}"
                    if self.labeling_is_active : full_label_display += f" -> {active_label_str}{label_status_str}"
                    else: full_label_display += label_status_str

                    draw.text((text_x_pos, y_offset), full_label_display, font=self.cv_thai_font_large, fill=text_color_pil_yellow)
                
                frame = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)
            else: 
                font_cv = cv2.FONT_HERSHEY_SIMPLEX; thickness_cv = 1; scale_small = 0.6; scale_large = 0.7
                color_cv_green = (0,255,0); color_cv_yellow = (0,255,255) 

                cv2.putText(frame, f"T: {self.current_time_ms} ms", (text_x_pos, y_offset), font_cv, scale_small, color_cv_green, thickness_cv, cv2.LINE_AA)
                y_offset += line_spacing
                cv2.putText(frame, self.text_record, (text_x_pos, y_offset), font_cv, scale_small, color_cv_green, thickness_cv, cv2.LINE_AA)
                y_offset += line_spacing

                if self.output_filename_base and self.record:
                    cv2.putText(frame, f"File: {self.output_filename_base}", (text_x_pos, y_offset), font_cv, scale_small, color_cv_green, thickness_cv, cv2.LINE_AA)
                    y_offset += line_spacing
                
                cv2.putText(frame, f"Date: {current_date_str}", (text_x_pos, y_offset), font_cv, scale_small, color_cv_green, thickness_cv, cv2.LINE_AA)
                y_offset += line_spacing
                
                if self.record:
                    active_label_str = self.current_continuous_label if self.labeling_is_active else DEFAULT_UNLABELED_VALUE
                    label_status_str = " (ACTIVE)" if self.labeling_is_active else " (INACTIVE)"
                    display_label_source = self.current_selected_label_var.get()
                    if self.random_labeling_mode_active.get() and self.current_random_label_index < self.total_random_labels_in_sequence:
                         display_label_source = self.random_label_sequence[self.current_random_label_index]
                    
                    full_label_display = f"Label: {display_label_source}"
                    if self.labeling_is_active : full_label_display += f" -> {active_label_str}{label_status_str}"
                    else: full_label_display += label_status_str
                    cv2.putText(frame, full_label_display, (text_x_pos, y_offset), font_cv, scale_large, color_cv_yellow, thickness_cv, cv2.LINE_AA)

            if self.record and self.video_writer: self.video_writer.write(frame)

            cv2image_for_tk = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            pil_img_for_tk = Image.fromarray(cv2image_for_tk)
            w_lw = self.image_label.winfo_width(); h_lw = self.image_label.winfo_height()
            if w_lw <= 1 or h_lw <= 1: # Initial sizing
                h_orig_frame, w_orig_frame = frame.shape[:2]
                w_lw, h_lw = w_orig_frame, h_orig_frame 
            if w_lw <=1 or h_lw <=1: # Still not sized, defer
                self.master.after(30, self.update_image)
                return

            w_p, h_p = pil_img_for_tk.size; img_ar = w_p / h_p; lbl_ar = w_lw / h_lw
            if img_ar > lbl_ar: new_w = w_lw; new_h = int(new_w / img_ar)
            else: new_h = h_lw; new_w = int(new_h * img_ar)
            new_w = max(1,new_w); new_h = max(1,new_h)
            try:
                rs_filter = Image.Resampling.LANCZOS if hasattr(Image,"Resampling") else Image.LANCZOS
                r_pil_img = pil_img_for_tk.resize((new_w,new_h), rs_filter)
            except Exception as e: 
                print(f"Resize err: {e}")
                self.master.after(30,self.update_image)
                return

            canvas = Image.new("RGB",(w_lw,h_lw),(0,0,0)) # Black background for letterboxing
            x_off = (w_lw - new_w)//2; y_off = (h_lw - new_h)//2
            canvas.paste(r_pil_img,(x_off,y_off))
            imgtk = ImageTk.PhotoImage(image=canvas)
            self.image_label.imgtk = imgtk; self.image_label.configure(image=imgtk)

        self.master.after(30, self.update_image) # Approx 33 FPS for UI update

    def refresh_ports(self):
        ports=[p.device for p in serial.tools.list_ports.comports()]
        self.port_combo['values']=ports
        if ports:
            if self.port_var.get() not in ports: self.port_var.set(ports[0])
        else: self.port_var.set('')

    def connect_serial(self):
        selected=self.port_var.get().strip()
        if not selected: messagebox.showwarning("No Port Selected","Please select a serial port."); return
        try:
            self.ser=serial.Serial(selected,BAUDRATE,timeout=SERIAL_TIMEOUT)
        except serial.SerialException as e: messagebox.showerror("Connection Error",f"Could not open {selected!r}:\n{e}"); return

        self.port_combo.config(state='disabled'); self.refresh_button.config(state='disabled')
        self.connect_button.config(state='disabled'); self.disconnect_button.config(state=tk.NORMAL)
        self.entry.config(state=tk.NORMAL); self.send_button.config(state=tk.NORMAL)
        self._clear_panes(); self._append_to_top(f"[Arduino] *** Connected to {selected} @ {BAUDRATE} baud ***\n")
        self.master.after(100, self._send_initial_reset)
        self.alive=True
        self.reader_thread=threading.Thread(target=self.read_from_serial,daemon=True); self.reader_thread.start()

    def _send_initial_reset(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b"reset\n")
                self._append_to_top("[You] reset (on connect)\n")
                time.sleep(BOARD_RESET_DELAY_MS / 1000.0) 
            except serial.SerialException as e:
                messagebox.showerror("Write Error",f"Failed to send initial reset command:\n{e}")
            except Exception as e:
                self._append_to_top(f"[Error] Exception during initial reset: {e}\n")

    def disconnect_serial(self):
        if self.ser and self.ser.is_open:
            self.alive=False
            if self.reader_thread and self.reader_thread.is_alive(): self.reader_thread.join(timeout=2)
            try: self.ser.close()
            except Exception as e: self._append_to_top(f"[Error] Failed to close serial port: {e}\n")
        self.entry.config(state=tk.DISABLED); self.send_button.config(state=tk.DISABLED)
        self.port_combo.config(state='readonly'); self.refresh_button.config(state=tk.NORMAL)
        self.connect_button.config(state=tk.NORMAL); self.disconnect_button.config(state=tk.DISABLED)
        self._append_to_top("[Arduino] *** Disconnected ***\n")
        
        # If recording was active, stop it (this will also handle random mode stop)
        if self.record:
            self.toggle_record()


    def read_from_serial(self):
        while self.alive:
            try:
                if self.ser and self.ser.is_open and self.ser.in_waiting > 0:
                    raw = self.ser.readline().decode(errors='ignore').rstrip('\r\n')
                    if not raw: continue
                    if raw.startswith("[Sensor]"):
                        if not self.sensor_paused: self._append_to_bottom(raw + "\n")
                        raw_data = raw.replace('[Sensor] ', ''); readings = raw_data.split(",")
                        if readings: self.current_time_ms = readings[0]
                        if self.record:
                            converted = []
                            for item in readings:
                                try: val = float(item); val = int(val) if val.is_integer() else val
                                except ValueError: val = item
                                converted.append(val)
                            label_for_this_row = DEFAULT_UNLABELED_VALUE
                            if self.labeling_is_active: label_for_this_row = self.current_continuous_label
                            converted.append(label_for_this_row)
                            self.sensor_data.append(converted)
                    else: self._append_to_top(raw + "\n")
                else: time.sleep(0.02)
            except (serial.SerialException, OSError) as e:
                if self.alive: self._append_to_top(f"[Error] Serial read error: {e}\n"); self.disconnect_serial(); break 
            except Exception as e:
                if self.alive: self._append_to_top(f"[Critical] Unexpected error in reader: {e}\n"); self.disconnect_serial(); break 
        
        main_thread_still_running = any(t.name == "MainThread" and t.is_alive() for t in threading.enumerate())
        if main_thread_still_running:
            if self.alive: 
                self._append_to_top("[Info] Serial reader thread stopped due to an issue.\n")
            else: 
                self._append_to_top("[Info] Serial reader thread stopped.\n")


    def send_data(self):
        text=self.entry.get().strip()
        if not text or not self.ser or not self.ser.is_open: return
        try: self.ser.write((text+"\n").encode()); self._append_to_top(f"[You] {text}\n")
        except serial.SerialException as e: messagebox.showerror("Write Error",f"Failed to write: {e}")
        self.entry.delete(0,tk.END)

    def toggle_pause_sensor(self):
        self.sensor_paused = not self.sensor_paused
        self.pause_button.config(text="Resume Sensor" if self.sensor_paused else "Pause Sensor")

    def toggle_record(self):
        if not self.record: # ---- STARTING RECORDING ----
            if not self.ser or not self.ser.is_open:
                messagebox.showwarning("Not Connected","Connect to a serial port before recording."); return
            try: self.ser.write(b"set_time\n"); self._append_to_top("[You] set_time\n")
            except serial.SerialException as e: messagebox.showerror("Write Error",f"Failed to send set_time: {e}"); return

            user_filename = self.filename_entry_var.get().strip()
            if not user_filename:
                self.output_filename_base = datetime.now().strftime("%Y-%m-%d_%H%M%S") # Fixed date format for better sorting
                self._append_to_top(f"[Info] No custom filename. Using default: {self.output_filename_base}\n")
            else:
                safe_filename = user_filename.replace(" ", "_")
                safe_filename = re.sub(r'[^\w\-.]+', '', safe_filename) 
                if not safe_filename: 
                    self.output_filename_base = datetime.now().strftime("%Y-%m-%d_%H%M%S")
                    self._append_to_top(f"[Warning] Invalid custom filename '{user_filename}'. Using default: {self.output_filename_base}\n")
                else:
                    self.output_filename_base = safe_filename
                    self._append_to_top(f"[Info] Using filename base: {self.output_filename_base}\n")

            self.filename_entry.config(state=tk.DISABLED) 
            self.random_mode_toggle_button.config(state=tk.NORMAL) # Enable random mode button

            self.sensor_data.clear()
            self.labeling_is_active = False # Reset labeling state
            self.current_continuous_label = DEFAULT_UNLABELED_VALUE
            self._append_to_top("[Info] Waiting for board to sync time...\n"); self.master.update_idletasks()
            video_path = os.path.join(OUTPUT_DIR, f"{self.output_filename_base}.mp4")
            self.master.after(BOARD_RESET_DELAY_MS, lambda: self._finalize_recording_start(video_path))
        
        else: # ---- STOPPING RECORDING ----
            self.record = False
            self.record_button.config(text="Record"); self.text_record = "Not Recording"
            self.filename_entry.config(state=tk.NORMAL) 
            self.random_mode_toggle_button.config(state=tk.DISABLED) # Disable random mode button

            if self.video_writer:
                self.video_writer.release(); self.video_writer = None
                self._append_to_top(f"[Info] Video recording stopped: {self.output_filename_base}.mp4\n")
            
            if self.random_labeling_mode_active.get(): # If random mode was active, stop it
                self.toggle_random_labeling_mode() 
            elif self.labeling_is_active: # If normal labeling was active, log its stop
                self._append_to_top(f"[Warning] Labeling of '{self.current_continuous_label}' was active and is now stopped due to recording stop.\n")
                self.labeling_is_active = False
                self.current_continuous_label = DEFAULT_UNLABELED_VALUE
            
            self._save_sensor_data()
            self.output_filename_base = None
            self._update_random_mode_status_display() # Ensure status is correct


    def _finalize_recording_start(self, video_path: str):
        if not (self.cap and self.cap.isOpened()):
            messagebox.showerror("Camera Error","Camera not available or not opened.")
            self._append_to_top("[Error] Camera not available for recording.\n")
            self.record_button.config(text="Record"); self.text_record = "Not Recording"
            self.filename_entry.config(state=tk.NORMAL) 
            self.random_mode_toggle_button.config(state=tk.DISABLED)
            self.output_filename_base = None
            self.labeling_is_active = False # Ensure reset
            self.current_continuous_label = DEFAULT_UNLABELED_VALUE
            return

        self.record = True; self.record_button.config(text="Stop Recording"); self.text_record = "Recording..."
        self._append_to_top(f"[Info] Recording started. Base filename: {self.output_filename_base}\n")
        width=int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or 640
        height=int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 480
        fourcc=cv2.VideoWriter_fourcc(*'mp4v') 
        try:
            self.video_writer=cv2.VideoWriter(video_path,fourcc,VIDEO_FPS,(width,height))
            if not self.video_writer.isOpened():
                messagebox.showerror("Video Writer Error",f"Could not open video writer for {video_path}.\nCheck codec availability (e.g., mp4v, XVID, MJPG).")
                self._append_to_top(f"[Error] Failed to open video writer for {video_path}\n")
                self.record=False; self.record_button.config(text="Record"); self.text_record="Not Recording"
                self.filename_entry.config(state=tk.NORMAL)
                self.random_mode_toggle_button.config(state=tk.DISABLED) 
                self.output_filename_base=None; self.video_writer=None
                self.labeling_is_active = False
                self.current_continuous_label = DEFAULT_UNLABELED_VALUE
                return
        except Exception as e:
            messagebox.showerror("Video Writer Error",f"Exception creating video writer: {e}")
            self._append_to_top(f"[Error] Exception creating video writer: {e}\n")
            self.record=False; self.record_button.config(text="Record"); self.text_record="Not Recording"
            self.filename_entry.config(state=tk.NORMAL)
            self.random_mode_toggle_button.config(state=tk.DISABLED)
            self.output_filename_base=None; self.video_writer=None
            self.labeling_is_active = False
            self.current_continuous_label = DEFAULT_UNLABELED_VALUE
            return
        self._append_to_top(f"[Info] Video recording to: {video_path}\n")

    def _save_sensor_data(self):
        if not self.sensor_data or not self.output_filename_base:
            if self.output_filename_base : self._append_to_top("[Info] No sensor data to save.\n")
            return
        filepath = os.path.join(OUTPUT_DIR, f"{self.output_filename_base}_sensor.csv")
        try:
            with open(filepath, 'w', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile); writer.writerow(column_names); writer.writerows(self.sensor_data)
            self._append_to_top(f"[Info] Sensor data saved to: {filepath}\n")
        except IOError as e: self._append_to_top(f"[Error] Could not save sensor data: {e}\n"); messagebox.showerror("Save Error",f"Failed to save: {e}")
        except Exception as e: self._append_to_top(f"[Critical] Error saving sensor data: {e}\n"); messagebox.showerror("Save Error",f"Unexpected error: {e}")
        finally: self.sensor_data.clear()

    def close(self):
        if messagebox.askokcancel("Quit","Do you really want to quit?"):
            self.alive=False # Signal threads to stop
            if self.reader_thread and self.reader_thread.is_alive(): 
                self.reader_thread.join(timeout=1) # Reduced timeout
            
            if self.ser and self.ser.is_open:
                try: self.ser.close()
                except Exception: pass
            
            if self.record : # If was recording, ensure files are closed
                if self.video_writer: 
                    self.video_writer.release()
                    self.video_writer=None
                self._save_sensor_data() # Save any pending data

            if self.cap and self.cap.isOpened(): self.cap.release()
            self.master.destroy()

    def _append_text_to_widget(self,widget:scrolledtext.ScrolledText,text:str):
        widget.configure(state=tk.NORMAL); widget.insert(tk.END,text); widget.see(tk.END); widget.configure(state=tk.DISABLED)
    def _append_to_top(self,text:str): 
        if self.master.winfo_exists(): # Check if master window still exists
            self.master.after(0,lambda: self._append_text_to_widget(self.top_text,text))
    def _append_to_bottom(self,text:str): 
        if self.master.winfo_exists():
            self.master.after(0,lambda: self._append_text_to_widget(self.bottom_text,text))
    def _clear_panes(self):
        def inner():
            if self.master.winfo_exists():
                for pane in(self.top_text,self.bottom_text):
                    pane.configure(state=tk.NORMAL); pane.delete('1.0',tk.END); pane.configure(state=tk.DISABLED)
        if self.master.winfo_exists():
            self.master.after(0,inner)

if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("1100x780") # Increased height slightly for new controls
    gui = SerialGUI(root)
    root.mainloop()