# Sign_Language_Detection
![](happy-cat-happy-happy-cat.gif)
![](https://media2.giphy.com/media/v1.Y2lkPTc5MGI3NjExNnY5ZTI1c3JkbTl5eTRkOHVncjY1ajM5aDFtMjM1MHFvZ2JsM2k1MiZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9cw/Xr9TlAqw3S7VPOrftK/giphy.gif)


# Sensor Data Collection & Labeling Tool

A comprehensive GUI application built with Python and Tkinter for the synchronized collection and real-time labeling of time-series sensor data and video footage. This tool is designed to streamline the data-gathering process for machine learning projects, particularly in fields like gesture recognition.

![image](https://github.com/user-attachments/assets/1ffb339d-2ff2-4c28-bbcd-96e00b555030)


## Key Features

- **Synchronized Data & Video**: Simultaneously records a webcam feed to an `.mp4` file and sensor data to a `.csv` file with aligned timestamps.
- **Live Video Overlay**: The GUI displays a live camera feed with overlaid status information, such as recording status, current timestamp, and labeling prompts, providing clear instructions to the user.
- **Dual Labeling Modes**:
    - **Normal (Sequential) Mode**: Guides the user through a predefined list of labels in order, for a set number of cycles.
    - **Random Mode**: Presents labels from a set in a randomized sequence, ideal for reducing bias in data collection.
- **On-the-fly Error Correction**: A "Flag & Redo" feature (`d` key) allows the user to instantly mark a misperformed action. The data is flagged, and a cooldown is initiated before prompting a retry.
- **Customizable Labels & Grouping**: Easily define a custom set of labels. Long lists can be broken down into smaller, manageable groups for separate collection sessions.
- **Configurable Breaks**: Automatically inserts a timed break between actions or cycles to reduce user fatigue and ensure data quality.
- **Serial Port Management**: Automatically detects available COM ports, with simple connect/disconnect controls and a raw data feed for monitoring.

## Requirements

- Python 3.7+
- A webcam
- A microcontroller (e.g., Arduino, ESP32) programmed to send sensor data over serial.
- A TrueType Font file (e.g., `tahoma.ttf`) for rendering text on the video overlay.

## Setup & Installation

1.  **Clone the Repository**
    ```bash
    git clone <your-repository-url>
    cd <your-repository-name>
    ```

2.  **Create a Virtual Environment (Recommended)**
    ```bash
    python -m venv venv
    # On Windows
    .\venv\Scripts\activate
    # On macOS/Linux
    source venv/bin/activate
    ```

3.  **Install Dependencies**
    The required libraries are listed in `requirements.txt`. Install them using pip:
    ```bash
    pip install -r requirements.txt
    ```

4.  **Prepare your Microcontroller**
    Your device must send sensor data over the serial port in a specific, comma-separated format, prefixed with `[Sensor]`. The script is configured to parse this line:
    ```
    [Sensor]timestamp_ms,ax_slav,ay_slav,...,flex_4
    ```
    - **Prefix**: The line **must** start with `[Sensor]`.
    - **Columns**: The number of data points must match the `COLUMN_NAMES` list in the script (excluding the `Label` and `timestamp_ms` columns).

## How to Use

1.  **Launch the Application**
    ```bash
    python main.py
    ```

2.  **Initial Configuration**
    - **Connect**: Select the correct `Port` from the dropdown and click `Connect`. You should see data streaming in the "Sensor Data" panel.
    - **Set Filename**: Enter a descriptive `Filename Base` (e.g., `subject_01_session_1`).
    - **Set Labels**: (Optional) Modify the comma-separated list in "Custom Label Set" and click `Update Labels`.
    - **Choose Mode**: In the "Setting" frame:
        - **Normal Mode**: Uncheck "Enable Random Mode". `Count per Label` sets the number of full *cycles* to perform.
        - **Random Mode**: Check "Enable Random Mode". `Count per Label` sets how many *times each label* will appear in the shuffled sequence.
    - **Set Break Time**: Adjust the `Break Time (s)` between actions.

3.  **Recording & Labeling Workflow**
    - **Start Recording**: Press the **`Record`** button or the **`a`** key. The camera overlay will show `RECORDING: True`.
    - **Follow Prompts**: The video overlay and status bar will prompt you for the first action (e.g., `PROMPT: Press 's' to START 'one'`).
    - **Label an Action**:
        - Press the **`s` key** to **START** labeling the prompted action. The `CURRENT LABEL` overlay will change color.
        - Perform the gesture.
        - Press the **`s` key again** to **STOP** labeling.
    - **Correcting an Error**:
        - If you make a mistake *while an action is being labeled* (between the two `s` presses), press the **`d` key**.
        - This flags the data as `error_redo`, stops labeling, and starts a cooldown before prompting you to try the **same action again**.
    - **Continue**: Repeat the start/stop cycle until the sequence is complete.
    - **Stop Recording**: Press the **`Stop Recording`** button or the **`a`** key.

## Output Files

All data is saved in the `out_data` directory, organized by date:
```
out_data/
└── YYYY-MM-DD/
    ├── data/
    │   └── TIMESTAMP_filename_base_sensor.csv
    └── video/
        └── TIMESTAMP_filename_base.mp4
```
The final column in the CSV file, `Label`, will contain your applied label or one of the special state labels (`nothing`, `break_time`, `error_redo`, `cooldown`).

## Configuration

Key parameters can be adjusted directly in the script's `CONFIGURATION` section:
- `BAUD_RATE`: Must match your microcontroller's baud rate.
- `FONT_FAMILY`, `FONT_SIZE_*`: GUI font settings.
- `OUTPUT_DIR`: The root directory for all saved files.
- `VIDEO_FPS`: Frames per second for the output video.
- `REDO_COOLDOWN_S`: The cooldown duration in seconds after flagging an error.
- `THAI_FONT_PATH`: Path to the `.ttf` font file for the video overlay.
- `SELECTABLE_LABELS_LIST`: The default list of labels.
- `COLUMN_NAMES`: The schema for the output CSV. **This must match your microcontroller's output.**
