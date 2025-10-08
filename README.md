# Mechatronics System Design: Theo Jansen Walking Mechanism
This repository contains the Python implementation and results for the kinematic analysis of the Theo Jansen linkage, based on the attached `MSD_report.pdf`.

## Demonstration Video

🎥 [Click to watch the walking gait simulation](Hardware_results/media1.mp4)

## Installation

Requires Python with standard scientific libraries (`numpy`, `matplotlib`, `pandas`).

### Install dependencies (adjust based on your requirements.txt if present)
```pip install numpy matplotlib pandas```

### Quick Run Sequence
Execute analysis scripts in order, as they rely on the output of previous steps:

#### 1. Forward Kinematics (Position Data):
```python main.py --output_dir main_results```

#### 2. Velocity Analysis:
```python vel_analysis.py --input_data main_results --output_dir vel_results```

#### 3. Acceleration Analysis:
```python accln_analysis.py --input_data main_results --output_dir accln_results```


## Documentation

* Review MSD_report.pdf for the underlying mathematical derivation (Sections 2.1 & 2.2).
* Refer to Techamronics_Presentation.pptx for project summary.
