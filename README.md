# Mechatronics System Design: Theo Jansen Walking Mechanism
This repository contains the Python implementation and results for the kinematic analysis of the Theo Jansen linkage, based on the attached `MSD_report.pdf`.

## Demonstration Video

The following provides a visual demonstration of the simulated gait cycle, generated from the kinematic model.

<video width="640" height="480" controls>
  <source src="/Hardware_results/media1.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

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
