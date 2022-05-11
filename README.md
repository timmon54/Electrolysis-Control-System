# Electrolysis-Control-System
A custom electrolysis control system for optimizing HHO generation efficiency.

A system that automatically regulates various input parameters to optimize efficiency in hydrogen and oxygen generation is being developed for small
production usage cases. A control board was created that supports many aspects of
functionality and safety features to increase efficiency and usability. Electrolysis cells
lose efficiency at elevated temperature, and additionally have an optimal voltage range
that can change with temperature and various other cell parameters. The developed
control system makes use of precise temperature measurements and a digital poten-
tiometer to control an external power supply, driving the cell to the highest efficiency it
is capable of given an environment. Additional features and sensors add a margin of
safety and ability to collect data for future cell optimization. A model will be created using data acquisition and either Ziegler-Nichols or Hammerstein identification techniques.

The current status of this project is a controller nearing the stages of completion to the point that data collection will commence and a full PID controller will be implemented in order to maximize the efficiency of an electrolyzer for hydroxy gas generation.
