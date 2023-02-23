# a-d-conversions<br>
A mixed assembly and C program that generates an analog servo position and converts it to a digital number to set the position of the servo. The FRDM-KL05Z board from NXP is used, including the pulse width modulation, DAC0, and ADC0 peripherals.

![ProgramResults](https://github.com/Helena-Lynd/a-d-conversions/blob/main/program-output.png?raw=true)

## Description<br>
Analog positions descibe the physical location that a servo needs to be moved to (e.g. "1" could be the left side of a piece of paper and "5" could be the right side), however you can't tell a microcontroller to "move to 5". That analog value must be converted into a digital value, which corresponds to the analog value in a way that is meaningful to the microcontroller. This program converts user-input analog values to these new digital values.
## Getting Started<br>
### Dependencies
- A method to compile the source files into an executable (e.g. Keil uVision5)
- KL05 board connected to a terminal (e.g. PuTTY)
- A servo connected to the KL05 board (e.g. Futaba S3003 Servo)
### Installing
- Download the source files provided to your directory of choice
```
git clone git@github.com:Helena-Lynd/a-d-conversions.git
```
- Compile the source files into an executable
  - If using an IDE, use the "Build" or "Rebuild" feature
### Executing
- Load the executable to your boards flash memory
  - If using an IDE, use the "Download" feature
- Run the program with a connected terminal window open and connected servo
  - The board has a button that can be pressed to initiate the program
- Input the analog position for the servo to move to
  - This was implemented with a Futaba S3003 servo attached to a piece of paper with analog positions written on it
## Modifying
The values for PWM_PERIOD, PWM_DUTY_10, and PWM_DUTY_5 may need to be adjusted to get the servo to move to the desired location. Note that modifiying these values can change the calculated digital values, and therefore the terminal window may display an incorrectly calculated "Analog servo position". This is expected behavior, and considered to be acceptable so long as the servo is moving to the correct location.
## Authors<br>
Helena Lynd
