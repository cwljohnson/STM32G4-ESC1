# STM32G4-ESC1

## STM32G4-ESC-DEMO
This is a standard STM32CubeIDE firmware project, that blinks the STATUS LED every 2s.

## STM32G4-ESC-SIMULINK
This is a simulink embedded coder project.

### MATLAB Requirements
- MATLAB R2023A (this is what I've used for everything)
- Simulink
- Embedded Coder
- MATLAB Coder
- Simulink Coder
- STM32 Embedder Coder Support Package

https://au.mathworks.com/matlabcentral/fileexchange/43093-embedded-coder-support-package-for-stmicroelectronics-stm32-processors

Follow installation instructions for above, and make sure to select STM32G4xxxx as target hardware.

After this, you will need to install STM32 stuff for code gen and flashing code, MATLAB recommends the following versions, and I've verified they work.

STM32CubeMX - Version 6.4.0
https://www.st.com/en/development-tools/stm32cubemx.html#st-get-software

STM32CubeProg - Version 2.4.0
https://www.st.com/en/development-tools/stm32cubeprog.html#st-get-software

I've then followed this tutorial for setting the project up
https://au.mathworks.com/help/supportpkg/stmicroelectronicsstm32f4discovery/ug/Getting-started-stm32cubemx.html

After you've installed everything above, you should be able to just load up the simulink project, make sure

``Hardware Settings -> Hardware Implementation -> Hardware board settings -> Target hardware resources -> STM32CubeMX project file`` 

points to

 ``ROOTDIR\STM32G4-ESC1\STM32G4-ESC-SIMULINK\STM32Code\STM32G4-ESC-Simulink.ioc``