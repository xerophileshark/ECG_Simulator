# ECG_Simulator

In this project, ECG signal simulation is done in order to test an actual heartbeat monitoring device. The simulations are based on three methods:
1. Generated ECG data -> ecgsimulator1.ino
  * The data used in "ecgsimulator1.ino" is sampled from "complete.m" in [this link](https://www.mathworks.com/matlabcentral/fileexchange/10858-ecg-simulation-using-matlab) which I found on Internet.
2. Fourier series method -> ecgsimulator2.ino
3. An accurate programmable method based on M. Nasor's paper -> ecgsimulator3.ino

All methods are implemented using Arduino Due.

## Practical Results

### Accurate Programmable Method

![20191102_103227](https://user-images.githubusercontent.com/30368346/120008463-365c3180-bff0-11eb-9bd1-b704d402224c.jpg)

### Fourier Series Method

![20191109_003349](https://user-images.githubusercontent.com/30368346/120008508-4247f380-bff0-11eb-8127-53180459caf1.jpg)

## References

### Fourier series method
The main references for this method's implementation are:
1. [Halawani, S., Kari, S., Albidewi, I., & Ahmad, A.R. (2014). ECG Simulation using Fourier Series: From Personal Computers to Mobile Devices.](https://www.semanticscholar.org/paper/ECG-Simulation-using-Fourier-Series%3A-From-Personal-Halawani-Kari/188c5a219c7ee238110d32a90677e0be27903c02)
2. karthik raviprakash (2021). ECG simulation using MATLAB (https://www.mathworks.com/matlabcentral/fileexchange/10858-ecg-simulation-using-matlab), MATLAB Central File Exchange. Retrieved May 28, 2021.
3. [ECG SIMULATION USING MATLAB: Principle of Fourier Series](http://azadproject.ir/wp-content/uploads/2014/04/ECG.pdf)

### accurate programmable method
1. [M. J. Burke, & M. Nasor (2001) An accurate programmable ECG simulator, Journal of Medical Engineering & Technology, 25:3, 97-102, DOI: 10.1080/03091900110051640](https://www.tandfonline.com/doi/citedby/10.1080/03091900110051640?scroll=top&needAccess=true)
