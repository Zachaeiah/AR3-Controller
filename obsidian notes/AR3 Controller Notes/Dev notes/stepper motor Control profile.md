
## 2.2 Fundamental stepper motor equations
To create rotational motion in a stepper motor, the current thru the windings must change in the correct order. This is obtained using a driver that gives the correct output sequence when subjected to a pulse (‘stepper motor pulse’) and a direction signal. To rotate the stepper motor at a constant speed, pulses must be generated at a steady rate,
![[Pasted image 20250329101942.png]]
A counter generates these pulses, running at the frequency $f_t [Hz]$ . The delay δt programmed by the counter c is
$\delta t= ct = \frac{c}{f_t} [s]$

The motor step angle α , position θ , and speed ω are given by 
$\alpha = \frac{2\pi}{spr} [Rad]$
$\theta =n\alpha$ 
$\omega = \frac{\alpha}{\delta t}$
## 2.3 Linear speed ramp
To start and stop the stepper motor in a smooth way, control of the acceleration and deceleration is needed. This shows the relation between acceleration, speed and position. Using a constant acceleration/deceleration gives a linear speed profile.

![[Pasted image 20250329095415.png]]

where spr is the number of steps per round, n is the number of steps, and 1 rad/sec = 9,55 rpm

The time delay δt between the stepper motor pulses controls the speed. These time delays must be calculated in order to make the speed of the stepper motor follow the speed ramp as closely as possible. Discrete steps control the stepper motor motion, and the resolution of the time delay between these steps is given by the frequency of the timer.
![[Pasted image 20250329101333.png]]

### 2.3.1 Exact calculations of the inter-step delay
The first counter delay c0 as well as succeeding counter delays cn, are given by:
$c_o = \frac{1}{t_t}\sqrt{\frac{2\alpha}{\dot{\omega}}}$ 
$c_n = c_o\left( \sqrt{n+1} - \sqrt{n}\right)$ 
The computational power of a microcontroller is limited, and calculating two square roots is time consuming. Therefore an approximation with less computational complexity is considered.
The counter value at the time n, using Taylor series approximation for the inter-step delay  is given by:
$c_n = c_{n-1} - \frac{2 c_{n-1}}{4n+1}$
This calculation is much faster than the double square root, but introduces an error of 0.44 at n =1. A way to compensate for this error is by multiplying 0 c with 0,676.

### 2.3.2 Change in acceleration

x, the acceleration is given by $c_n$ and n. If a change in acceleration (or deceleration) is done, a new n must be calculated. The time $t_n$ and n as a function of the motor acceleration, speed and step angle are given by:
$t_n = \frac{\omega_n}{\dot\omega}$
$n = \frac{\dot\omega t_n^2}{2\alpha}$
Merging these equation gives the relationship
$n\dot\omega = \frac{\omega^2}{2\alpha}$
This shows that the number of steps needed to reach a given speed is inversely proportional to the acceleration: $n_1\dot\omega_1 = n_2\dot\omega_2$ 
This means that changing the acceleration from $\dot\omega_1$  to $\dot\omega_2$  & is done by changing n .
![[Pasted image 20250329103415.png]]
Moving a given number of steps, deceleration must start at the right step to end at zero speed. The following equation is used to find $n_1$:

$n_1 = \frac{\left( n_1 + n_2\right)\dot\omega_2}{\left(\dot\omega_1 + \dot\omega_2\right)}$
