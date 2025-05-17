
# Best micro stepping value for each motor
To setup each DM542T driver for each motor we first need to know:
Max stepping frequency: $f_{max} \left[H_z\right]$
Max speed of the motor: $R_{max}\left[ \frac{Rev}{Min}\right]$
Steps per revelation: $S_{\micro}\left[\frac{\micro Step}{rev_{link}}\right]$
Gear reduction on each motor: $N$

#### Max stepping frequency
Frequency at witch we can step the motor in [[DM542T_V4.0.pdf#page=3|Electrical Specifications chapter 2]] we can see that $f_{max} = 150\left[kH_z\right]$

#### Max speed of the motor
The max Angular Velocity and Acceleration that each link of the robot has bean design to be 
$15\left[ \frac{Rev}{Min}\right]$ and $\pi\left[ \frac{Rad}{sec}\right]$ 
Max speed of the motor: $15\left[ \frac{Rev}{Min}\right]$

#### Steps per revelation
The motors we are using, from its Data sheet we can see each has a $1.8\left[\frac{Deg}{step}\right]$ without any micro stepping for gear reduction.
$\frac{360}{1}\left[\frac{Deg}{rev}\right] \times \frac{1}{1.8} \left[\frac{step}{Deg}\right] = 200 \left[\frac{step}{Deg}\right]$   

### How to find the best micro stepping value for each motor
with all the info we can now fide that value of micro stepping each motors need to run at its specification, the general formula for the amount of steps needed to rotated any motor one revelation is:
#### $\alpha$ Calculation 
$\frac{360}{1}\left[\frac{Deg}{rev_{motor}}\right] \times \frac{1}{1.8} \left[\frac{step}{Deg}\right] \times \frac{S_{\micro}}{1}\left[\frac{\micro Step}{step}\right] \times  A\left[\frac{rev_{motor}}{rev_{shaft}}\right] \times N \left[\frac{rev_{shaft}}{rev_{link}}\right] = 200\times A N S_{\micro} \left[\frac{\micro Step}{rev_{link}}\right] = \alpha \left[\frac{\micro Step}{rev_{link}}\right]$   
#### $f_{max}$ Calculation
$R_{max}\left[ \frac{rev_{link}}{Min}\right] \times \alpha \left[\frac{\micro Step}{rev_{link}}\right] \times \frac{1}{60}\left[ \frac{Min}{Sec}\right] = f_{max} \left[ \frac{\micro Step}{Sec}\right]$   
$\frac{R \alpha}{60} = f_{max} \left[ \frac{\micro Step}{sec}\right]$
$\frac{10 \times R  A  N  S_{\micro}}{3} = f_{max} \left[ \frac{\micro Step}{sec}\right]\space$
$S_{\micro}\left[\frac{\micro Step}{rev_{link}}\right] = \frac{3f_{max}}{10\times RAN}$ 

if we now plug in all the values for the each motor and pick the closes micro stepping value that the DM542T can do we get the following 

 ##### J1 Motor micro stepping value:
$S_{\micro} = \frac{3f_{max}}{10\times RAN} = \frac{3 \times 150,000}{10\times 15 \times 10 \times 4} = 75$ 
$\lfloor 75 \rfloor = 64\left[\frac{\micro Step}{rev_{link}}\right]$  

##### J2 Motor micro stepping value: 
$S_{\micro} = \frac{3f_{max}}{10\times RAN} = \frac{3 \times 150,000}{10\times 15 \times 50} = 60$ 
$\lfloor 60 \rfloor = 50$ 

##### J3 Motor micro stepping value:
$S_{\micro} = \frac{3f_{max}}{10\times RAN} = \frac{3 \times 150,000}{10\times 15 \times 50} = 60$ 
$\lfloor 60 \rfloor = 50$  

##### J4 Motor micro stepping value:
$S_{\micro} = \frac{3f_{max}}{10\times RAN} = \frac{3 \times 150,000}{10\times 15 \times 14 \times \frac{28}{10}} = 76.53061224$ 
$\lfloor 76.53061224 \rfloor = 64$  

##### J5 Motor micro stepping value:
$A$ Calculation for J5 Lead screw 
Lead $\frac{Travel}{Revolution} = 8mm$ 
$S_{arc}=R\theta$
$\theta = \frac{S_{arc}}{R} = \frac{8mm}{30mm} = \frac{4}{15} \simeq 0.2 \overline{6}$
$\frac{4}{15}A=2\pi, A = \frac{15\pi}{2}\simeq 23.5619449$

$S_{\micro} = \frac{3f_{max}}{10\times RAN} = \frac{3 \times 150,000}{10\times 15 \times \frac{15\pi}{2}} = 127.3239545$ 
$\lfloor 127.3239545 \rfloor = 128$

##### J6 Motor micro stepping value:
$S_{\micro} = \frac{3f_{max}}{10\times RAN} = \frac{3 \times 150,000}{10\times 15 \times 19} = 157.8947368$ 
$\lfloor 157.8947368 \rfloor = 128$  

# Final Setup Configuration
Here we have the settings for all of the motors here
1: switch is up/ON
0: switch is down/OFF

| MOTOR | SW1 | SW2 | SW3 | SW4 | SW5 | SW6 | SW7 | SW8 |
| ----- | --- | --- | --- | --- | --- | --- | --- | --- |
| J1    | 0   | 1   | 1   | 1   | 1   | 0   | 0   | 1   |
| J2    | 1   | 1   | 0   | 1   | 0   | 1   | 0   | 0   |
| J3    | 0   | 1   | 1   | 1   | 0   | 1   | 0   | 0   |
| J4    | 1   | 1   | 1   | 1   | 1   | 0   | 0   | 1   |
| J5    | 0   | 1   | 1   | 1   | 0   | 0   | 0   | 1   |
| J6    | 1   | 1   | 1   | 1   | 0   | 0   | 0   | 1   |


