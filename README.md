# README #

**Warning:** this module does not use any of the built-in safety features used by ALMotion to prevent the robot from damaging itself. Incorrect usage can lead to damage of your robot. Use at own risk.

### Setup ###

Clone the repository to a qibuild workspace (see [here](http://doc.aldebaran.com/qibuild/beginner/getting_started.html) for help on setting up qibuild). Make sure to build the package using the cross toolchain 
(see [here](https://community.ald.softbankrobotics.com/en/resources/software/language/en-gb) for download of the correspondent NAOqi version, and [here](http://doc.aldebaran.com/qibuild/beginner/qibuild/aldebaran.html#qibuild-using-aldebaran-packages) for help).
If you do not know your NAOqi version, execute this command in your Pepper:
```
lscpu
lsb_release -a
```
#### Create qitoolchain environemnt ####
```
qitoolchain create pepper2.5 ./ctc-linux64-atom-2.5.2.74/toolchain.xml
qitoolchain info
```

#### Create a qibuild configuration ####
```
qibuild add-config pepper2.5 --toolchain pepper2.5
```

#### Configure the project ####
```
qibuild init
qibuild config --wizard (specify the build directory)
qibuild configure -c pepper2.5 motion_controller
```

#### Build and deploy ####
```
qibuild make -c pepper2.5 motion_controller
qibuild deploy -c pepper2.5 --url nao@10.0.204.154:/home/nao/dev/motion_controller motion_controller
```

Otherwise, once the module has been built, move the compiled shared object file to the robot, and its location to /home/nao/naoqi/preferences/autoload.ini

Restart NaoQI:
```
nao restart
```

Running qicli info on the robot should now show a module called **MotionController**

If needed, kill the ALMotion:
```
qicli call ALMotion.exit
```

### Usage ###

To use the module, create an ALProxy for the module and call the move method, as seen below:
```
qi::AnyObject controller = session->service("MotionController");
controller.call<void>("move", vel_x, vel_y, vel_th );
```

Other methods are exposed:
```
controller.call<void>("set_linear_velocity", linear_vel );
controller.call<void>("set_angular_velocity", angular_vel );
controller.call<void>("set_acceleration", acceleration );
```

**Note:** For safety reasons, MotionController will disengage if ALMotion attempts to drive the robot. This means that behaviours such as body rotation will prevent MotionController from working correctly. These behaviours can be disabled manually by opening the power hatch at the back of the robot.


### Contact ###
Maintainer: Ferran Garcia (ferran.garcia@softbankrobotics.com)

### Disclaimer ###
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO
EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
USE OR OTHER DEALINGS IN THE SOFTWARE.
