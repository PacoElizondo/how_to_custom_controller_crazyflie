# How to write a Custom Controller for crazyflie drones

## 0. Dependencies
from: https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/building-and-flashing/build/
#### Ubuntu
```
sudo apt-get install make gcc-arm-none-eabi
```
#### macOS

```
brew install gcc-arm-embedded
```

No toolchain for windows, wsl required:
```
wsl --install
```

## 1. Clone firmware repo

```
git clone https://github.com/bitcraze/crazyflie-firmware.git
```

## 2. Make an app

#### 2.1 Copy an example 
Here you have some options:

####  Copy Hello World and write the controller from scratch:
```
cp -r examples/app_hello_world examples/my_controller 
cd examples/my_controller/
mv src/hello_world.c src/my_controller.c #or whatever name you´d like
```

edit ./src/Kbuild and add the following line:
```
obj-y += my_controller.o
```

edit ./app-config:
```
CONFIG_APP_ENABLE=y
CONFIG_APP_PRIORITY=1
CONFIG_APP_STACKSIZE=350
CONFIG_CONTROLLER_OOT=y
```

#### Copy out-of-tree controller as a base to write yours:
 
```
cp -r examples/app_out_of_tree_controller examples/my_controller 
cd examples/my_controller/
mv src/hello_world.c src/my_controller.c #or whatever name you´d like
```
No need to edit Kbuild or other files

#### Copy either option and replace the my_controller.c with another out-of-tree controller:
I would recommend this one as it has a more comprehensive structure, since everything should be in the same file, contrary to the in-built controllers provided by bit-craze. 

The following controller would work as such and it will be explained further below:
[4. Controller code structure](#4-controller-code-structure)

#### 2.2 Test
Build:
```
make -j8
```

Flash (with crazy radio module connected):
```
make cload
```


## 3. Python API Control selection

The Python API allows you to send messages, for example, a trajectory given by a set of waypoints, and receive messages from the log and custom log variables written on your control code. 
Here you can select the controller you want to use to control the drone for:

```
def set_controller(scf):
    cf = scf.cf
	# The number 5 refers to the out_of_tree controller, 0-4 refer to in-built
	# controllers
    cf.param.set_value('stabilizer.controller', '5') 
    time.sleep(2.1)
```

If you would like an example python API code that shows how to set up the drone and access to your custom logs (to see control state/inputs) see:
[example_API_rutine.py](example_API_rutine.py)

## 4. Controller code structure

The custom controller app has a specific required structure, here is a simplified explanation:

```
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "controller.h"

#define DEBUG_MODULE "MYCONTROLLER"
#include "debug.h"


// This needs to be here because of the app framework
void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

  while(1) {
    vTaskDelay(M2T(2000));
  }
}

// Other includes

// Global variable definitions

static bool isInit;

// Custom functions to be called during the control loop (for example, 
// quaternion algebra functions that are not included in the standard
// math.h or cuda)


void controllerOutOfTreeInit() {
  if (isInit) {
    return;
  }
  
#define UPDATE_RATE RATE_100_HZ // or whatever rate you consider
static const float DELTA_T = 0.01f;
static float t = 0;
  
void controllerOutOfTree(control_t *control,
                          const setpoint_t *setpoint,
                          const sensorData_t *sensors,
                          const state_t *state,
                          const stabilizerStep_t stabilizerStep) {
  

	// define local variables
	
	 if (RATE_DO_EXECUTE(UPDATE_RATE, stabilizerStep)) {

		// Get state e.g.
		// Current attitude
	    struct quat orientation = mkquat(
		  state->attitudeQuaternion.x,
	      state->attitudeQuaternion.y,
	      state->attitudeQuaternion.z,
	      state->attitudeQuaternion.w
	    );
	    
	    t += DELTA_T;
		
		// Compute errors and inputs; whatever you do on your control loop
		
		// Set inputs
		// Control Input
		if (setpoint->mode.z == modeDisable) {
		  control->thrustSi = 0.0f;
		  control->torque[0] =  0.0f;
		  control->torque[1] =  0.0f;
		  control->torque[2] =  0.0f;
		} else {
		  // control the body torques
		  control->thrustSi = control_thrust;
		  control->torqueX  = control_torque.x;
		  control->torqueY  = control_torque.y;
		  control->torqueZ  = control_torque.z;
		}  

		control->controlMode = controlModeForceTorque;
		
	}

bool controllerOutOfTreeTest(){
  return true;
}


// Log variables to access from the API, for example

LOG_GROUP_START(adaptive_control)
/**
 * @brief Thrust
 */
LOG_ADD(LOG_FLOAT, thrust, &control_thrust)
/**
 * @brief Trans kp x
 */
LOG_ADD(LOG_FLOAT, torque_x, &control_torque.x)
/**
 * @brief Torque y
 */
LOG_ADD(LOG_FLOAT, torque_y, &control_torque.y)
/**
 * @brief Torque z
 */
LOG_ADD(LOG_FLOAT, torque_z, &control_torque.z)
LOG_GROUP_STOP(adaptive_control)

```

This is a simplification of [controller_example.c](controller_example.c)

## bitcraze Resources

https://www.bitcraze.io/documentation/tutorials/getting-started-with-development/

https://www.bitcraze.io/documentation/tutorials/getting-started-with-development/#modify-the-source-code

https://www.bitcraze.io/2023/02/adding-an-estimator-or-controller/




