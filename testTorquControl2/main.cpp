#include <iostream>
#include <dlfcn.h> //Ubuntu
#include <KinovaTypes.h>
#include <Kinova.API.CommLayerUbuntu.h>
#include <Kinova.API.UsbCommandLayerUbuntu.h>
#include <unistd.h>
//Note that under windows, you may/will have to perform other #include
using namespace std;
int main()
{
        int result;
        int programResult = 0;
        //Handle for the library's command layer.
        void * commandLayer_handle;
        //Function pointers to the functions we need
        int(*MyInitAPI)();
        int(*MyCloseAPI)();
        int(*MyGetAngularCommand)(AngularPosition &);
        int(*MyGetAngularPosition)(AngularPosition &);
        int(*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
        int(*MySetActiveDevice)(KinovaDevice device);
        int(*MyGetActuatorAcceleration)(AngularAcceleration &Response);
        int(*MyGetAngularVelocity)(AngularPosition &Response);
        int(*MyMoveHome)();
        int(*MyRunGravityZEstimationSequence)(ROBOT_TYPE type, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE]);
        int(*MySwitchTrajectoryTorque)(GENERALCONTROL_TYPE);
        int(*MySetTorqueSafetyFactor)(float factor);
        int(*MySendAngularTorqueCommand)(float Command[COMMAND_SIZE]);
        int(*MySendCartesianForceCommand)(float Command[COMMAND_SIZE]);
        int(*MySetGravityVector)(float Command[3]);
        int(*MySetGravityPayload)(float Command[GRAVITY_PAYLOAD_SIZE]);
        int(*MySetGravityOptimalZParam)(float Command[GRAVITY_PARAM_SIZE]);
        int(*MySetGravityType)(GRAVITY_TYPE Type);
        int(*MySetTorqueVibrationController)(float value);
        int(*MyGetAngularForceGravityFree)(AngularPosition &);
        int(*MyGetCartesianForce)(CartesianPosition &);
        int(*MySetTorqueControlType)(TORQUECONTROL_TYPE type);
        int (*MySetFrameType)(int);
        int (*MySetTorqueRobotProtection)(int);
        int (*MySendBasicTrajectory)(TrajectoryPoint command);
        int(*MySetTorqueActuatorDamping)(float Command[COMMAND_SIZE]);
        //We load the library (Under Windows, use the function LoadLibrary)
        commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);
        MyInitAPI = (int(*)()) dlsym(commandLayer_handle, "InitAPI");
        MyCloseAPI = (int(*)()) dlsym(commandLayer_handle, "CloseAPI");
        MyGetAngularCommand = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularCommand");
        MyGetAngularPosition = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularPosition");
        MyGetDevices = (int(*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle, "GetDevices");
        MySetActiveDevice = (int(*)(KinovaDevice devices)) dlsym(commandLayer_handle, "SetActiveDevice");
        MyGetActuatorAcceleration = (int(*)(AngularAcceleration &)) dlsym(commandLayer_handle, "GetActuatorAcceleration");
        MyGetAngularVelocity = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularVelocity");
        MyRunGravityZEstimationSequence = (int(*)(ROBOT_TYPE, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE])) dlsym(commandLayer_handle, "RunGravityZEstimationSequence");
        MySwitchTrajectoryTorque = (int(*)(GENERALCONTROL_TYPE)) dlsym(commandLayer_handle, "SwitchTrajectoryTorque");
        MySetTorqueSafetyFactor = (int(*)(float)) dlsym(commandLayer_handle, "SetTorqueSafetyFactor");
        MySendAngularTorqueCommand = (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "SendAngularTorqueCommand");
        MySendCartesianForceCommand = (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "SendCartesianForceCommand");
        MySetGravityVector = (int(*)(float Command[3])) dlsym(commandLayer_handle, "SetGravityVector");
        MySetGravityPayload = (int(*)(float Command[GRAVITY_PAYLOAD_SIZE])) dlsym(commandLayer_handle, "SetGravityPayload");
        MySetGravityOptimalZParam = (int(*)(float Command[GRAVITY_PARAM_SIZE])) dlsym(commandLayer_handle, "SetGravityOptimalZParam");
        MySetGravityType = (int(*)(GRAVITY_TYPE Type)) dlsym(commandLayer_handle, "SetGravityType");
        MyGetAngularForceGravityFree = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularForceGravityFree");
        MyGetCartesianForce = (int(*)(CartesianPosition &)) dlsym(commandLayer_handle, "GetCartesianForce");
        MySetTorqueVibrationController = (int(*)(float)) dlsym(commandLayer_handle, "SetTorqueVibrationController");
        MySetTorqueControlType = (int(*)(TORQUECONTROL_TYPE)) dlsym(commandLayer_handle, "SetTorqueControlType");
        MyMoveHome = (int(*)()) dlsym(commandLayer_handle, "MoveHome");
        MySetFrameType = (int (*)(int)) dlsym(commandLayer_handle,"SetFrameType");
        MySetTorqueRobotProtection = (int (*)(int)) dlsym(commandLayer_handle,"SetTorqueRobotProtection");
        MySendBasicTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle,"SendBasicTrajectory");
        MySetTorqueActuatorDamping = (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "SetTorqueActuatorDamping");
        //Verify that all functions has been loaded correctly
        if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MyGetAngularCommand == NULL) || (MyGetAngularPosition == NULL)
                || (MySetActiveDevice == NULL) || (MyGetDevices == NULL) || (MyGetAngularVelocity == NULL))
        {
                cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << endl;
                programResult = 0;
        }
        else
        {
                cout << "I N I T I A L I Z A T I O N   C O M P L E T E D" << endl << endl;
                result = (*MyInitAPI)();
                int resultComm;
                AngularPosition DataCommand;
                TrajectoryPoint trajectoryPoint;
                // Get the angular command to test the communication with the robot
                resultComm = MyGetAngularCommand(DataCommand);
                cout << "Initialization's result :" << result << endl;
                cout << "Communication result :" << resultComm << endl;
                // If the API is initialized and the communication with the robot is working
                if (result == 1 && resultComm == 1)
                {


                        cout << "API initialization worked" << endl;
                        cout << "The robot will swich to torque control mode and move. Be cautious." << endl;
                        // Set to position mode
                        MySwitchTrajectoryTorque(POSITION);
                        // Move to home position
                        MyMoveHome();
                        // ----- testing direct torque -----
                        //MySetTorqueControlType(DIRECTTORQUE);     // Set the torque control type to Direct Torque Control
                        MySetTorqueControlType(DIRECTTORQUE);
                        MySetTorqueSafetyFactor(0.6);               // Set the safety factor to 0.6
                        result = MySetTorqueRobotProtection(1);
                        MySetTorqueVibrationController(0.5);        // Set the vibration controller to 0.5

                        // Set damping to 0.4
                        float ActuatorDamping[COMMAND_SIZE];
                        for (int i = 0; i < COMMAND_SIZE; i++)
                        {
                                ActuatorDamping[i] = 0;
                        }
                        ActuatorDamping[0] = 0.4;  // 0-2
                        ActuatorDamping[1] = 0.4;
                        ActuatorDamping[2] = 0.4;
                        ActuatorDamping[3] = 0.4;
                        ActuatorDamping[4] = 0.4;
                        ActuatorDamping[5] = 0.4;
                        MySetTorqueActuatorDamping(ActuatorDamping);

                        // Switch to torque control
                        // (Here we switch before sending torques. The switch is possible because the gravity torques are already taken into account.)

                        MySwitchTrajectoryTorque(TORQUE);
                        cout << "hehr" << endl;
                        float zeroForceCommand[COMMAND_SIZE];
                        for (int i = 0; i < COMMAND_SIZE; i++)
                        {
                                zeroForceCommand[i] = 0;
                        }
                        // Send the torque commands for 20 seconds
                        for (int i = 0; i < 20000; i++)
                        {
                            cout << ".";
                            MySendCartesianForceCommand(zeroForceCommand);
                            usleep(1000);
                        }
                        result = MySetTorqueRobotProtection(2);
                        result = MySetTorqueSafetyFactor(0.6);
                        // Set the damping back to zero
                        for (int i = 0; i < COMMAND_SIZE; i++)
                        {
                                ActuatorDamping[i] = 0;
                        }
                        MySetTorqueActuatorDamping(ActuatorDamping);
                        result = MySwitchTrajectoryTorque(POSITION);

                  }
                else
                {
                        cout << "API initialization failed" << endl;
                }
                cout << endl << "C L O S I N G   A P I" << endl;
                result = (*MyCloseAPI)();
                programResult = 1;
        }
        dlclose(commandLayer_handle);
        cout << "xxxx" << endl;
        return programResult;
}



/*


                    bool directTorqueFlag = true;
                    if (directTorqueFlag)
                    {
                    // ----- testing direct torque -----
                    result << MySetTorqueControlType(DIRECTTORQUE);
                    cout << result << endl;
                    result = MySetTorqueSafetyFactor(1.0);
                    cout << result << endl;
                    result = MySetTorqueVibrationController(0.5);
                    cout << result << endl;
                    result = MySetTorqueRobotProtection(1);
                    cout << result << endl;
                    MySwitchTrajectoryTorque(TORQUE);
                    cout << "hehr" << endl;
                    float zeroTorqueCommand[COMMAND_SIZE];
                    for (int i = 0; i < COMMAND_SIZE; i++)
                    {
                            zeroTorqueCommand[i] = 0;
                    }
                    // Send the torque commands for 20 seconds
                    for (int i = 0; i < 20000; i++)
                    {
                        cout << ".";
                        MySendAngularTorqueCommand(zeroTorqueCommand);
                        usleep(1000);
                    }
                    result = MySetTorqueRobotProtection(2);
                    result = MySetTorqueSafetyFactor(0.6);
                    result = MySwitchTrajectoryTorque(POSITION);
                    result = (*MyCloseAPI)();
                    dlclose(commandLayer_handle);
                    return 0;
                    }

*/
/*
// Initialize the torque commands
cout << "using angular torque command" << endl;
float TorqueCommand[COMMAND_SIZE];
for (int i = 0; i < COMMAND_SIZE; i++)
{
        TorqueCommand[i] = 0;
}
// Send the torque commands for 2 seconds
for (int i = 0; i < 200; i++)
{
        // Torque of 0.5Nm on joint 6
        //TorqueCommand[3] = 0.5;
        // Send the torques
        MySendAngularTorqueCommand(TorqueCommand);
        // Sleep 10 ms
        usleep(10000);
}

// Wait
usleep(2000000);
cout << "using Cartesian force command" << endl;
// Initialize the Cartesian force commands
float CartForceCommand[COMMAND_SIZE];
for (int i = 0; i < COMMAND_SIZE; i++)
{
        CartForceCommand[i] = 0;
}
// Send the force commands for 2 seconds
for (int i = 0; i < 200; i++)
{
        // Force command in Y direction of Fy = -2
        CartForceCommand[1] = -3;
        // Send the force command
        MySendCartesianForceCommand(CartForceCommand);
        // Sleep of 10 ms
        usleep(10000);
}
//Wait
usleep(2000000);
// Switch back to position
MySwitchTrajectoryTorque(POSITION);
*/
