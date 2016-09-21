#include <iostream>
#include <dlfcn.h> //Ubuntu
#include <KinovaTypes.h>
#include <Kinova.API.CommLayerUbuntu.h>
#include <Kinova.API.UsbCommandLayerUbuntu.h>
#include <unistd.h>

using namespace std;
int main()
{
        cout << endl << endl;
        cout << "======================================================" << endl;
        cout << "=====  Simulating A Spring Damper with Jaco2     =====" << endl;
        cout << "======================================================" << endl;
        cout << "code: Reza Ahmadzadeh (IRIM, 2016)." << endl;

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
        int(*MyMoveHome)();
        int(*MySwitchTrajectoryTorque)(GENERALCONTROL_TYPE);
        int(*MySetTorqueSafetyFactor)(float factor);
        int(*MySendAngularTorqueCommand)(float Command[COMMAND_SIZE]);
        int(*MySendCartesianForceCommand)(float Command[COMMAND_SIZE]);
        int(*MySetTorqueVibrationController)(float value);
        int(*MySetTorqueControlType)(TORQUECONTROL_TYPE type);
        int(*MySetTorqueActuatorDamping)(float Command[COMMAND_SIZE]);
        int (*MySetTorqueRobotProtection)(int);
        //We load the library (Under Windows, use the function LoadLibrary)
        commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);
        MyInitAPI = (int(*)()) dlsym(commandLayer_handle, "InitAPI");
        MyCloseAPI = (int(*)()) dlsym(commandLayer_handle, "CloseAPI");
        MyGetAngularCommand = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularCommand");
        MyGetAngularPosition = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularPosition");
        MyGetDevices = (int(*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle, "GetDevices");
        MySetActiveDevice = (int(*)(KinovaDevice devices)) dlsym(commandLayer_handle, "SetActiveDevice");
        MySwitchTrajectoryTorque = (int(*)(GENERALCONTROL_TYPE)) dlsym(commandLayer_handle, "SwitchTrajectoryTorque");
        MySetTorqueSafetyFactor = (int(*)(float)) dlsym(commandLayer_handle, "SetTorqueSafetyFactor");
        MySendAngularTorqueCommand = (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "SendAngularTorqueCommand");
        MySendCartesianForceCommand = (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "SendCartesianForceCommand");
        MySetTorqueVibrationController = (int(*)(float)) dlsym(commandLayer_handle, "SetTorqueVibrationController");
        MySetTorqueControlType = (int(*)(TORQUECONTROL_TYPE)) dlsym(commandLayer_handle, "SetTorqueControlType");
        MyMoveHome = (int(*)()) dlsym(commandLayer_handle, "MoveHome");
        MySetTorqueActuatorDamping = (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "SetTorqueActuatorDamping");
        MySetTorqueRobotProtection = (int (*)(int)) dlsym(commandLayer_handle,"SetTorqueRobotProtection");

        //Verify that all functions has been loaded correctly
        if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MyGetAngularCommand == NULL) || (MyGetAngularPosition == NULL)
                || (MySetActiveDevice == NULL) || (MyGetDevices == NULL))
        {
                cout << "***  Initialization Failed!  ***" << endl;
                programResult = 0;
        }
        else
        {
                result = (*MyInitAPI)();

                int resultComm;
                AngularPosition DataCommand;
                resultComm = MyGetAngularCommand(DataCommand);  // Get the angular command to test the communication with the robot

                cout << "Initialization's result :" << result << endl;
                cout << "Communication result :" << resultComm << endl;
                cout << "current angular data: " << DataCommand.Actuators.Actuator1 << ", " << DataCommand.Actuators.Actuator2 <<
                        ", " << DataCommand.Actuators.Actuator3 << ", " << DataCommand.Actuators.Actuator4 << ", " << DataCommand.Actuators.Actuator5 << ", " << DataCommand.Actuators.Actuator6 << endl;
                // If the API is initialized and the communication with the robot is working
                if (result == 1 && resultComm == 1)
                {
                        cout << "API initialized." << endl;
                        AngularPosition PositionReference;
                        MyGetAngularPosition(PositionReference);
                        cout << "Reference position: " << PositionReference.Actuators.Actuator1 << " ," <<  PositionReference.Actuators.Actuator2 << " ," <<  PositionReference.Actuators.Actuator3
                                 << " ," <<  PositionReference.Actuators.Actuator4 << " ," <<  PositionReference.Actuators.Actuator5 << " ," <<  PositionReference.Actuators.Actuator6 << endl;

                        result = MySwitchTrajectoryTorque(TORQUE);
                        cout << "switched to torque mode: " << result << endl;


                        result = MySetTorqueSafetyFactor(0.6);  // Set the safety factor to 0.6
                        cout << "safety factor set: " << result << endl;

                        //result = MySetTorqueRobotProtection(1);
                        //cout << "Protection zones off: " << result << endl;

                        result = MySetTorqueVibrationController(0.5);   // Set the Vibration controller to 0.5
                        cout << "vibration controller set: " << result << endl;

                        float ActuatorDamping[COMMAND_SIZE];
                        for (int i = 0; i < COMMAND_SIZE; i++)
                        {
                                ActuatorDamping[i] = 0;
                        }
                        ActuatorDamping[0] = 0.2;   // Set damping to 0.4
                        ActuatorDamping[1] = 0.2;
                        ActuatorDamping[2] = 0.2;
                        ActuatorDamping[3] = 0.2;
                        ActuatorDamping[4] = 0.2;
                        ActuatorDamping[5] = 0.2;
                        result = MySetTorqueActuatorDamping(ActuatorDamping);
                        cout << "Dampings are set: " << result << endl;

                        AngularPosition PositionActual;

                        float TorqueCommand[COMMAND_SIZE];
                        for (int i = 0; i < COMMAND_SIZE; i++)
                        {
                                TorqueCommand[i] = 0;   // Initialize the torque command
                        }

                        float Stiffness = 0.4;  // Set the stiffness to 0.5 (Caution: if the stiffness is high and damping is too low, the device may become unstable. Turn off the robot in case of any problem.)

                        cout << "Simulate the spring-damper system for 30 secondes." << endl;
                        for (int i = 0; i < 3000; i++)
                        {
                                MyGetAngularPosition(PositionActual);   // Get the actual position
                                // Compute the torque to send to the robot (stiffness part. The damping part is sent directly through "MySetTorqueActuatorDamping").
                                TorqueCommand[0] = 0; TorqueCommand[1] = 0; TorqueCommand[2] = 0; TorqueCommand[3] = 0; TorqueCommand[4] = 0; TorqueCommand[5] = 0;
                                TorqueCommand[0] = -Stiffness * ((PositionActual.Actuators.Actuator1) - (PositionReference.Actuators.Actuator1));
                                TorqueCommand[1] = -Stiffness * ((PositionActual.Actuators.Actuator2) - (PositionReference.Actuators.Actuator2));
                                TorqueCommand[2] = -Stiffness * ((PositionActual.Actuators.Actuator3) - (PositionReference.Actuators.Actuator3));
                                TorqueCommand[3] = -Stiffness * ((PositionActual.Actuators.Actuator4) - (PositionReference.Actuators.Actuator4));
                                TorqueCommand[4] = -Stiffness * ((PositionActual.Actuators.Actuator5) - (PositionReference.Actuators.Actuator5));
                                TorqueCommand[5] = -Stiffness * ((PositionActual.Actuators.Actuator6) - (PositionReference.Actuators.Actuator6));

                                MySendAngularTorqueCommand(TorqueCommand);  // Send the torque command

                                usleep(10000);  // Wait 10 ms
                        }

                        //result = MySetTorqueRobotProtection(2);
                        //cout << "Protection zones on: " << result << endl;

                        result = MySwitchTrajectoryTorque(POSITION);
                        cout << "switch to position mode: " << result << endl;
                        if (result != 1)
                        {
                            cout << "could not switch back to position mode." << endl;

                        }


                        for (int i = 0; i < COMMAND_SIZE; i++)
                        {
                                ActuatorDamping[i] = 0; // Set the damping back to zero
                        }

                        result = MySetTorqueActuatorDamping(ActuatorDamping);
                        cout << "set damping back to normal: " << result << endl;

                }
                else
                {
                        cout << "API initialization failed" << endl;
                }
                cout << endl << "Done!" << endl;
                result = (*MyCloseAPI)();
                programResult = 1;
        }
        dlclose(commandLayer_handle);
        return programResult;
}
