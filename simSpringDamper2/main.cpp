#include <iostream>
#include <dlfcn.h> //Ubuntu
#include <KinovaTypes.h>
#include <Kinova.API.CommLayerUbuntu.h>
#include <Kinova.API.UsbCommandLayerUbuntu.h>
#include <unistd.h>
#include <string>
#include <math.h>
//Note that under windows, you may/will have to perform other #include
using namespace std;



void getResult(string &executionResult,int result)
{
    if (result)
    {
        executionResult = "OK";
    }
    else
    {
        executionResult = "Failed";
    }
}






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
        int(*MySetTorqueActuatorDamping)(float Command[COMMAND_SIZE]);
        int (*MySetTorqueRobotProtection)(int);
        int (*MySendBasicTrajectory)(TrajectoryPoint angcommand);
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
        MySetTorqueActuatorDamping = (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "SetTorqueActuatorDamping");
        MySetTorqueRobotProtection = (int (*)(int)) dlsym(commandLayer_handle,"SetTorqueRobotProtection");
        MySendBasicTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle,"SendBasicTrajectory");

        //Verify that all functions has been loaded correctly
        if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MyGetAngularCommand == NULL) || (MyGetAngularPosition == NULL)
                || (MySetActiveDevice == NULL) || (MyGetDevices == NULL) || (MyGetAngularVelocity == NULL))
        {
                cout << "***  Initialization Failed!  ***" << endl;
                programResult = 0;
        }
        else
        {
                result = (*MyInitAPI)();
                int resultComm;
                AngularPosition DataCommand;
                // Get the angular command to test the communication with the robot
                resultComm = MyGetAngularCommand(DataCommand);
                string strResult = "Failed";
                getResult(strResult,result);
                cout << "Initialization's result :" << strResult << endl;
                getResult(strResult,resultComm);
                cout << "Communication result :" << strResult << endl;
                cout << "current angular data: " << DataCommand.Actuators.Actuator1 << ", " << DataCommand.Actuators.Actuator2 <<
                        ", " << DataCommand.Actuators.Actuator3 << ", " << DataCommand.Actuators.Actuator4 << ", " << DataCommand.Actuators.Actuator5 << ", " << DataCommand.Actuators.Actuator6 << endl;
                // If the API is initialized and the communication with the robot is working
                if (result == 1 && resultComm == 1)
                {
                        cout << "API initialized: OK" << endl;
                        result = MySwitchTrajectoryTorque(POSITION);
                        getResult(strResult,result);
                        cout << "switched to position mode: " << strResult << endl;
                        // Get the reference position

                        cout << "sending the robot to the inital pose ..." << endl;

                        TrajectoryPoint trajectoryPoint;
                        trajectoryPoint.InitStruct();
                        trajectoryPoint.Position.Type = ANGULAR_POSITION;
                        trajectoryPoint.Position.Actuators.Actuator1 = 235; // 225-215-235
                        trajectoryPoint.Position.Actuators.Actuator2 = 220; // 200-220-220
                        trajectoryPoint.Position.Actuators.Actuator3 = 110; // 100-110-110
                        trajectoryPoint.Position.Actuators.Actuator4 = 178;
                        trajectoryPoint.Position.Actuators.Actuator5 = 30;
                        trajectoryPoint.Position.Actuators.Actuator6 = 75;
                        (*MySendBasicTrajectory)(trajectoryPoint);
                        usleep(5000000);
                        cout << "Done." << endl;

                        AngularPosition PositionReference;
                        MyGetAngularPosition(PositionReference);
                        cout << "Reference position: " << PositionReference.Actuators.Actuator1 << " ," <<  PositionReference.Actuators.Actuator2 << " ," <<  PositionReference.Actuators.Actuator3
                                 << " ," <<  PositionReference.Actuators.Actuator4 << " ," <<  PositionReference.Actuators.Actuator5 << " ," <<  PositionReference.Actuators.Actuator6 << endl;


                        // Switch to torque control
                        //cout << "The robot will swich to torque control mode and move. Be cautious." << endl;
                        //result = MySetTorqueControlType(DIRECTTORQUE);
                        //getResult(strResult,result);
                        //cout << "Direct torque mode set:" << strResult << endl;

                        result = MySwitchTrajectoryTorque(TORQUE);
                        getResult(strResult,result);
                        cout << "switched to torque mode: " << strResult << endl;

                        /*
                        cout << "switched to torque mode: ";
                        for (int ii=0; ii<3; ii++)
                        {
                            result = MySwitchTrajectoryTorque(TORQUE);
                            getResult(strResult,result);
                            cout << strResult << " - ";
                            usleep(1000000);
                        }
                        cout << endl;
                        */

                        // Set the safety factor to 0.6
                        result = MySetTorqueSafetyFactor(0.6); //0.6
                        getResult(strResult,result);
                        cout << "safety factor set: " << strResult << endl;
                        // Set the Vibration controller to 0.5
                        result = MySetTorqueRobotProtection(1);
                        getResult(strResult,result);
                        cout << "Protection zones off: " << strResult << endl;
                        result = MySetTorqueVibrationController(0.5); //0.5
                        getResult(strResult,result);
                        cout << "vibration controller set: " << strResult << endl;
                        // Set damping to 0.4
                        float ActuatorDamping[COMMAND_SIZE];
                        for (int i = 0; i < COMMAND_SIZE; i++)
                        {
                                ActuatorDamping[i] = 0;
                        }
                        ActuatorDamping[0] = 0.4;
                        ActuatorDamping[1] = 0.4;
                        ActuatorDamping[2] = 0.4;
                        ActuatorDamping[3] = 0.4;
                        ActuatorDamping[4] = 0.4;
                        ActuatorDamping[5] = 0.4;
                        result = MySetTorqueActuatorDamping(ActuatorDamping);
                        getResult(strResult,result);
                        cout << "Dampings are set: " << strResult << endl;
                        // Get the actual position
                        AngularPosition PositionActual;
                        // Initialize the torque command
                        float TorqueCommand[COMMAND_SIZE];
                        for (int i = 0; i < COMMAND_SIZE; i++)
                        {
                                TorqueCommand[i] = 0;
                        }

                        // Set the stiffness to 0.5 (Caution: if the stiffness is high and damping is too low, the device may become unstable. Turn off the robot in case of any problem.)

                        //float Stiffness = 0.5;


                        float Stiffness[COMMAND_SIZE];
                        for (int i = 0; i < COMMAND_SIZE; i++)
                        {
                                Stiffness[i] = 0.5; // initialize the stiffness to 0.5
                        }
                        Stiffness[0] = 0.4;
                        Stiffness[1] = 0.4;
                        Stiffness[2] = 0.4;
                        Stiffness[3] = 0.4;
                        Stiffness[4] = 0.2;
                        Stiffness[5] = 0.2;





                        // Simulate the spring-damper system for 30 secondes
                        cout << "Simulate the spring-damper system while following a trajectory in torque mode." << endl;
                        for (int i = 0; i < 1500; i++)
                        {
                            cout << ".";
                            // Get the actual position
                            MyGetAngularPosition(PositionActual);
                            //PositionActual.Actuators.Actuator1 += 0.1f;
                            // Compute the torque to send to the robot (stiffness part. The damping part is sent directly through "MySetTorqueActuatorDamping").
                            TorqueCommand[0] = 0; TorqueCommand[1] = 0; TorqueCommand[2] = 0; TorqueCommand[3] = 0; TorqueCommand[4] = 0; TorqueCommand[5] = 0;
                            TorqueCommand[0] = -Stiffness[0] * ((PositionActual.Actuators.Actuator1) - (PositionReference.Actuators.Actuator1));
                            TorqueCommand[1] = -Stiffness[1] * ((PositionActual.Actuators.Actuator2) - (PositionReference.Actuators.Actuator2));
                            TorqueCommand[2] = -Stiffness[2] * ((PositionActual.Actuators.Actuator3) - (PositionReference.Actuators.Actuator3));
                            TorqueCommand[3] = -Stiffness[3] * ((PositionActual.Actuators.Actuator4) - (PositionReference.Actuators.Actuator4));
                            TorqueCommand[4] = -Stiffness[4] * ((PositionActual.Actuators.Actuator5) - (PositionReference.Actuators.Actuator5));
                            TorqueCommand[5] = -Stiffness[5] * ((PositionActual.Actuators.Actuator6) - (PositionReference.Actuators.Actuator6));
                            // Send the torque command
                            MySendAngularTorqueCommand(TorqueCommand);
                            // Wait 10 ms
                            usleep(10000);
                            PositionReference.Actuators.Actuator1 += 0.1f;
                            PositionReference.Actuators.Actuator2 += sin(PositionReference.Actuators.Actuator1);
                        }

                        /*
                        TrajectoryPoint trajectoryPoint;
                        trajectoryPoint.InitStruct();
                        trajectoryPoint.Position.Type = ANGULAR_POSITION;
                        MyGetAngularPosition(PositionActual);
                        trajectoryPoint.Position.Actuators.Actuator1 = PositionActual.Actuators.Actuator1;
                        trajectoryPoint.Position.Actuators.Actuator2 = PositionActual.Actuators.Actuator2;
                        trajectoryPoint.Position.Actuators.Actuator3 = PositionActual.Actuators.Actuator3;
                        trajectoryPoint.Position.Actuators.Actuator4 = PositionActual.Actuators.Actuator4;
                        trajectoryPoint.Position.Actuators.Actuator5 = PositionActual.Actuators.Actuator5;
                        trajectoryPoint.Position.Actuators.Actuator6 = PositionActual.Actuators.Actuator6;
                        for (int i = 0; i < 300; i++)
                        {
                            MySwitchTrajectoryTorque(POSITION);
                            //We prepare the virtual joystick command that will be sent to the robotic arm.
                            trajectoryPoint.Position.Actuators.Actuator1 += 0.1f;
                            trajectoryPoint.Position.Actuators.Actuator2 += 0.1f;
                            //trajectoryPoint.Position.Actuators.Actuator3 = PositionActual.Actuators.Actuator3;
                            //trajectoryPoint.Position.Actuators.Actuator4 = PositionActual.Actuators.Actuator4;
                            //trajectoryPoint.Position.Actuators.Actuator5 = PositionActual.Actuators.Actuator5;
                            //trajectoryPoint.Position.Actuators.Actuator6 = PositionActual.Actuators.Actuator6;
                            (*MySendBasicTrajectory)(trajectoryPoint);
                            usleep(100);


                            MySetTorqueControlType(DIRECTTORQUE);
                            MySetTorqueSafetyFactor(0.6);
                            MySetTorqueRobotProtection(1);
                            MySetTorqueVibrationController(0.5);
                            MySwitchTrajectoryTorque(TORQUE);
                            for (int i = 0; i < COMMAND_SIZE; i++)
                            {
                                    ActuatorDamping[i] = 0;
                            }
                            ActuatorDamping[0] = 0.2; ActuatorDamping[1] = 0.2; ActuatorDamping[2] = 0.2; ActuatorDamping[3] = 0.2; ActuatorDamping[4] = 0.2; ActuatorDamping[5] = 0.2;
                            MySetTorqueActuatorDamping(ActuatorDamping);


                            // Get the actual position
                            MyGetAngularPosition(PositionActual);
                            // Compute the torque to send to the robot (stiffness part. The damping part is sent directly through "MySetTorqueActuatorDamping").
                            TorqueCommand[0] = 0; TorqueCommand[1] = 0; TorqueCommand[2] = 0; TorqueCommand[3] = 0; TorqueCommand[4] = 0; TorqueCommand[5] = 0;
                            TorqueCommand[0] = -Stiffness * ((PositionActual.Actuators.Actuator1) - (PositionReference.Actuators.Actuator1));
                            TorqueCommand[1] = -Stiffness * ((PositionActual.Actuators.Actuator2) - (PositionReference.Actuators.Actuator2));
                            TorqueCommand[2] = -Stiffness * ((PositionActual.Actuators.Actuator3) - (PositionReference.Actuators.Actuator3));
                            TorqueCommand[3] = -Stiffness * ((PositionActual.Actuators.Actuator4) - (PositionReference.Actuators.Actuator4));
                            TorqueCommand[4] = -Stiffness * ((PositionActual.Actuators.Actuator5) - (PositionReference.Actuators.Actuator5));
                            TorqueCommand[5] = -Stiffness * ((PositionActual.Actuators.Actuator6) - (PositionReference.Actuators.Actuator6));
                            // Send the torque command
                            MySendAngularTorqueCommand(TorqueCommand);
                                // Wait 10 ms
                                usleep(10000);
                        }

                         usleep(1000000);
                        */

                        // Switch back to position control
                        result = MySetTorqueRobotProtection(2);
                        getResult(strResult,result);
                        cout << endl << "Protection zones on: " << strResult << endl;
                        //result = MySetTorqueSafetyFactor(0.6);
                        //getResult(strResult,result);
                        //cout << "safety factor set: " << strResult << endl;

                        result = MySwitchTrajectoryTorque(POSITION);
                        getResult(strResult,result);
                        cout << "switch to position mode: " << strResult << endl;
                        if (result != 1)
                        {
                            cout << "could not switch back to position mode." << endl;

                        }

                        // Set the damping back to zero
                        for (int i = 0; i < COMMAND_SIZE; i++)
                        {
                                ActuatorDamping[i] = 0;
                        }

                        result = MySetTorqueActuatorDamping(ActuatorDamping);
                        getResult(strResult,result);
                        cout << "set damping back to normal: " << strResult << endl;

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
