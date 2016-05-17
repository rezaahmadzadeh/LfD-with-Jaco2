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
        cout << endl << endl;
        cout << "===================================================" << endl;
        cout << "=====  Set Gravity Parameters for Jaco2 arm   =====" << endl;
        cout << "===================================================" << endl;
        cout << "code: Reza Ahmadzadeh (IRIM, 2016)." << endl;
        cout << "new parameters has to be set inside the code for safety reasons." << endl << endl;

        bool setGravityVectorForPrentice;
        int result;
        int programResult = 0;
        int resultComm;
        int devicesCount;
        AngularPosition DataCommand;
        void * commandLayer_handle;     //Handle for the library's command layer.

        // ----- Function pointers to the functions we need -----
        int(*MyInitAPI)();
        int(*MyCloseAPI)();
        int(*MyGetAngularCommand)(AngularPosition &);
        int(*MySwitchTrajectoryTorque)(GENERALCONTROL_TYPE);
        int(*MySetGravityOptimalZParam)(float Command[GRAVITY_PARAM_SIZE]);
        int(*MySetGravityType)(GRAVITY_TYPE Type);
        int(*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
        int(*MySetActiveDevice)(KinovaDevice device);
        int(*MySetGravityVector)(float Command[3]);
        //int(*MyGetAngularPosition)(AngularPosition &);
        //int(*MyGetActuatorAcceleration)(AngularAcceleration &Response);
        //int(*MyGetAngularVelocity)(AngularPosition &Response);
        //int(*MyRunGravityZEstimationSequence)(ROBOT_TYPE type, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE]);
        //int(*MySetTorqueSafetyFactor)(float factor);
        //int(*MySendAngularTorqueCommand)(float Command[COMMAND_SIZE]);
        //int(*MySendCartesianForceCommand)(float Command[COMMAND_SIZE]);
        //int(*MySetGravityVector)(float Command[3]);
        //int(*MySetGravityPayload)(float Command[GRAVITY_PAYLOAD_SIZE]);
        //int(*MyGetAngularForceGravityFree)(AngularPosition &);
        //int(*MyGetCartesianForce)(CartesianPosition &);

        //We load the library (Under Windows, use the function LoadLibrary)
        commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);
        MyInitAPI = (int(*)()) dlsym(commandLayer_handle, "InitAPI");
        MyCloseAPI = (int(*)()) dlsym(commandLayer_handle, "CloseAPI");
        MyGetAngularCommand = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularCommand");
        MySwitchTrajectoryTorque = (int(*)(GENERALCONTROL_TYPE)) dlsym(commandLayer_handle, "SwitchTrajectoryTorque");
        MySetGravityOptimalZParam = (int(*)(float Command[GRAVITY_PARAM_SIZE])) dlsym(commandLayer_handle, "SetGravityOptimalZParam");
        MySetGravityType = (int(*)(GRAVITY_TYPE Type)) dlsym(commandLayer_handle, "SetGravityType");
        MyGetDevices = (int(*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle, "GetDevices");
        MySetActiveDevice = (int(*)(KinovaDevice devices)) dlsym(commandLayer_handle, "SetActiveDevice");
        MySetGravityVector = (int(*)(float Command[3])) dlsym(commandLayer_handle, "SetGravityVector");

        // MyGetAngularPosition = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularPosition");
        // MyGetActuatorAcceleration = (int(*)(AngularAcceleration &)) dlsym(commandLayer_handle, "GetActuatorAcceleration");
        // MyGetAngularVelocity = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularVelocity");
        // MyRunGravityZEstimationSequence = (int(*)(ROBOT_TYPE, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE])) dlsym(commandLayer_handle, "RunGravityZEstimationSequence");
        // MySetTorqueSafetyFactor = (int(*)(float)) dlsym(commandLayer_handle, "SetTorqueSafetyFactor");
        // MySendAngularTorqueCommand = (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "SendAngularTorqueCommand");
        // MySendCartesianForceCommand = (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "SendCartesianForceCommand");
        // MySetGravityVector = (int(*)(float Command[3])) dlsym(commandLayer_handle, "SetGravityVector");
        // MySetGravityPayload = (int(*)(float Command[GRAVITY_PAYLOAD_SIZE])) dlsym(commandLayer_handle, "SetGravityPayload");
        // MyGetAngularForceGravityFree = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularForceGravityFree");
        // MyGetCartesianForce = (int(*)(CartesianPosition &)) dlsym(commandLayer_handle, "GetCartesianForce");

        //Verify that all functions has been loaded correctly
        if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MyGetAngularCommand == NULL) ||
                (MySwitchTrajectoryTorque == NULL) || (MySetGravityOptimalZParam == NULL) || (MySetGravityType == NULL) || (MySetGravityVector == NULL))
        {
                cout << "* * *  ERROR: initialization failed!  * * *" << endl;
                programResult = 0;
        }
        else
        {
                setGravityVectorForPrentice = false;
                cout << "Initialization completed." << endl << endl;
                result = (*MyInitAPI)();

                KinovaDevice list[MAX_KINOVA_DEVICE];

                 devicesCount = MyGetDevices(list, result);
                 if (devicesCount == 0)
                 {
                     cout << "\n WARNING : The robot is off or is not in the loop!" << endl;
                     return 0;
                 }
                 else if (devicesCount > 1)
                 {
                     cout << "\n WARNING: There are multiple robots connected. This process is considered for a single robot." << endl;
                    return 0;
                 }

                 cout << "Found a robot on the USB bus (" << list[0].SerialNumber << ") (" << list[0].DeviceType << ")" << endl;

                 MySetActiveDevice(list[0]);  //Setting the current device as the active device.

                resultComm = MyGetAngularCommand(DataCommand);  // Get the angular command to test the communication with the robot

                // If the API is initialized and the communication with the robot is working
                if (result == 1 && resultComm == 1)
                {
                        // Make sure we are in position mode (and not in torque mode)
                        MySwitchTrajectoryTorque(POSITION);

                        /*
                        // Set the gravity mode to Manual input
                        MySetGravityType(MANUAL_INPUT);
                        // Get and print the Torques and Forces (with gravity removed)
                        AngularPosition TorqueFree;
                        MyGetAngularForceGravityFree(TorqueFree);
                        CartesianPosition CartForce;
                        MyGetCartesianForce(CartForce);
                        cout << "*********************************" << endl;
                        cout << "*** Manual mode gravity estimation" << endl;
                        cout << "Actuator 1   Torque : " << TorqueFree.Actuators.Actuator1 << "°" << "     Force X : " << CartForce.Coordinates.X << endl;
                        cout << "Actuator 2   Torque : " << TorqueFree.Actuators.Actuator2 << "°" << "     Force Y : " << CartForce.Coordinates.Y << endl;
                        cout << "Actuator 3   Torque : " << TorqueFree.Actuators.Actuator3 << "°" << "     Force Z : " << CartForce.Coordinates.Z << endl;
                        cout << "Actuator 4   Torque : " << TorqueFree.Actuators.Actuator4 << "°" << "     Torque X : " << CartForce.Coordinates.ThetaX << endl;
                        cout << "Actuator 5   Torque : " << TorqueFree.Actuators.Actuator5 << "°" << "     Torque Y : " << CartForce.Coordinates.ThetaY << endl;
                        cout << "Actuator 6   Torque : " << TorqueFree.Actuators.Actuator6 << "°" << "     Torque Z : " << CartForce.Coordinates.ThetaZ << endl << endl;
                        */


                        // -------------  Set the Optimal parameters obtained from the identification sequence -------------------------------------
                        // 1- values for Nimbus
                        //float OptimalParam[OPTIMAL_Z_PARAM_SIZE] = {1.30469, -0.021818, 0.00503935, -1.36279, 0.00707375, 0.718922, 0.000474846, 0.249909, -0.00164482, -0.0102302, -0.0970402, -0.10606, 0.569738, -0.0528163, -0.00276341, 0.0694895};
                        // 2- values for Prentice
                        //float OptimalParam[OPTIMAL_Z_PARAM_SIZE] = {1.29878,  0.0454845,-0.0182455, -1.37306, 0.00480272, 0.719564, 0.00581768,  0.253633, 0.00242839, -0.00456613, 0.37502, -0.366981, 0.250789, 0.0856243,    -0.0030033,  0.0488819};
                        // 3- older values (default)
                        // float OptimalParam[OPTIMAL_Z_PARAM_SIZE] = { 1.22781, 0.0550204, -0.0148855, -1.15, -0.00524289, 0.563342, 0.0013925, 0.182611, -0.00396236, -0.00237999, 0.288417, -0.224536, 0.0526025, -0.0335503, 0.0246604, -0.00237218 };
                        // 4- first gravity estimation for Prentice (5/5/2016)
                        // float OptimalParam[OPTIMAL_Z_PARAM_SIZE] = {1.30385,0.03884,-0.0177897,-1.37458,0.00733365,0.720785,0.00542076,0.254201,0.00159902,-0.00495904,0.370557,0.0191459,0.458593,0.0880957,-0.0664954,0.0377676};
                        // 5- second gravity estimation for Prentice (5/6/2016)
                        // float OptimalParam[OPTIMAL_Z_PARAM_SIZE] = {1.30429,0.0261865,-0.0139155,-1.37051,0.00576831,0.719486,0.00509437,0.252955,0.00231727,-0.00552679,0.444551,0.115209,0.358188,0.182018,-0.066636,0.0205258};
                        // 6- Martine gravity estimation for Prentice (5/5/2016)
                        // float OptimalParam[OPTIMAL_Z_PARAM_SIZE] = {1.18315,-0.00455447,-0.0090834,-1.11046,-0.000249209,0.554853,-0.000344073,0.181086,-0.00264444,-0.00278941,0.179827,-0.164505,0.0341622,-0.0273777,-0.0568545,-0.00272235};
                        // 7- after a good calibration (5/6/2016)
                        // float OptimalParam[OPTIMAL_Z_PARAM_SIZE] = { 1.2866,0.0493694,-0.026624,-1.36103,0.0107946,0.715808,0.00667519,0.251691,0.00104736,-0.00474472,0.4016,-0.436709,0.486683,0.0481139,-0.120568,0.00334937};
                        // 8- after short-cirsuit
                        //float OptimalParam[OPTIMAL_Z_PARAM_SIZE] = {1.28652,0.0430668,-0.0174641,-1.35257,0.00558113,0.705339,0.00475949,0.250731,0.00151768,-0.00571242,0.364516,-0.12309,0.423939,-0.0476983,-0.0529807,0.0819299};
                        float OptimalParam[OPTIMAL_Z_PARAM_SIZE] = {1.28761,0.041719,-0.0122856,-1.35148,0.0103189,0.707362,0.00734889,0.247521,0.000899133,-0.00504736,0.319657,-0.229033,0.414252,-0.0468793,-0.0562333,0.11089};

                        MySetGravityOptimalZParam(OptimalParam);
                        MySetGravityType(OPTIMAL);  // Set gravity type to optimal
                        //MySetGravityType(MANUAL_INPUT);  // Set gravity type to optimal
                        usleep(30000);
                        cout << "The parameters are set." << endl;

                        if (setGravityVectorForPrentice)
                        {
                            // Gravity vector in -Y
                            float GravityVector[3];
                            GravityVector[0] = 0;// -9.81;
                            GravityVector[1] = -9.81;// 0;
                            GravityVector[2] = 0;// 0;
                            // Set the gravity vector
                            MySetGravityVector(GravityVector);
                            cout << "The gravity vector was changed for Prentice configuration." << endl;
                            usleep(20000);
                        }

                        // ---------------------------------------------------------------------------------------------------------------------------

                        /*
                        // Get and print the Torques and Forces (with gravity removed)
                        MyGetAngularForceGravityFree(TorqueFree);
                        MyGetCartesianForce(CartForce);
                        cout << "*********************************" << endl;
                        cout << "*** Optimal mode gravity estimation" << endl;
                        cout << "Actuator 1   Torque : " << TorqueFree.Actuators.Actuator1 << "°" << "     Force X : " << CartForce.Coordinates.X << endl;
                        cout << "Actuator 2   Torque : " << TorqueFree.Actuators.Actuator2 << "°" << "     Force Y : " << CartForce.Coordinates.Y << endl;
                        cout << "Actuator 3   Torque : " << TorqueFree.Actuators.Actuator3 << "°" << "     Force Z : " << CartForce.Coordinates.Z << endl;
                        cout << "Actuator 4   Torque : " << TorqueFree.Actuators.Actuator4 << "°" << "     Torque X : " << CartForce.Coordinates.ThetaX << endl;
                        cout << "Actuator 5   Torque : " << TorqueFree.Actuators.Actuator5 << "°" << "     Torque Y : " << CartForce.Coordinates.ThetaY << endl;
                        cout << "Actuator 6   Torque : " << TorqueFree.Actuators.Actuator6 << "°" << "     Torque Z : " << CartForce.Coordinates.ThetaZ << endl << endl;
                        */
                }
                else
                {
                        cout << "WARNING: API initialization failed!" << endl;
                }
                cout << endl << "Program terminated successfully." << endl;
                result = (*MyCloseAPI)();
                programResult = 1;
        }
        dlclose(commandLayer_handle);
        return programResult;
}

