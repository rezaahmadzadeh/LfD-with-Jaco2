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
        int(*MyRunGravityZEstimationSequence)(ROBOT_TYPE type, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE]);
        int(*MySwitchTrajectoryTorque)(GENERALCONTROL_TYPE);
        int(*MySetTorqueSafetyFactor)(float factor);
        int(*MySendAngularTorqueCommand)(float Command[COMMAND_SIZE]);
        int(*MySendCartesianForceCommand)(float Command[COMMAND_SIZE]);
        int(*MySetGravityVector)(float Command[3]);
        int(*MySetGravityPayload)(float Command[GRAVITY_PAYLOAD_SIZE]);
        int(*MySetGravityOptimalZParam)(float Command[GRAVITY_PARAM_SIZE]);
        int(*MySetGravityType)(GRAVITY_TYPE Type);
        int(*MyGetAngularForceGravityFree)(AngularPosition &);
        int(*MyGetCartesianForce)(CartesianPosition &);
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
        //Verify that all functions has been loaded correctly
        if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MyGetAngularCommand == NULL) ||
                (MySwitchTrajectoryTorque == NULL))
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
                // Get the angular command to test the communication with the robot
                resultComm = MyGetAngularCommand(DataCommand);
                cout << "Initialization's result :" << result << endl;
                cout << "Communication result :" << resultComm << endl;
                // If the API is initialized and the communication with the robot is working
                if (result == 1 && resultComm == 1)
                {
                        cout << "API initialization worked" << endl;
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
                        // Set the Optimal parameters obtained from the identification sequence
                        float OptimalParam[OPTIMAL_Z_PARAM_SIZE] = {1.30469, -0.021818, 0.00503935, -1.36279, 0.00707375, 0.718922, 0.000474846, 0.249909, -0.00164482, -0.0102302, -0.0970402, -0.10606, 0.569738, -0.0528163, -0.00276341, 0.0694895};
                        // float OptimalParam[OPTIMAL_Z_PARAM_SIZE] = { 1.22781, 0.0550204, -0.0148855, -1.15, -0.00524289, 0.563342, 0.0013925, 0.182611, -0.00396236, -0.00237999, 0.288417, -0.224536, 0.0526025, -0.0335503, 0.0246604, -0.00237218 };
                        MySetGravityOptimalZParam(OptimalParam);
                        // Set gravity type to optimal
                        MySetGravityType(OPTIMAL);
                        usleep(30000);
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
                        cout << "API initialization failed" << endl;
                }
                cout << endl << "C L O S I N G   A P I" << endl;
                result = (*MyCloseAPI)();
                programResult = 1;
        }
        dlclose(commandLayer_handle);
        return programResult;
}

