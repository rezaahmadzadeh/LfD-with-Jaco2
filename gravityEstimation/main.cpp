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
        cout << "========================================================" << endl;
        cout << "=====  Estimate Gravity Parameters for Jaco2 arm   =====" << endl;
        cout << "========================================================" << endl;
        cout << "code: Reza Ahmadzadeh (IRIM, 2016)." << endl;
        cout << "WARNING: Read the documents before running this code!" << endl << endl;

        int result;
        int programResult = 0;
        int devicesCount;
        //Handle for the library's command layer.
        void * commandLayer_handle;
        //Function pointers to the functions we need
        int(*MyInitAPI)();
        int(*MyCloseAPI)();
        int(*MyGetAngularCommand)(AngularPosition &);
        int(*MyRunGravityZEstimationSequence)(ROBOT_TYPE type, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE]);
        int(*MySwitchTrajectoryTorque)(GENERALCONTROL_TYPE);
        int(*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
        int(*MySetActiveDevice)(KinovaDevice device);
        //int(*MyGetAngularPosition)(AngularPosition &);
        //int(*MyGetActuatorAcceleration)(AngularAcceleration &Response);
        //int(*MyGetAngularVelocity)(AngularPosition &Response);
        //int(*MySetTorqueSafetyFactor)(float factor);
        //int(*MySendAngularTorqueCommand)(float Command[COMMAND_SIZE]);
        //int(*MySendCartesianForceCommand)(float Command[COMMAND_SIZE]);
        //int(*MySetGravityVector)(float Command[3]);
        //int(*MySetGravityPayload)(float Command[GRAVITY_PAYLOAD_SIZE]);
        //int(*MySetGravityOptimalZParam)(float Command[GRAVITY_PARAM_SIZE]);
        //int(*MySetGravityType)(GRAVITY_TYPE Type);
        //int(*MyGetAngularForceGravityFree)(AngularPosition &);
        //int(*MyGetCartesianForce)(CartesianPosition &);

        //We load the library (Under Windows, use the function LoadLibrary)
        commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);
        MyInitAPI = (int(*)()) dlsym(commandLayer_handle, "InitAPI");
        MyCloseAPI = (int(*)()) dlsym(commandLayer_handle, "CloseAPI");
        MyGetAngularCommand = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularCommand");
        MyRunGravityZEstimationSequence = (int(*)(ROBOT_TYPE, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE])) dlsym(commandLayer_handle, "RunGravityZEstimationSequence");
        MySwitchTrajectoryTorque = (int(*)(GENERALCONTROL_TYPE)) dlsym(commandLayer_handle, "SwitchTrajectoryTorque");
        MyGetDevices = (int(*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle, "GetDevices");
        MySetActiveDevice = (int(*)(KinovaDevice devices)) dlsym(commandLayer_handle, "SetActiveDevice");
        //MyGetAngularPosition = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularPosition");
        //MyGetActuatorAcceleration = (int(*)(AngularAcceleration &)) dlsym(commandLayer_handle, "GetActuatorAcceleration");
        //MyGetAngularVelocity = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularVelocity");
        //MySetTorqueSafetyFactor = (int(*)(float)) dlsym(commandLayer_handle, "SetTorqueSafetyFactor");
        //MySendAngularTorqueCommand = (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "SendAngularTorqueCommand");
        //MySendCartesianForceCommand = (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "SendCartesianForceCommand");
        //MySetGravityVector = (int(*)(float Command[3])) dlsym(commandLayer_handle, "SetGravityVector");
        //MySetGravityPayload = (int(*)(float Command[GRAVITY_PAYLOAD_SIZE])) dlsym(commandLayer_handle, "SetGravityPayload");
        //MySetGravityOptimalZParam = (int(*)(float Command[GRAVITY_PARAM_SIZE])) dlsym(commandLayer_handle, "SetGravityOptimalZParam");
        //MySetGravityType = (int(*)(GRAVITY_TYPE Type)) dlsym(commandLayer_handle, "SetGravityType");
        //MyGetAngularForceGravityFree = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularForceGravityFree");
        //MyGetCartesianForce = (int(*)(CartesianPosition &)) dlsym(commandLayer_handle, "GetCartesianForce");
        //Verify that all functions has been loaded correctly

        if ( (MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MyGetAngularCommand == NULL) || (MyRunGravityZEstimationSequence == NULL) )
        {
                cout << "* * *  Error during initialization!  * * *" << endl;
                programResult = 0;
        }
        else
        {
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

                int resultComm;
                MySwitchTrajectoryTorque(POSITION);
                cout << "set the robot in position trajectory mode." << endl;
                AngularPosition DataCommand;
                // Get the angular command to test the communication with the robot
                resultComm = MyGetAngularCommand(DataCommand);
                cout << "Communication result :" << resultComm << endl;
                // If the API is initialized and the communication with the robot is working
                if (result == 1 && resultComm == 1)
                {
                        // Choose robot type
                        ROBOT_TYPE type = JACOV2_6DOF_SERVICE; //  //MICO_6DOF_SERVICE;   JACOV2_6DOF_SERVICE = 3, JACOV2_6DOF_ASSISTIVE = 6
                        double OptimalzParam[OPTIMAL_Z_PARAM_SIZE];
                        // Run identification sequence
                        // CAUTION READ THE FUNCTION DOCUMENTATION BEFORE
                        MyRunGravityZEstimationSequence(type, OptimalzParam);
                }
                cout << endl << "Estimation process has finished! Set the parameters using the command {setparam}." << endl;
                result = (*MyCloseAPI)();
                programResult = 1;
        }
        dlclose(commandLayer_handle);
        return programResult;
}

