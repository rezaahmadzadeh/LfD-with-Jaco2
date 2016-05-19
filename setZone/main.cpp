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
        cout << "======================================================" << endl;
        cout << "=====  Set Safe Zone for Jaco2 arm on Prentice   =====" << endl;
        cout << "======================================================" << endl;
        cout << "code: Reza Ahmadzadeh (IRIM, 2016)." << endl;
        cout << "Note that " << endl << endl;

        //bool setGravityVectorForPrentice;
        int result;
        int programResult = 0;
        int resultComm;
        int devicesCount;
        AngularPosition DataCommand;
        void * commandLayer_handle;     //Handle for the library's command layer.

        // ----- Function pointers to the functions we need -----
        int(*MyInitAPI)();
        int(*MyCloseAPI)();
        int(*MySetProtectionZone)(ZoneList);
        int(*MyGetProtectionZone)(ZoneList &Response);
        int(*MyGetAngularCommand)(AngularPosition &);
        //int(*MySwitchTrajectoryTorque)(GENERALCONTROL_TYPE);
        //int(*MySetGravityOptimalZParam)(float Command[GRAVITY_PARAM_SIZE]);
        //int(*MySetGravityType)(GRAVITY_TYPE Type);
        int(*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
        int(*MySetActiveDevice)(KinovaDevice device);
        //int(*MySetGravityVector)(float Command[3]);
        //int(*MySetGravityPayload)(float Command[GRAVITY_PAYLOAD_SIZE]);
        //int(*MyGetAngularForceGravityFree)(AngularPosition &);
        //int(*MyGetCartesianForce)(CartesianPosition &);
        //int(*MyGetAngularPosition)(AngularPosition &);
        //int(*MyGetActuatorAcceleration)(AngularAcceleration &Response);
        //int(*MyGetAngularVelocity)(AngularPosition &Response);
        //int(*MyRunGravityZEstimationSequence)(ROBOT_TYPE type, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE]);
        //int(*MySetTorqueSafetyFactor)(float factor);
        //int(*MySendAngularTorqueCommand)(float Command[COMMAND_SIZE]);
        //int(*MySendCartesianForceCommand)(float Command[COMMAND_SIZE]);
        //int(*MySetGravityVector)(float Command[3]);
        //int(*MySetGravityPayload)(float Command[GRAVITY_PAYLOAD_SIZE]);

        //We load the library (Under Windows, use the function LoadLibrary)
        commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);
        MyInitAPI = (int(*)()) dlsym(commandLayer_handle, "InitAPI");
        MyCloseAPI = (int(*)()) dlsym(commandLayer_handle, "CloseAPI");
        MySetProtectionZone = (int(*)(ZoneList)) dlsym(commandLayer_handle, "SetProtectionZone");
        MyGetProtectionZone = (int (*)(ZoneList &)) dlsym(commandLayer_handle,"GetProtectionZone");
        MyGetAngularCommand = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularCommand");
        //MySwitchTrajectoryTorque = (int(*)(GENERALCONTROL_TYPE)) dlsym(commandLayer_handle, "SwitchTrajectoryTorque");
        //MySetGravityOptimalZParam = (int(*)(float Command[GRAVITY_PARAM_SIZE])) dlsym(commandLayer_handle, "SetGravityOptimalZParam");
        //MySetGravityType = (int(*)(GRAVITY_TYPE Type)) dlsym(commandLayer_handle, "SetGravityType");
        MyGetDevices = (int(*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle, "GetDevices");
        MySetActiveDevice = (int(*)(KinovaDevice devices)) dlsym(commandLayer_handle, "SetActiveDevice");
        //MySetGravityVector = (int(*)(float Command[3])) dlsym(commandLayer_handle, "SetGravityVector");
        //MySetGravityPayload = (int(*)(float Command[GRAVITY_PAYLOAD_SIZE])) dlsym(commandLayer_handle, "SetGravityPayload");
        //MyGetAngularForceGravityFree = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularForceGravityFree");
        //MyGetCartesianForce = (int(*)(CartesianPosition &)) dlsym(commandLayer_handle, "GetCartesianForce");
        // MyGetAngularPosition = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularPosition");
        // MyGetActuatorAcceleration = (int(*)(AngularAcceleration &)) dlsym(commandLayer_handle, "GetActuatorAcceleration");
        // MyGetAngularVelocity = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularVelocity");
        // MyRunGravityZEstimationSequence = (int(*)(ROBOT_TYPE, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE])) dlsym(commandLayer_handle, "RunGravityZEstimationSequence");
        // MySetTorqueSafetyFactor = (int(*)(float)) dlsym(commandLayer_handle, "SetTorqueSafetyFactor");
        // MySendAngularTorqueCommand = (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "SendAngularTorqueCommand");
        // MySendCartesianForceCommand = (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "SendCartesianForceCommand");
        // MySetGravityVector = (int(*)(float Command[3])) dlsym(commandLayer_handle, "SetGravityVector");
        // MySetGravityPayload = (int(*)(float Command[GRAVITY_PAYLOAD_SIZE])) dlsym(commandLayer_handle, "SetGravityPayload");

        //Verify that all functions has been loaded correctly
        if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MySetProtectionZone == NULL) || (MyGetAngularCommand == NULL))
        {
                cout << "***  ERROR: initialization failed!  ***" << endl;
                programResult = 0;
        }
        else
        {
                //setGravityVectorForPrentice = true;
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

                    ZoneList zone;
                    result = (*MyGetProtectionZone)(zone);
                    cout << "result: " << result << endl;
                    cout << "Active zone count: " << zone.NbZones << endl;


                    //zone.NbZones = 1;

                    //zone.Zones[0].zoneShape.shapeType = PrismTriangularBase_Z;
                    //zone.Zones[0].zoneShape.Points[0].





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

