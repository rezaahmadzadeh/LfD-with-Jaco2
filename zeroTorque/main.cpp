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
        cout << "=====  Activating Zero Torque Gravity Mode     =====" << endl;
        cout << "======================================================" << endl;
        cout << "code: Reza Ahmadzadeh (IRIM, 2016)." << endl;

        int result;
        int programResult = 0;
        bool setGravityVectorForPrentice;
        void * commandLayer_handle;
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
                setGravityVectorForPrentice = false;
                AngularPosition DataCommand;
                resultComm = MyGetAngularCommand(DataCommand);
                string strResult = "Failed";
                getResult(strResult,result);
                cout << "Initialization's result :" << strResult << endl;
                getResult(strResult,resultComm);
                cout << "Communication result :" << strResult << endl;
                cout << "current angular data: " << DataCommand.Actuators.Actuator1 << ", " << DataCommand.Actuators.Actuator2 <<
                        ", " << DataCommand.Actuators.Actuator3 << ", " << DataCommand.Actuators.Actuator4 << ", " << DataCommand.Actuators.Actuator5 << ", " << DataCommand.Actuators.Actuator6 << endl;
                if (result == 1 && resultComm == 1)
                {
                        cout << "API initialized: OK" << endl;
                        result = MySwitchTrajectoryTorque(POSITION);
                        getResult(strResult,result);
                        cout << "switched to position mode: " << strResult << endl;

                        // 13- Sept 7 2016
                        float OptimalParam[OPTIMAL_Z_PARAM_SIZE] = {1.31011,0.0311761,-0.011198,-1.35773,0.00976884,0.710675,0.00755337,0.247009,0.00066524,-0.00634159,-0.134054,0.0219935,0.271503,0.0447736,-0.0639545,-0.00111969};

                        result = MySetGravityOptimalZParam(OptimalParam);
                        getResult(strResult,result);
                        cout << "gravity parameteres are set:" << strResult << endl;
                        MySetGravityType(OPTIMAL);  // // MySetGravityType(MANUAL_INPUT);
                        usleep(30000);
                        cout << "OPTIMAL gravity is set." << endl;

                        if (setGravityVectorForPrentice)
                        {
                            // Gravity vector in -Y
                            float GravityVector[3];
                            GravityVector[0] = 0;// -9.81;  //####################### CHECK ####################################
                            GravityVector[1] = -9.81;// 0;
                            GravityVector[2] = 0;// 0;
                            // Set the gravity vector
                            MySetGravityVector(GravityVector);
                            cout << "The gravity vector was changed for Prentice configuration to [" << GravityVector[0] << "," << GravityVector[1] << "," << GravityVector[2] << "]." << endl;
                            usleep(20000);

                            // Gravity payload initialization (1.8 kg for the normal hand 0.85 for Robotiq hand)
                            float GravityPayload[4];
                            GravityPayload[0] = 0.85;   //####################### CHECK ####################################
                            GravityPayload[1] = 0;
                            GravityPayload[2] = 0;
                            GravityPayload[3] = 0;
                            // Set payload params
                            MySetGravityPayload(GravityPayload);
                            cout << "The gravity payload was set for Robotiq gripper." << endl;
                            usleep(20000);

                        }


                        // Set the safety factor to 0.6
                        result = MySetTorqueSafetyFactor(1.0); //0.6
                        getResult(strResult,result);
                        cout << "safety factor set: " << strResult << endl;
                        result = MySetTorqueRobotProtection(1);
                        getResult(strResult,result);
                        cout << "Protection zones off: " << strResult << endl;
                        result = MySetTorqueVibrationController(0.5); //0.5
                        getResult(strResult,result);
                        cout << "vibration controller set: " << strResult << endl;
                        result = MySwitchTrajectoryTorque(TORQUE);
                        getResult(strResult,result);
                        cout << "switched to torque mode: " << strResult << endl;

                        float ActuatorDamping[COMMAND_SIZE];
                        for (int i = 0; i < COMMAND_SIZE; i++){ActuatorDamping[i] = 0;}
                        ActuatorDamping[0] = 0.4;
                        ActuatorDamping[1] = 0.4;
                        ActuatorDamping[2] = 0.4;
                        ActuatorDamping[3] = 0.4;
                        ActuatorDamping[4] = 0.4;
                        ActuatorDamping[5] = 0.4;
                        result = MySetTorqueActuatorDamping(ActuatorDamping);
                        getResult(strResult,result);
                        cout << "Dampings are set: " << strResult << endl;
                        float TorqueCommand[COMMAND_SIZE];
                        for (int i = 0; i < COMMAND_SIZE; i++){TorqueCommand[i] = 0;}


                        cout << "Simulate the spring-damper system while following a trajectory in torque mode." << endl;
                        for (int i = 0; i < 2000; i++)
                        {
                            cout << ".";
                            TorqueCommand[0] = 0; TorqueCommand[1] = 0; TorqueCommand[2] = 0; TorqueCommand[3] = 0; TorqueCommand[4] = 0; TorqueCommand[5] = 0;
                            MySendAngularTorqueCommand(TorqueCommand);
                            usleep(10000);
                        }

                        // Switch back to position control
                        result = MySetTorqueRobotProtection(2);
                        getResult(strResult,result);
                        cout << endl << "Protection zones on: " << strResult << endl;
                        result = MySetTorqueSafetyFactor(0.6);
                        getResult(strResult,result);
                        cout << "safety factor set: " << strResult << endl;

                        result = MySwitchTrajectoryTorque(POSITION);
                        getResult(strResult,result);
                        cout << "switch to position mode: " << strResult << endl;
                        if (result != 1){cout << "could not switch back to position mode." << endl;}

                        for (int i = 0; i < COMMAND_SIZE; i++) {ActuatorDamping[i] = 0;}
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
