#include <iostream>
#include <dlfcn.h> //Ubuntu
#include <KinovaTypes.h>
#include <Kinova.API.CommLayerUbuntu.h>
#include <Kinova.API.UsbCommandLayerUbuntu.h>
#include <unistd.h>
#include <string>
#include <math.h>
#include <stdio.h>
#include <fstream>
#include <termios.h>
#include <vector>
#include <fcntl.h>
#include <sstream>


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
                        trajectoryPoint.Position.Type = ANGULAR_POSITION; //226.287  202.525  97.8848  178.22  29.9561  75.1491  0.509246
                        trajectoryPoint.Position.Actuators.Actuator1 = -60;//-60;//-50;//-59;//-227;//-245; //312;
                        trajectoryPoint.Position.Actuators.Actuator2 = 180;//239;//180;//230;//205;//135; //164; //245;
                        trajectoryPoint.Position.Actuators.Actuator3 = 102;//112;//148;//139;//292;//300;//117;
                        trajectoryPoint.Position.Actuators.Actuator4 = 270;//314;//316;//160.6;//208;//-15;
                        trajectoryPoint.Position.Actuators.Actuator5 = 0;//480;//495;//273.6;//308;//61;
                        trajectoryPoint.Position.Actuators.Actuator6 = 90;//132;//118;//29;//0;//112     -59.7233	180.358	101.769	269.942
                        (*MySendBasicTrajectory)(trajectoryPoint);
                        usleep(5000000);

                        cout << "Done." << endl;
                        //result = (*MyCloseAPI)();
                        //return 0;
                        AngularPosition PositionReference;
                        MyGetAngularPosition(PositionReference);
                        cout << "Reference position: " << PositionReference.Actuators.Actuator1 << " ," <<  PositionReference.Actuators.Actuator2 << " ," <<  PositionReference.Actuators.Actuator3
                                 << " ," <<  PositionReference.Actuators.Actuator4 << " ," <<  PositionReference.Actuators.Actuator5 << " ," <<  PositionReference.Actuators.Actuator6 << endl;


                         //result = (*MyCloseAPI)();
                         //return 0;

                        cout << "Enter the filename to execute the trajectory from: " << endl;
                        string fileName;
                        getline(cin, fileName);
                        ifstream ifs;
                        ifs.open(fileName.c_str());
                        if (!ifs.is_open())
                        {
                            cout << "no such file! try again." << endl;
                            result = (*MyCloseAPI)();
                            return 0;
                        }
                        int count;
                        float j1,j2,j3,j4,j5,j6,x,y,z,tx,ty,tz;
                        float M1,M2,M3,M4,M5,M6,Fx,Fy,Fz,Mx,My,Mz;


                        ifs >> count >> j1 >> j2 >> j3 >> j4 >> j5 >> j6 >> x >> y >> z >> tx >> ty >> tz >> M1 >> M2 >> M3 >> M4 >> M5 >> M6 >> Fx >> Fy >> Fz >> Mx >> My >> Mz;
                        printf("First Desired Pose: [%f\t%f\t%f\t%f\t%f\t%f]\n", x,y,z,tx,ty,tz);
                        printf("First Desired Angle: [%f\t%f\t%f\t%f\t%f\t%f]\n", j1,j2,j3,j4,j5,j6);

                        std::string answer;
                        std::string yes = "y"; //.c_str();

                        result = MySwitchTrajectoryTorque(TORQUE);
                        getResult(strResult,result);
                        cout << "switched to torque mode: " << strResult << endl;

                        result = MySetTorqueSafetyFactor(0.6); //0.6
                        getResult(strResult,result);
                        cout << "safety factor set: " << strResult << endl;
                        result = MySetTorqueRobotProtection(1);
                        getResult(strResult,result);
                        cout << "Protection zones off: " << strResult << endl;
                        result = MySetTorqueVibrationController(0.5); //0.5
                        getResult(strResult,result);
                        cout << "vibration controller set: " << strResult << endl;

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
                        // Get the actual position
                        AngularPosition PositionActual;
                        // Initialize the torque command
                        float TorqueCommand[COMMAND_SIZE];
                        for (int i = 0; i < COMMAND_SIZE; i++){TorqueCommand[i] = 0;}

                        float Stiffness[COMMAND_SIZE];
                        for (int i = 0; i < COMMAND_SIZE; i++){Stiffness[i] = 0.5;} // initialize the stiffness to 0.5
                        Stiffness[0] = 0.4;
                        Stiffness[1] = 0.4;
                        Stiffness[2] = 0.4;
                        Stiffness[3] = 0.4;
                        Stiffness[4] = 0.4;
                        Stiffness[5] = 0.4;

                        cout << "Start executing the trajectory?";
                        getline(cin, answer);
                        if (answer.compare(yes) == 0 )
                        {

                            while (ifs.is_open() &&  ifs >> count >> j1 >> j2 >> j3 >> j4 >> j5 >> j6 >> x >> y >> z >> tx >> ty >> tz >> M1 >> M2 >> M3 >> M4 >> M5 >> M6 >> Fx >> Fy >> Fz >> Mx >> My >> Mz)
                            {

                                MyGetAngularPosition(PositionActual);
                                TorqueCommand[0] = 0; TorqueCommand[1] = 0; TorqueCommand[2] = 0; TorqueCommand[3] = 0; TorqueCommand[4] = 0; TorqueCommand[5] = 0;
                                cout << "Reference:" << j1 << "," << j2 << "," << j3 << "," << j4 << "," << j5 << "," << j6 << endl;
                                cout << "error:" << PositionActual.Actuators.Actuator1 - j1 << "," << PositionActual.Actuators.Actuator2 - j2 << "," << PositionActual.Actuators.Actuator3 - j3 << "," << PositionActual.Actuators.Actuator4 - j4 << "," << PositionActual.Actuators.Actuator5 - j5 << "," << PositionActual.Actuators.Actuator6 - j6 << endl;

                                TorqueCommand[0] = -Stiffness[0] * ((PositionActual.Actuators.Actuator1) - (j1));
                                TorqueCommand[1] = -Stiffness[1] * ((PositionActual.Actuators.Actuator2) - (j2));
                                TorqueCommand[2] = -Stiffness[2] * ((PositionActual.Actuators.Actuator3) - (j3));
                                TorqueCommand[3] = -Stiffness[3] * ((PositionActual.Actuators.Actuator4) - (j4));
                                TorqueCommand[4] = -Stiffness[4] * ((PositionActual.Actuators.Actuator5) - (j5));
                                TorqueCommand[5] = -Stiffness[5] * ((PositionActual.Actuators.Actuator6) - (j6));
                                MySendAngularTorqueCommand(TorqueCommand);
                                usleep(10000);

                            }

                        }
                        else
                        {
                            ifs.close();
                            result = (*MyCloseAPI)();
                            return 0;
                        }

                        result = MySetTorqueRobotProtection(2);
                        getResult(strResult,result);
                        cout << endl << "Protection zones on: " << strResult << endl;

                        result = MySwitchTrajectoryTorque(POSITION);
                        getResult(strResult,result);
                        cout << "switch to position mode: " << strResult << endl;
                        if (result != 1)
                        {
                            cout << "could not switch back to position mode." << endl;

                        }

                        // Set the damping back to zero
                        for (int i = 0; i < COMMAND_SIZE; i++){ActuatorDamping[i] = 0;}

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





/* // DEMO 1
TrajectoryPoint trajectoryPoint;
trajectoryPoint.InitStruct();
trajectoryPoint.Position.Type = ANGULAR_POSITION;
trajectoryPoint.Position.Actuators.Actuator1 = 312;
trajectoryPoint.Position.Actuators.Actuator2 = 245;
trajectoryPoint.Position.Actuators.Actuator3 = 117;
trajectoryPoint.Position.Actuators.Actuator4 = -15;
trajectoryPoint.Position.Actuators.Actuator5 = 61;
trajectoryPoint.Position.Actuators.Actuator6 = 112;
(*MySendBasicTrajectory)(trajectoryPoint);
usleep(5000000);

trajectoryPoint.Position.Actuators.Actuator2 = 225;
trajectoryPoint.Position.Actuators.Actuator3 = 119;
trajectoryPoint.Position.Actuators.Actuator4 = -21.5;
trajectoryPoint.Position.Actuators.Actuator5 = 83;
trajectoryPoint.Position.Actuators.Actuator6 = 103;

(*MySendBasicTrajectory)(trajectoryPoint);
usleep(5000000);


trajectoryPoint.Position.Actuators.Actuator1 = 401;
(*MySendBasicTrajectory)(trajectoryPoint);
usleep(5000000);

trajectoryPoint.Position.Actuators.Actuator2 = 245;
trajectoryPoint.Position.Actuators.Actuator3 = 117;
trajectoryPoint.Position.Actuators.Actuator4 = -15;
trajectoryPoint.Position.Actuators.Actuator5 = 61;
trajectoryPoint.Position.Actuators.Actuator6 = 112;
(*MySendBasicTrajectory)(trajectoryPoint);
usleep(5000000);

// DEMO2
TrajectoryPoint trajectoryPoint;
trajectoryPoint.InitStruct();
trajectoryPoint.Position.Type = ANGULAR_POSITION;
trajectoryPoint.Position.Actuators.Actuator1 = 312;
trajectoryPoint.Position.Actuators.Actuator2 = 245;
trajectoryPoint.Position.Actuators.Actuator3 = 117;
trajectoryPoint.Position.Actuators.Actuator4 = -15;
trajectoryPoint.Position.Actuators.Actuator5 = 61;
trajectoryPoint.Position.Actuators.Actuator6 = 112;
(*MySendBasicTrajectory)(trajectoryPoint);
usleep(5000000);

trajectoryPoint.Position.Actuators.Actuator1 = 307;
trajectoryPoint.Position.Actuators.Actuator2 = 207;
trajectoryPoint.Position.Actuators.Actuator3 = 102;
trajectoryPoint.Position.Actuators.Actuator4 = -21.5;
trajectoryPoint.Position.Actuators.Actuator5 = 85;
trajectoryPoint.Position.Actuators.Actuator6 = 103;

(*MySendBasicTrajectory)(trajectoryPoint);
usleep(5000000);


trajectoryPoint.Position.Actuators.Actuator1 = 407;
(*MySendBasicTrajectory)(trajectoryPoint);
usleep(5000000);

trajectoryPoint.Position.Actuators.Actuator1 = 401;
trajectoryPoint.Position.Actuators.Actuator2 = 245;
trajectoryPoint.Position.Actuators.Actuator3 = 117;
trajectoryPoint.Position.Actuators.Actuator4 = -15;
trajectoryPoint.Position.Actuators.Actuator5 = 61;
trajectoryPoint.Position.Actuators.Actuator6 = 112;
(*MySendBasicTrajectory)(trajectoryPoint);
usleep(5000000);


//DEMO3
TrajectoryPoint trajectoryPoint;
trajectoryPoint.InitStruct();
trajectoryPoint.Position.Type = ANGULAR_POSITION;
trajectoryPoint.Position.Actuators.Actuator1 = 312;
trajectoryPoint.Position.Actuators.Actuator2 = 245;
trajectoryPoint.Position.Actuators.Actuator3 = 117;
trajectoryPoint.Position.Actuators.Actuator4 = -15;
trajectoryPoint.Position.Actuators.Actuator5 = 61;
trajectoryPoint.Position.Actuators.Actuator6 = 112;
(*MySendBasicTrajectory)(trajectoryPoint);
usleep(5000000);

trajectoryPoint.Position.Actuators.Actuator1 = 326;
trajectoryPoint.Position.Actuators.Actuator2 = 226;
trajectoryPoint.Position.Actuators.Actuator3 = 133;
trajectoryPoint.Position.Actuators.Actuator4 = -21.5;
trajectoryPoint.Position.Actuators.Actuator5 = 85;
trajectoryPoint.Position.Actuators.Actuator6 = 103;

(*MySendBasicTrajectory)(trajectoryPoint);
usleep(5000000);


trajectoryPoint.Position.Actuators.Actuator1 = 386;
(*MySendBasicTrajectory)(trajectoryPoint);
usleep(5000000);

trajectoryPoint.Position.Actuators.Actuator1 = 401;
trajectoryPoint.Position.Actuators.Actuator2 = 245;
trajectoryPoint.Position.Actuators.Actuator3 = 117;
trajectoryPoint.Position.Actuators.Actuator4 = -15;
trajectoryPoint.Position.Actuators.Actuator5 = 61;
trajectoryPoint.Position.Actuators.Actuator6 = 112;
(*MySendBasicTrajectory)(trajectoryPoint);
usleep(5000000);


//DEMO4
TrajectoryPoint trajectoryPoint;
trajectoryPoint.InitStruct();
trajectoryPoint.Position.Type = ANGULAR_POSITION;
trajectoryPoint.Position.Actuators.Actuator1 = 312;
trajectoryPoint.Position.Actuators.Actuator2 = 245;
trajectoryPoint.Position.Actuators.Actuator3 = 117;
trajectoryPoint.Position.Actuators.Actuator4 = -15;
trajectoryPoint.Position.Actuators.Actuator5 = 61;
trajectoryPoint.Position.Actuators.Actuator6 = 112;
(*MySendBasicTrajectory)(trajectoryPoint);
usleep(5000000);

trajectoryPoint.Position.Actuators.Actuator1 = 323;
trajectoryPoint.Position.Actuators.Actuator2 = 206;
trajectoryPoint.Position.Actuators.Actuator3 = 134;
trajectoryPoint.Position.Actuators.Actuator4 = -45;
trajectoryPoint.Position.Actuators.Actuator5 = 120;
trajectoryPoint.Position.Actuators.Actuator6 = 103;

(*MySendBasicTrajectory)(trajectoryPoint);
usleep(5000000);


trajectoryPoint.Position.Actuators.Actuator1 = 390;
(*MySendBasicTrajectory)(trajectoryPoint);
usleep(5000000);

trajectoryPoint.Position.Actuators.Actuator1 = 401;
trajectoryPoint.Position.Actuators.Actuator2 = 245;
trajectoryPoint.Position.Actuators.Actuator3 = 117;
trajectoryPoint.Position.Actuators.Actuator4 = -15;
trajectoryPoint.Position.Actuators.Actuator5 = 61;
trajectoryPoint.Position.Actuators.Actuator6 = 112;
(*MySendBasicTrajectory)(trajectoryPoint);
usleep(5000000);
*/

