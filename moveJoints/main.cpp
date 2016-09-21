/* -------------------------------------
 *  A Simple Trajectory Executer
 *
 *
 *
 * set singlePointTest for sending a single point to the module
 * set incrementalTest for sending points in an incremental loop
 * input the file including trajectory information recorded using the recorder module in the repo
 *
 *
 * <<< SINCE THERE IS NO OBSTACLE AVOIDANCE MONITORING THE ROBOT, USING THIS MODULE CAN BE DANGEREOUS >>>
 *
 *
 *
 * Reza Ahmadzadeh (IRIM, April-2016)
 *
 * -------------------------------------*/

#include <iostream>
#include <dlfcn.h>
#include <KinovaTypes.h>
#include <Kinova.API.CommLayerUbuntu.h>
#include <stdio.h>
#include <fstream>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <fcntl.h>
#include <string>
#include <sstream>
#include <unistd.h>  // for sleep
#include <ncurses.h> // for keyboard events

using namespace std;

int kbhit(void)
{
    int ch = getch();

    if (ch != ERR) {
        ungetch(ch);
        return 1;
    } else {
        return 0;
    }
}

// ---------------------------------------------------------------------------
// ------------------------------ MAIN ---------------------------------------
// ---------------------------------------------------------------------------
int main()
{
    cout << endl << endl;
    cout << "===================================================" << endl;
    cout << "=====  Move Joints V.0.1              =====" << endl;
    cout << "===================================================" << endl;
    cout << "code: Reza Ahmadzadeh (IRIM, 2016)." << endl;
    cout << "You can record a trajectory using the recorder provided in this repository." << endl << endl;

    int result;
    bool singlePointTest, incrementalTest, cartesianMode, firstCartesian, intensiveTest;
    CartesianPosition data, data1, data2;
    AngularPosition dataang;
    int devicesCount;


    // ====== important flags =============
    singlePointTest = false;
    incrementalTest = false;
    intensiveTest = false;       // plays a trajectory with different number of points to show the smoothness of the movement
    firstCartesian = false;     // go to the first point of the trajectory using cartesian info (false = use angular info)
    cartesianMode = false;       // run the trajectory using cartesian info
    // ====================================


    cout << "Executing trajectories using SendAdvanceTrajectory/SendBasicTrajectory" << endl;


    void * commandLayer_handle;
    int (*MyInitAPI)();
    int (*MyCloseAPI)();
    int (*MySendBasicTrajectory)(TrajectoryPoint command);
    int (*MyStartControlAPI)();
    int (*MyMoveHome)();
    int (*MyGetCartesianCommand)(CartesianPosition &);
    int (*MyGetCartesianPosition)(CartesianPosition &);
    int (*MyGetAngularPosition)(AngularPosition &);
    int (*MySendAdvanceTrajectory)(TrajectoryPoint command);
    int (*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
    int (*MySetActiveDevice)(KinovaDevice device);
    int (*MySetCartesianControl)();
    int (*MySetAngularControl)();

    commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);
    MyInitAPI = (int (*)()) dlsym(commandLayer_handle,"InitAPI");
    MyCloseAPI = (int (*)()) dlsym(commandLayer_handle,"CloseAPI");
    MySendBasicTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle,"SendBasicTrajectory");
    MyStartControlAPI = (int (*)()) dlsym(commandLayer_handle,"StartControlAPI");
    MyGetCartesianCommand = (int (*)(CartesianPosition &)) dlsym(commandLayer_handle,"GetCartesianCommand");
    MyGetCartesianPosition = (int (*)(CartesianPosition &)) dlsym(commandLayer_handle,"GetCartesianPosition");
    MyGetAngularPosition = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularPosition");
    MyMoveHome = (int (*)()) dlsym(commandLayer_handle,"MoveHome");
    MySendAdvanceTrajectory = (int(*)(TrajectoryPoint)) dlsym(commandLayer_handle,"SendAdvanceTrajectory");
    MyGetDevices = (int(*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle, "GetDevices");
    MySetActiveDevice = (int(*)(KinovaDevice devices)) dlsym(commandLayer_handle, "SetActiveDevice");
    MySetCartesianControl = (int (*)()) dlsym(commandLayer_handle,"SetCartesianControl");
    MySetAngularControl = (int (*)()) dlsym(commandLayer_handle,"SetAngularControl");

    if((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MySendBasicTrajectory == NULL) || (MyStartControlAPI == NULL) ||
    (MyMoveHome == NULL) || (MyGetCartesianCommand == NULL) || (MyGetCartesianPosition == NULL) || (MySendAdvanceTrajectory == NULL))
    {
        cout << "* * *  ERROR: initialization failed!  * * *"  << endl;
    }
    else
    {
        result = (*MyInitAPI)();
        cout << "Initialization's result :" << result << endl;

        KinovaDevice list[MAX_KINOVA_DEVICE];

        devicesCount = MyGetDevices(list, result);
        if (devicesCount == 0)
        {
             cout << "\n WARNING : The robot is off or is not in the loop!" << endl;
             result = (*MyCloseAPI)();
             return 0;
        }

        cout << "Found a robot on the USB bus (" << list[0].SerialNumber << ") (" << list[0].DeviceType << ")" << endl;

        MySetActiveDevice(list[0]);  //Setting the current device as the active device.
        //cout << "moving home..." << endl;
        //MyMoveHome();
        //usleep(3000000);
        //result = (*MyGetCartesianPosition)(data);
        //cout << " Home: [ " << data.Coordinates.X << "," << data.Coordinates.Y << "," << data.Coordinates.Z << " ]" << endl;



        // -----------------------------------------------------------------------------------------------------
        // --------------------------------------- MOVING TO THE FIRST POINT -----------------------------------
        // -----------------------------------------------------------------------------------------------------


        std::string answer;
        std::string yes = "y"; //.c_str();
        TrajectoryPoint trajectoryPoint;
        result = (*MySetAngularControl)();
        cout << "Angular control has been set." << endl;

        // ====================== Go to the first point using Joint info ====================
        cout << "The robot will move to the first angular point of the given trajectory file. Agree?";
        getline(cin, answer);
        if (answer.compare(yes) == 0 )
        {
            trajectoryPoint.InitStruct();
            trajectoryPoint.Position.Type = ANGULAR_POSITION;

            // Manual
             trajectoryPoint.Position.Actuators.Actuator1 = -60;//-60;//-60;//-50;//-50;//227.4;
             trajectoryPoint.Position.Actuators.Actuator2 = 186;//180;//239;//211;//
             trajectoryPoint.Position.Actuators.Actuator3 = 87;//112;//56;//112;//
             trajectoryPoint.Position.Actuators.Actuator4 = 270;
             trajectoryPoint.Position.Actuators.Actuator5 = 0;
             trajectoryPoint.Position.Actuators.Actuator6 = 90;
            (*MySendBasicTrajectory)(trajectoryPoint);
            cout << "command sent" << endl;
            usleep(5000000);
            cout << "completed." << endl;
        }
        else
        {
            result = (*MyCloseAPI)();
            return 0;
        }
        // -----------------------------------------------------------------------------------------------------
        // --------------------------------------- TRAJECTORY EXECUTION ----------------------------------------
        // -----------------------------------------------------------------------------------------------------


        cout << "Start executing the trajectory?";
        getline(cin, answer);
        if (answer.compare(yes) == 0 )
        {
            //usleep(1000000);
            for (int i=1; i<=120; i++)
            {
                trajectoryPoint.Position.Type = ANGULAR_POSITION;
                trajectoryPoint.Position.HandMode = POSITION_MODE;
                trajectoryPoint.Position.Actuators.Actuator1 += 1;
                //trajectoryPoint.Position.Actuators.Actuator2 = j2;
                //trajectoryPoint.Position.Actuators.Actuator3 = j3;
                //trajectoryPoint.Position.Actuators.Actuator4 = j4;
                //trajectoryPoint.Position.Actuators.Actuator5 = j5;
                //trajectoryPoint.Position.Actuators.Actuator6 = j6;
                (*MySendBasicTrajectory)(trajectoryPoint);
                usleep(20000);
                //cout << count << "--> " << trajectoryPoint.Position.Actuators.Actuator1 << "-" << trajectoryPoint.Position.Actuators.Actuator2 << "-" << trajectoryPoint.Position.Actuators.Actuator3 << "-"
                 //       << trajectoryPoint.Position.Actuators.Actuator4 << "-" << trajectoryPoint.Position.Actuators.Actuator5 << "-"  << trajectoryPoint.Position.Actuators.Actuator6 << endl;
            }
            cout << "\n\n Trajectory Executed Successfully!" << endl;
        }
        else
        {
            cout << "aborted.";
        }

        result = (*MyCloseAPI)();
        cout << endl << "Closing API and the file!" << endl;
        cout << endl << "Done!" << endl;
    }
    return 0;
}



