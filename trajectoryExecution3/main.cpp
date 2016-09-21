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
    cout << "=====  Trajectory Executor V.0.3              =====" << endl;
    cout << "===================================================" << endl;
    cout << "code: Reza Ahmadzadeh (IRIM, 2016)." << endl;
    cout << "**** use only with trajectories recorded with recorder2" << endl;
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
        // --------------------------------------- READING THE FILE---------------------------------------------
        // -----------------------------------------------------------------------------------------------------
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


        // -----------------------------------------------------------------------------------------------------
        // --------------------------------------- MOVING TO THE FIRST POINT -----------------------------------
        // -----------------------------------------------------------------------------------------------------
        TrajectoryPoint trajectoryPoint;

        // get the first point from the trajectory
        ifs >> count >> j1 >> j2 >> j3 >> j4 >> j5 >> j6 >> x >> y >> z >> tx >> ty >> tz >> M1 >> M2 >> M3 >> M4 >> M5 >> M6 >> Fx >> Fy >> Fz >> Mx >> My >> Mz;
        printf("First Desired Pose: [%f\t%f\t%f\t%f\t%f\t%f]\n", x,y,z,tx,ty,tz);
        printf("First Desired Angle: [%f\t%f\t%f\t%f\t%f\t%f]\n", j1,j2,j3,j4,j5,j6);

        std::string answer;
        std::string yes = "y"; //.c_str();

        if (firstCartesian)
        {
            result = (*MySetCartesianControl)();
            cout << "Cartesian control has been set." << result << endl;

            // ====================== Go to the first point using Cartesian info ====================
            cout << "The robot will move to the first Cartesian pose of the given trajectory file. Agree?";
            getline(cin, answer);
            if (answer.compare(yes) == 0 )
            {
                trajectoryPoint.InitStruct();
                trajectoryPoint.Position.Type = CARTESIAN_POSITION;
                trajectoryPoint.Position.CartesianPosition.X = x;
                trajectoryPoint.Position.CartesianPosition.Y = y;
                trajectoryPoint.Position.CartesianPosition.Z = z;
                trajectoryPoint.Position.CartesianPosition.ThetaX = tx;
                trajectoryPoint.Position.CartesianPosition.ThetaY = ty;
                trajectoryPoint.Position.CartesianPosition.ThetaZ = tz;
                (*MySendBasicTrajectory)(trajectoryPoint);
                usleep(10000000);
                cout << "checking for the first point success..." << endl;
                result = (*MyGetCartesianPosition)(data1);
                result = (*MyGetCartesianCommand)(data2);
                cout << " Error: [ " << data1.Coordinates.X - data2.Coordinates.X << "," << data1.Coordinates.Y - data2.Coordinates.Y << "," << data1.Coordinates.Z - data2.Coordinates.Z << " ]" << endl;
            }
            else
            {
                ifs.close();
                result = (*MyCloseAPI)();
                return 0;
            }
        }
        else
        {
            // make sure the robot is in cartesian control mode
            result = (*MySetAngularControl)();
            cout << "Angular control has been set." << endl;

            // ====================== Go to the first point using Joint info ====================
            cout << "The robot will move to the first angular point of the given trajectory file. Agree?";
            getline(cin, answer);
            if (answer.compare(yes) == 0 )
            {
                trajectoryPoint.InitStruct();
                trajectoryPoint.Position.Type = ANGULAR_POSITION;
                // read from file
                trajectoryPoint.Position.Actuators.Actuator1 = j1;
                trajectoryPoint.Position.Actuators.Actuator2 = j2;
                trajectoryPoint.Position.Actuators.Actuator3 = j3;
                trajectoryPoint.Position.Actuators.Actuator4 = j4;
                trajectoryPoint.Position.Actuators.Actuator5 = j5;
                trajectoryPoint.Position.Actuators.Actuator6 = j6;

                // Manual
                // trajectoryPoint.Position.Actuators.Actuator1 = 227.4;
                // trajectoryPoint.Position.Actuators.Actuator2 = 198.89;
                // trajectoryPoint.Position.Actuators.Actuator3 = 140.07;
                // trajectoryPoint.Position.Actuators.Actuator4 = -72.08;
                // trajectoryPoint.Position.Actuators.Actuator5 = 37.43;
                // trajectoryPoint.Position.Actuators.Actuator6 = 43.52;
                (*MySendBasicTrajectory)(trajectoryPoint);
                cout << "command sent" << endl;
                usleep(5000000);
                cout << "completed." << endl;
            }
            else
            {
                ifs.close();
                result = (*MyCloseAPI)();
                return 0;
            }
        }

        // -----------------------------------------------------------------------------------------------------
        // --------------------------------------- TRAJECTORY EXECUTION ----------------------------------------
        // -----------------------------------------------------------------------------------------------------
        if (cartesianMode)
        {
            result = (*MySetCartesianControl)();
            cout << "Cartesian control has been set." << result << endl;

            cout << "Start executing the trajectory?";
            getline(cin, answer);
            if (answer.compare(yes) == 0 )
            {

                trajectoryPoint.InitStruct();
                trajectoryPoint.Limitations.accelerationParameter1 = 0.9f;  //not implemented
                trajectoryPoint.Limitations.accelerationParameter2 = 0.9f;  //not implemented
                trajectoryPoint.Limitations.accelerationParameter3 = 0.9f;  //not implemented
                trajectoryPoint.Limitations.forceParameter1 = 0.0f;         //not implemented
                trajectoryPoint.Limitations.forceParameter2 = 0.0f;         //not implemented
                trajectoryPoint.Limitations.forceParameter3 = 0.0f;         //not implemented
                trajectoryPoint.Limitations.speedParameter1 = 0.5f;         // limit the translational velocity to 8cm/s
                trajectoryPoint.Limitations.speedParameter2 = 0.6f;         // limit the rotational velocity to 0.6 RAD/s
                trajectoryPoint.Limitations.speedParameter3 = 0.5f;         // limit the translational velocity to 8cm/s
                trajectoryPoint.LimitationsActive = true;
                trajectoryPoint.Position.Type = CARTESIAN_POSITION;
                trajectoryPoint.Position.Actuators.Actuator1 = 0.0f;
                trajectoryPoint.Position.Actuators.Actuator2 = 0.0f;
                trajectoryPoint.Position.Actuators.Actuator3 = 0.0f;
                trajectoryPoint.Position.Actuators.Actuator4 = 0.0f;
                trajectoryPoint.Position.Actuators.Actuator5 = 0.0f;
                trajectoryPoint.Position.Actuators.Actuator6 = 0.0f;
                trajectoryPoint.Position.Delay = 0.0f;
                trajectoryPoint.Position.HandMode = POSITION_MODE;

                while (ifs.is_open() &&  ifs >> count >> j1 >> j2 >> j3 >> j4 >> j5 >> j6 >> x >> y >> z >> tx >> ty >> tz >> M1 >> M2 >> M3 >> M4 >> M5 >> M6 >> Fx >> Fy >> Fz >> Mx >> My >> Mz)
                {
                    //trajectoryPoint.Position.Type = CARTESIAN_POSITION;
                    //trajectoryPoint.Position.HandMode = POSITION_MODE;
                    //trajectoryPoint.Limitations.speedParameter2 = 20;
                    trajectoryPoint.LimitationsActive = true;
                    trajectoryPoint.Position.CartesianPosition.X = x;
                    trajectoryPoint.Position.CartesianPosition.Y = y;
                    trajectoryPoint.Position.CartesianPosition.Z = z;
                    trajectoryPoint.Position.CartesianPosition.ThetaX = tx;
                    trajectoryPoint.Position.CartesianPosition.ThetaY = ty;
                    trajectoryPoint.Position.CartesianPosition.ThetaZ = tz;
                    (*MySendAdvanceTrajectory)(trajectoryPoint);
                    usleep(5000);
                    cout << count << "--> " << trajectoryPoint.Position.CartesianPosition.X << "-" << trajectoryPoint.Position.CartesianPosition.Y << "-" << trajectoryPoint.Position.CartesianPosition.Z << "-"
                            << trajectoryPoint.Position.CartesianPosition.ThetaX << "-" << trajectoryPoint.Position.CartesianPosition.ThetaY << "-"  << trajectoryPoint.Position.CartesianPosition.ThetaZ << endl;
                }

                /* using basic trajectory
                while (ifs.is_open() &&  ifs >> count >> j1 >> j2 >> j3 >> j4 >> j5 >> j6 >> x >> y >> z >> tx >> ty >> tz)
                {
                    trajectoryPoint.Position.Type = CARTESIAN_POSITION;
                    trajectoryPoint.Position.HandMode = POSITION_MODE;
                    trajectoryPoint.Position.CartesianPosition.X = x;
                    trajectoryPoint.Position.CartesianPosition.Y = y;
                    trajectoryPoint.Position.CartesianPosition.Z = z;
                    trajectoryPoint.Position.CartesianPosition.ThetaX = tx;
                    trajectoryPoint.Position.CartesianPosition.ThetaY = ty;
                    trajectoryPoint.Position.CartesianPosition.ThetaZ = tz;
                    (*MySendBasicTrajectory)(trajectoryPoint);
                    usleep(5000);
                    cout << count << "--> " << trajectoryPoint.Position.CartesianPosition.X << "-" << trajectoryPoint.Position.CartesianPosition.Y << "-" << trajectoryPoint.Position.CartesianPosition.Z << "-"
                            << trajectoryPoint.Position.CartesianPosition.ThetaX << "-" << trajectoryPoint.Position.CartesianPosition.ThetaY << "-"  << trajectoryPoint.Position.CartesianPosition.ThetaZ << endl;
                }
                */

                cout << "\n\n Trajectory Executed Successfully!" << endl;
                ifs.close();
            }
            else
            {
                cout << "aborted.";
            }
        }
        else  // executing trajectory in Angular mode
        {
            result = (*MySetAngularControl)();
            cout << "Angular control has been set." << result << endl;

            cout << "Start executing the trajectory?";
            getline(cin, answer);
            if (answer.compare(yes) == 0 )
            {

                // using advanced trajectory
                /*
                trajectoryPoint.InitStruct();
                trajectoryPoint.Limitations.accelerationParameter1 = 0.9f;  //not implemented
                trajectoryPoint.Limitations.accelerationParameter2 = 0.9f;  //not implemented
                trajectoryPoint.Limitations.accelerationParameter3 = 0.9f;  //not implemented
                trajectoryPoint.Limitations.forceParameter1 = 0.0f;         //not implemented
                trajectoryPoint.Limitations.forceParameter2 = 0.0f;         //not implemented
                trajectoryPoint.Limitations.forceParameter3 = 0.0f;         //not implemented
                trajectoryPoint.Limitations.speedParameter1 = 1.0f;         // limit the translational velocity to 8cm/s
                trajectoryPoint.Limitations.speedParameter2 = 1.6f;         // limit the rotational velocity to 0.6 RAD/s
                trajectoryPoint.Limitations.speedParameter3 = 1.5f;         // limit the translational velocity to 8cm/s
                trajectoryPoint.LimitationsActive = true;
                trajectoryPoint.Position.Type = ANGULAR_POSITION;
                trajectoryPoint.Position.Actuators.Actuator1 = 0.0f;
                trajectoryPoint.Position.Actuators.Actuator2 = 0.0f;
                trajectoryPoint.Position.Actuators.Actuator3 = 0.0f;
                trajectoryPoint.Position.Actuators.Actuator4 = 0.0f;
                trajectoryPoint.Position.Actuators.Actuator5 = 0.0f;
                trajectoryPoint.Position.Actuators.Actuator6 = 0.0f;
                trajectoryPoint.Position.Delay = 0.0f;
                trajectoryPoint.Position.HandMode = POSITION_MODE;

                int ch;

                while (ifs.is_open() &&  ifs >> count >> j1 >> j2 >> j3 >> j4 >> j5 >> j6 >> x >> y >> z >> tx >> ty >> tz && ch != 's')
                {

                    initscr();
                    cbreak();
                    noecho();
                    nodelay(stdscr, TRUE);
                    scrollok(stdscr, TRUE);
                    if (kbhit()) // a key has been pressed!
                    {
                        ch = getch();
                        printw("Key pressed! It was: %d\n",ch );
                        refresh();
                        sleep(1);
                    }
                    else // no key has been pressed
                    {
                        //trajectoryPoint.Position.Type = ANGULAR_POSITION;
                        //trajectoryPoint.Position.HandMode = POSITION_MODE;
                        //trajectoryPoint.Limitations.speedParameter2 = 20;
                        trajectoryPoint.LimitationsActive = false;
                        trajectoryPoint.Position.Actuators.Actuator1 = j1;
                        trajectoryPoint.Position.Actuators.Actuator2 = j2;
                        trajectoryPoint.Position.Actuators.Actuator3 = j3;
                        trajectoryPoint.Position.Actuators.Actuator4 = j4;
                        trajectoryPoint.Position.Actuators.Actuator5 = j5;
                        trajectoryPoint.Position.Actuators.Actuator6 = j6;
                        //(*MySendAdvanceTrajectory)(trajectoryPoint);
                        (*MySendBasicTrajectory)(trajectoryPoint);
                        //printw("point %d\n",count);
                        cout << count << ",\n";
                        usleep(5000);

                        //cout << count << "--> " << trajectoryPoint.Position.Actuators.Actuator1 << "-" << trajectoryPoint.Position.Actuators.Actuator2 << "-" << trajectoryPoint.Position.Actuators.Actuator3 << "-"
                                //<< trajectoryPoint.Position.Actuators.Actuator4 << "-" << trajectoryPoint.Position.Actuators.Actuator5 << "-"  << trajectoryPoint.Position.Actuators.Actuator6 << endl;

                        refresh();

                    }
                    endwin();

                }
                */

                // 7sec = 7000000 / 140 points = 50000
                //using basic trajectory
                while (ifs.is_open() &&  ifs >> count >> j1 >> j2 >> j3 >> j4 >> j5 >> j6 >> x >> y >> z >> tx >> ty >> tz >> M1 >> M2 >> M3 >> M4 >> M5 >> M6 >> Fx >> Fy >> Fz >> Mx >> My >> Mz)
                {
                    trajectoryPoint.Position.Type = ANGULAR_POSITION;
                    trajectoryPoint.Position.HandMode = POSITION_MODE;
                    trajectoryPoint.Position.Actuators.Actuator1 = j1;
                    trajectoryPoint.Position.Actuators.Actuator2 = j2;
                    trajectoryPoint.Position.Actuators.Actuator3 = j3;
                    trajectoryPoint.Position.Actuators.Actuator4 = j4;
                    trajectoryPoint.Position.Actuators.Actuator5 = j5;
                    trajectoryPoint.Position.Actuators.Actuator6 = j6;
                    (*MySendBasicTrajectory)(trajectoryPoint);
                    usleep(3000);
                    cout << count << "--> " << trajectoryPoint.Position.Actuators.Actuator1 << "-" << trajectoryPoint.Position.Actuators.Actuator2 << "-" << trajectoryPoint.Position.Actuators.Actuator3 << "-"
                            << trajectoryPoint.Position.Actuators.Actuator4 << "-" << trajectoryPoint.Position.Actuators.Actuator5 << "-"  << trajectoryPoint.Position.Actuators.Actuator6 << endl;
                }


                cout << "\n\n Trajectory Executed Successfully!" << endl;
                ifs.close();
            }
            else
            {
                cout << "aborted.";
            }
        }

        result = (*MyCloseAPI)();
        ifs.close();
        cout << endl << "Closing API and the file!" << endl;
        cout << endl << "Done!" << endl;
    }
    return 0;
}



