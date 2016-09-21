#include <iostream>
#include <dlfcn.h>
#include <vector>
#include <Kinova.API.CommLayerUbuntu.h>
#include <KinovaTypes.h>
#include <stdio.h>
#include <fstream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std;

int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}

// ---------------------------------------------------------------------------
// ------------------------------ MAIN ---------------------------------------
// ---------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    cout << endl << endl;
    cout << "===================================================" << endl;
    cout << "===== A Demonstration Recorder for Jaco2 arm  =====" << endl;
    cout << "===================================================" << endl;
    cout << "code: Reza Ahmadzadeh (IRIM, 2016)." << endl;
    cout << "structure: i j1 j2 j3 j4 j5 j6 x y z thetax thetay thetaz" << endl;
    cout << "{Position, Angle} : {mm, deg}" << endl << endl;
    if ( argc != 2 ) {// argc should be 2 for correct execution
        // We print argv[0] assuming it is the program name
        cout << "Usage: "<< argv[0] <<" <filename>" << endl;
        cout << "Error: a filename has to be provided, for instance [demo_1.dat]" << endl  << endl << endl;
        return 0;
    }
    else {
        // We assume argv[1] is a filename to open
        cout << "recording in a file saved as: " << argv[1] << endl;
        }


    int result;
    AngularPosition dataPosition;
    CartesianPosition cartPosition;
    //AngularPosition dataCommand;
    //CartesianPosition endeffector;

    //Handle for the library's command layer.
    void * commandLayer_handle;
    //Function pointers to the functions we need
    int (*MyInitAPI)();
    int (*MyCloseAPI)();
    //int (*MyGetAngularCommand)(AngularPosition &);
    int (*MyGetAngularPosition)(AngularPosition &);
    int (*MyGetCartesianPosition)(CartesianPosition &);
    int (*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
    int (*MySetActiveDevice)(KinovaDevice device);
    int (*MySendBasicTrajectory)(TrajectoryPoint command);
    int (*MyMoveHome)();

    //We load the library
    commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);

    //We load the functions from the library (Under Windows, use GetProcAddress)
    MyInitAPI = (int (*)()) dlsym(commandLayer_handle,"InitAPI");
    MyCloseAPI = (int (*)()) dlsym(commandLayer_handle,"CloseAPI");
    //MyGetAngularCommand = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularCommand");
    MyGetAngularPosition = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularPosition");
    MyGetCartesianPosition = (int (*) (CartesianPosition &)) dlsym(commandLayer_handle,"GetCartesianPosition");
    MyGetDevices = (int (*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle,"GetDevices");
    MySetActiveDevice = (int (*)(KinovaDevice devices)) dlsym(commandLayer_handle,"SetActiveDevice");
    MySendBasicTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle,"SendBasicTrajectory");
    MyMoveHome = (int(*)()) dlsym(commandLayer_handle, "MoveHome");

    //If the was loaded correctly
    if((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MyGetCartesianPosition == NULL) || (MyGetAngularPosition == NULL)
       || (MySetActiveDevice == NULL) || (MyGetDevices == NULL))
    {
        cout << "* * *  ERROR: initialization failed!  * * *" << endl;
    }
    else
    {

        result = (*MyInitAPI)();
        cout << "Initialization's result :" << result << endl;

        ofstream outfile;       // lets open a file
        outfile.open(argv[1]); //("trajectory.dat");
        cout << "An empty file has been created to record the trajectory." << endl;

        KinovaDevice list[MAX_KINOVA_DEVICE];

        int devicesCount = MyGetDevices(list, result);
        if (devicesCount == 0)
        {
            cout << "\n WARNING: The robot is off or is not in the loop!" << endl;
            return 0;
        }

        for(int i = 0; i < devicesCount; i++)
        {
            cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ") (" << list[i].DeviceType << ")" << endl;

            //Setting the current device as the active device.
            MySetActiveDevice(list[i]);

            //MyMoveHome();  // move the robot home

            std::string answer;
            std::string yes = "y"; //.c_str();
            TrajectoryPoint trajectoryPoint;
            cout << "The robot will move to HOME position first. Agree (y)?";
            getline(cin, answer);
            if (answer.compare(yes) == 0 )
            {
                trajectoryPoint.InitStruct();
                trajectoryPoint.Position.Type = ANGULAR_POSITION;
                trajectoryPoint.Position.Actuators.Actuator1 = 180; //227.4;
                trajectoryPoint.Position.Actuators.Actuator2 = 180; //198.89;
                trajectoryPoint.Position.Actuators.Actuator3 = 90; //140.07;
                trajectoryPoint.Position.Actuators.Actuator4 = 90; //-72.08;
                trajectoryPoint.Position.Actuators.Actuator5 = 0; //37.43;
                trajectoryPoint.Position.Actuators.Actuator6 = 0; //43.52;
                (*MySendBasicTrajectory)(trajectoryPoint);
                usleep(3000000);
                cout << "Home now!" << endl;
            }


            cout << "start recording (y)?";
            getline(cin, answer);
            if (answer.compare(yes)!=0)
            {
                result = (*MyCloseAPI)();
                outfile.close();
                cout << endl << "Closing the file and the API!" << endl;
                cout << endl << "Done!" << endl;
                return 0;
            }


            int pointCounter = 0;
            while(!kbhit())
            {
                // -------------- to keep the end-effectors orientation fixed
                /*
                trajectoryPoint.InitStruct();
                trajectoryPoint.Position.Type = CARTESIAN_POSITION;
                // get the current pose of the end-effector
                //trajectoryPoint.Position.CartesianPosition.X = x;
                //trajectoryPoint.Position.CartesianPosition.Y = y;
                //trajectoryPoint.Position.CartesianPosition.Z = z;
                trajectoryPoint.Position.CartesianPosition.ThetaX = 0;
                trajectoryPoint.Position.CartesianPosition.ThetaY = 0;
                //trajectoryPoint.Position.CartesianPosition.ThetaZ = tz;
                (*MySendBasicTrajectory)(trajectoryPoint);
                //usleep(3000);
                */

                // (*MyGetAngularCommand)(dataCommand);
                (*MyGetAngularPosition)(dataPosition);
                (*MyGetCartesianPosition)(cartPosition);

                outfile << pointCounter << "  " << dataPosition.Actuators.Actuator1 << "  " << dataPosition.Actuators.Actuator2 << "  " << dataPosition.Actuators.Actuator3 << "  "
                        << dataPosition.Actuators.Actuator4 << "  " << dataPosition.Actuators.Actuator5 << "  " << dataPosition.Actuators.Actuator6 << "  "
                        << cartPosition.Coordinates.X << "  " << cartPosition.Coordinates.Y << "  " << cartPosition.Coordinates.Z << "  "
                        << cartPosition.Coordinates.ThetaX << "  " << cartPosition.Coordinates.ThetaY << "  " << cartPosition.Coordinates.ThetaZ << endl;
                cout << pointCounter << "  " << dataPosition.Actuators.Actuator1 << "  " << dataPosition.Actuators.Actuator2 << "  " << dataPosition.Actuators.Actuator3 << "  "
                     << dataPosition.Actuators.Actuator4 << "  " << dataPosition.Actuators.Actuator5 << "  " << dataPosition.Actuators.Actuator6 << "  "
                     << cartPosition.Coordinates.X << "  " << cartPosition.Coordinates.Y << "  " << cartPosition.Coordinates.Z << "  "
                     << cartPosition.Coordinates.ThetaX << "  " << cartPosition.Coordinates.ThetaY << "  " << cartPosition.Coordinates.ThetaZ << endl;
                pointCounter++;
            }

            cout << "The robot will move back to HOME position. Agree (y)?";
            getline(cin, answer);
            if (answer.compare(yes) == 0 )
            {
                trajectoryPoint.InitStruct();
                trajectoryPoint.Position.Type = ANGULAR_POSITION;
                trajectoryPoint.Position.Actuators.Actuator1 = 270;
                trajectoryPoint.Position.Actuators.Actuator2 = 180;
                trajectoryPoint.Position.Actuators.Actuator3 = 90;
                trajectoryPoint.Position.Actuators.Actuator4 = 0;
                trajectoryPoint.Position.Actuators.Actuator5 = 90;
                trajectoryPoint.Position.Actuators.Actuator6 = 45;
                (*MySendBasicTrajectory)(trajectoryPoint);
                usleep(3000000);
            }
            else
            {
                result = (*MyCloseAPI)();
                outfile.close();
                cout << endl << "Closing the file and the API!" << endl;
                cout << endl << "Done!" << endl;
                return 0;
            }
        }

        outfile.close();
        cout << endl << "Closing the file...!" << endl;
        cout << endl << "Closing API...!" << endl;
        result = (*MyCloseAPI)();
        cout << endl << "Done!" << endl;
    }
    dlclose(commandLayer_handle);
    return 0;
}
