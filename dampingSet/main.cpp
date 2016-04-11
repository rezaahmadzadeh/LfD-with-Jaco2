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


int main()
{

        AngularPosition currentCommand;
        int result;
        cout << "SetCartesianForceMinMax function example" << endl;
        //Handle for the library's command layer.
        void * commandLayer_handle;
        //Function pointers to the functions we need
        int (*MyInitAPI)();
        int (*MyCloseAPI)();
        int (*MyStartForceControl)();
        int (*MySetCartesianForceMinMax)(CartesianInfo, CartesianInfo);    // min and max forces
        int (*MySetCartesianInertiaDamping)(CartesianInfo, CartesianInfo); // inertia and damping
        int (*MySetAngularInertiaDamping)(AngularInfo, AngularInfo);
        int (*MySendBasicTrajectory)(TrajectoryPoint command);
        int (*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
        int (*MySetActiveDevice)(KinovaDevice device);
        int (*MyMoveHome)();
        int (*MyInitFingers)();
        int (*MyGetAngularCommand)(AngularPosition &);


        //We load the library (Under Windows, use the function LoadLibrary)
        commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);
        //We load the functions from the library (Under Windows, use GetProcAddress)
        MyInitAPI = (int (*)()) dlsym(commandLayer_handle,"InitAPI");
        MyCloseAPI = (int (*)()) dlsym(commandLayer_handle,"CloseAPI");
        MySetCartesianForceMinMax = (int (*)(CartesianInfo, CartesianInfo)) dlsym(commandLayer_handle,"SetCartesianForceMinMax");
        MySetCartesianInertiaDamping = (int (*)(CartesianInfo, CartesianInfo))dlsym(commandLayer_handle,"SetCartesianInertiaDamping");
        MySetAngularInertiaDamping = (int (*)(AngularInfo, AngularInfo)) dlsym(commandLayer_handle,"SetAngularInertiaDamping");
        MyStartForceControl = (int(*)()) dlsym(commandLayer_handle,"StartForceControl");
        MyMoveHome = (int (*)()) dlsym(commandLayer_handle,"MoveHome");
        MyInitFingers = (int (*)()) dlsym(commandLayer_handle,"InitFingers");
        MyGetDevices = (int (*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle,"GetDevices");
        MySetActiveDevice = (int (*)(KinovaDevice devices)) dlsym(commandLayer_handle,"SetActiveDevice");
        MySendBasicTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle,"SendBasicTrajectory");
        MyGetAngularCommand = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularCommand");




        //If the was loaded correctly

        if((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MySendBasicTrajectory == NULL) ||
           (MySendBasicTrajectory == NULL) || (MyMoveHome == NULL) || (MyInitFingers == NULL) ||
           (MySetCartesianForceMinMax == NULL) || (MySetAngularInertiaDamping == NULL) || (MySetCartesianInertiaDamping == NULL))
        {
            cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << endl;
            cout << MyInitAPI  << MyCloseAPI  << MySendBasicTrajectory << MySendBasicTrajectory  << MyMoveHome << MyInitFingers << MySetCartesianForceMinMax  << MySetAngularInertiaDamping  << MySetCartesianInertiaDamping << endl;
        }
        else
        {
                cout << "The command has been initialized correctly." << endl << endl;
                cout << "Calling the method InitAPI()" << endl;
                result = (*MyInitAPI)();
                cout << "result of InitAPI() = " << result << endl << endl;



                KinovaDevice list[MAX_KINOVA_DEVICE];

                int devicesCount = MyGetDevices(list, result);

                for(int i = 0; i < devicesCount; i++)
                {
                    cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << endl;

                    //Setting the current device as the active device.
                    MySetActiveDevice(list[i]);

                    cout << "Send the robot to HOME position" << endl;
                    MyMoveHome();


                    TrajectoryPoint pointToSend;
                    pointToSend.InitStruct();
                    //We specify that this point will be an angular(joint by joint) position.
                    pointToSend.Position.Type = ANGULAR_POSITION;

                    //We get the actual angular command of the robot.
                    MyGetAngularCommand(currentCommand);

                    pointToSend.Position.Actuators.Actuator1 = currentCommand.Actuators.Actuator1 + 30;
                    pointToSend.Position.Actuators.Actuator2 = currentCommand.Actuators.Actuator2;
                    pointToSend.Position.Actuators.Actuator3 = currentCommand.Actuators.Actuator3;
                    pointToSend.Position.Actuators.Actuator4 = currentCommand.Actuators.Actuator4;
                    pointToSend.Position.Actuators.Actuator5 = currentCommand.Actuators.Actuator5;
                    pointToSend.Position.Actuators.Actuator6 = currentCommand.Actuators.Actuator6;

                    cout << "*********************************" << endl;
                    cout << "Sending the first point to the robot." << endl;
                    MySendBasicTrajectory(pointToSend);




                    CartesianInfo commandMin;
                    CartesianInfo commandMax;
                    commandMin.InitStruct();
                    commandMax.InitStruct();
                    commandMin.X = 8.0f;
                    commandMin.Y = 8.0f;
                    commandMin.Z = 8.0f;
                    commandMin.ThetaX = 1.0f;
                    commandMin.ThetaY = 1.0f;
                    commandMin.ThetaZ = 1.0f;

                    //float cmax = 200.0;
                    commandMax.X = 12.0f;
                    commandMax.Y = 15.0f;
                    commandMax.Z = 20.0f;
                    commandMax.ThetaX = 2.0f;
                    commandMax.ThetaY = 2.0f;
                    commandMax.ThetaZ = 2.0f;
                    result = (*MySetCartesianForceMinMax)(commandMin, commandMax);
                    cout << "Cartesian force min and max has been set: " << result << endl;
                    /*
                    CartesianInfo Inertia;
                    CartesianInfo Damping;
                    // not setting the intertia yet


                    float inertiaValue = 1.0;  // km*m^2
                    Inertia.X = inertiaValue;
                    Inertia.Y = inertiaValue;
                    Inertia.Z = inertiaValue;
                    Inertia.ThetaX = inertiaValue;
                    Inertia.ThetaY = inertiaValue;
                    Inertia.ThetaZ = inertiaValue;

                    float dmpValue = 1.0;    // N*s/m,  N*s/RAD
                    Damping.X = dmpValue;
                    Damping.Y = dmpValue;
                    Damping.Z = dmpValue;
                    Damping.ThetaX = dmpValue;
                    Damping.ThetaY = dmpValue;
                    Damping.ThetaZ = dmpValue;

                    cout << "reached here" << endl;
                    //result = (*MySetCartesianInertiaDamping)(Inertia,Damping);
                    cout << "Cartesian inertial and damping values have been set." << endl;
                    */



                    AngularInfo Inertia;
                    AngularInfo Damping;
                    Inertia.InitStruct();
                    Damping.InitStruct();


                    Inertia.Actuator1 = 0.006f;
                    Inertia.Actuator2 = 0.006f;
                    Inertia.Actuator3 = 0.005f;
                    Inertia.Actuator4 = 0.005f; //0.050f;
                    Inertia.Actuator5 = 0.005f; //0.050f;
                    Inertia.Actuator6 = 0.005f; //0.050f;

                    Damping.Actuator1 = 0.015f;
                    Damping.Actuator2 = 0.015f;
                    Damping.Actuator3 = 0.01f;
                    Damping.Actuator4 = 0.01f;
                    Damping.Actuator5 = 0.01f;
                    Damping.Actuator6 = 0.01f;

                    result = (*MySetAngularInertiaDamping)(Inertia,Damping);
                    cout << "Angular inertia and damping values have been set: " << result << endl;

                    cout << "Damping " << Damping.Actuator1 << " Inertia " << Inertia.Actuator1 << endl;
                    result = MyStartForceControl();
                    cout << "Force Control is started: " << result << endl;

                }

                cout << endl << "WARNING: Your robot is now set to angular control. If you use the joystick, it will be a joint by joint movement." << endl;
                cout << endl << "C L O S I N G   A P I" << endl;
                result = (*MyCloseAPI)();
            }

            dlclose(commandLayer_handle);





        return 0;
}




/*




    KinovaDevice list[MAX_KINOVA_DEVICE];

    int devicesCount = MyGetDevices(list, result);

    for(int i = 0; i < devicesCount; i++)
    {
        cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << endl;

        //Setting the current device as the active device.
        MySetActiveDevice(list[i]);

        cout << "Send the robot to HOME position" << endl;
        MyMoveHome();

        cout << "Initializing the fingers" << endl;
        MyInitFingers();

        TrajectoryPoint pointToSend;
        pointToSend.InitStruct();

        //We specify that this point will be used an angular(joint by joint) velocity vector.
        pointToSend.Position.Type = ANGULAR_VELOCITY;

        pointToSend.Position.Actuators.Actuator1 = 0;
        pointToSend.Position.Actuators.Actuator2 = 0;
        pointToSend.Position.Actuators.Actuator3 = 0;
        pointToSend.Position.Actuators.Actuator4 = 0;
        pointToSend.Position.Actuators.Actuator5 = 0;
        pointToSend.Position.Actuators.Actuator6 = 48; //joint 6 at 48 degrees per second.

        pointToSend.Position.Fingers.Finger1 = 0;
        pointToSend.Position.Fingers.Finger2 = 0;
        pointToSend.Position.Fingers.Finger3 = 0;

        for(int i = 0; i < 300; i++)
        {
            //We send the velocity vector every 5 ms as long as we want the robot to move along that vector.
            MySendBasicTrajectory(pointToSend);
            usleep(5000);
        }

        pointToSend.Position.Actuators.Actuator6 = -20; //joint 6 at -20 degrees per second.

        for(int i = 0; i < 300; i++)
        {
            //We send the velocity vector every 5 ms as long as we want the robot to move along that vector.
            MySendBasicTrajectory(pointToSend);
            usleep(5000);
        }

        cout << "Send the robot to HOME position" << endl;
        MyMoveHome();

        //We specify that this point will be an angular(joint by joint) position.
        pointToSend.Position.Type = ANGULAR_POSITION;

        //We get the actual angular command of the robot.
        MyGetAngularCommand(currentCommand);

        pointToSend.Position.Actuators.Actuator1 = currentCommand.Actuators.Actuator1 + 30;
        pointToSend.Position.Actuators.Actuator2 = currentCommand.Actuators.Actuator2;
        pointToSend.Position.Actuators.Actuator3 = currentCommand.Actuators.Actuator3;
        pointToSend.Position.Actuators.Actuator4 = currentCommand.Actuators.Actuator4;
        pointToSend.Position.Actuators.Actuator5 = currentCommand.Actuators.Actuator5;
        pointToSend.Position.Actuators.Actuator6 = currentCommand.Actuators.Actuator6;

        cout << "*********************************" << endl;
        cout << "Sending the first point to the robot." << endl;
        MySendBasicTrajectory(pointToSend);

        pointToSend.Position.Actuators.Actuator1 = currentCommand.Actuators.Actuator1 - 60;
        cout << "Sending the second point to the robot." << endl;
        MySendBasicTrajectory(pointToSend);

        pointToSend.Position.Actuators.Actuator1 = currentCommand.Actuators.Actuator1;
        cout << "Sending the third point to the robot." << endl;
        MySendBasicTrajectory(pointToSend);

        cout << "*********************************" << endl << endl << endl;
    }

    cout << endl << "WARNING: Your robot is now set to angular control. If you use the joystick, it will be a joint by joint movement." << endl;
    cout << endl << "C L O S I N G   A P I" << endl;
    result = (*MyCloseAPI)();
}

dlclose(commandLayer_handle);

return 0;
*/
