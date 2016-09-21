#include <iostream>
#include <dlfcn.h> //Ubuntu
#include <KinovaTypes.h>
#include <Kinova.API.CommLayerUbuntu.h>

using namespace std;
int main()
{
        int result;
        cout << "Erasing all buffered points...";
        void * commandLayer_handle;
        int (*MyInitAPI)();
        int (*MyCloseAPI)();
        int (*MyEraseAllTrajectories)();
        commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);
        MyInitAPI = (int (*)()) dlsym(commandLayer_handle,"InitAPI");
        MyCloseAPI = (int (*)()) dlsym(commandLayer_handle,"CloseAPI");
        MyEraseAllTrajectories = (int (*)()) dlsym(commandLayer_handle,"EraseAllTrajectories");
        if((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MyEraseAllTrajectories == NULL))
        {
                cout << "*** Initialization Failed! ***" << endl;
        }
        else
        {
                result = (*MyInitAPI)();
                result = (*MyEraseAllTrajectories)();
                cout << "Done." << endl;
                result = (*MyCloseAPI)();
        }
        return 0;
}

