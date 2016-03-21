#include <iostream>
#include <dlfcn.h>
#include <vector>
#include <Kinova.API.CommLayerUbuntu.h>
#include <KinovaTypes.h>
#include <stdio.h>
#include <unistd.h>

using namespace std;

int main()
{
	int result;

	//Handle for the library's command layer.
	void * commandLayer_handle;

	//Function pointers to the functions we need
	int (*MyInitAPI)();
	int (*MyCloseAPI)();
	int (*MyStartForceControl)();

	//We load the library
	commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);

	//We load the functions from the library
	MyInitAPI = (int (*)()) dlsym(commandLayer_handle,"InitAPI");
	MyCloseAPI = (int (*)()) dlsym(commandLayer_handle,"CloseAPI");
	MyStartForceControl = (int (*)()) dlsym(commandLayer_handle,"StartForceControl");


	//If the was loaded correctly
    if((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MyStartForceControl == NULL))
	{
		cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << endl;
	}
	else
	{
		//cout << "I N I T I A L I Z A T I O N   C O M P L E T E D" << endl << endl;

		result = (*MyInitAPI)();

		// cout << "Initialization's result :" << result << endl;

  		MyStartForceControl();
		cout << "ForceControl Mode is activated!" << endl;
		result = (*MyCloseAPI)();
	}

	dlclose(commandLayer_handle);

	return 0;
}
