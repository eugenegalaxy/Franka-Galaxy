// Pity attempt to conver "web_interface_python.py" into C++ version. 
// Did not succeed. Maybe you can take on from here and do it?
// How to do it:
// Option 1: Write a C++ implementation "web_interface_python.py". 
//           What stopped me is now knowing how to do "encode_password" function in C++...
// Option 2: (THIS FILE) Call python functions through C++ using "Python.h" library.
             // Update HOSTNAME/LOGIN/PASSWORD if needed and try to run the code, see where it stops.
// Good luck!
// Author: Jevgenijs Galaktionovs, jgalak16@student.aau.dk (until 2021 June); jga@reallyarobot.com 

#include <Python.h>
#include <tuple>
#include <iostream>

const char HOSTNAME[] = "172.27.23.65";
const char LOGIN[] = "Panda";
const char PASSWORD[] = "panda1234";

std::string get_working_path(){
   char temp[200];
   return ( getcwd(temp, sizeof(temp)) ? std::string( temp ) : std::string("") );
}


// DOESNT WORK BECAUSE IT CALLS PYTHON2 FOR SOME REASON...
int main(int argc, char** argv)
{

    std::string path = get_working_path();
    const char *c_path = path.c_str(); // convert std::string to const char* for setenv()
    // Set PYTHONPATH TO working directory
    setenv("PYTHONPATH",c_path,1);
    //setenv("PYTHONPATH","/home/robotlab/catkin_ws/src/franka_plugin/src",1);
    
    PyObject *pName = NULL;
    PyObject *pModule = NULL;
    PyObject *pDict = NULL;
    PyObject *pFunc = NULL;
    PyObject *pValue1 = NULL;
    PyObject *pValue2 = NULL;
    PyObject *pValue3 = NULL;
    PyObject *presult = NULL;
    PyObject *pArgs = NULL;
    // Initialize the Python Interpreter
    Py_Initialize();
    Py_SetPythonHome((char *)"/usr/bin/python3");
    std::cout << Py_GetPythonHome() << std::endl;
    // Build the name object
    if ((pName = PyUnicode_FromString((char*)"web_interface_python")))
    {
        std::cout << "here? 3" << std::endl;   
        // Load the module object
        std::cout << Py_GetPythonHome() << std::endl;
        if ((pModule = PyImport_Import(pName)))
        {
            std::cout << "here? 4" << std::endl;   
            // pDict is a borrowed reference 
            pDict = PyModule_GetDict(pModule);
            std::cout << "here? 5" << std::endl;   
            // pFunc is also a borrowed reference 
            pFunc = PyDict_GetItemString(pDict, (char*)"franka_gripper_homing");
            std::cout << "here? 6" << std::endl;   
            if (PyCallable_Check(pFunc))
            {
                std::cout << "here? 7" << std::endl;   
                pValue1=Py_BuildValue("(z)",(char*)HOSTNAME);
                pValue2=Py_BuildValue("(z)",(char*)LOGIN);
                pValue3=Py_BuildValue("(z)",(char*)PASSWORD);
                std::cout << "here? 8" << std::endl;   
                pArgs = PyTuple_New(3);
                PyTuple_SetItem(pArgs, 0, pValue1);
                PyTuple_SetItem(pArgs, 1, pValue2);
                PyTuple_SetItem(pArgs, 2, pValue3);
                PyErr_Print();
                printf("Let's give this a shot!\n");
                presult=PyObject_CallObject(pFunc,pArgs);
                PyErr_Print();
            }
        }
    }
    if (PyErr_Occurred()) PyErr_Print();
    printf("Result is %d\n", (int)PyInt_AsLong(presult));

    if(pValue1){
        // Clean up
        Py_DECREF(pValue1);
        Py_DECREF(pValue2);
        Py_DECREF(pValue3);
        Py_DECREF(pModule);
        Py_DECREF(pName);
    }
    // Finish the Python Interpreter
    Py_Finalize();
    return 0;
}