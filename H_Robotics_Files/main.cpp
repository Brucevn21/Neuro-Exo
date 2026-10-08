
/// The includes below are needed to compile and execute all the methods in the class.
#include "imu.hpp"
#include "amp.hpp"
#include <stdio.h>
#include <cstdlib>
#include <sys/time.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <chrono>
#include <sstream>
#include <list>
#include <string.h>
#include <time.h>
#include "globalVariables.hpp"
#include "Calibration_Collection.hpp"
// #include <boost/algorithm/string.hpp>
using namespace std;

/// Libraries from libsoc that is for gpio management because of the interrupt.
#include <libsoc_gpio.h>
#include <libsoc_debug.h>
#include "Testing_SVM.hpp" //contains includes to channel and hardware_interface class

// Include for threads
#include <thread>
#include <atomic>
#include <mutex>

/// Define the GPIO that will be used for input and the interrupt in this case is GPIO 115 for the BBB-W
#define GPIO_INPUT 115

/// Global variables for amplifier object, interrupt, imu object, and file so that it can be used by the interrupt service routine.
gpio *gpio_input;
Amplifier amp;
Calibration cal;
Imu imu;
Testing test;
Hardware_Interface top_hI; // used for debug eeg/arm and measure impedance functions
ofstream myfile;
ofstream cleanfile;
int samples = 0;

/// Callback interrupt handler, this can be used if libsoc library is installed and imported.
int interrupt_count = 0;
int callback_interrupt_handler(void *args)
{
    /// Collect from amplifier and collect from IMU devices.
    samples++;
    amp.collectEeg();
    imu.collect();
    return EXIT_SUCCESS;
}

// Define Global struct used to hold global variables sent/recieved from mobile app -> passed to all files through extern
globalVals gV;

// Used to read data coming in from the app
// atomic<bool> emergencyStop(false);
mutex inputMutex;
string lastCommand;
std::atomic<bool> stopRequested(false); // needed to stop amp measure impedance
// Moved here from main in order for inputThread to change it
bool running = true;
// Thread-safe command variable using atomic
std::atomic<int> command(7); // 7 is default "do nothing"

void inputThread()
{
    string line;
    while (true)
    {
        if (getline(cin, line))
        { // blocks, but only this thread is blocked
            lock_guard<mutex> lock(inputMutex);
            lastCommand = line;

            if (stoi(line) == 1) // send settings,  NOTE: stoi(line) will only work if integers are passed, used to ignore white space
            {
                cout << "[INPUT THREAD]Reading settings from python server" << endl;
                connectToSettingsServer();
                receiveSettingsFromPython();
            }
            else if (line.find("2") != string::npos) // start impedance
            {
                cout << "[INPUT THREAD]Starting measure impedance" << endl;
                command.store(0);
            }
            else if (line.find("3") != std::string::npos) // stop impedance
            {
                cout << "[INPUT THREAD]Stop command received! ending measure impedance" << endl;
                stopRequested.store(true);         // Signal stop
                amp.impedanceStreamActive = false; // Stop impedance loop
                command.store(7);                  // go back to default do nothing state
            }
            else if (line.find("4") != std::string::npos) // start sequence
            {
                // cout << "[INPUT THREAD]Running a sequence" << endl;
                if (gV.PROCEDURE.find("Training") != string::npos) // start calibration collection
                {
                    cout << "[INPUT THREAD]Running calibration collection" << endl;
                    command.store(1);
                }
                else if (gV.PROCEDURE.find("Testing") != string::npos) // start testing svm
                {
                    cout << "[INPUT THREAD]Running testing svm" << endl;
                    command.store(3);
                }
                else
                {
                    cout << "[INPUT THREAD] None ran" << endl;
                }
                // cout << "[INPUT THREAD] After start sequence command is: " << command.load() << endl;
            }
            else if (line.find("5") != std::string::npos) // send file
            {
                cout << "[INPUT THREAD]Sending file(s)" << endl;
            }
            else if (line.find("6") != std::string::npos) // emergency stop
            {
                cout << "[INPUT THREAD]Setting emergency stop to true" << endl;
                gV.EMERGENCY_STOP = true;
            }
            else if (line.find("7") != std::string::npos) // end stage
            {
                cout << "[INPUT THREAD]Setting end stage to true" << endl;
                gV.END_STAGE = true;
            }
            else if (line.find("8") != std::string::npos) // train svm
            {
                cout << "[INPUT THREAD]Calling train svm python script" << endl;
                command.store(2);
            }
            else if (line.find("9") != std::string::npos) // debug eeg
            {
                cout << "[INPUT THREAD]Running debug eeg" << endl;
                command.store(4);
            }
            else if (stoi(line) == 0) // debug arm
            {
                cout << "[INPUT THREAD]Running debug arm" << endl;
                command.store(5);
            }
            else if (stoi(line) == 10) // iPad case
            {
                cout << "[INPUT THREAD]Performing synchron test" << endl;
                command.store(6);
            }
        }
    }
}

// Function declaration
string train_svm(
    const std::string &folder,
    const std::string &save_dir,
    int max_passes,
    const std::string &removal_method,
    const std::string &channels,
    const std::string &metric,
    const std::string &repeat,
    float outlier_prop);

int main(int argc, char **argv)
{
    // Configure BLE endpoints without connecting during sensor initialization.
    if (const char* address = std::getenv("NEUROEXO_NANO_ADDRESS")) {
        cal.hI.setBluetoothDevice(address);
        test.hI.setBluetoothDevice(address);
        top_hI.setBluetoothDevice(address);
    }

    // Calling thread USED TO READ DATA COMING IN FROM APP
    std::thread reader(inputThread);
    reader.detach(); // runs independently

    gV.VI_RUNNING = "1";

    /// Set up variables for GPIO including input/output, edge.
    gpio_input = libsoc_gpio_request(GPIO_INPUT, LS_GPIO_SHARED);
    libsoc_gpio_set_direction(gpio_input, INPUT);
    libsoc_gpio_set_edge(gpio_input, FALLING);

    /// These variables are taken from the command line we have the accelerometer sensitivity, gyroscope sensitivity, gain of EOG, gain of EEG, and time of trial.
    // float sens_acc = stof(argv[1]);
    // float sens_gyr = stof(argv[2]);
    // int amp_eog = std::stoi(argv[3]);
    // int amp_eeg = std::stoi(argv[4]);
    // int time_command = std::stoi(argv[5]);

    // HARD CODED BASED OFF: ./main 6384 131 6 0 2
    float sens_acc = 6384;
    float sens_gyr = 131;
    int amp_eog = 6;
    int amp_eeg = 0;
    int time_command = 2;

    cout << "[TOP OF MAIN]SENS_ACC: " << sens_acc << ", SENS_GYR: " << sens_gyr << endl;

    /// Initialize IMU object within the cal->hardware interface and test->hardware interface and set up.
    // cout << "[RUNNING MAIN WITH UPDATES]" << endl;
    cal.hI.imu_cp = Imu();
    cal.hI.imu_cp.startImu();
    cal.hI.imu_cp.imuSet();
    cal.hI.imu_cp.setSensAcc(sens_acc);
    cal.hI.imu_cp.setSensGyr(sens_gyr);

    test.hI.imu_cp = Imu();
    test.hI.imu_cp.startImu();
    test.hI.imu_cp.imuSet();
    test.hI.imu_cp.setSensAcc(sens_acc);
    test.hI.imu_cp.setSensGyr(sens_gyr);

    top_hI.imu_cp = Imu();
    top_hI.imu_cp.startImu();
    top_hI.imu_cp.imuSet();
    top_hI.imu_cp.setSensAcc(sens_acc);
    top_hI.imu_cp.setSensGyr(sens_gyr);

    /// Intialize Amplifier object and set up.
    amp = Amplifier();
    amp.startAmp();
    amp.setAmpEeg(amp_eeg);
    amp.setAmpEog(amp_eog);
    amp.startUp();
    /// Initialize Calibration parameters
    cal.hI.startAmp();
    cal.hI.setAmpEeg(amp_eeg);
    cal.hI.setAmpEog(amp_eog);
    cal.hI.startUp();
    // Initialize Testing SVM parameters
    test.hI.startAmp();
    test.hI.setAmpEeg(amp_eeg);
    test.hI.setAmpEog(amp_eog);
    test.hI.startUp();

    // Global hardware interface class used for debug eeg and debug arm
    top_hI.startAmp();
    top_hI.setAmpEeg(amp_eeg);
    top_hI.setAmpEog(amp_eog);
    top_hI.startUp();
    top_hI.debugMode();

    // Connect to Update server
    connectToUpdateServer();

    while (!gV.EMERGENCY_STOP)
    {
        int current_command = command.load(); // Use .load() for atomic read

        // Debug output to see what command is being executed
        // if (current_command != 7)
        // {
        //     cout << "[MAIN] Executing command: " << current_command << endl;
        // }

        switch (current_command)
        {
        case 0: // Impedance_Check
        {
            gV.THERAPY_STAGE = "impedance";
            // Send updated therapy_stage to app
            sendUpdateToPython("therapy_stage", gV.THERAPY_STAGE);
            amp.impedance();
            std::this_thread::sleep_for(std::chrono::milliseconds(1500)); // delay for a 1.5 seconds to allow input thread to change command for a cal col or test svm
            int current_command = command.load();
            if (current_command == 0) // this is meant to stop the race condition that occurs when impedance check ends and a sequence starts
                command.store(7);     // go back to default do nothing state (just in case but it should end completely)
            // cout << "[END OF IMPEDANCE CHECK CASE] command is: " << command.load() << endl; // DEBUG
            break;
        }
        case 1:                         // Calibration_Collection
            gV.THERAPY_STAGE = "stare"; // stare now
            // Send updated therapy_stage to app
            sendUpdateToPython("therapy_stage", gV.THERAPY_STAGE);
            // std::this_thread::sleep_for(std::chrono::milliseconds(2500));
            cal.executeModule(); // blocking, waits until all submodules have finished
            cal.endModule();
            break;
        case 2: // Training_SVM
        {
            gV.THERAPY_STAGE = "";
            // Send updated therapy_stage to app
            sendUpdateToPython("therapy_stage", gV.THERAPY_STAGE);
            // Run Lianne's SVM python script using c++ function
            string optimalFile = train_svm(
                "/home/debian/Desktop/subject",
                "/home/debian/Desktop/subject_model",
                stoi(gV.Max_p),
                gV.Rem_meth,
                gV.Ch_sel,
                gV.Opt_met,
                gV.Repeat_train,
                stod(gV.Prop));
            // Send the newly created file to c++
            // cout << "[TRAIN SVM CASE]The optimal model file path is: " << optimalFile << endl;
            send_specific_file(optimalFile);
            command.store(7); // go back to default do nothing state while it runs
            break;
        }
        case 3:                         // Testing_SVM
            gV.THERAPY_STAGE = "stare"; // stare now
            // Send updated therapy_stage to app
            sendUpdateToPython("therapy_stage", gV.THERAPY_STAGE);
            // std::this_thread::sleep_for(std::chrono::milliseconds(2500));
            test.executeModule(); // blocking, waits until all submodules have finished
            test.endModule();
            break;
        case 4: // debug_eeg
            gV.THERAPY_STAGE = "";
            // Send updated therapy_stage to app
            // cout << "RUNNING DEBUG EEG CASE" << endl;
            sendUpdateToPython("therapy_stage", gV.THERAPY_STAGE);
            try
            {
                top_hI.callEEG(nullptr, nullptr); // in debug mode so no channels passsed
            }
            catch (const std::exception &e)
            {
                std::cerr << "debug eeg thread exiting due to error: " << e.what() << std::endl;
            }
            command.store(7); // go back to default do nothing state (just in case but it should end completely)
            break;
        case 5: // debug_arm
            gV.THERAPY_STAGE = "";
            // Send updated therapy_stage to app
            sendUpdateToPython("therapy_stage", gV.THERAPY_STAGE);
            try {
                top_hI.debugArm();
            } catch (const std::exception& error) {
                std::cerr << "Arm BLE debug failed: " << error.what() << std::endl;
            }
            top_hI.disconnectBluetooth();
            command.store(7);            // go back to default do nothing state (just in case but it should end completely)
            break;
        case 6: // Synchron / iPad test
            test.synchronEnable();
            cout << "Performing iPad test" << endl;
            test.executeModule();
            test.endModule();
            break;
        case 7: // Nothing
            // Do nothing
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            break;
        default:
            // cout << "do nothing" << endl;
            cout << "[MAIN] Unknown command: " << current_command << endl;
            command.store(7); // Reset unknown commands
            break;
        }
    }
    cout << "[TOP] Main loop is ending." << endl;
    // Disconnect from settings server
    disconnectFromSettingsServer();

    gV.VI_RUNNING = "0";
    return 0;
}

// Simple function to call Python SVM training script
string train_svm(
    const std::string &folder,
    const std::string &save_dir,
    int max_passes,
    const std::string &removal_method,
    const std::string &channels,
    const std::string &metric,
    const std::string &repeat,
    float outlier_prop)
{
    std::ostringstream command;

    command << "python3 main.py"; // hard coded, expects main.py to be in same directory as executable
    command << " --folder \"" << folder << "\"";
    command << " --save_dir \"" << save_dir << "\"";
    command << " --max_passes " << max_passes;
    command << " --removal_method \"" << removal_method << "\"";
    command << " --channels \"" << channels << "\"";
    command << " --metric \"" << metric << "\"";
    command << " --repeat \"" << repeat << "\"";
    command << " --outlier_prop " << outlier_prop;

    std::string cmd = command.str();
    std::cout << "Executing: " << cmd << std::endl;

    // Capture output using popen
    std::string model_filename = "";
    std::array<char, 128> buffer;
    std::string result;

    FILE *pipe = popen(cmd.c_str(), "r");
    if (!pipe)
    {
        std::cerr << "Failed to run Python script" << std::endl;
        return "";
    }

    // Read output line by line
    while (fgets(buffer.data(), buffer.size(), pipe) != nullptr)
    {
        std::string line = buffer.data();
        std::cout << line; // Print to console

        // Look for the model filename marker
        if (line.find("MODEL_FILE:") != std::string::npos)
        {
            size_t pos = line.find("MODEL_FILE:") + 11;
            model_filename = line.substr(pos);
            // Remove trailing newline
            model_filename.erase(model_filename.find_last_not_of("\n\r") + 1);
        }
    }

    int status = pclose(pipe);

    if (status == 0)
    {
        std::cout << "Training completed successfully!" << std::endl;
        std::cout << "Model saved to: " << model_filename << std::endl;
    }
    else
    {
        std::cerr << "Training failed with exit code: " << status << std::endl;
    }

    return model_filename;
}