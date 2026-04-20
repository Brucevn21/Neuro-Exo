#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <iostream>
#include <cstdio>
#include "FIR-filter-class/filt.h"
#include "Hardware_Interface.hpp"
#include "globalVariables.hpp" //includes global variable struct
#include <string>
#include <iomanip>
#include <sstream>
#include <regex>
#include <vector>
#include <algorithm> // for std::remove
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <errno.h>
#include <random>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <chrono>
/// Libraries from libsoc that is for gpio management because of the interrupt.
// #include <libsoc_gpio.h>
// #include <libsoc_debug.h>
/// Define the GPIO that will be used for input and the interrupt in this case is GPIO 115 for the BBB-W
// #define GPIO_INPUT 115
/// Global variables for interrupt, imu object so that it can be used by the interrupt service routine.
// gpio *gpio_input;
/// Callback interrupt handler, this can be used if libsoc library is installed and imported.
// int interrupt_count = 0;
// int callback_interrupt_handler(void *args)
// {
//     {
//         imu_cp.collect();
//         return EXIT_SUCCESS;
//     }
// }

using namespace std;

void Hardware_Interface::startAmp()
{
    /// Define the transfer buffers for SPI communication.
    trx.tx_buf = (unsigned long)tx_buff_2;
    trx.rx_buf = (unsigned long)rx_buff_2;

    /// Define the bits per word, speed, delay, and length of the SPI communication buffers.
    trx.bits_per_word = 0;
    trx.speed_hz = spi_speed;
    trx.delay_usecs = 0;
    trx.len = len_data;

    /// Open the file where SPI communication buffer will happen in.
    fd = open(SPI_DEVICE, O_RDWR);

    /// The filter will be used for the measure impedance method.
    filtering = new Filter(BPF, 4, 250.0, 6.0, 8.0);
    high_pass = new Filter(HPF, 4, 250.0, 0.5);

    // Initialize for 5 EEG channels, 3 states per channel
    wh = MatrixXd::Zero(3, 8);   // 3x8 (3 states x 8 channels) - changed from 3x5 (3 states × 5 channels)
    Pt1 = MatrixXd::Zero(24, 3); // 24x3 (8 channels x 3 states, by 3) - changed from 15x3 (5 channels × 3 states, by 3)

    // Initialize Pt1 diagonal blocks
    for (int i = 0; i < 8; i++) // Changed to 8 from 3
    {                           // All 8 channels from 5 channels
        Pt1.block(i * 3, 0, 3, 3).diagonal().setConstant(0.5);
    }

    /// The if statement returns whether the file/buffer pointer can be accessed or not.
    if (fd < 0)
    {
        printf("Could not open the SPI device...\r\n");
        exit(EXIT_FAILURE);
    }

    /// The if statement is to check if the read mode can be accessed. If not error will be shown in terminal.
    ret = ioctl(fd, SPI_IOC_RD_MODE32, &scratch32);
    if (ret != 0)
    {
        printf("Could not read SPI mode...\r\n");
        close(fd);
        exit(EXIT_FAILURE);
    }

    /// The following statements are to set up writing mode for SPI bus. If not error will show up in the terminal.
    scratch32 |= SPI_MODE_1;
    ret = ioctl(fd, SPI_IOC_WR_MODE32, &scratch32);
    if (ret != 0)
    {
        printf("Could not write SPI mode...\r\n");
        close(fd);
        exit(EXIT_FAILURE);
    }

    /// The statements below are to verify if the speed of the buffer can be observed. If not error will show up in terminal.
    ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &scratch32);
    if (ret != 0)
    {
        printf("Could not read the SPI max speed...\r\n");
        close(fd);
        exit(EXIT_FAILURE);
    }

    /// The statement is to write the SPI speed. If not possible the error will show up in the terminal.
    scratch32 = 15000000;
    ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &scratch32);
    if (ret != 0)
    {
        printf("Could not write the SPI max speed...\r\n");
        close(fd);
        exit(EXIT_FAILURE);
    }
}
int Hardware_Interface::getAmpEog()
{
    return this->ampEog;
}
/***
 * METHOD: The setAmpEeg() setter method to change the EEG amplification.
 * PARAMETERS:  int amplification - integer referring to the index of the option from 0-6 where each number corresponds to an amplification.
 * RETURN: None
 ***/
void Hardware_Interface::setAmpEeg(int amplification)
{
    this->ampEeg = amplification;
}
void Hardware_Interface::setAmpEog(int amplification)
{
    this->ampEog = amplification;
}
/***
***
*METHOD: The startUpSequence() is the sequence of commands needed to set up the amplifier for all data collection and interaction.Please look at the manual for each command's meaning.
* PARAMETERS : int fd - File that is being used for SPI purposes.
* spi_ioc_transfer* transfer - Transfer struct needed for SPI purposes.
* RETURN : This method returns nothing, just sends the message to the appropriate bus.
***/
void Hardware_Interface::startUpSequence(int fd, spi_ioc_transfer *transfer)
{
    /// "hexes" array is initially filled with 27 0x00's
    /// The 0x11 command stops the continuously reading if data.
    hexes[0] = 0x11;
    changeBuff(hexes, fd, transfer);
    usleep(20000);

    /// Default BIAS estimation sequence.
    hexes[0] = 0x43;
    hexes[1] = 0x00;
    hexes[2] = 0xE0;
    changeBuff(hexes, fd, transfer);
    usleep(20000);

    /// Defines sampling frequency at 0x95 = 500Hz, 0x96 = 250Hz. There is a discrepancy between this and the documentation.
    hexes[0] = 0x41;
    hexes[1] = 0x00;
    hexes[2] = 0x96;
    changeBuff(hexes, fd, transfer);
    usleep(20000);

    /// This sequence allows for NO TESTING SEQUENCE INTERNALLY. If need to change testing this is the line that might need to be changed.
    hexes[0] = 0x42;
    hexes[1] = 0x00;
    hexes[2] = 0xC0;
    changeBuff(hexes, fd, transfer);
    usleep(20000);

    /// This sequence establishes NO P-channel used for the BIAS estimation/derivation.
    hexes[0] = 0x4D;
    hexes[1] = 0x00;
    hexes[2] = 0x00;
    changeBuff(hexes, fd, transfer);
    usleep(20000);

    /// This sequence establishes NO N-channel used for the BIAS estimation/derivation.
    hexes[0] = 0x4E;
    hexes[1] = 0x00;
    hexes[2] = 0x00;
    changeBuff(hexes, fd, transfer);
    usleep(20000);

    /// This sequence established the amplifier inputs shorted.
    hexes[0] = 0x45;
    hexes[1] = 0x07;
    hexes[2] = 0x01;
    hexes[3] = 0x01;
    hexes[4] = 0x01;
    hexes[5] = 0x01;
    hexes[6] = 0x01;
    hexes[7] = 0x01;
    hexes[8] = 0x01;
    hexes[9] = 0x01;
    changeBuff(hexes, fd, transfer);
    usleep(20000);

    /// The sequence enables the "read data continously" mode.
    hexes[0] = 0x10;
    hexes[1] = 0x00;
    hexes[2] = 0x00;
    hexes[3] = 0x00;
    hexes[4] = 0x00;
    hexes[5] = 0x00;
    hexes[6] = 0x00;
    hexes[7] = 0x00;
    hexes[8] = 0x00;
    hexes[9] = 0x00;
    changeBuff(hexes, fd, transfer);
    usleep(20000);

    /// The sequence allows to start reading data from the amplifier.
    hexes[0] = 0x08;
    changeBuff(hexes, fd, transfer);
    usleep(20000);

    /// The 0x11 command stops the continuously reading of data.
    hexes[0] = 0x11;
    changeBuff(hexes, fd, transfer);
}
/***
 * METHOD: The startEegStream() is the sequence of commands needed to set up the amplifier for all data collection and interaction. Please look at the manual for each command's meaning.
 * PARAMETERS:  int fd - File that is being used for SPI purposes.
 *              spi_ioc_transfer *transfer - Transfer struct needed for SPI purposes.
 * RETURN: This method returns nothing, just sends the message to the appropriate bus.
 ***/
void Hardware_Interface::startEegStream(int fd, spi_ioc_transfer *transfer)
{
    /// The following sequence  from line 218 until the next changeBuff() call will be to set up the gains of each EEG and EOG channel.
    ///  Ch 1-5 have a gain of 24 (0x68) and Ch6-8 have a gain of 1 (0x08)
    hexes[0] = 0x45;
    hexes[1] = 0x07;

    /// The lines access the setting inserted by the user on the command line. These can be hardcoded if needed.
    uint8_t a_eeg = this->gainHexValues[this->ampEeg];
    uint8_t a_eog = this->gainHexValues[this->ampEog];

    /// Ending the sequence of setting up the gains.
    hexes[2] = a_eeg;
    hexes[3] = a_eeg;
    hexes[4] = a_eeg;
    hexes[5] = a_eeg;
    hexes[6] = a_eeg;
    hexes[7] = a_eog;
    hexes[8] = a_eog;
    hexes[9] = a_eog;
    changeBuff(hexes, fd, transfer);
    usleep(20000);

    // The below lines are for reading the receiver buffer; This is for debugging purposes please do not DELETE only comment out.
    hexes[0] = 0x25;
    hexes[1] = 0x07;
    hexes[2] = 0x00;
    hexes[3] = 0x00;
    hexes[4] = 0x00;
    hexes[5] = 0x00;
    hexes[6] = 0x00;
    hexes[7] = 0x00;
    hexes[8] = 0x00;
    hexes[9] = 0x00;
    changeBuff(hexes, fd, transfer);
    usleep(20000);

    /// Uncomment the following printf line to see the values intilized when doing set up.
    printf("%.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x", rx_buff_2[0], rx_buff_2[1], rx_buff_2[2], rx_buff_2[3], rx_buff_2[4], rx_buff_2[5], rx_buff_2[6], rx_buff_2[7], rx_buff_2[8], rx_buff_2[9], rx_buff_2[10], rx_buff_2[11], rx_buff_2[12], rx_buff_2[13], rx_buff_2[14], rx_buff_2[15], rx_buff_2[16], rx_buff_2[17], rx_buff_2[18], rx_buff_2[19], rx_buff_2[20], rx_buff_2[21], rx_buff_2[22], rx_buff_2[23], rx_buff_2[24], rx_buff_2[25], rx_buff_2[26]);

    /// The following sequence is to register to control if SRB1, reference, is connected to all inverting inputs. This is used in positive mode. In this case SRB1 is connected.
    hexes[0] = 0x55;
    hexes[1] = 0x00;
    hexes[2] = 0x20;
    hexes[3] = 0x00;
    hexes[4] = 0x00;
    hexes[5] = 0x00;
    hexes[6] = 0x00;
    hexes[7] = 0x00;
    hexes[8] = 0x00;
    hexes[9] = 0x00;
    changeBuff(hexes, fd, transfer);
    usleep(20000);

    /// All positive channels used for bias derivation. This can be adjusted depending on the case/application.
    hexes[0] = 0x4D;
    hexes[1] = 0x00;
    hexes[2] = 0xFF;
    changeBuff(hexes, fd, transfer);
    usleep(20000);

    /// All negative channels used for bias derivation. This can be adjusted depending on the case/application.
    hexes[0] = 0x4E;
    changeBuff(hexes, fd, transfer);
    usleep(20000);

    /// Enable internal reference buffer for bias estimation. Bias internal estimation is enable. This can be adapted depending of application/case.
    hexes[0] = 0x43;
    hexes[2] = 0xEC; // CHANGE 2 JOSE
    changeBuff(hexes, fd, transfer);
    sleep(0.20);

    /// Read data discrete mode. Could be changed for 0x10 which would be reading data continously.
    hexes[0] = 0x10;
    hexes[2] = 0x00;
    changeBuff(hexes, fd, transfer);
    usleep(20000);

    /// Command to start reading data.
    hexes[0] = 0x08;
    changeBuff(hexes, fd, transfer);
    usleep(20000);
}
/***
 * METHOD: The measureEEGEOG() is the sequence of commands needed to start streaming EEG and EOG.
 * PARAMETERS:  int fd - File that is being used for SPI purposes.
 *              spi_ioc_transfer *transfer - Transfer struct needed for SPI purposes.
 * RETURN: This method returns nothing, just sends the message to the appropriate bus.
 ***/
void Hardware_Interface::measureEEGEOG(int fd, spi_ioc_transfer *transfer, Channel<Eigen::MatrixXd> *channel1, Channel<Eigen::MatrixXd> *channel2)
{
    // cout << "Running measure EEG and EOG" << endl;
    // Connect to Python for BLE transmission (if running debug)
    connectToPythonServer();
    if (!pythonConnected)
    {
        std::cerr << "Failed to connect to Python server in measureEEGEOG" << std::endl;
        // return;
    }
    // Used for IMU collection
    // gpio_input = libsoc_gpio_request(GPIO_INPUT, LS_GPIO_SHARED);
    // libsoc_gpio_set_direction(gpio_input, INPUT);
    // libsoc_gpio_set_edge(gpio_input, FALLING);

    // // RUN AGAIN JUST IN CASE IMU DIDNT GET SET RIGHT
    imu_cp.startImu();
    imu_cp.imuSet();
    imu_cp.setSensAcc(6384); // hard coded
    imu_cp.setSensGyr(131);  // hard coded

    const int sampleCount = 25; // collect 25 samples (100 ms worth)

    // FOR BASELINE COLLECTION
    auto startTime = chrono::steady_clock::now();
    auto maxDuration = chrono::minutes(1);
    while (!gV.END_STAGE && !gV.EMERGENCY_STOP)
    {
        // Create voltage matrix for filtering (1 sample x 8 channels)
        Eigen::MatrixXd voltageMatrix(1, 8);
        // for (int k = 0; k < sampleCount; k++)  // commenting out to save data at 250 Hz sampling rate
        //{
        memset(hexes, 0x00, sizeof(hexes));
        hexes[0] = 0x00;
        changeBuff(hexes, fd, transfer);
        usleep(4000); // 250 Hz

        // Get imu data  -> also being collected at 250 Hz
        // cout << "BEFORE TRYING IMU DATA" << endl;
        imu_cp.collect();
        // cout << "BEFORE AFTER TRYING IMU DATA" << endl;
        //  libsoc_gpio_callback_interrupt(gpio_input, &callback_interrupt_handler, (void *)&interrupt_count);

        // Fill num_to_convert with all 8 channels (24 bytes)
        for (int b = 0; b < 24; b++)
        {
            num_to_convert[b] = rx_buff_2[3 + b];
        }
        toVoltage(num_to_convert);

        for (int ch = 0; ch < 8; ch++)
        {
            voltageMatrix(0, ch) = volts[ch];
        }
        //}

        // Apply filtering (HPF -> H-inf -> LPF for EEG channels)
        Eigen::MatrixXd filteredMatrix = FilterVoltage(voltageMatrix, 8.0);
        // TEST(PRINT CONTENTS OF CHANNEL MATRIX)------------------------------------------------------
        // cout << "PRINTING EEG AND EOG VALUES" << endl;
        // for (int i = 0; i < filteredMatrix.rows(); i++)
        // {
        //     for (int j = 0; j < filteredMatrix.cols(); j++)
        //     {
        //         cout << filteredMatrix(i, j) << " ";
        //     }
        //     cout << endl;
        // }
        // cout << endl;
        //---------------------------------------------------------

        // Get most recent imu data element in s vector
        string lastData = imu_cp.s.back();
        // cout << "COLLECTED IMU DATA: " << lastData << endl;
        //  Parse the string
        //  Format is: "IMU: accel_x accel_y accel_z gyro_x gyro_y gyro_z "
        std::istringstream iss(lastData);
        std::vector<double> values;
        string word;

        // Skip "IMU:" prefix
        iss >> word;

        // Read all 6 float values
        double val;
        while (iss >> val)
        {
            values.push_back(val);
        }

        // Extract accelerometer XYZ (first 3 values)
        double accel_x = values[0];
        double accel_y = values[1];
        double accel_z = values[2];
        if (accel_x == 0.0 && accel_y == 0.0 && accel_z == 0.0 && !debug) // dont let this print during debug eeg
        {
            cout << "[ERROR] IMU VALUES ARE ZERO" << endl;
            sendUpdateToPython("ERROR", "ZEROS"); // sends error message: ERROR=ZEROS
        }
        // cout << "EXTRACTED ACCELEROMETER XYZ: " << accel_x << ", " << accel_y << ", " << accel_z << endl;

        // Collect EEG Data -> no longer used
        Eigen::MatrixXd EEG_Data = filteredMatrix.leftCols(5);

        // FOR BASELINE
        bool baselineDone = ((gV.THERAPY_STAGE == "stare") && (chrono::steady_clock::now() - startTime >= maxDuration));

        if (debug) // for debug eeg mode
        {
            // Send filteredMatrix containing all eeg and eog channels
            if (pythonConnected)
            {
                sendEEGEOGToApp(filteredMatrix);
            }
        }
        else
        {
            // Compute Matrix to send to other modules
            Eigen::MatrixXd outputMatrix;
            if (module == "callibration_collection")
                outputMatrix.resize(1, 14); // 1 row, 14 elements  (all 8 channels, imu accel xyz (3 cols), position, imagine, move)
            else                            // assumes testing_svm is called
                outputMatrix.resize(1, 15); // 1 row, 15 elements (all 8 channels,  imu accel xyz (3 cols), position, move_pr, imagine, move)

            // if (gV.IMAGINE_MOVEMENT || gV.FIXATE || gV.MOVE)
            if (true) // CHANGE BY JULIO R SIBRIAN
            {
                if (gV.END_OF_TRIAL || baselineDone) // checks if trial ended OR if baseline time has completed
                {
                    // outputMatrix(0, 0) = 9999; // pass terminal value
                    outputMatrix.setConstant(9999);
                    usleep(50000); // sleep for 50 ms
                }
                else
                {
                    outputMatrix.block(0, 0, 1, 8) = filteredMatrix; // pass EEG and EOG data in first 8 columns
                    outputMatrix(0, 8) = accel_x;                    // imu accel x
                    outputMatrix(0, 9) = accel_y;                    // imu accel y
                    outputMatrix(0, 10) = accel_z;                   // imu accel z
                    // Set 11th index val (position)
                    if (module == "callibration_collection")
                    {
                        if (gV.TASK == "flexion")
                            outputMatrix(0, 11) = 0 - stoi(gV.POSITION);
                        else // (gV.TASK == "centerout") //DEFAULT
                            outputMatrix(0, 11) = 50 - stoi(gV.POSITION);
                        outputMatrix(0, 12) = gV.IMAGINE_MOVEMENT ? 1 : 0;
                        outputMatrix(0, 13) = gV.MOVE ? 1 : 0;
                    }
                    else // if module is testing_svm
                    {
                        outputMatrix(0, 11) = stoi(gV.POSITION);
                        outputMatrix(0, 12) = gV.MOVEMENT_PREDICTED ? 1 : 0;
                        outputMatrix(0, 13) = gV.IMAGINE_MOVEMENT ? 1 : 0;
                        outputMatrix(0, 14) = gV.MOVE ? 1 : 0;
                    }
                }
            }
            else
            {
                outputMatrix.setZero(); // send all 0s
            }

            // Send data to channels
            if (channel1 != nullptr)
                channel1->send(outputMatrix);
            if (channel2 != nullptr)
                channel2->send(outputMatrix);

            // TEST(PRINT CONTENTS OF SENT MATRIX)------------------------------------------------------
            // cout << "PRINTING OUTPUT MATRIX SENT TO MODULES" << endl;
            // for (int i = 0; i < outputMatrix.rows(); i++)
            // {
            //     for (int j = 0; j < outputMatrix.cols(); j++)
            //     {
            //         cout << outputMatrix(i, j) << " ";
            //     }
            //     cout << endl;
            // }
            // cout << endl;
            //---------------------------------------------------------
        }

        // FOR BASELINE -> Check if a minute has passed
        if (baselineDone)
        {
            cout << "[ENDING]Baseline collection is complete, now breaking" << endl;
            break;
        }
    }
    cout << "[ENDING]Measure eeg and eog" << endl;

    disconnectPython();
}
// Filters both EOG and EEG channels
Eigen::MatrixXd Hardware_Interface::FilterVoltage(const Eigen::MatrixXd &voltageMatrix, double HighBound)
{
    int rows = voltageMatrix.rows(); // samples (25)
    int cols = voltageMatrix.cols(); // channels (8)

    Eigen::MatrixXd outputMatrix(rows, cols);

    // === Process all 8 channels (EEG + EOG) ===

    // Step 1: High-pass filter
    Eigen::MatrixXd hpFiltered(rows, cols); // changed from 5 to cols
    for (int ch = 0; ch < cols; ch++)       // changed from 5 to cols
    {
        Filter filteringHP(HPF, 2, 250.0, HighBound);
        for (int i = 0; i < rows; i++)
        {
            hpFiltered(i, ch) = filteringHP.do_sample(voltageMatrix(i, ch));
        }
    }

    // Step 2: H-infinity filter (sample by sample)
    Eigen::MatrixXd hinfFiltered(rows, cols); // changed from 5 to cols

    double gamma = 0.1;
    double q = 0.001;
    // Rf must be 1x3 (row vector) so that Rf.transpose() becomes 3x1 (column vector)
    Eigen::MatrixXd Rf = Eigen::MatrixXd::Zero(1, 3);
    Rf(0, 0) = 1.0; // [1, 0, 0]

    for (int sample = 0; sample < rows; sample++)
    {
        // Yf should be numChannels x 1 so (8x1) instead of (5x1)
        Eigen::VectorXd Yf = hpFiltered.row(sample).transpose(); // should be 8x1 now not 5x1

        try
        {
            ::FilterResult hinfResult = Hinf_RT_Filter_v2016_2_11(
                Yf, // 8x1 (8 channels) - changed from 5x1
                Rf, // 3x1 (3 states)
                gamma,
                Pt1, // 24x3 (8 channels x 3 states, by 3) - changed from 15x3 (5 channels × 3 states, by 3)
                wh,  // 3x8 (3 states x 8 channels) - changed from 3x5 (3 states × 5 channels)
                q);

            // Update state for next sample
            Pt1 = hinfResult.Pt;
            wh = hinfResult.wh;

            // Store result
            if (hinfResult.shsh.size() == cols) // changed from 5 to cols
            {
                hinfFiltered.row(sample) = hinfResult.shsh.transpose();
            }
            else
            {
                std::cerr << "H∞ size mismatch at sample " << sample << "\n";
                hinfFiltered.row(sample) = Yf.transpose();
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "H∞ error at sample " << sample << ": " << e.what() << "\n";
            hinfFiltered.row(sample) = Yf.transpose();
        }
    }

    // Step 3: Low-pass filter
    for (int ch = 0; ch < cols; ch++) // changed from 5 to cols
    {
        Filter filteringLP(LPF, 2, 250.0, 30.0);
        for (int i = 0; i < rows; i++)
        {
            outputMatrix(i, ch) = filteringLP.do_sample(hinfFiltered(i, ch));
        }
    }
    // No longer doing this
    // === Copy EOG channels (6-8) without filtering ===
    // for (int ch = 5; ch < cols; ch++)
    // {
    //     outputMatrix.col(ch) = voltageMatrix.col(ch);
    // }

    return outputMatrix;
}

void Hardware_Interface::toVoltage(uint8_t number[])
{
    double gain = 24.0;
    double multiplier = (9.0 / gain) / pow(2, 24); // Conversion Factor

    for (int i = 0; i < 8; i++)
    {
        int index_one = i * 3;
        int index_two = index_one + 1;
        int index_three = index_one + 2;

        uint8_t number_one = number[index_one];
        uint8_t number_two = number[index_two];
        uint8_t number_three = number[index_three];

        bool is_positive = number_one <= 0x7F;

        uint8_t number_volts[4];
        number_volts[0] = is_positive ? 0x00 : 0xFF;
        number_volts[1] = number_one;
        number_volts[2] = number_two;
        number_volts[3] = number_three;

        // Combine into signed 32-bit int
        int32_t signed_value = (int32_t)((number_volts[0] << 24) |
                                         (number_volts[1] << 16) |
                                         (number_volts[2] << 8) |
                                         number_volts[3]);

        volts[i] = signed_value * multiplier;
    }
}

Eigen::MatrixXd Hardware_Interface::HInfFilter(const Eigen::MatrixXd &voltageMatrix)
{
    int rows = voltageMatrix.rows();
    int cols = voltageMatrix.cols();

    // Create the H-infinity filtered matrix with same dimensions
    Eigen::MatrixXd hinfinityMatrix(rows, cols);

    // Apply H-infinity filter to each channel (column)
    for (int ch = 0; ch < cols; ch++)
    {
        for (int sample = 0; sample < rows; sample++)
        {
            // For now, just copy the input (placeholder for actual H-inf algorithm)
            hinfinityMatrix(sample, ch) = voltageMatrix(sample, ch);
        }
    }

    return hinfinityMatrix;
}
void Hardware_Interface::testEegStream(int fd, spi_ioc_transfer *transfer)
{
    /// The following sequence is to stop reading in continous mode.
    hexes[0] = 0x11;
    hexes[1] = 0x00;
    hexes[2] = 0x00;
    hexes[3] = 0x00;
    hexes[4] = 0x00;
    hexes[5] = 0x00;
    hexes[6] = 0x00;
    hexes[7] = 0x00;
    hexes[8] = 0x00;
    hexes[9] = 0x00;
    changeBuff(hexes, fd, transfer);
    usleep(20000);

    /// This sequence defines that the test signal is going to be generated internally. Notifying the corresponding register of the test mode.
    hexes[0] = 0x42;
    hexes[2] = 0xD0;
    changeBuff(hexes, fd, transfer);
    usleep(20000);

    /// The following sequence  from line 218 until the next changeBuff() call will be to set up the gains of each EEG and EOG channel.
    ///  Ch 1-5 have a gain of 24 (0x68) and Ch6-8 have a gain of 1 (0x08)
    hexes[0] = 0x45;
    hexes[1] = 0x07;

    /// The lines access the setting inserted by the user on the command line. These can be hardcoded if needed.
    uint8_t a_eeg = this->gainHexValues[this->ampEeg];
    uint8_t a_eog = this->gainHexValues[this->ampEog];

    /// Ending the sequence of setting up the gains.
    hexes[2] = a_eeg;
    hexes[3] = a_eeg;
    hexes[4] = a_eeg;
    hexes[5] = a_eeg;
    hexes[6] = a_eeg;
    hexes[7] = a_eog;
    hexes[8] = a_eog;
    hexes[9] = a_eog;
    changeBuff(hexes, fd, transfer);
    usleep(20000);

    /// Print statement to verify the gains; can be uncommented if needed.
    //        printf("%.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x", rx_buff_2[0],rx_buff_2[1],rx_buff_2[2],rx_buff_2[3],rx_buff_2[4],rx_buff_2[5],rx_buff_2[6],rx_buff_2[7],rx_buff_2[8],rx_buff_2[9],rx_buff_2[10],rx_buff_2[11],rx_buff_2[12],rx_buff_2[13],rx_buff_2[14],rx_buff_2[15],rx_buff_2[16],rx_buff_2[17], rx_buff_2[18],rx_buff_2[19],rx_buff_2[20],rx_buff_2[21],rx_buff_2[22],rx_buff_2[23],rx_buff_2[24],rx_buff_2[25],rx_buff_2[26],rx_buff_2[27]);

    /// The following sequence is to start reading the data continously.
    hexes[0] = 0x10;
    hexes[1] = 0x00;
    hexes[2] = 0x00;
    hexes[3] = 0x00;
    hexes[4] = 0x00;
    hexes[5] = 0x00;
    hexes[6] = 0x00;
    hexes[7] = 0x00;
    hexes[8] = 0x00;
    hexes[9] = 0x00;
    changeBuff(hexes, fd, transfer);
    usleep(20000);

    /// Start reading data
    hexes[0] = 0x08;
    changeBuff(hexes, fd, transfer);
    usleep(20000);
}
/***
 *METHOD: The startUp() method is a helper method that calls startUpSequence() and startEegStream().
 * PARAMETERS : None
 * RETURN : None
 * **/
void Hardware_Interface::startUp()
{
    startUpSequence(fd, &trx);
    startEegStream(fd, &trx);
}
// below is test version
// void Hardware_Interface::callEEG(Channel<Eigen::MatrixXd> *channel1, Channel<Eigen::MatrixXd> *channel2, std::atomic<bool> *doneFlag)
void Hardware_Interface::callEEG(Channel<Eigen::MatrixXd> *channel1, Channel<Eigen::MatrixXd> *channel2)
{
    // measureEEGEOG(fd, &trx, channel1, channel2, doneFlag); //test
    measureEEGEOG(fd, &trx, channel1, channel2);
}
void Hardware_Interface::testAmp()
{
    testEegStream(fd, &trx);
}
void Hardware_Interface::sendCommand(const uint8_t *command)
{
    if (!command)
    {
        std::cerr << "Null command passed to sendCommand()" << std::endl;
        return;
    }

    for (int i = 0; i < 27; ++i)
    {
        tx_buff_2[i] = command[i];
    }

    if (fd < 0)
    {
        std::cerr << "Invalid file descriptor in sendCommand()" << std::endl;
        return;
    }

    ioctl(fd, SPI_IOC_MESSAGE(1), &trx);
}
const unsigned char *Hardware_Interface::getRawRx() const
{
    return rx_buff_2;
}

void Hardware_Interface::changeBuff(unsigned char *hex, int fd, spi_ioc_transfer *transfer)
{
    for (int m = 0; m < len_data; m++)
    {
        tx_buff_2[m] = hex[m];
    }
    ioctl(fd, SPI_IOC_MESSAGE(1), transfer);
}
void Hardware_Interface::Arm_setup(double &maxPosition)
{
    cout << "[RUNNING] Arm setup" << endl;

    int samplingRate = 50;
    gV.TARGET = "100";
    gV.HOME = "0";
    // string ipAddress = "192.168.0.120"; // Replace with your robot arm's IP -> USE THIS
    string ipAddress = "127.0.0.1"; // LOCALHOST IP ADDRESS -> USED FOR AT HOME TESTS ONLY
    int port = 11999;               // Replace with your robot arm's port
    maxPosition = 0.0;

    // Send target to app
    sendUpdateToPython("target", gV.TARGET);

    for (int i = 0; i < 5000 / samplingRate; i++)
    {
        if (gV.END_STAGE || gV.EMERGENCY_STOP)
        {
            // stop everyting
            std::exit(0);
        }
        usleep(samplingRate * 1000);
    }

    // RESET
    gV.REST = false;
    gV.MOVE = false;
    gV.IMAGINE_MOVEMENT = false;
    gV.FIXATE = false;
    gV.POSITION = (0 > 100) ? "100" : "0";

    // Send position to app
    sendUpdateToPython("position", gV.POSITION);

    // Create socket
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
    {
        perror("Can't create socket");
        return;
    }

    // Connect
    sockaddr_in hint{};
    hint.sin_family = AF_INET;
    hint.sin_port = htons(port);
    inet_pton(AF_INET, ipAddress.c_str(), &hint.sin_addr);

    if (connect(sock, (sockaddr *)&hint, sizeof(hint)) < 0) // IF CANT CONNECT, ARM ISNT CONNECTED SO INFORM APP
    {
        perror("Can't connect to server");
        sendUpdateToPython("ERROR", "ARM"); // sends error message: ERROR=ARM
        close(sock);
        return;
    }

    // Send one command
    std::string msgToSend = "3\t0.500000\t\n";

    sendWithTimeout(sock, msgToSend, 5000); // only timeout this send

    std::string response;
    if (recvImmediate(sock, response, 5000, 1000))
    {
        // cout << "First receive successful";
    }
    else
    {
        cerr << "No data received or error occurred\n";
    }
    for (int i = 0; i < 3000 / samplingRate; i++)
    {
        if (gV.END_STAGE || gV.EMERGENCY_STOP)
        {
            // stop everyting
            std::exit(0);
        }
        usleep(samplingRate * 1000);
    }

    gV.THERAPY_STAGE = "prepareForMax";

    // Send therapy_stage to app
    sendUpdateToPython("therapy_stage", gV.THERAPY_STAGE);

    std::string msgToSendtwo = "7\t0.500000\t\n";
    sendWithTimeout(sock, msgToSendtwo, 5000); // only timeout this send
    std::string responsetwo;
    if (recvImmediate(sock, responsetwo, 5000, 1000))
    {
        // cout << "Second receive successful";
    }
    else
    {
        cerr << "No data received or error occurred\n";
    }

    usleep(3000 * 1000);
    cout << "Start now" << endl;
    std::string msgToSendthree = "0\t0.500000\t\n";
    for (int i = 0; i < 10000 / samplingRate; i++)
    {
        gV.THERAPY_STAGE = "max";

        // Send therapy_stage to app
        sendUpdateToPython("therapy_stage", gV.THERAPY_STAGE);

        send(sock, msgToSendthree.c_str(), msgToSendthree.size(), 0);
        string responsethree;
        if (recvImmediate(sock, responsethree, 5000, 28))
        {
            // Cleanup
            responsethree.erase(std::remove(responsethree.begin(), responsethree.end(), '\r'), responsethree.end());
            responsethree.erase(std::remove(responsethree.begin(), responsethree.end(), '\n'), responsethree.end());

            // Replace ".." with "."
            size_t pos = 0;
            while ((pos = responsethree.find("..", pos)) != std::string::npos)
            {
                responsethree.replace(pos, 2, ".");
                pos += 1;
            }

            // Extract third value
            std::string thirdValue = extractThirdToken(responsethree);
            double thirdNum = std::stod(thirdValue);
            if (thirdNum < 1.0)
            {
                thirdNum = 1.0;
            }
            gV.POSITION = (thirdNum > 100) ? "100" : thirdValue;

            // Send position to app
            sendUpdateToPython("position", gV.POSITION);

            if (thirdNum > maxPosition)
            {
                maxPosition = thirdNum;
            }
            // cout << "Current position: " << thirdNum << endl;
        }

        usleep(50 * 1000);
    }
    for (int i = 0; i < 4000 / samplingRate; i++)
    {
        if (gV.END_STAGE || gV.EMERGENCY_STOP)
        {
            // stop everyting
            std::exit(0);
        }
        usleep(samplingRate * 1000);
        gV.THERAPY_STAGE = "soon";

        // Send therapy_stage to app
        sendUpdateToPython("therapy_stage", gV.THERAPY_STAGE);
    }
    std::string msgToSendfour = "4\t0.500000\t\n";
    std::string responsefour;
    sendWithTimeout(sock, msgToSendfour, 5000);
    if (recvImmediate(sock, responsefour, 1000, 100))
    {
        // cout << "fourth data received succesfully" << endl;
        // cout << "Fourth response: " << responsefour << endl;
    }
    else
    {
        cerr << "Fourth data failed to receive";
    }
    std::string msgToSendfifth = "7\t0.500000\t\n";
    sendWithTimeout(sock, msgToSendfifth, 5000);
    std::string responsefifth;
    if (recvImmediate(sock, responsefifth, 1000, 100))
    {
        // cout << "fifth data received succesfully" << endl;
        // cout << "Fifth response: " << responsefifth << endl;
    }
    else
    {
        cerr << "Fifth data failed to receive";
    }
    // cout << "\n---" << endl;
    cout << "Max position reaced: " << maxPosition << endl;
    // cout << "---\n"
    //<< endl;
    // Cleanup
    // close(sock);
}

void Hardware_Interface::debugArm()
{
    cout << "[RUNNING] Debug arm Module" << endl;

    int samplingRate = 50;
    // string ipAddress = "192.168.0.120"; // Replace with your robot arm's IP -> USE THIS
    string ipAddress = "127.0.0.1"; // LOCALHOST IP ADDRESS -> USED FOR AT HOME TESTS ONLY
    int port = 11999;               // Replace with your robot arm's port

    gV.POSITION = (0 > 100) ? "100" : "0";
    gV.HOME = "0";

    // Send position to app
    sendUpdateToPython("position", gV.POSITION);

    // Create socket
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
    {
        perror("Can't create socket");
        return;
    }

    // Connect
    sockaddr_in hint{};
    hint.sin_family = AF_INET;
    hint.sin_port = htons(port);
    inet_pton(AF_INET, ipAddress.c_str(), &hint.sin_addr);

    if (connect(sock, (sockaddr *)&hint, sizeof(hint)) < 0) // IF CANT CONNECT, ARM ISNT CONNECTED SO INFORM APP
    {
        perror("Can't connect to server");
        sendUpdateToPython("ERROR", "ARM"); // sends error message: ERROR=ARM
        close(sock);
        return;
    }

    // Send one command
    std::string msgToSend = "3\t0.500000\t\n";

    sendWithTimeout(sock, msgToSend, 5000); // only timeout this send

    std::string response;
    if (recvImmediate(sock, response, 5000, 1000))
    {
        // cout << "First receive successful";
    }
    else
    {
        cerr << "No data received or error occurred\n";
    }

    std::string msgToSendtwo = "7\t0.500000\t\n";
    sendWithTimeout(sock, msgToSendtwo, 5000); // only timeout this send
    std::string responsetwo;
    if (recvImmediate(sock, responsetwo, 5000, 1000))
    {
        // cout << "Second receive successful";
    }
    else
    {
        cerr << "No data received or error occurred\n";
    }

    std::string msgToSendthree = "0\t0.500000\t\n";
    bool stop = false;
    while (!stop)
    {
        if (gV.END_STAGE || gV.EMERGENCY_STOP) // end next iteration
            stop = true;

        send(sock, msgToSendthree.c_str(), msgToSendthree.size(), 0);
        string responsethree;
        if (recvImmediate(sock, responsethree, 5000, 50))
        {
            // Cleanup
            responsethree.erase(std::remove(responsethree.begin(), responsethree.end(), '\r'), responsethree.end());
            responsethree.erase(std::remove(responsethree.begin(), responsethree.end(), '\n'), responsethree.end());

            // Replace ".." with "."
            size_t pos = 0;
            while ((pos = responsethree.find("..", pos)) != std::string::npos)
            {
                responsethree.replace(pos, 2, ".");
                pos += 1;
            }

            // Extract third value
            std::string thirdValue = extractThirdToken(responsethree);
            double thirdNum = std::stod(thirdValue);
            gV.POSITION = (thirdNum > 100) ? "100" : thirdValue;

            // Send position to app
            sendUpdateToPython("position", gV.POSITION);
        }

        usleep(50 * 1000); // arm sampling rate
    }
}

// Helper functions -----------------------------------------------------------------------------------------------
bool Hardware_Interface::recvImmediate(int sock, std::string &out, int timeoutMs, int maxBytesToRead)
{
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(sock, &readfds);

    timeval timeout;
    timeout.tv_sec = timeoutMs / 1000;
    timeout.tv_usec = (timeoutMs % 1000) * 1000;

    int sel = select(sock + 1, &readfds, nullptr, nullptr, &timeout);
    if (sel > 0 && FD_ISSET(sock, &readfds))
    {
        char buf[1000];
        int bytesReceived = recv(sock, buf, maxBytesToRead, 0);
        if (bytesReceived > 0)
        {
            out.assign(buf, bytesReceived);
            return true;
        }
    }
    else if (sel == 0) // WHEN THIS OCCURS, ARM CONNECTION HAS FAILED SO INFORM APP
    {
        cerr << "Timeout: No data within " << timeoutMs << " ms\n";
        sendUpdateToPython("ERROR", "ARM"); // sends error message: ERROR=ARM
    }
    else
    {
        perror("select failed");
    }
    return false;
}

bool Hardware_Interface::sendWithTimeout(int sock, const std::string &data, int timeoutMs)
{
    fd_set writefds;
    FD_ZERO(&writefds);
    FD_SET(sock, &writefds);

    timeval timeout;
    timeout.tv_sec = timeoutMs / 1000;
    timeout.tv_usec = (timeoutMs % 1000) * 1000;

    int sel = select(sock + 1, nullptr, &writefds, nullptr, &timeout);
    if (sel > 0 && FD_ISSET(sock, &writefds))
    {
        int bytesSent = send(sock, data.c_str(), data.size(), 0);
        return bytesSent >= 0;
    }
    cerr << "Send timeout or error\n";
    return false;
}
std::string Hardware_Interface::extractThirdToken(const std::string &str)
{
    std::istringstream iss(str);
    std::string token;
    int index = 0;

    while (iss >> token)
    {
        if (index == 2)
        {
            return token; // third token found
        }
        ++index;
    }

    return ""; // if there weren't at least 3 tokens
}

// PYTHON SERVER FUNCTIONS -----------------------------------------------------------------------------------------------

void Hardware_Interface::connectToPythonServer()
{
    pythonSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (pythonSocket < 0)
    {
        std::cerr << "❌ Failed to create socket for Python server" << std::endl;
        pythonConnected = false;
        return;
    }

    sockaddr_in hint{};
    hint.sin_family = AF_INET;
    hint.sin_port = htons(65432);                    // Python server port
    inet_pton(AF_INET, "127.0.0.1", &hint.sin_addr); // localhost

    // std::cout << "🔌 Connecting to Python BLE server..." << std::endl; // not needed

    if (connect(pythonSocket, (sockaddr *)&hint, sizeof(hint)) < 0)
    {
        std::cerr << "❌ Failed to connect to Python server on port 65432" << std::endl;
        std::cerr << "   Make sure server.py is running!" << std::endl;
        close(pythonSocket);
        pythonConnected = false;
        return;
    }

    std::cout << "✅ Connected to Python BLE server on port 65432" << std::endl;
    pythonConnected = true;
}

void Hardware_Interface::sendPositionToPython(double position)
{
    if (!pythonConnected || pythonSocket < 0)
    {
        return;
    }

    // Send just the position value as a string with newline
    std::string message = std::to_string(position) + "\n";

    ssize_t sent = send(pythonSocket, message.c_str(), message.size(), 0);
    if (sent < 0)
    {
        std::cerr << "❌ Failed to send position to Python server" << std::endl;
        pythonConnected = false;
    }
    else
    {
        std::cout << "📤 Sent position to BLE: " << position << std::endl;
    }
}

void Hardware_Interface::disconnectPython()
{
    if (pythonSocket >= 0)
    {
        close(pythonSocket);
        pythonSocket = -1;
    }
    pythonConnected = false;
    std::cout << "🔌 Disconnected from Python server" << std::endl;
}
void Hardware_Interface::sendCommandToPython(const std::string &command)
{
    if (!pythonConnected || pythonSocket < 0)
    {
        return;
    }

    std::string message = command + "\n";
    ssize_t sent = send(pythonSocket, message.c_str(), message.size(), 0);

    if (sent < 0)
    {
        std::cerr << "❌ Failed to send command: " << command << std::endl;
    }
    else
    {
        // std::cout << "📤 Sent command: " << command << std::endl;
    }
}

void Hardware_Interface::sendFullDataToPython(double eeg1, double eeg2, double eeg3,
                                              double eeg4, double eeg5, double position,
                                              int imagine, int move)
{
    if (!pythonConnected || pythonSocket < 0)
        return;

    // Format: "eeg1,eeg2,eeg3,eeg4,eeg5,position,imagine,move\n"
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(9);
    oss << eeg1 << "," << eeg2 << "," << eeg3 << ","
        << eeg4 << "," << eeg5 << ","
        << position << "," << imagine << "," << move << "\n";

    std::string message = oss.str();
    ssize_t sent = send(pythonSocket, message.c_str(), message.size(), 0);

    if (sent < 0)
    {
        std::cerr << "❌ Failed to send data to Python" << std::endl;
    }
}
void Hardware_Interface::sendEEGEOGToApp(MatrixXd eeg_eog)
{
    if (!pythonConnected || pythonSocket < 0)
        return;

    std::ostringstream oss;
    // oss << std::fixed << std::setprecision(2);

    for (int i = 0; i < 8; i++)
    {
        oss << eeg_eog(0, i);
        if (i < 7)
            oss << ";";
    }
    oss << "\n";

    std::string message = oss.str();

    // ADD THIS DEBUG LINE: -> NOT NEEDED ANYMORE
    // std::cout << "\n Sending to Python: " << message << std::flush;

    ssize_t sent = send(pythonSocket, message.c_str(), message.size(), 0);

    if (sent < 0)
    {
        std::cerr << "Failed to send EEG and EOG to Python" << std::endl;
    }
}
void Hardware_Interface::debugMode()
{
    debug = true;
    cout << "Running Hardware Interface in debug mode" << endl;
}
void Hardware_Interface::leaveDebugMode()
{
    debug = false;
    cout << "Running Hardware Interface in regular mode" << endl;
}
void Hardware_Interface::setModule(string m)
{
    module = m;
}

void Hardware_Interface::closeTCPConnection() /////////////////////////////////////////TCIP Block
{
    for (int i = 0; i < 10; i++)
    {
        string msgToSend = "0\t0.000000\t\n";
        this->sendWithTimeout(sock, msgToSend, 25000);

        std::string response;
        if (this->recvImmediate(sock, response, 2000, 50))
        {
            // cout << "First receive from protocol successful";
        }
        else
        {
            cerr << "No data received or error occurred\n";
        }
    }
    for (int i = 0; i < 5; i++)
    {
        string msgToSend = "8\t0.500000\t\n";
        this->sendWithTimeout(sock, msgToSend, 2000);

        std::string response;
        if (this->recvImmediate(sock, response, 25000, 50))
        {
            // cout << "First receive from protocol successful";
        }
        else
        {
            cerr << "No data received or error occurred\n";
        }
    }
    // close it
    close(sock);
    cout << "[closeTCPConnection]TCP connection to arm has been closed" << endl;
}