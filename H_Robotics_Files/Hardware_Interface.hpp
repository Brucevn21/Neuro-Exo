#ifndef HARDWARE_INTERFACE_HPP
#define HARDWARE_INTERFACE_HPP

#include <cstdint>
#include <string>
#include <list>
#include <linux/spi/spidev.h>
#include "FIR-filter-class/filt.h"
#include <eigen3/Eigen/Dense>
#include <vector>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <eigen3/Eigen/Core>
#include "channel.hpp" //contains class to act as EEG channel between submodules
#include "imu.hpp"     //contains imu class
#include <thread>
#include <future>
#include <atomic>
#include <filesystem>
#define SPI_DEVICE "/dev/spidev1.0"

using namespace std;
using namespace Eigen;

class Hardware_Interface
{
public:
    /// Nicknames to shorten declaration of matrices
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixXd;
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorXd;
    typedef Eigen::Matrix<double, 1, Eigen::Dynamic> VectorXd2;

    // SPI configuration
    uint32_t len_data = 27;
    uint8_t tx_buff_2[27];
    uint8_t rx_buff_2[27];
    uint32_t spi_speed = 16000000;
    int fd;
    int ret;
    double volts[8]; // Stores converted voltages per channel
    struct spi_ioc_transfer trx;
    uint32_t scratch32;
    Filter *filtering;
    Filter *high_pass;
    uint8_t num_to_convert[4];
    list<string> s;

    list<VectorXd> filtered;

    list<int> times;

    VectorXd eeg_data = VectorXd(5, 1);
    VectorXd2 eog_data = VectorXd2(1, 3);

    MatrixXd Pt1 = MatrixXd(24, 3); // 24x3 (8 channels x 3 states, by 3) - changed from 15x3 (5 channels × 3 states, by 3)

    MatrixXd Pt = Pt1.block<3, 3>(0, 0, 3, 3);
    MatrixXd wh = MatrixXd(3, 5); // 3x8 (3 states x 8 channels) - changed from 3x5 (3 states × 5 channels) <- Initialize wh matrix
    int hinfSampleCount = 0;      // used in hinf function to reset Pt1 and wh

    // Command + gain control
    uint8_t hexes[27] = {0};
    int gainValues[7] = {1, 2, 4, 6, 8, 12, 24};
    uint8_t gainHexValues[7] = {0x08, 0x18, 0x28, 0x38, 0x48, 0x58, 0x68};
    int ampEog;
    int ampEeg;
    int sock; // used as handle to arm throughout the trials

    // Bluetooth (BlueZ RFCOMM)
    int btSocket = -1;
    bool btConnected = false;
    std::string btAddress = "";
    uint8_t btChannel = 1;

    // IMU
    Imu imu_cp;

    void startAmp();
    void sendCommand(const uint8_t *command);
    void setAmpEeg(int amplification);
    int getAmpEeg();
    void setAmpEog(int amplification);
    int getAmpEog();
    void testEegStream(int fd, spi_ioc_transfer *transfer);
    // Arm setup
    void Arm_setup(double &maxPosition);
    // Debug arm
    void debugArm();
    // Close connection
    void closeTCPConnection();

    Eigen::MatrixXd FilterVoltage(const Eigen::MatrixXd &voltageMatrix, double HighBound);
    Eigen::MatrixXd HInfFilter(const Eigen::MatrixXd &voltageMatrix);
    const unsigned char *getRawRx() const;
    void changeBuff(unsigned char *hex, int fd, spi_ioc_transfer *transfer);

    void startUp();
    void startUpSequence(int fd, spi_ioc_transfer *transfer);
    void startEegStream(int fd, spi_ioc_transfer *transfer);
    // Get EEG Data
    void measureEEGEOG(int fd, spi_ioc_transfer *transfer, Channel<Eigen::MatrixXd> *channel1 = nullptr, Channel<Eigen::MatrixXd> *channel2 = nullptr);
    void callEEG(Channel<Eigen::MatrixXd> *channel1 = nullptr, Channel<Eigen::MatrixXd> *channel2 = nullptr);
    void testAmp();
    void toVoltage(uint8_t number[]);

    // Socket communication to Python BLE server------------------------------------------------
    int pythonSocket;
    bool pythonConnected;
    void sendCommandToPython(const string &command);
    void connectToPythonServer();
    void sendPositionToPython(double position);
    void disconnectPython();
    void sendFullDataToPython(double eeg1, double eeg2, double eeg3,
                              double eeg4, double eeg5, double position,
                              int imagine, int move);
    void sendEEGEOGToApp(MatrixXd eeg_eog);

    // BlueZ RFCOMM Bluetooth helpers ------------------------------------------------------------
    void setBluetoothDevice(const string &bdaddr, uint8_t channel = 1);
    bool connectToBluetoothDevice(const string &bdaddr, uint8_t channel = 1);
    bool sendBluetooth(const string &message);
    void disconnectBluetooth();
    bool receiveBluetooth(std::string &out, int timeoutMs, int maxBytesToRead = 1000);
    // ------------------------------------------------------------------------------------------------

    // Enter debug mode (used when not connected to ARM)
    void debugMode();
    void leaveDebugMode();
    // Let class know which module it belongs to
    void setModule(string m);

    // Helper functions --------------------------------------------------------------------
    bool recvImmediate(int sock, string &out, int timeoutMs, int maxBytesToRead = 1000);
    bool sendWithTimeout(int sock, const string &data, int timeoutMs);
    std::string extractThirdToken(const string &str);
    //----------------------------------------------------------------------------------------------

private:
    bool debug = false;
    string module = "";
};

#endif