#ifndef HARDWARE_INTERFACE_HPP
#define HARDWARE_INTERFACE_HPP

#include <cstdint>
#include <array>
#include <memory>
#include <functional>
#include "BleTrialClient.hpp"
#include <string>
#include <list>
#include <linux/spi/spidev.h>
#include "FIR-filter-class/filt.h"
#include <eigen3/Eigen/Dense>
#include <vector>
#include <sys/types.h>
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
    Hardware_Interface();
    ~Hardware_Interface();
    Hardware_Interface(const Hardware_Interface&) = delete;
    Hardware_Interface& operator=(const Hardware_Interface&) = delete;
    /// Nicknames to shorten declaration of matrices
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixXd;
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorXd;
    typedef Eigen::Matrix<double, 1, Eigen::Dynamic> VectorXd2;

    // SPI configuration
    uint32_t len_data = 27;
    uint8_t tx_buff_2[27];
    uint8_t rx_buff_2[27];
    uint32_t spi_speed = 16000000;
    int fd = -1;
    int ret;
    double volts[8]; // Stores converted voltages per channel
    struct spi_ioc_transfer trx{};
    uint32_t scratch32;
    Filter *filtering = nullptr;
    Filter *high_pass = nullptr;
    uint8_t num_to_convert[24]{};
    list<string> s;

    list<VectorXd> filtered;

    list<int> times;

    VectorXd eeg_data = VectorXd(5, 1);
    VectorXd2 eog_data = VectorXd2(1, 3);

    MatrixXd Pt1 = MatrixXd(24, 3); // 24x3 (8 channels x 3 states, by 3) - changed from 15x3 (5 channels × 3 states, by 3)

    MatrixXd Pt = MatrixXd::Zero(3, 3);
    MatrixXd wh = MatrixXd::Zero(3, 8); // Three states for each of eight channels
    int hinfSampleCount = 0;      // used in hinf function to reset Pt1 and wh

    // Command + gain control
    uint8_t hexes[27] = {0};
    int gainValues[7] = {1, 2, 4, 6, 8, 12, 24};
    uint8_t gainHexValues[7] = {0x08, 0x18, 0x28, 0x38, 0x48, 0x58, 0x68};
    int ampEog = 6;
    int ampEeg = 0;
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

    // BlueZ BLE GATT trial protocol. Use one owner thread for arm operations.
    void setBluetoothDevice(const std::string& address, const std::string& adapter = "hci0");
    bool connectToBluetoothDevice();
    void disconnectBluetooth();
    bool isBluetoothConnected() const;
    bool isBluetoothSimulated() const;
    void calibrateArm();
    void setMaxPosition(double degrees);
    void configureTrial(const neuroexo::TrialSettings& settings);
    void startTrial(uint16_t trial);
    neuroexo::ArmPosition readArmPosition(uint16_t trial);
    void endTrial(uint16_t trial);
    double lastArmPositionDegrees() const { return double(armPositionMilliDegrees_.load()) / neuroexo::SCALE; }
    neuroexo::TrialTiming runArmTrial(
        const neuroexo::TrialSettings& settings, std::chrono::milliseconds duration,
        std::function<void(const neuroexo::ArmPosition&)> onPosition = {},
        std::function<bool()> shouldStop = {});

    // Enter debug mode (used when not connected to ARM)
    void debugMode();
    void leaveDebugMode();
    // Let class know which module it belongs to
    void setModule(string m);

private:
    void requireBluetooth();
    void logEEGEOG(const MatrixXd& values);
    std::unique_ptr<neuroexo::BleTrialClient> ble_;
    std::string btAddress;
    std::string btAdapter = "hci0";
    std::atomic<int32_t> armPositionMilliDegrees_{0};
    std::array<std::unique_ptr<Filter>, 8> highPassFilters_, lowPassFilters_;
    double highPassCutoff_ = -1.0;
    bool debug = false;
    string module = "";
};

#endif