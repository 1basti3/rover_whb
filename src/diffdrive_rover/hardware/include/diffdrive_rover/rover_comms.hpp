#ifndef DIFFDRIVE_ROVER_ROVER_COMMS_HPP
#define DIFFDRIVE_ROVER_ROVER_COMMS_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <cmath>
#include <wiringPi.h>
#include <iostream>
#include <utility>

using namespace LibSerial;

class Lummerland
{
public:
    Lummerland() = default;
    

    void connect()//const std::string &serial_device, int32_t timeout_ms)
    {
        try
        {
            ser.Open("/dev/ttyUSB1");
            ser.SetBaudRate(BaudRate::BAUD_9600);
        }
        catch (const OpenFailed&)
        {
            ser.Open("/dev/ttyUSB0");
            ser.SetBaudRate(BaudRate::BAUD_9600);
        }

        wiringPiSetupGpio(); // Initialize wiringPi to use BCM GPIO numbering

        // Initialize Encoder
        initLeftEncoder(24, 23);  
        initRightEncoder(27, 17);
    }

    void disconnect()
    {
        ser.Close();
    }

    bool connected() const
    {
        return ser.IsOpen();
    }

    double convert(double x, double outMax, double outMin)
    {
        x = x * 60 / (2 * M_PI); // rad/s to RPM
        std::cout << "RPM: " << x << std::endl;
        double retVal = (x - -170) * (outMax - outMin) / (170 - -170) + outMin;
        return retVal;
    }

    std::pair<double, double> ddrIk(double vX, double omega, double L = 0.353, double r = 0.079)
    {
        double left = (vX - (L / 2) * omega) / r;
        double right = (vX + (L / 2) * omega) / r;
        return {left, right};
    }

    void set_motor_values (int left_motors, int right_motors)
    {
        //auto [links, rechts] = ddrIk(speed, omega);

        int leftSpeed = static_cast<int>(convert(left_motors, 127, 1));
        int rightSpeed = static_cast<int>(convert(right_motors, 128, 256));

        std::cout << "Rechts: " << rightSpeed << std::endl;
        std::cout << "Links: " << leftSpeed << std::endl;

        unsigned char leftWheelSpeed = static_cast<unsigned char>(leftSpeed);
        unsigned char rightWheelSpeed = static_cast<unsigned char>(rightSpeed);

        ser.FlushIOBuffers(); // Just in case
        ser.WriteByte(rightWheelSpeed);
        ser.WriteByte(leftWheelSpeed);
    }

    void initLeftEncoder(int leftPin, int rightPin)
    {
        this->leftPin = leftPin;
        this->rightPin = rightPin;
        value = 0;
        state = "00";
        direction = ' ';

        pinMode(leftPin, INPUT);
        pinMode(rightPin, INPUT);
        pullUpDnControl(leftPin, PUD_DOWN);
        pullUpDnControl(rightPin, PUD_DOWN);

        instance = this;
    }

    void read_left_encoder (int &val_1)
    {
        // WiringPi interrupt handling
        wiringPiISR(leftPin, INT_EDGE_BOTH, &Lummerland::transitionOccurredWrapper);
        wiringPiISR(rightPin, INT_EDGE_BOTH, &Lummerland::transitionOccurredWrapper);
        val_1 = getValue();
    }

    void initRightEncoder(int leftPin, int rightPin)
    {
        this->leftPin = leftPin;
        this->rightPin = rightPin;
        value = 0;
        state = "00";
        direction = ' ';

        pinMode(leftPin, INPUT);
        pinMode(rightPin, INPUT);
        pullUpDnControl(leftPin, PUD_DOWN);
        pullUpDnControl(rightPin, PUD_DOWN);

        instance = this;
    }

    void read_right_encoder(int &val_2)
    {
        // WiringPi interrupt handling
        wiringPiISR(leftPin, INT_EDGE_BOTH, &Lummerland::transitionOccurredWrapper);
        wiringPiISR(rightPin, INT_EDGE_BOTH, &Lummerland::transitionOccurredWrapper);
        val_2 = getValue();
    }

private:

    int getValue() const
    {
        return value;
    }

    static void transitionOccurredWrapper()
    {
        if (instance)
        {
            instance->transitionOccurred();
        }
    }

    void transitionOccurred()
    {
        int p1 = digitalRead(leftPin);
        int p2 = digitalRead(rightPin);
        std::string newState = std::to_string(p1) + std::to_string(p2);

        if (state == "00")
        {
            if (newState == "01")
                direction = 'R';
            else if (newState == "10")
                direction = 'L';
        }
        else if (state == "01")
        {
            if (newState == "11")
                direction = 'R';
            else if (newState == "00" && direction == 'L')
                value--;
        }
        else if (state == "10")
        {
            if (newState == "11")
                direction = 'L';
            else if (newState == "00" && direction == 'R')
                value++;
        }
        else if (state == "11")
        {
            if (newState == "01")
                direction = 'L';
            else if (newState == "10")
                direction = 'R';
            else if (newState == "00")
            {
                if (direction == 'L')
                    value--;
                else if (direction == 'R')
                    value++;
            }
        }

        state = newState;
    }

    int leftPin;
    int rightPin;
    int value;
    std::string state;
    char direction;

    static Lummerland* instance; // Static pointer to the current instance

    SerialPort ser;
};

// Initialize static member
Lummerland* Lummerland::instance = nullptr;

#endif //DIFFDRIVE_ROVER_ROVER_COMMS_HPP
