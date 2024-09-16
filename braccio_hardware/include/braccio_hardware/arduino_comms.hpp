#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP

#include <cstring>
#include <sstream>
// #include <cstdlib>
// #include <libserial/SerialPort.h>
#include <iostream>
#include <vector>

#include <termios.h>


class ArduinoComms
{

public:

  ArduinoComms() = default;

  ~ArduinoComms()
  {
    // Implement close function
    /*if (serial_conn_.IsOpen())
    {
      try
      {
        serial_conn_.Close();
      }
      catch(...)
      {
        std::cerr << "Something went wrong while closing connection with serial ports." << std::endl;
      }
      
    }*/
  }

  // ================================
  // Connect function
  // ================================
  bool connect(const std::string &serial_device, int32_t /*baud_rate*/, int32_t timeout_ms)
  {  
    timeout_ms_ = timeout_ms;
    
    SerialPort = open(serial_device.c_str(), O_RDWR);

    if (SerialPort < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ArduinoComms"), "Error %i from open: %s", errno, strerror(errno));
        return false;
    }
    if (tcgetattr(SerialPort, &tty) != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ArduinoComms"), "Error %i from tcgetattr: %s", errno, strerror(errno));
        close(SerialPort);
        return false;
    }

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;
    tty.c_lflag &= ~(IXON | IXOFF | IXANY);
    tty.c_lflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN] = 0;

    speed_t speed = B115200;
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    sleep(2); // delay due to USB serial port flushing management.
    tcflush(SerialPort, TCIOFLUSH);

    if (tcsetattr(SerialPort, TCSANOW, &tty) != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ArduinoComms"), "Error %i from tcgetattr: %s", errno, strerror(errno));
        close(SerialPort);
        return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("ArduinoComms"), "SERIAL PORT OPENED: %d! WAITING...", SerialPort);
    auto t_start = std::chrono::high_resolution_clock::now();
    while(true)
    {
        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_time_ms = std::chrono::duration<double, std::milli> (t_end-t_start).count();
        if (elapsed_time_ms > timeout_ms_)
        {
            break;
        }
    }

    return true;
  }

  // ================================
  // Disconnect function
  // ================================
  void disconnect()
  {
    if (SerialPort == -1)
    {
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("ArduinoComms"), "CLOSING SERIAL PORT: %d! WAITING...", SerialPort);
    
    tcflush(SerialPort, TCIOFLUSH);
    close(SerialPort);

    auto t_start = std::chrono::high_resolution_clock::now();
    while(true)
    {
        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_time_ms = std::chrono::duration<double, std::milli> (t_end-t_start).count();
        if (elapsed_time_ms > 3000)
        {
            break;
        }
    }
    return;
  }

  // ================================
  // Check connection function
  // ================================
  bool connected(bool hw_test)
  {
    // Implement verification function

    unsigned char l[4] = {'<','l','>','\0'};

    WriteToSerial(l, sizeof(uint8_t)*sizeof(l));

    if (hw_test)
    {
      RCLCPP_INFO(rclcpp::get_logger("ArduinoComms"), "=== live command sent ===");
    }

    char res[10] = {};
    uint8_t* v = (uint8_t*)res;

    ReadSerial(hw_test, v, sizeof(uint8_t)*sizeof(res));

    if (hw_test)
    {
      RCLCPP_INFO(rclcpp::get_logger("ArduinoComms"), "Received message: %s", res);
    }

    if ((res[1] == 'o') && (res[2] == 'k'))
    {

      RCLCPP_INFO(rclcpp::get_logger("ArduinoComms"), "Arduino connected");
      return true;
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("ArduinoComms"), "Arduino NOT connected");
      return false;      
    }
  }

  // ================================
  // Read robot joints (state) function
  // ================================
  void read_joints_values(bool hw_test, int &val0, int &val1, int &val2, int &val3, int &val4, int &val5)
  {
    int val[6];
    char * strtokIndx;

    unsigned char r[4] = {'<','r','>','\0'};

    WriteToSerial(r, sizeof(uint8_t)*sizeof(r));

    if (hw_test)
    {
      RCLCPP_INFO(rclcpp::get_logger("ArduinoComms"), "=== <r> command sent ===");
    }

    char ret[35] = {};
    uint8_t* v = (uint8_t*)ret;

    ReadSerial(hw_test, v, sizeof(uint8_t)*sizeof(ret));

    if (hw_test)
    {
      RCLCPP_INFO(rclcpp::get_logger("ArduinoComms"), "Received message (deg): %s", ret);
    }

    strtokIndx = std::strtok(ret, ",");
    
    for (int i = 0; i < 6; i++)
    {
      strtokIndx = strtok(NULL, ",");
      val[i] = atoi(strtokIndx);
    }

    if (hw_test)
    {
      RCLCPP_INFO(rclcpp::get_logger("ArduinoComms"), "Received (deg): %i, %i, %i, %i, %i, %i", val[0], val[1], val[2], val[3], val[4], val[5]);
    }

    val0 = val[0];
    val1 = val[1];
    val2 = val[2];
    val3 = val[3];
    val4 = val[4];
    val5 = val[5];

    return;
  }
  
  // ================================
  // Set robot joints function
  // ================================
  void set_joints_values(bool hw_test, float val0, float val1, float val2, float val3, float val4, float val5)
  {
    char w[35]; w[0] = {'<'}; w[1] = {'w'}; w[2] = {'\0'};
    char delimiter[2] = {',', '\0'};
    char endMark[2] = {'>', '\0'};
    
    int r_val0; int r_val1; int r_val2; int r_val3; int r_val4; int r_val5;

    if (!std::isnan(val0) || !std::isnan(val1) || !std::isnan(val2) || !std::isnan(val3) || !std::isnan(val4))
    {
      r_val0 = std::roundf(val0);
      r_val1 = std::roundf(val1);
      r_val2 = std::roundf(val2);
      r_val3 = std::roundf(val3);
      r_val4 = std::roundf(val4);
    }
    if (!std::isnan(val5))
    {
      r_val5 = std::roundf(val5);
    }

    if (hw_test)
    {
      RCLCPP_INFO(rclcpp::get_logger("ArduinoComms"), "Wrote (deg): %i, %i, %i, %i, %i, %i", r_val0, r_val1, r_val2, r_val3, r_val4, r_val5);
    }

    std::strcat(w, delimiter);

    std::string command0_string = std::to_string(r_val0);
    std::strcat(w, command0_string.c_str());
    
    std::strcat(w, delimiter);

    std::string command1_string = std::to_string(r_val1);
    std::strcat(w, command1_string.c_str());
    
    std::strcat(w, delimiter);
    
    std::string command2_string = std::to_string(r_val2);
    std::strcat(w, command2_string.c_str());
    
    std::strcat(w, delimiter);

    std::string command3_string = std::to_string(r_val3);
    std::strcat(w, command3_string.c_str());
    
    std::strcat(w, delimiter);

    std::string command4_string = std::to_string(r_val4);
    std::strcat(w, command4_string.c_str());
    
    std::strcat(w, delimiter);

    std::string command5_string = std::to_string(r_val5);
    std::strcat(w, command5_string.c_str());
    
    std::strcat(w, endMark);

    if (hw_test)
    {
      RCLCPP_INFO(rclcpp::get_logger("ArduinoComms"), "Wrote (char command): %s", w);
    }

    WriteToSerial((unsigned char*)w, sizeof(uint8_t)*sizeof(w));

    return;
  }

private:

    int timeout_ms_;

    int SerialPort = -1;
    struct termios tty;

  // ================================
    int WriteToSerial(unsigned char* buf, int nBytes)
    {
        
        return ::write(SerialPort, buf, nBytes);
    }

  // ================================
    int ReadSerial(bool hw_test, unsigned char* buf, int nBytes)
    {
        //auto t_start = std::chrono::high_resolution_clock::now();
        int n = 0;
        
        while (n < nBytes)
        {
            int ret = ::read(SerialPort, &(buf[n]), 1);
            
            //RCLCPP_INFO(rclcpp::get_logger("ArduinoComms"), "== Char received: %s", &buf[n]);

            if (ret < 0)
            {
                return ret;
            }

            if (buf[n] == '\0')
            {
              if (hw_test)
              {
                RCLCPP_INFO(rclcpp::get_logger("ArduinoComms"), "== End of received message ==");
              }
              break;
            }
            n += ret;

            // Implementation of a reading timeout
            /*auto t_end = std::chrono::high_resolution_clock::now();
            double elapsed_time_ms = std::chrono::duration<double, std::milli> (t_end-t_start).count();
            if (elapsed_time_ms > 100)
            {
                //RCLCPP_ERROR(rclcpp::get_logger("ArduinoComms"), "Timeout reading from serial!!!");
                break;
            }*/
        }
        return n;
    }

};

#endif