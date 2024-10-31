#ifndef SOCKET_TCP_CLIENT_FX3U_H
#define SOCKET_TCP_CLIENT_FX3U_H

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <iostream>
#include <unistd.h>
#include <vector>


class SkTcpClientFx3u
{
  private:
    // std::string server_addr;
    // uint16_t port_addr;
  public:
    // SkTcpClientFx3u(std::string server_addr_ = "192.168.0.250", uint16_t port_addr_ = 8000){
    // };
    SkTcpClientFx3u();
    ~SkTcpClientFx3u();

    int sockfd;
    // float radian_pos = 0.0f;
    struct sockaddr_in servaddr;
    

    bool setup(std::string server_addr_, uint16_t port_addr_);
    
    uint8_t* convert_uint16_to_2uint8(uint16_t number);
    uint8_t* convert_int16_to_2uint8(int16_t number);
    uint8_t* convert_uint32_to_4uint8(uint32_t number);
    uint8_t* convert_int32_to_4uint8(int32_t number);
    uint8_t* convert_2int32_to_8uint8(int32_t number1, int32_t number2);
    uint8_t read_bit_FX3U(char name_bit, uint32_t number_name);
    uint8_t* read_bit_FX3U(char name_bit, uint32_t number_name, uint8_t length);
    uint8_t* read_word_FX3U(char name_word, uint32_t number_name, uint8_t length_word);
    bool write_bit_FX3U(char name_bit, uint32_t number_name, uint8_t bit_cmd);
    bool write_bit_FX3U(char name_bit, uint32_t number_name, uint8_t length_bit, uint8_t bit_cmd[]);
    bool write_word_FX3U(char name_word, uint32_t number_name, uint8_t length_word, uint8_t data_word[]);

    // Convert interger to bytearray
    // std::vector<uint8_t> numberToBytes(uint16_t number, uint8_t length);

    // Convert pulse/s to RPM
    int32_t convertPulse_StoRPM(int32_t value);
    // Convert RPM to pulse
    int32_t convertRPMtoPulse_S(int32_t value);

    // Convert pulse/s to rad/s
    float convertPulse_StoVelocity(int32_t value);
    // Convert rad/s to pulse/s
    int32_t converVelocitytoPulse_S(float velocity);

    // Convert pulse encoder (position) to rad
    float convertValue2Radian(int32_t value_old, int32_t value_new, float encoder_resolution, double gear_ratio);
    float convertValue2Radian(int32_t value, int32_t max_position, int32_t min_position, float max_radian, float min_radian);
    // Convert rad to pulse encoder (position)
    int32_t convertRadian2Value(float radian, int32_t max_position, int32_t min_position, float max_radian, float min_radian);

    // convert velocity from rpm to rad/s
    float convertValue2Velocity(int32_t value, double gear_ratio);
    // Convert velocity from rad/s to rpm
    int32_t convertVelocity2Value(float velocity, double gear_ratio);

    // Convert pulse encoder to meter (position)
    float convertPulse2Meter(int32_t value, int32_t encoder_resolution, float gear_ratio, float radius_rot, float efficiency);
    // Convert meter to pulse encoder (position)
    int32_t convertMeter2Pulse(float metter, int32_t encoder_resolution, float gear_ratio, float radius_rot);

    int8_t sign(int32_t number);
};
#endif // SOCKET_TCP_CLIENT_FX3U_H