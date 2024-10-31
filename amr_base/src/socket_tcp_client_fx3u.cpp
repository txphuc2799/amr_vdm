
#include "amr_base/socket_tcp_client_fx3u.h"
#include "math.h"
// # include "/home/tannhat/ros2_ws/src/pallet_truck_vdm_v1.0/pallet_truck_hardware/include/pallet_truck_hardware/socket_tcp_client_fx3u.h"

#define SIZEOF(arr) sizeof(arr) / sizeof(*arr)

SkTcpClientFx3u::SkTcpClientFx3u(){}

SkTcpClientFx3u::~SkTcpClientFx3u()
{
    close(sockfd);
}

// = "192.168.1.250"
// = 8000
bool SkTcpClientFx3u::setup(std::string server_addr_, uint16_t port_addr_ )
{
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
    {
        // printf("\nError creating socket! ");
        return false;
    }
    bzero(&servaddr, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr(server_addr_.c_str());
    servaddr.sin_port = htons(port_addr_);

    if (connect(sockfd, (struct sockaddr *) &servaddr, sizeof(servaddr)) < 0)
    {
        // printf("\nError connecting socket!");
        close(sockfd);
        return false;
    }
    else{
        // printf("\nPC_client connecting socket TCP with PLC_server success!");
        return true;
    }
}

uint8_t* SkTcpClientFx3u::convert_uint16_to_2uint8(uint16_t number)
{
    static uint8_t frame_number_2byte[2];
    frame_number_2byte[1] = (number >> 8) & 0xFF;
    frame_number_2byte[0] = (number >> 0) & 0xFF;
    return frame_number_2byte;
}

uint8_t* SkTcpClientFx3u::convert_int16_to_2uint8(int16_t number)
{
    static uint8_t frame_number_2byte[2];
    frame_number_2byte[1] = (number >> 8) & 0xFF;
    frame_number_2byte[0] = (number >> 0) & 0xFF;
    return frame_number_2byte;
}

uint8_t* SkTcpClientFx3u::convert_uint32_to_4uint8(uint32_t number)
{
    static uint8_t frame_number_4byte[4];
    frame_number_4byte[3] = (number >> 24) & 0xFF;
    frame_number_4byte[2] = (number >> 16) & 0xFF;
    frame_number_4byte[1] = (number >> 8) & 0xFF;
    frame_number_4byte[0] = (number >> 0) & 0xFF;
    return frame_number_4byte;
}

uint8_t* SkTcpClientFx3u::convert_int32_to_4uint8(int32_t number)
{
    static uint8_t frame_number_4byte[4];
    frame_number_4byte[3] = (number >> 24) & 0xFF;
    frame_number_4byte[2] = (number >> 16) & 0xFF;
    frame_number_4byte[1] = (number >> 8) & 0xFF;
    frame_number_4byte[0] = (number >> 0) & 0xFF;
    return frame_number_4byte;
}

uint8_t* SkTcpClientFx3u::convert_2int32_to_8uint8(int32_t number1, int32_t number2)
{   
    static uint8_t frame_number_8byte[10];
    if (number2 == 0) {
        frame_number_8byte[9] = 0;
        frame_number_8byte[8] = 0;
    }else{
        frame_number_8byte[9] = 0;
        frame_number_8byte[8] = 1;
    }

    frame_number_8byte[7] = (number2 >> 24) & 0xFF;
    frame_number_8byte[6] = (number2 >> 16) & 0xFF;
    frame_number_8byte[5] = (number2 >> 8) & 0xFF;
    frame_number_8byte[4] = (number2 >> 0) & 0xFF;
    frame_number_8byte[3] = (number1 >> 24) & 0xFF;
    frame_number_8byte[2] = (number1 >> 16) & 0xFF;
    frame_number_8byte[1] = (number1 >> 8) & 0xFF;
    frame_number_8byte[0] = (number1 >> 0) & 0xFF;
    return frame_number_8byte;
}


uint8_t SkTcpClientFx3u::read_bit_FX3U(char name_bit, uint32_t number_name)
{
    uint8_t name_bit_ascii = (uint8_t)name_bit;
    uint8_t* frame_number_name = SkTcpClientFx3u::convert_uint32_to_4uint8(number_name);
    uint8_t command[] = {0x00, 0xff, 0x0A, 0x00, frame_number_name[0], frame_number_name[1], frame_number_name[2], 
                        frame_number_name[3], 0x20, name_bit_ascii, 0x01, 0x00};
    
    if (send(sockfd, command, 12,0) < 0)
    {
        // printf("\nSocket_client(read_bit): Send command Error");
        return 2;
    }
    uint8_t buffer[3];
    recv(sockfd, buffer, 3, 0);
    // printf("\nSocket client: Received parameters:    ");
    if (buffer[0] != 128 || buffer[1] != 0)
    {
        // printf("\nSocket client(read_bit): parameters is wrong");
        return 2;
    }

    if ( buffer[2] == 0) return 0;
    else return 1;
}

uint8_t* SkTcpClientFx3u::read_bit_FX3U(char name_bit, uint32_t number_name, uint8_t length)
{
    uint8_t name_bit_ascii = (uint8_t)name_bit;
    uint8_t* frame_number_name = SkTcpClientFx3u::convert_uint32_to_4uint8(number_name);
    uint8_t command[] = {0x00, 0xff, 0x0A, 0x00, frame_number_name[0], frame_number_name[1], frame_number_name[2], 
                        frame_number_name[3], 0x20, name_bit_ascii, length, 0x00};

    send(sockfd, command, 12,0);
    uint8_t size_buf;
    if (length % 2 == 0) size_buf = 2 + (length / 2);
    else size_buf = 2 + ((length + 1) / 2);
    uint8_t buffer[size_buf];
    
    recv(sockfd, buffer, size_buf , 0);
    return buffer;
}

uint8_t* SkTcpClientFx3u::read_word_FX3U(char name_word, uint32_t number_name, uint8_t length_word)
{
    // static uint8_t res_data[20] = {0};
    uint8_t name_word_ascii = (uint8_t)name_word;
    static uint8_t buffer[24];
    uint8_t* frame_number_name = SkTcpClientFx3u::convert_uint32_to_4uint8(number_name);
    uint8_t command[] = {0x01, 0xff, 0x0A, 0x00, frame_number_name[0], frame_number_name[1], frame_number_name[2], 
                        frame_number_name[3], 0x20, name_word_ascii, length_word, 0x00};

    if (send(sockfd, command, 12, 0) < 0)
    {
        // printf("\nSocket_client(read_word): Send command Error");
        // res_data[19] = 11;
        // return res_data;
        buffer[19] = 11;
        return buffer;
    }           

    int bufsize = 2 + length_word*2;
    
    recv(sockfd, buffer, bufsize, 0);
    if (buffer[0] != 129 || buffer[1] != 0)
    {
        // printf("\nSocket client(read_word): parameters is wrong");
        buffer[23] = 11;
        return buffer;
    }

    return buffer;
    // for (int i=0; i<bufsize; i++)
    // {
    //     res_data[i] = buffer[i];
    // }
    // return res_data;
}

bool SkTcpClientFx3u::write_bit_FX3U(char name_bit, uint32_t number_name, uint8_t bit_cmd)
{
    // printf("\nVao doc mot bit");
    uint8_t state = bit_cmd != 0 ? 16 : 0;  
    uint8_t name_bit_ascii = (uint8_t)name_bit;
    uint8_t* frame_number_name = SkTcpClientFx3u::convert_uint32_to_4uint8(number_name);
    uint8_t command[] = {0x02, 0xff, 0x0A, 0x00, frame_number_name[0], frame_number_name[1], frame_number_name[2], 
                    frame_number_name[3], 0x20, name_bit_ascii, 0x01, 0x00, state};

    if (send(sockfd, command, sizeof(command), 0) < 0)
    {
        // printf("\nSocket_client(write_bit): Send command Error");
        return false;
    }
    uint8_t buffer[2];
    recv(sockfd, buffer, 2, 0);
    if (buffer[0] != 130 || buffer[1] != 0)
    {
        // printf("\nSocket client(write_bit): parameters is wrong");
        return false;
    }
    else return true;
}

bool SkTcpClientFx3u::write_bit_FX3U(char name_bit, uint32_t number_name, uint8_t length_bit, uint8_t bit_cmd[])
{
    uint8_t name_bit_ascii = (uint8_t)name_bit;
    uint8_t* frame_number_name = SkTcpClientFx3u::convert_uint32_to_4uint8(number_name);
    int array_length;
    if (length_bit % 2 == 0) {
        array_length = 12 + length_bit / 2;
    } else {
        array_length = 12 + (length_bit + 1) / 2;
    }
    uint8_t command[array_length] = {0x02, 0xff, 0x0A, 0x00, frame_number_name[0], frame_number_name[1], frame_number_name[2], 
                    frame_number_name[3], 0x20, name_bit_ascii, length_bit, 0x00};
    for (int i=0; i< length_bit; i+=2){
        // printf("\nBIT cmd: %d", bit_cmd[i]);
        if (i + 1 >= length_bit) {
            if (bit_cmd[i] == 0) {
                command[12+i/2] = 0;
            } else {
                command[12+i/2] = 16;
            }
        } else {
            if (bit_cmd[i] == 0 && bit_cmd[i + 1] == 0) {
                command[12+i/2] = 0;
            } else if (bit_cmd[i] == 1 && bit_cmd[i + 1] == 0) {
                command[12+i/2] = 16;
            } else if (bit_cmd[i] == 0 && bit_cmd[i + 1] == 1) {
                command[12+i/2] = 1;
            } else {
                command[12+i/2] = 17;
            }
        }
    }

    if (send(sockfd, command, sizeof(command), 0) < 0) {
        // printf("\nSocket_client(write_bit): Send command Error");
        return false;
    }

    uint8_t buffer[2];
    recv(sockfd, buffer, 2, 0);
    if (buffer[0] != 130 || buffer[1] != 0) {
        // printf("\nSocket client(write_bit): parameters is wrong");
        return false;
    } else return true;
}


bool SkTcpClientFx3u::write_word_FX3U(char name_word, uint32_t number_name, uint8_t length_word, uint8_t data_word[])
{   
    uint8_t name_word_ascii = (uint8_t)name_word;
    uint8_t* frame_number_name = SkTcpClientFx3u::convert_uint32_to_4uint8(number_name);
    uint8_t command[12+length_word*2] = {0x03, 0xff, 0x0A, 0x00, frame_number_name[0], frame_number_name[1], frame_number_name[2], 
                    frame_number_name[3], 0x20, name_word_ascii, length_word, 0x00};

    for (int i=0; i< 2*length_word; i++){
        command[12+i] = data_word[i];
    }
    if (send(sockfd, command, sizeof(command), 0) < 0)
    {
        // printf("\nSocket_client(write_word): Send command Error");
        return false;
    }
    uint8_t buffer[2];
    recv(sockfd, buffer, 2, 0);
    if (buffer[0] != 131 || buffer[1] != 0)
    {
        // printf("\nSocket client(write_word): parameters is wrong");
        return false;
    }
    else return true;
}

// Chuyen doi xung sang goc cho dieu khien theo toc do
float SkTcpClientFx3u::convertValue2Radian(int32_t value_old, int32_t value_new, float encoder_resolution, double gear_ratio)
{
    float radian = 0.0;
    const float pi = 3.14159265359f;

    radian = 2*pi*(value_new - value_old) / (encoder_resolution * gear_ratio);
    return radian;
}

// Chuyen doi xung sang goc cho dieu khien theo vi tri
float SkTcpClientFx3u::convertValue2Radian(int32_t value, int32_t max_position, int32_t min_position,
                                           float max_radian, float min_radian)
{
    float radian = 0.0;
    int32_t zero_position = 0;

    if (value > zero_position){
        radian = (float)(value - zero_position) * max_radian / (float)(max_position - zero_position);
    }
    else if( value < zero_position){
        radian = (float)(value - zero_position) * min_radian / (float)(min_position -zero_position);
    }

    return radian;
}

int32_t SkTcpClientFx3u::convertRadian2Value(float radian, int32_t max_position, int32_t min_position,
                                             float max_radian, float min_radian)
{
    int32_t value = 0;
    int32_t zero_position = 0;
    if (radian > 0)
    {
        value = (radian * (max_position - zero_position) / max_radian) + zero_position;
    }
    else if (radian < 0)
    {
        value = (radian * (min_position - zero_position) / min_radian) + zero_position;
    }
    else
    {
        value = zero_position;
    }

    return value;
}

float SkTcpClientFx3u::convertPulse_StoVelocity(int32_t value){
    float velocity = 0.0f;
    const float RPM2RADPERSEC = 0.104719755f;
    velocity = (60 * RPM2RADPERSEC * value)/ 10000;
    return velocity;
}

int32_t SkTcpClientFx3u::converVelocitytoPulse_S(float velocity)
{
    int32_t value_Pulses = 0;
    const float RPM2RADPERSEC = 0.104719755f;
    value_Pulses = (10000 * velocity) / (60 * RPM2RADPERSEC);
    return value_Pulses;
}


int32_t SkTcpClientFx3u::convertPulse_StoRPM(int32_t value)
{
    int32_t value_RPM = 0;
    value_RPM = (int32_t)(60 * value / 10000);
    return value_RPM;
}

int32_t SkTcpClientFx3u::convertRPMtoPulse_S(int32_t value)
{
    int32_t value_PulseS = 0;
    value_PulseS = (int32_t)(10000 * value / 60);
    return value_PulseS;
}


float SkTcpClientFx3u::convertValue2Velocity(int32_t value, double gear_ratio)
{
    float velocity = 0.0f;
    const float RPM2RADPERSEC = 0.104719755f;
    velocity = value * RPM2RADPERSEC / gear_ratio;
    return velocity;
}

int32_t SkTcpClientFx3u::convertVelocity2Value(float velocity, double gear_ratio)
{
    int32_t value = 0;
    const float RPM2RADPERSEC = 0.104719755f;
    value = velocity * gear_ratio / RPM2RADPERSEC;
    return value;
}

float SkTcpClientFx3u::convertPulse2Meter(int32_t value, int32_t encoder_resolution, float gear_ratio, float radius_rot, float efficiency)
{
    float metter = 0.0f;
    metter = 2 * M_PI * radius_rot * value * efficiency / (encoder_resolution * gear_ratio);
    return metter;
}

int32_t SkTcpClientFx3u::convertMeter2Pulse(float metter, int32_t encoder_resolution, float gear_ratio, float radius_rot)
{
    int32_t value = 0;
    value = metter * encoder_resolution * gear_ratio / ( 2* M_PI * radius_rot);
    return value;
}

// std::vector<uint8_t> SkTcpClientFx3u::numberToBytes(uint16_t number, uint8_t length){

//     std::vector<uint8_t> byteArray(length);

//     uint64_t intNumber = *(reinterpret_cast<uint64_t*>(&number));

//     for (int i=0; i<length; i++){
//         byteArray[i] = static_cast<uint8_t>((intNumber >> (8 * (length - 1 - i)) & 0xFF));
//     }
//     return byteArray;
// }

int8_t SkTcpClientFx3u::sign(int32_t number)
{
    if (number >= 0){
        return 1;
    }
    else{
        return 0;
    }
}

// SkTcpClientFx3u nhat_test;
// int main()
// {   
//     char name_bit = 'M';
//     uint32_t number_name = 100;
    
//     if (nhat_test.setup())
//     {
//         printf("\nTest setup OKAY");
//     }
//     else printf("\nTest setup ERROR");

//     int8_t a = nhat_test.read_bit_FX3U(name_bit,number_name);
//     if (a == 1)
//     {
//         printf("\nBit M100: ON");
//     }
//     else if (a == 0)
//     {
//         printf("\nBit M100: OFF");
//     }
//     else printf("\nREAD bit error");
    
    
//     if (nhat_test.write_bit_FX3U(name_bit,number_name,0))
//     {
//         printf("\nWrite Bit M100: SUCCESS");
//     }
//     else printf("\nWrite Bit M100: ERROR");

//     // char name_word ='D';
//     // uint32_t number_name = 100;
//     // uint8_t leg

//     uint8_t* data_word_test;
//     data_word_test = nhat_test.read_word_FX3U('D',100,2);
//     if (data_word_test[19] == 11){
//         printf("\nRead word D100: ERROR");
//     }
//     else{
//         printf("\nReceived parameters:    ");
//         for(int i = 0; i < 8; i++)
//         {
//             printf("%d  ", data_word_test[i]);
//         }
//     }
//     uint8_t res_data[4] = {data_word_test[2],data_word_test[3],data_word_test[4],data_word_test[5]};
//     int32_t test_data_32int;
//     test_data_32int = *((int32_t*) res_data);
//     printf("\nReceived number 32bit:    ");
//     printf("%d  ", test_data_32int);
//     // uint8_t data[] = {0x64,0x00,0x64,0x00,0x64,0x00,0x64,0x00};
//     // size_t length_array = sizeof(data);
//     // if (8 != length_array){
//     //     printf("\nSocket_client(write_word): Length_word and data_word is not correct");
//     //     return false;
//     // }
//     // if (nhat_test.write_word_FX3U('D',105,4,data)){
//     //     printf("\nWrite word D105-107: SUCCESS");
//     // }
//     // else printf("\nWrite word D105-107: ERROR");

//     return 0;


// }
