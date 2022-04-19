// -*- lsst-c++ -*-
/**
 * @file modbus_interface.h
 *
 * @brief modbus_interface header
 */

#ifndef MODBUS_INTERFACE_H
#define MODBUS_INTERFACE_H

//Modbus include
#include <modbus/modbus-tcp.h>

//Standard includes
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <set>
#include <algorithm>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

namespace Modbus {

class ModbusInterface
{
public:

    ModbusInterface(){};

    enum STATE_CODE
    {
     NO_ISSUE = 0,                       /*!< The node is running without encountering any issue                          */
     INITIALIZING = 1,                   /*!< The node is initializing itself                                             */
     NOT_CONNECTED = 2,                  /*!< The node could not establish, or lost the connection with the Modbus device */
     INVALID_CONFIGURATION_FILE = 3,     /*!< The provided configuration file is not valid                                */
     INVALID_IO_TYPE = 4,                /*!< An IO was given a type other than input or output                           */
     INVALID_IO_DATA_TYPE = 5,           /*!< An IO was given a data type other than digital or analog                    */
     INVALID_IO_KEY = 6,                 /*!< A provided key to publish, or write on is not given a configuration         */
     INVALID_IO_TO_WRITE = 7             /*!< Tried to write on a non output IO                                           */
    };

    /**
     * @struct m_IO_struct
     * @brief Structure defining an IO
     *
     * @var m_IO_struct::type
     * IO type, can be input or output
     * @var m_IO_struct::data_type
     * IO data type, can be digital or analog
     * @var m_IO_struct::address
     * IO address
     */
    struct IO_struct{
        std::string type;        /*!< The IO type, can be input or output                                                                                      */
        std::string data_type;   /*!< The IO data type, can be digital or analog                                                                               */
        int address;             /*!< The IO address in its register, starting at 1 on configuration file to match reference device Bechkoff KL1889 and EL2809 */
    };

    struct modbus_memory{
        int nb_input_coils;
        int nb_output_coils;
        int nb_input_registers;
        int nb_output_registers;
        std::vector<uint8_t> input_coils;
        std::vector<uint8_t> output_coils;
        std::vector<uint16_t> input_registers;
        std::vector<uint16_t> output_registers;
    };

    void setContext(std::string address, int port);
    std::pair<std::string, int> getContext();

    void setDevice(int nb_input_coils, int nb_output_coils, int nb_input_registers, int nb_output_registers);

    void addIO(std:: string, std::string, std::string, int);
    bool verifyIO();
    bool hasIODeclared(std::string);

    void updateMemory();

    uint8_t getSingleInputCoil(int);
    uint8_t getSingleOutputCoil(int);
    bool setSingleOutputCoil(int, uint8_t);
    uint16_t getSingleInputRegister(int);
    uint16_t getSingleOutputRegister(int);
    bool setSingleOutputRegister(int, uint16_t);

    std::vector<uint8_t> getMultipleInputCoils();
    std::vector<uint8_t> getMultipleOutputCoils();
    bool setMultipleOutputCoils(std::vector<uint8_t>);
    std::vector<uint16_t> getMultipleInputRegisters();
    std::vector<uint16_t> getMultipleOutputRegisters();
    bool setMultipleOutputRegisters(std::vector<uint16_t>);

    uint16_t getIOvalue(std::string);
    std::map<std::string, IO_struct> getIOMap();

    void setConnectionState(bool);
    bool getConnectionState();
    bool initiateConnection();
    bool restartConnection();

    void convert(uint8_t *, std::vector<uint8_t>, int);
    void convert(uint16_t *, std::vector<uint16_t>, int);


private:

    modbus_t *m_ctx;               /*!< Modbus context                           */
    modbus_memory m_memory;
    std::map<std::string, IO_struct> m_IO_map;              /*!< Map of all declared IO and their definition             */

    std::mutex m_ctx_guard;        /*!< Modbus context guard                     */
    std::mutex m_IO_map_guard;     /*!< IO map guard                             */
    std::mutex m_memory_guard;     /*!< IO map guard                             */
    std::mutex m_connection_state_guard;     /*!< connection state guard                            */

    bool m_connected{false};       /*!< Connection state to the modbus device    */
    std::string m_address;
    int m_port;

    int m_temp_nb_to_get;
    bool m_success;

};

}

#endif
