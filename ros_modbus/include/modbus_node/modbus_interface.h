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
#include <cstring>

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
     * @struct IO_struct
     * @brief Structure defining an IO
     *
     * @var IO_struct::type
     * IO type, can be input or output
     * @var IO_struct::data_type
     * IO data type, can be digital or analog
     * @var IO_struct::address
     * IO address
     */
    struct IO_struct{
        std::string type;        /*!< The IO type, can be input or output                                                                                      */
        std::string data_type;   /*!< The IO data type, can be digital or analog                                                                               */
        int address;             /*!< The IO address in its register, starting at 1 on configuration file to match reference device Bechkoff KL1889 and EL2809 */
    };

    /**
     * @struct modbus_memory
     * @brief Structure defining the number of each IO type on the device and their values
     *
     * @var modbus_memory::nb_input_coils
     * Number of input coils of the device
     * @var modbus_memory::nb_output_coils
     * Number of output coils of the device
     * @var modbus_memory::nb_input_registers
     * Number of input registers of the device
     * @var modbus_memory::nb_output_registers
     * Number of output registers of the device
     * @var modbus_memory::input_coils
     * Vector of input coils values of the device
     * @var modbus_memory::output_coils
     * Vector of output coils values of the device
     * @var modbus_memory::input_registers
     * Vector of input registers values of the device
     * @var modbus_memory::output_registers
     * Vector of output registers values of the device
     */
    struct modbus_memory{
        int nb_input_coils;                        /*!< Number of input coils of the device             */
        int nb_output_coils;                       /*!< Number of output coils of the device            */
        int nb_input_registers;                    /*!< Number of input registers of the device         */
        int nb_output_registers;                   /*!< Number of output registers of the device        */
        std::vector<uint8_t> input_coils;          /*!< Vector of input coils values of the device      */
        std::vector<uint8_t> output_coils;         /*!< Vector of output coils values of the device     */
        std::vector<uint16_t> input_registers;     /*!< Vector of input registers values of the device  */
        std::vector<uint16_t> output_registers;    /*!< Vector of output registers values of the device */
    };

    /**
     * @brief Initiate a new TCP context for Modbus device.
     *
     * @param address The Modbus device address*
     * @param port The port the Modbus device is listening to
     */
    void setContext(std::string, int);

    /**
     * @brief Return the Modbus device address and port.
     *
     * @return pair Contains Modbus address as first and Modbus port as second
     */
    std::pair<std::string, int> getContext();

    /**
     * @brief Set the number of each IO type for the device.
     *
     * @param nb_input_coils Number of input coils of the device
     * @param nb_output_coils Number of output coils of the device
     * @param nb_input_registers Number of input registers of the device
     * @param nb_output_registers Number of output registers of the device
     */
    void setDevice(int, int, int, int);

    /**
     * @brief Declares an IO to the device
     *
     * @param name The name of the IO
     * @param io_type IO type, can be "input" or "output"
     * @param io_data_type IO data type, can be "digital" or "analog"
     * @param address IO address (starts at 0)
     */
    void addIO(std::string, std::string, std::string, int);

    /**
     * @brief simple verification, verify that declared IOs have correct type, data type and are provided an address
     */
    bool verifyIO();

    /**
     * @brief Check if an IO is declared
     * @return true if the IO is declared
     * @return false otherwise
     */
    bool hasIODeclared(std::string);

    /**
     * @brief Update the Modbus memory
     *
     * @throw MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE if update of a vector fails and set m_connected to false
     */
    void updateMemory();

    /**
     * @brief get the value of the input coil at given address
     *
     * @param address The coil address
     *
     * @return value The input coil value
     */
    uint8_t getSingleInputCoil(int);

    /**
     * @brief get the value of the output coil at given address
     *
     * @param address The coil address
     *
     * @return value The output coil value
     */
    uint8_t getSingleOutputCoil(int);

    /**
     * @brief set the value of the coil at given address
     *
     * @param address The coil address
     * @param value The coil value
     *
     * @return true If the value was set
     * @return false otherwise
     */
    bool setSingleOutputCoil(int, uint8_t);

    /**
     * @brief get the value of the input register at given address
     *
     * @param address The register address
     *
     * @return value The input register value
     */
    uint16_t getSingleInputRegister(int);

    /**
     * @brief get the value of the output register at given address
     *
     * @param address The register address
     *
     * @return value The output register value
     */
    uint16_t getSingleOutputRegister(int);

    /**
     * @brief set the value of the register at given address
     *
     * @param address The register address
     * @param value The register value
     *
     * @return true If the value was set
     * @return false otherwise
     */
    bool setSingleOutputRegister(int, uint16_t);

    /**
     * @brief get the values of the input coils
     *
     * @return values The input coils values as vector
     */
    std::vector<uint8_t> getMultipleInputCoils();

    /**
     * @brief get the values of the output coils
     *
     * @return values The output coils values as vector
     */
    std::vector<uint8_t> getMultipleOutputCoils();

    /**
     * @brief set the values of the coils
     *
     * @param value The coils values
     *
     * @return true If the value was set
     * @return false otherwise
     */
    bool setMultipleOutputCoils(std::vector<uint8_t>);

    /**
     * @brief get the values of the input registers
     *
     * @return values The input registers values as vector
     */
    std::vector<uint16_t> getMultipleInputRegisters();

    /**
     * @brief get the values of the output registers
     *
     * @return values The output registers values as vector
     */
    std::vector<uint16_t> getMultipleOutputRegisters();

    /**
     * @brief set the values of the registers
     *
     * @param value The registers values
     *
     * @return true If the value was set
     * @return false otherwise
     */
    bool setMultipleOutputRegisters(std::vector<uint16_t>);

    /**
     * @brief Get a given IO value
     *
     * @param name The given IO name
     *
     * @return the IO value
     */
    uint16_t getIOvalue(std::string);

    /**
     * @brief Get IO map
     * @return a copy of the map with all declared IOs
     */
    std::map<std::string, IO_struct> getIOMap();

    /**
     * @brief Set connection state to a given value
     *
     * @param state The state to set the connection state at
     */
    void setConnectionState(bool);

    /**
     * @brief Get connection state
     *
     * @return state The connection state value
     */
    bool getConnectionState();

    /**
     * @brief Initiate connection
     *
     * Tries to open a connection with the device using it's context via TCP
     *
     * @return true if the connection openned
     * @return false otherwise
     */
    bool initiateConnection();

    /**
     * @brief Restart connection
     *
     * Tries to reconnect
     *
     * @return true when reconnected
     * @return false otherwise
     */
    bool restartConnection();

private:

    modbus_t *m_ctx;                               /*!< Modbus context                              */
    modbus_memory m_memory;                        /*!< Modbus memory state                         */
    std::map<std::string, IO_struct> m_IO_map;     /*!< Map of all declared IO and their definition */

    std::mutex m_ctx_guard;                        /*!< Modbus context guard                        */
    std::mutex m_IO_map_guard;                     /*!< IO map guard                                */
    std::mutex m_memory_guard;                     /*!< Memory guard                                */
    std::mutex m_connection_state_guard;           /*!< connection state guard                      */

    bool m_connected{false};                       /*!< Connection state to the modbus device       */
    std::string m_address;                         /*!< Address of the modbus device                */
    int m_port;                                    /*!< Port of the modbus device                   */

    int m_temp_nb_to_get;                          /*!< Temp value of IO to read                    */
    bool m_success;                                /*!< Success of a reading                        */

};

}

#endif
