// -*- lsst-c++ -*-
/**
 * @file modbus_interface.cpp
 *
 * @brief modbus_interface functions definition
 */

#include <modbus_node/modbus_interface.h>
#include <iostream>

using namespace Modbus;

/**
 * @brief Initiate a new TCP context for Modbus device.
 *
 * @param address The Modbus device address*
 * @param port The port the Modbus device is listening to
 */
void ModbusInterface::setContext(std::string address, int port)
{
    m_address = address;
    m_port = port;
    m_ctx = modbus_new_tcp(address.c_str(), port);
}

/**
 * @brief Return the Modbus device address and port.
 *
 * @return pair Contains Modbus address as first and Modbus port as second
 */
std::pair<std::string, int> ModbusInterface::getContext()
{
    return std::pair<std::string, int>(m_address, m_port);
}

/**
 * @brief Set the number of each IO type for the device.
 *
 * @param nb_input_coils Number of input coils of the device
 * @param nb_output_coils Number of output coils of the device
 * @param nb_input_registers Number of input registers of the device
 * @param nb_output_registers Number of output registers of the device
 */
void ModbusInterface::setDevice(int nb_input_coils, int nb_output_coils, int nb_input_registers, int nb_output_registers)
{
    m_memory_guard.lock();

    m_memory.nb_input_coils = nb_input_coils;
    m_memory.input_coils = std::vector<uint8_t>(nb_input_coils, 0);

    m_memory.nb_output_coils = nb_output_coils;
    m_memory.output_coils = std::vector<uint8_t>(nb_output_coils, 0);

    m_memory.nb_input_registers = nb_input_registers;
    m_memory.input_registers = std::vector<uint16_t>(nb_input_registers, 0);

    m_memory.nb_output_registers = nb_output_registers;
    m_memory.output_registers = std::vector<uint16_t>(nb_output_registers, 0);

    m_memory_guard.unlock();
}

/**
 * @brief Declares an IO to the device
 *
 * @param name The name of the IO
 * @param io_type IO type, can be "input" or "output"
 * @param io_data_type IO data type, can be "digital" or "analog"
 * @param address IO address (starts at 0)
 */
void ModbusInterface::addIO(std::string name, std::string io_type, std::string io_data_type, int address)
{
    IO_struct temp;
    temp.type = io_type;
    temp.data_type = io_data_type;
    temp.address = address;

    m_IO_map_guard.lock();
    m_IO_map.insert(std::pair(name, temp));
    m_IO_map_guard.unlock();
}

/**
 * @brief simple verification, verify that declared IOs have correct type, data type and are provided an address
 */
bool ModbusInterface::verifyIO()
{
    // Only verify type, data type and if an address is provided for IO of interest
    m_IO_map_guard.lock();
    auto temp_map = m_IO_map;
    m_IO_map_guard.unlock();

    for(auto &[key, structure]: temp_map)
    {
        if(structure.type != "input" && structure.type != "output")
        {
            return false;
        }
        else if(structure.data_type != "digital" && structure.type != "analog")
        {
            return false;
        }
        else if(structure.address == -1)
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief Check if an IO is declared
 * @return true if the IO is declared
 * @return false otherwise
 */
bool ModbusInterface::hasIODeclared(std::string key)
{
    m_IO_map_guard.lock();
    bool result = m_IO_map.find(key) != m_IO_map.end();
    m_IO_map_guard.unlock();
    return result;
}

/**
 * @brief Update the Modbus memory
 *
 * @throw MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE if update of a vector fails and set m_connected to false
 */
void ModbusInterface::updateMemory()
{
    //update input coils
    m_memory_guard.lock();
    m_temp_nb_to_get =  m_memory.nb_input_coils;
    m_memory_guard.unlock();

    m_ctx_guard.lock();
    m_memory_guard.lock();
    m_success = modbus_read_input_bits(m_ctx, 0, m_temp_nb_to_get , m_memory.input_coils.data()) != -1;
    m_memory_guard.unlock();
    m_ctx_guard.unlock();
    if(!m_success)
    {
        setConnectionState(false);
        throw MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE;
    }

    //update output coils
    m_memory_guard.lock();
    m_temp_nb_to_get =  m_memory.nb_output_coils;
    m_memory_guard.unlock();

    m_ctx_guard.lock();
    m_memory_guard.lock();
    m_success = modbus_read_bits(m_ctx, 0,m_temp_nb_to_get , m_memory.output_coils.data()) != -1;
    m_memory_guard.unlock();
    m_ctx_guard.unlock();
    if(!m_success)
    {
        setConnectionState(false);
        throw MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE;
    }

    //update input registers
    m_memory_guard.lock();
    m_temp_nb_to_get =  m_memory.nb_input_registers;
    m_memory_guard.unlock();

    m_ctx_guard.lock();
    m_memory_guard.lock();
    m_success = modbus_read_input_registers(m_ctx, 0,m_temp_nb_to_get , m_memory.input_registers.data()) != -1;
    m_memory_guard.unlock();
    m_ctx_guard.unlock();
    if(!m_success)
    {
        setConnectionState(false);
        throw MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE;
    }


    //update output registers
    m_memory_guard.lock();
    m_temp_nb_to_get =  m_memory.nb_output_registers;
    m_memory_guard.unlock();

    m_ctx_guard.lock();
    m_memory_guard.lock();
    m_success = modbus_read_registers(m_ctx, 0,m_temp_nb_to_get , m_memory.output_registers.data()) != -1;
    m_memory_guard.unlock();
    m_ctx_guard.unlock();
    if(!m_success)
    {
        setConnectionState(false);
        throw MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE;
    }
}

/**
 * @brief get the value of the input coil at given address
 *
 * @param address The coil address
 *
 * @return value The input coil value
 */
uint8_t ModbusInterface::getSingleInputCoil(int address)
{
    m_memory_guard.lock();
    uint8_t temp = m_memory.input_coils[address];
    m_memory_guard.unlock();
    return temp;

}

/**
 * @brief get the value of the output coil at given address
 *
 * @param address The coil address
 *
 * @return value The output coil value
 */
uint8_t ModbusInterface::getSingleOutputCoil(int address)
{
    m_memory_guard.lock();
    uint8_t temp = m_memory.output_coils[address];
    m_memory_guard.unlock();
    return temp;

}

/**
 * @brief set the value of the coil at given address
 *
 * @param address The coil address
 * @param value The coil value
 *
 * @return true If the value was set
 * @return false otherwise
 */
bool ModbusInterface::setSingleOutputCoil(int address, uint8_t value)
{
    m_ctx_guard.lock();
    bool result = modbus_write_bit(m_ctx, address, value);
    m_ctx_guard.unlock();
    return result;
}

/**
 * @brief get the value of the input register at given address
 *
 * @param address The register address
 *
 * @return value The input register value
 */
uint16_t ModbusInterface::getSingleInputRegister(int address)
{
    m_memory_guard.lock();
    uint16_t temp = m_memory.input_registers[address];
    m_memory_guard.unlock();
    return temp;

}

/**
 * @brief get the value of the output register at given address
 *
 * @param address The register address
 *
 * @return value The output register value
 */
uint16_t ModbusInterface::getSingleOutputRegister(int address)
{
    m_memory_guard.lock();
    uint16_t temp = m_memory.output_registers[address];
    m_memory_guard.unlock();
    return temp;

}

/**
 * @brief set the value of the register at given address
 *
 * @param address The register address
 * @param value The register value
 *
 * @return true If the value was set
 * @return false otherwise
 */
bool ModbusInterface::setSingleOutputRegister(int address, uint16_t value)
{
    m_ctx_guard.lock();
    bool result = modbus_write_register(m_ctx, address, value);
    m_ctx_guard.unlock();
    return result;

}

/**
 * @brief get the values of the input coils
 *
 * @return values The input coils values as vector
 */
std::vector<uint8_t> ModbusInterface::getMultipleInputCoils()
{
    m_memory_guard.lock();
    std::vector<uint8_t> temp = m_memory.input_coils;
    m_memory_guard.unlock();
    return temp;
}

/**
 * @brief get the values of the output coils
 *
 * @return values The output coils values as vector
 */
std::vector<uint8_t> ModbusInterface::getMultipleOutputCoils()
{
    m_memory_guard.lock();
    std::vector<uint8_t> temp = m_memory.output_coils;
    m_memory_guard.unlock();
    return temp;
}

/**
 * @brief set the values of the coils
 *
 * @param value The coils values
 *
 * @return true If the value was set
 * @return false otherwise
 */
bool ModbusInterface::setMultipleOutputCoils(std::vector<uint8_t> values)
{
    m_memory_guard.lock();
    int temp_to_write = m_memory.nb_output_coils;
    m_memory_guard.unlock();
    m_ctx_guard.lock();
    bool result = modbus_write_bits(m_ctx, 0, temp_to_write, values.data());
    m_ctx_guard.unlock();
    return result;
}

/**
 * @brief get the values of the input registers
 *
 * @return values The input registers values as vector
 */
std::vector<uint16_t> ModbusInterface::getMultipleInputRegisters()
{
    m_memory_guard.lock();
    std::vector<uint16_t> temp = m_memory.input_registers;
    m_memory_guard.unlock();
    return temp;
}

/**
 * @brief get the values of the output registers
 *
 * @return values The output registers values as vector
 */
std::vector<uint16_t> ModbusInterface::getMultipleOutputRegisters()
{
    m_memory_guard.lock();
    std::vector<uint16_t> temp = m_memory.output_registers;
    m_memory_guard.unlock();
    return temp;
}

/**
 * @brief set the values of the registers
 *
 * @param value The registers values
 *
 * @return true If the value was set
 * @return false otherwise
 */
bool ModbusInterface::setMultipleOutputRegisters(std::vector<uint16_t> values)
{
    m_memory_guard.lock();
    int temp_to_write = m_memory.nb_output_registers;
    m_memory_guard.unlock();
    m_ctx_guard.lock();
    bool result = modbus_write_registers(m_ctx, 0, temp_to_write, values.data());
    m_ctx_guard.unlock();
    return result;
}

/**
 * @brief Get a given IO value
 *
 * @param name The given IO name
 *
 * @return the IO value
 */
uint16_t ModbusInterface::getIOvalue(std::string name)
{
    m_IO_map_guard.lock();
    IO_struct temp_def = m_IO_map.at(name);
    m_IO_map_guard.unlock();

    if (temp_def.type == "input")
    {
        if (temp_def.data_type == "digital")
        {
            return getSingleInputCoil(temp_def.address);
        }
        else if (temp_def.data_type == "analog")
        {
            return getSingleInputRegister(temp_def.address);
        }
    }
    else if (temp_def.type == "output")
    {
        if (temp_def.data_type == "digital")
        {
            return getSingleOutputCoil(temp_def.address);
        }
        else if (temp_def.data_type == "analog")
        {
            return getSingleOutputRegister(temp_def.address);
        }
    }
    throw MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    return 0;
}

/**
 * @brief Get IO map
 *
 * @return a copy of the map with all declared IOs
 */
std::map<std::string, ModbusInterface::IO_struct> ModbusInterface::getIOMap()
{
    m_IO_map_guard.lock();
    std::map<std::string, IO_struct> map = m_IO_map;
    m_IO_map_guard.unlock();
    return map;
}

/**
 * @brief Set connection state to a given value
 *
 * @param state The state to set the connection state at
 */
void ModbusInterface::setConnectionState(bool state)
{
    m_connection_state_guard.lock();
    m_connected = state;
    m_connection_state_guard.unlock();
}

/**
 * @brief Get connection state
 *
 * @return state The connection state value
 */
bool ModbusInterface::getConnectionState()
{
    m_connection_state_guard.lock();
    bool state = m_connected;
    m_connection_state_guard.unlock();
    return state;
}

/**
 * @brief Initiate connection
 *
 * Tries to open a connection with the device using it's context via TCP
 *
 * @return true if the connection openned
 * @return false otherwise
 */
bool ModbusInterface::initiateConnection()
{
    m_ctx_guard.lock();
    if (modbus_connect(m_ctx) == 0)
    {
        m_ctx_guard.unlock();
        setConnectionState(true);
        return true;
    }
    else
    {
        m_ctx_guard.unlock();
        setConnectionState(false);
        return false;
    }
}

/**
 * @brief Restart connection
 *
 * Tries to reconnect
 *
 * @return true when reconnected
 * @return false otherwise
 */
bool ModbusInterface::restartConnection()
{
    try {
        setConnectionState(false);
        m_ctx_guard.lock();
        modbus_close(m_ctx);
        while(modbus_connect(m_ctx) == -1)
        {
            m_ctx_guard.unlock();
            sleep(1);
            m_ctx_guard.lock();
        }
        m_ctx_guard.unlock();
        setConnectionState(true);
        return true;
    } catch (...) {
        return false;
    }
}
