// -*- lsst-c++ -*-
/**
 * @file modbus_interface.cpp
 *
 * @brief modbus_interface functions definition
 */

#include <modbus_node/modbus_interface.h>
#include <iostream>

using namespace Modbus;

void ModbusInterface::setContext(std::string address, int port)
{
    m_address = address;
    m_port = port;
    m_ctx = modbus_new_tcp(address.c_str(), port);
}

std::pair<std::string, int> ModbusInterface::getContext()
{
    return std::pair<std::string, int>(m_address, m_port);
}

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

bool ModbusInterface::hasIODeclared(std::string key)
{
    m_IO_map_guard.lock();
    bool result = m_IO_map.find(key) != m_IO_map.end();
    m_IO_map_guard.unlock();
    return result;
}

void ModbusInterface::updateMemory()
{
    //update input coils
    m_memory_guard.lock();
    m_temp_nb_to_get =  m_memory.nb_input_coils;
    m_memory_guard.unlock();
    uint8_t bufferid[m_temp_nb_to_get];
    uint8_t *m_temp8id = bufferid;
    m_ctx_guard.lock();
    m_success = modbus_read_input_bits(m_ctx, 0, m_temp_nb_to_get , m_temp8id) != -1;
    m_ctx_guard.unlock();
    if(!m_success)
    {
        throw MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE;
    }
    m_memory_guard.lock();
    std::copy(m_temp8id, m_temp8id+m_temp_nb_to_get, m_memory.input_coils.begin());
    m_memory_guard.unlock();

    //update output coils
    m_memory_guard.lock();
    m_temp_nb_to_get =  m_memory.nb_output_coils;
    m_memory_guard.unlock();
    uint8_t bufferod[m_temp_nb_to_get];
    uint8_t *m_temp8od = bufferod;
    m_ctx_guard.lock();
    m_success = modbus_read_bits(m_ctx, 0,m_temp_nb_to_get , m_temp8od) != -1;
    m_ctx_guard.unlock();
    if(!m_success)
    {
        throw MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE;
    }
    m_memory_guard.lock();
    std::copy(m_temp8od, m_temp8od+m_temp_nb_to_get, m_memory.output_coils.begin());
    m_memory_guard.unlock();

    //update input registers
    m_memory_guard.lock();
    m_temp_nb_to_get =  m_memory.nb_input_registers;
    m_memory_guard.unlock();
    uint16_t bufferia[2*m_temp_nb_to_get];
    uint16_t *m_temp16ia = bufferia;
    m_ctx_guard.lock();
    m_success = modbus_read_input_registers(m_ctx, 0,m_temp_nb_to_get , m_temp16ia) != -1;
    m_ctx_guard.unlock();
    if(!m_success)
    {
        throw MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE;
    }
    m_memory_guard.lock();
    std::copy(m_temp16ia, m_temp16ia+m_temp_nb_to_get, m_memory.input_registers.begin());
    m_memory_guard.unlock();

    //update output registers
    m_memory_guard.lock();
    m_temp_nb_to_get =  m_memory.nb_output_registers;
    m_memory_guard.unlock();
    uint16_t bufferoa[2*m_temp_nb_to_get];
    uint16_t *m_temp16oa = bufferoa;
    m_ctx_guard.lock();
    m_success = modbus_read_registers(m_ctx, 0,m_temp_nb_to_get , m_temp16oa) != -1;
    m_ctx_guard.unlock();
    if(!m_success)
    {
        throw MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE;
    }
    m_memory_guard.lock();
    std::copy(m_temp16oa, m_temp16oa+m_temp_nb_to_get, m_memory.output_registers.begin());
    m_memory_guard.unlock();

}

uint8_t ModbusInterface::getSingleInputCoil(int address)
{
    m_memory_guard.lock();
    uint8_t temp = m_memory.input_coils[address];
    m_memory_guard.unlock();
    return temp;

}

uint8_t ModbusInterface::getSingleOutputCoil(int address)
{
    m_memory_guard.lock();
    uint8_t temp = m_memory.output_coils[address];
    m_memory_guard.unlock();
    return temp;

}

bool ModbusInterface::setSingleOutputCoil(int address, uint8_t value)
{
    m_ctx_guard.lock();
    bool result = modbus_write_bit(m_ctx, address, value);
    m_ctx_guard.unlock();
    return result;
}

uint16_t ModbusInterface::getSingleInputRegister(int address)
{
    m_memory_guard.lock();
    uint16_t temp = m_memory.input_registers[address];
    m_memory_guard.unlock();
    return temp;

}

uint16_t ModbusInterface::getSingleOutputRegister(int address)
{
    m_memory_guard.lock();
    uint16_t temp = m_memory.output_registers[address];
    m_memory_guard.unlock();
    return temp;

}

bool ModbusInterface::setSingleOutputRegister(int address, uint16_t value)
{
    m_ctx_guard.lock();
    bool result = modbus_write_register(m_ctx, address, value);
    m_ctx_guard.unlock();
    return result;

}

std::vector<uint8_t> ModbusInterface::getMultipleInputCoils()
{
    m_memory_guard.lock();
    std::vector<uint8_t> temp = m_memory.input_coils;
    m_memory_guard.unlock();
    return temp;
}

std::vector<uint8_t> ModbusInterface::getMultipleOutputCoils()
{
    m_memory_guard.lock();
    std::vector<uint8_t> temp = m_memory.output_coils;
    m_memory_guard.unlock();
    return temp;
}

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

std::vector<uint16_t> ModbusInterface::getMultipleInputRegisters()
{
    m_memory_guard.lock();
    std::vector<uint16_t> temp = m_memory.input_registers;
    m_memory_guard.unlock();
    return temp;
}

std::vector<uint16_t> ModbusInterface::getMultipleOutputRegisters()
{
    m_memory_guard.lock();
    std::vector<uint16_t> temp = m_memory.output_registers;
    m_memory_guard.unlock();
    return temp;
}

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

std::map<std::string, ModbusInterface::IO_struct> ModbusInterface::getIOMap()
{
    m_IO_map_guard.lock();
    std::map<std::string, IO_struct> map = m_IO_map;
    m_IO_map_guard.unlock();
    return map;
}

void ModbusInterface::setConnectionState(bool state)
{
    m_connection_state_guard.lock();
    m_connected = state;
    m_connection_state_guard.unlock();
}

bool ModbusInterface::getConnectionState()
{
    m_connection_state_guard.lock();
    bool state = m_connected;
    m_connection_state_guard.unlock();
    return state;
}

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
