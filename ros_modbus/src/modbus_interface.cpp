// -*- lsst-c++ -*-
/**
 * @file modbus_interface.cpp
 *
 * @brief modbus_interface functions definition
 */

#include <modbus_node/modbus_interface.h>

using namespace Modbus;

void ModbusInterface::setContext(std::string address, int port)
{
    m_address = address;
    m_port = port;
    m_ctx = modbus_new_tcp(address.c_str(), port);
}

void ModbusInterface::setDevice(int nb_input_coils, int nb_output_coils, int nb_input_registers, int nb_output_registers)
{
    m_memory_guard.lock();

    m_memory.nb_input_coils = nb_input_coils;
    m_memory.input_coils.resize(nb_input_coils);

    m_memory.nb_output_coils = nb_output_coils;
    m_memory.output_coils.resize(nb_output_coils);

    m_memory.nb_input_registers = nb_input_registers;
    m_memory.input_registers.resize(nb_input_registers);

    m_memory.nb_output_registers = nb_output_registers;
    m_memory.output_registers.resize(nb_output_registers);

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

void ModbusInterface::updateMemory()
{
    //update input coils
    m_memory_guard.lock();
    m_temp_nb_to_get =  m_memory.nb_input_coils;
    m_memory_guard.unlock();
    m_ctx_guard.lock();
    m_success = modbus_read_input_bits(m_ctx, 0,m_temp_nb_to_get , &m_temp8) != -1;
    m_ctx_guard.unlock();
    if(!m_success)
    {
        throw MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE;
    }
    m_memory_guard.lock();
    m_memory.input_coils = std::vector<uint8_t>(m_temp8, m_temp_nb_to_get);
    m_memory_guard.unlock();

    //update output coils
    m_memory_guard.lock();
    m_temp_nb_to_get =  m_memory.nb_output_coils;
    m_memory_guard.unlock();
    m_ctx_guard.lock();
    m_success = modbus_read_bits(m_ctx, 0,m_temp_nb_to_get , &m_temp8) != -1;
    m_ctx_guard.unlock();
    if(!m_success)
    {
        throw MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE;
    }
    m_memory_guard.lock();
    m_memory.output_coils = std::vector<uint8_t>(m_temp8, m_temp_nb_to_get);
    m_memory_guard.unlock();

    //update input registers
    m_memory_guard.lock();
    m_temp_nb_to_get =  m_memory.nb_input_registers;
    m_memory_guard.unlock();
    m_ctx_guard.lock();
    m_success = modbus_read_input_registers(m_ctx, 0,m_temp_nb_to_get , &m_temp16) != -1;
    m_ctx_guard.unlock();
    if(!m_success)
    {
        throw MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE;
    }
    m_memory_guard.lock();
    m_memory.input_registers = std::vector<uint16_t>(m_temp16, m_temp_nb_to_get);
    m_memory_guard.unlock();

    //update output registers
    m_memory_guard.lock();
    m_temp_nb_to_get =  m_memory.nb_output_registers;
    m_memory_guard.unlock();
    m_ctx_guard.lock();
    m_success = modbus_read_registers(m_ctx, 0,m_temp_nb_to_get , &m_temp16) != -1;
    m_ctx_guard.unlock();
    if(!m_success)
    {
        throw MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE;
    }
    m_memory_guard.lock();
    m_memory.output_registers = std::vector<uint16_t>(m_temp16, m_temp_nb_to_get);
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
