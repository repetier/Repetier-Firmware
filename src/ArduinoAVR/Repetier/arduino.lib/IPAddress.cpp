
#include <Arduino.h>
#include <IPAddress.h>

IPAddress::IPAddress()
{
    memset(_address.a8, 0, sizeof(_address));
}

IPAddress::IPAddress(uint8_t first_octet, uint8_t second_octet, uint8_t third_octet, uint8_t fourth_octet)
{
    _address.a8[0] = first_octet;
    _address.a8[1] = second_octet;
    _address.a8[2] = third_octet;
    _address.a8[3] = fourth_octet;
}

IPAddress::IPAddress(uint32_t address)
{
	_address.a32=address;
}

IPAddress::IPAddress(const uint8_t *address)
{
    memcpy(_address.a8, address, sizeof(_address));
}

IPAddress& IPAddress::operator=(const uint8_t *address)
{
    memcpy(_address.a8, address, sizeof(_address));
    return *this;
}

IPAddress& IPAddress::operator=(uint32_t address)
{
	_address.a32=address;
    return *this;
}

bool IPAddress::operator==(const uint8_t* addr)
{
    return memcmp(addr, _address.a8, sizeof(_address)) == 0;
}

size_t IPAddress::printTo(Print& p) const
{
    size_t n = 0;
    for (int i =0; i < 3; i++)
    {
        n += p.print(_address.a8[i], DEC);
        n += p.print('.');
    }
    n += p.print(_address.a8[3], DEC);
    return n;
}

