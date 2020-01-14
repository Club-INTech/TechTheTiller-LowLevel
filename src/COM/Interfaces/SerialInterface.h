#ifndef _SERIALHL_h
#define _SERIALHL_h

#include <Arduino.h>
#include <vector>

#include "Utils/Singleton.hpp"
#include <cstdarg>
#include "Config/Defines.h"
#include "Utils/Average.hpp"
#include "AbstractComInterface.h"


class SerialInterface : public AbstractComInterface
{
public:
	SerialInterface() = default;

	/* RECEPTION */

	bool read(char*);
	bool read(int32_t&);
	bool read(int16_t&);
	bool read(volatile int8_t &);
	bool read(float&);

	void printf(const char*);
	void printfln(const char*);

private:
	bool read_char(char &);
	String data;
	uint8_t available();
};
#endif
