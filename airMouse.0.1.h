#pragma once

#include "resource.h"

#include <SerialClass.h>

class AirSerial : public Serial
{
private:
	int mBaudRate;
	unsigned int mInputInterval;
public:
	AirSerial(const char * portName, unsigned int baudRate); 
	int ReadDataUntil(char *buffer, unsigned int nbChar, char delim);
	int WaitForLine(std::string & buffer, unsigned int nbChar, double seconds);	//seconds - how long to wait for input
	int WaitForLine(std::string buffer);
	void CalculateInputInterval(std::string & buff, unsigned int nbChar, double seconds);
	int Unbuffer();

	class NotLineErr : public std::runtime_error
	{
	private:
		int chRead;
	public:
		NotLineErr(int bytesRead);
		NotLineErr(int bytesRead, const char * message);
		int GetByteCount() { return chRead; }
	};
	class WaitedTooLong : public std::runtime_error
	{
	private:
		double time;
	public:
		WaitedTooLong(double t) : std::runtime_error("Waited too long in Serial::WaitForLine()"), time(t) {}
		WaitedTooLong() : time(-1), std::runtime_error("Waited too long in Serial::WaitForLine()") {}
		WaitedTooLong(std::string strGotten) : runtime_error(strGotten) {}
		int GetTime() { return time; }
	};

};