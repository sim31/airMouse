#include "airMouse.0.1.h"
#include <Log.h>
#include <string>
#include <sstream>
#include <Status.h>


extern Log gLog;
extern Status active;

//mano (Tado)
int AirSerial::ReadDataUntil(char * buff, unsigned int nbChar, char delim)
{
	int chRead = ReadData(buff, nbChar);
	int i;
	for (i = 0; i < chRead; i++)
	{
		if (buff[i] == delim)
		{
			buff[i] = '\0';
			break;
		}
	}
	return i;
}

int AirSerial::WaitForLine(std::string  & line, unsigned int nbChar, double seconds)
{
	Win32Timer timer;
	char ch;
	int chRead = 0;
	double timePassed = 0;
	timer.GetCurrTime();
	double timeEllapsed = 0;
	line = "";
	while (IsConnected() && line.size() <= nbChar && (line.size() == 0 || line.back() != '\n'))
	{
		chRead = ReadData(&ch, 1);
		if (chRead == 1)
			line += ch;
		timeEllapsed = timer.GetTimeEllapsed();
		timePassed += timeEllapsed;
		if (timePassed > seconds)
		{
			std::ostringstream stream;
			stream << "Waited too long in WaitForLine(). " << line;
			gLog.Write(stream.str());
			throw WaitedTooLong(stream.str());
		}
		if (mInputInterval > 0)
		{
			gLog.Write("Sleeping in AirSerial::WaitForLine()");
			Sleep(mInputInterval);
		}
	}
	if (!IsConnected())
	{
		throw std::runtime_error("Disconected from COM. In AirSerial::WaitForLine()");
	}
	if (line.back() != '\n')
	 {
		if (line.size() >=nbChar - 1)
		{
			gLog.Write(line);
			throw NotLineErr(line.size(), "Not line read in AirSerial::WaitForLine(). Bad input.");
		}
		else
		{
			gLog.Write(line);
			throw NotLineErr(line.size(), "Not line read in AirSerial::WaitForLine(). Too small buffer or bad input");
		}
	}
	return line.size();
}

void AirSerial::CalculateInputInterval(std::string & buff, unsigned int nbChar, double time)
{
	Win32Timer timer;
	timer.GetCurrTime();
	int bytes = WaitForLine(buff, nbChar, time);
	unsigned int timePassed = timer.GetTimeEllapsed() * 1000;
	mInputInterval = timePassed / bytes;
}

int AirSerial::Unbuffer()
{
	int chRead = 0;
	int chReadTotal = 0;
	char buff[10000];
	while (chRead > 0)
	{
		chRead = ReadData(buff, 10000);
		if (chRead > 0)
			chReadTotal += chRead;
	}
	return chRead;
}

AirSerial::NotLineErr::NotLineErr(int bytesRead)
	: std::runtime_error("No line read in AirSerial::WaitForLine(), too small buffer"),
	chRead(bytesRead)
{
}

AirSerial::NotLineErr::NotLineErr(int bytesRead, const char * message)
	: std::runtime_error(message),
	chRead(bytesRead)
{
}

int AirSerial::WaitForLine(std::string buffer)
{
	DWORD mask = 0;
	buffer = "";
	while ((buffer.size() == 0 || buffer.back() != '\n'))
	{
		while (mask != EV_RXCHAR)
		{
			if (!WaitCommEvent(hSerial, &mask, NULL))
			{
				std::ostringstream stream;
				stream << "Error while waiting for comm event: " << GetLastError();
				gLog.Write(stream.str());
				throw std::runtime_error(stream.str());
			}
		}
		char buff[500];
		int chRead = ReadData(buff, 500);
		if (chRead > 0)
		{
			active.Set(true);
			buffer += buff;
		}
	}
	return buffer.size();

}

AirSerial::AirSerial(const char * portName, unsigned int baudRate) : Serial(portName, baudRate), mInputInterval(0)
{
	//We're not yet connected
//	this->connected = false;
//	mBaudRate = baudRate;

	//Try to connect to the given port throuh CreateFile
//	this->hSerial = CreateFile(portName,
//		GENERIC_READ | GENERIC_WRITE,
//		0,
//		NULL,
//		OPEN_EXISTING,
//		FILE_ATTRIBUTE_NORMAL,
//		NULL);

	//Check if the connection was successfull
//	if (this->hSerial == INVALID_HANDLE_VALUE)
//	{
//		//If not success full display an Error
//		if (GetLastError() == ERROR_FILE_NOT_FOUND){
//
			//Print Error if neccessary
//			printf("ERROR: Handle was not attached. Reason: %s not available.\n", portName);

//		}
//		else
//		{
//			printf("ERROR!!!");
//		}
//	}
//	else
//	{
//		//If connected we try to set the comm parameters
//		DCB dcbSerialParams = { 0 };

		//Try to get the current
//		if (!GetCommState(this->hSerial, &dcbSerialParams))
//		{
//			//If impossible, show an error
//			throw std::runtime_error("Failed to get comm state in AirSerial::AirSerial()");
//		}
//		else
//		{

//			if (!SetCommMask(hSerial, EV_RXCHAR))
//			{
//				std::ostringstream stream;
//				stream << "SetCommMask() failed with error: " << GetLastError();
//				std::runtime_error(stream.str());
//			}
//			//Define serial connection parameters for the arduino board
//			dcbSerialParams.BaudRate = baudRate;
//			dcbSerialParams.ByteSize = 8;
//			dcbSerialParams.StopBits = ONESTOPBIT;
//			dcbSerialParams.Parity = NOPARITY;
//			dcbSerialParams.EvtChar = '\n';
//
//			//Set the parameters and check for their proper application
//			if (!SetCommState(hSerial, &dcbSerialParams))
//			{
//				throw std::runtime_error("ALERT: Could not set Serial Port parameters");
//			}
//			else
//			{
//				//If everything went fine we're connected
//				this->connected = true;
//				//We wait 2s as the arduino board will be reseting
//				Sleep(ARDUINO_WAIT_TIME);
//			}
//
//		}
//	}

}