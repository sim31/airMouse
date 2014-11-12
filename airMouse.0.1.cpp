// airMouse.0.1.cpp : Defines the entry point for the application.
//

#include <tchar.h>
#include <Windows.h>
#include "airMouse.0.1.h"
#include <Log.h>
#include <SerialClass.h>
#include <fstream>
#include <sstream>
#include <thread>
#include <Status.h>
#include <string>

#define MAX_LOADSTRING 100
#define TRAY_MESSAGE WM_USER+1
#define SETTINGS_ID 3
#define EXIT_ID 2
#define TRAY_ID 1
#define QUIT_HOTKEY_ID 4

struct Settings
{
	std::string portName;
	double xDeadZone;
	double yDeadZone;
	double xSpeed;
	double ySpeed;
	double rollDiff;
	double yawDiff;
} settings;

struct YawPitchRoll
{
	double yaw;
	double pitch;
	double roll;
	YawPitchRoll() : yaw(0), pitch(0), roll(0) {}
	bool operator!=(YawPitchRoll ang2) { return (yaw != ang2.yaw || pitch != ang2.pitch || roll != ang2.roll); }
};

// Global Variables:
HINSTANCE hInst;								// current instance
TCHAR szTitle[MAX_LOADSTRING];					// The title bar text
TCHAR szWindowClass[MAX_LOADSTRING];			// the main window class name
HWND gHwnd;
NOTIFYICONDATA iconData = { 0 };
INPUT input = { 0 };
Log gLog;
Win32Timer gTimer;
Status run(true);
HACCEL hAccelTable;
const std::string recognitionCode = "air.0.1-ypr\r\n";
int xScreen, yScreen;
Status active(true);

// Forward declarations of functions included in this code module:
ATOM				MyRegisterClass(HINSTANCE hInstance);
BOOL				InitInstance(HINSTANCE, int);
LRESULT CALLBACK	WndProc(HWND, UINT, WPARAM, LPARAM);
void				Notification(char * text, UINT flags = NIIF_INFO, bool realTime = true);	//rest is set in iconData 
void				GetSettings(Settings & settings);
void				InitializeInputStruct();
void				MoveCursor(int x, int y);
void				Input(INPUT * input);
void				CalculateMovementData();
void				MyLoop();
bool				Check();		//check run and calls DestroyWindow if not
void				WaitForCalm();
double				GetInputInterval(AirSerial & serial);		//discard some inputs and returns interval between inputs from arduino (seconds)
void				LeftMouseBt(bool down);	//if down = true - button down event if false - released
void				RightMouseBt(bool down);
int					ReadLine(AirSerial & serial, std::string & buff, int chToRead, double waitTime);
double				GetBiggestInputInterval(AirSerial & serial);

int APIENTRY _tWinMain(_In_ HINSTANCE hInstance,
                     _In_opt_ HINSTANCE hPrevInstance,
                     _In_ LPTSTR    lpCmdLine,
                     _In_ int       nCmdShow)
{
	// Initialize global strings
	LoadString(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
	LoadString(hInstance, IDC_AIRMOUSE01, szWindowClass, MAX_LOADSTRING);

	gLog.Open(szTitle);
	std::thread t1(MyLoop);
	try
	{


		UNREFERENCED_PARAMETER(hPrevInstance);
		UNREFERENCED_PARAMETER(lpCmdLine);

		MyRegisterClass(hInstance);

		// Perform application initialization:
		if (!InitInstance(hInstance, nCmdShow))
		{
			return FALSE;
		}

		hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_AIRMOUSE01));

		RegisterHotKey(gHwnd, QUIT_HOTKEY_ID, MOD_CONTROL, 0x51);	//Q key

		gLog.Write("Initialized win32 program.");
		gLog.Write("Starting Win32 message loop");
		MSG msg;
		while (GetMessage(&msg, NULL, 0, 0))
		{
			if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
			{
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}
		}
		if (run())		//means window was destroyed and we need to stop other thread (other thread stops program by sending destroy message)
			run.Set(false);
	}

	catch (std::runtime_error & err)
	{
		MessageBox(NULL, err.what(), "runtime error", MB_ICONERROR);
		gLog.Write(err.what());
		SendMessage(gHwnd, WM_DESTROY, 0, 0);
		run.Set(false);
	}
	if (t1.joinable())
		t1.join();
	return 0;
}



//
//  FUNCTION: MyRegisterClass()
//
//  PURPOSE: Registers the window class.
//
ATOM MyRegisterClass(HINSTANCE hInstance)
{
	WNDCLASSEX wcex;

	wcex.cbSize = sizeof(WNDCLASSEX);

	wcex.style			= CS_HREDRAW | CS_VREDRAW;
	wcex.lpfnWndProc	= WndProc;
	wcex.cbClsExtra		= 0;
	wcex.cbWndExtra		= 0;
	wcex.hInstance		= hInstance;
	wcex.hIcon			= LoadIcon(hInstance, MAKEINTRESOURCE(IDI_AIRMOUSE01));
	wcex.hCursor		= LoadCursor(NULL, IDC_ARROW);
	wcex.hbrBackground	= (HBRUSH)(COLOR_WINDOW+1);
	wcex.lpszMenuName	= MAKEINTRESOURCE(IDC_AIRMOUSE01);
	wcex.lpszClassName	= szWindowClass;
	wcex.hIconSm = 0;

	return RegisterClassEx(&wcex);
}

//
//   FUNCTION: InitInstance(HINSTANCE, int)
//
//   PURPOSE: Saves instance handle and creates main window
//
//   COMMENTS:
//
//        In this function, we save the instance handle in a global variable and
//        create and display the main program window.
//
BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
   HWND hWnd;

   hInst = hInstance; // Store instance handle in our global variable

   hWnd = CreateWindow(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
      CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, NULL, NULL, hInstance, NULL);

   if (!hWnd)
   {
      return FALSE;
   }
   gHwnd = hWnd;

   //create tray icon
   ZeroMemory(&iconData, sizeof(iconData));
   iconData.cbSize = sizeof iconData;
   iconData.hWnd = hWnd;
   iconData.uID = TRAY_ID;
   iconData.uFlags = NIF_ICON | NIF_MESSAGE | NIF_TIP;
   iconData.hIcon = LoadIcon(hInst, MAKEINTRESOURCE(IDI_SMALL));
   iconData.uCallbackMessage = TRAY_MESSAGE;
   strcpy_s(iconData.szTip, 128, szTitle);
   strcpy_s(iconData.szInfo, 128, szTitle);
   Shell_NotifyIcon(NIM_ADD, &iconData);

   //ShowWindow(hWnd, nCmdShow);
   UpdateWindow(hWnd);

   return TRUE;
}

//
//  FUNCTION: WndProc(HWND, UINT, WPARAM, LPARAM)
//
//  PURPOSE:  Processes messages for the main window.
//

LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	switch (message)
	{
	case TRAY_MESSAGE:
	{
		switch (lParam)
		{
		case WM_LBUTTONDOWN:
			Notification("airMouse working", NIIF_INFO, true);
			break;
		case WM_RBUTTONDOWN:
		case WM_CONTEXTMENU:
		{
			//showing context menu
			HMENU hM = CreatePopupMenu();
			InsertMenu(hM, 0, MF_BYPOSITION | MF_STRING, EXIT_ID, "Exit");
			InsertMenu(hM, 0, MF_BYPOSITION | MF_STRING, SETTINGS_ID, "Settings");
			POINT point;
			GetCursorPos(&point);
			TrackPopupMenu(hM, TPM_BOTTOMALIGN | TPM_LEFTALIGN, point.x, point.y, 0, hWnd, NULL);
		}
		}
		break;
	}
	case WM_HOTKEY:
	{
		gLog.Write("Terminated by user (ctrl+q pressed)");
		Notification("AirMouse quiting...");
		Sleep(2500);
		DestroyWindow(hWnd);
		break;
	}
	case WM_COMMAND:
		if (LOWORD(wParam) == EXIT_ID)
		{
			gLog.Write("Terminated by user");
			Notification("AirMouse quiting...");
			Sleep(2000);
			DestroyWindow(hWnd);	//sends WM_DESTROY message
		}
		else if (LOWORD(wParam) == SETTINGS_ID)
		{
			system("notepad Settings.txt");
		}
		break;
	case WM_DESTROY:
		Shell_NotifyIcon(NIM_DELETE, &iconData);
		DestroyIcon(iconData.hIcon);
		PostQuitMessage(0);
		break;
	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}
	return 0;
}

void Notification(char * text, UINT infoFlags, bool realTime)
{
	std::mutex mu;
	mu.lock();
	strcpy_s(iconData.szInfo, 256, text);
	iconData.dwInfoFlags = infoFlags;
	iconData.uFlags = realTime ? NIF_INFO | NIF_REALTIME : NIF_INFO;
	Shell_NotifyIcon(NIM_MODIFY, &iconData);
	mu.unlock();
}

void GetSettings(Settings & settings)
{
	std::ifstream fin("Settings.txt");
	std::string str;
	if (!fin.is_open())
		throw std::runtime_error("failed to open settings file. In GetSettings().");
	fin.ignore(1000, '=');
	std::getline(fin, settings.portName);

	std::stringstream stream;
	stream << "\\\\.\\" << settings.portName;
	settings.portName = stream.str();
	if (!fin.good())
		throw std::runtime_error("failed reading port name in settings file. In GetSettings().");

	fin.ignore(1000, '=');
	fin >> settings.xDeadZone;
	if (!fin.good())
		throw std::runtime_error("failed reading xDeadZone in settings file. In GetSettings().");
	
	fin.ignore(1000, '=');
	fin >> settings.yDeadZone;
	if (!fin.good())
		throw std::runtime_error("Failed when reading yDeadZone in settings file. In GetSettings().");

	fin.ignore(1000, '=');
	fin >> settings.xSpeed;
	if (!fin.good())
		throw std::runtime_error("Failed when reading xSpeed in Settings.txt file.");

	fin.ignore(1000, '=');
	fin >> settings.ySpeed;
	if (!fin.good())
		throw std::runtime_error("Failed when reading ySpeed from Settings.txt");

	fin.close();
}

void InitializeInputStruct()
{
	input.type = INPUT_MOUSE;
	input.mi.dwFlags = MOUSEEVENTF_MOVE;
}

void MoveCursor(int x, int y)
{
	input.mi.dx = x;
	input.mi.dy = y;
	input.mi.dwFlags = MOUSEEVENTF_MOVE;
	Input(&input);
}

void Input(INPUT * input)
{
	UINT rval = SendInput(1, input, sizeof *input);
	if (rval = 0)
		throw std::runtime_error("SendInput failed. In Input()");
}

void CalculateMovementData()
{
    xScreen = GetSystemMetrics(SM_CXSCREEN);
	yScreen = GetSystemMetrics(SM_CYSCREEN);
}


void MyLoop()
{
	try
	{
		std::string line;
		gLog.Write("Reading settings file");
		GetSettings(settings);
		gLog.Write("Read settings file succesfully. Opening port");

		AirSerial serial(settings.portName.c_str(), 115200);
		if (!serial.IsConnected())
			throw std::runtime_error("Failed to open port.");
		serial.Unbuffer();
		gLog.Write("Port opened. Checking communications");
		char ch = 'w';
		serial.WriteData(&ch, 1);	//wake arduino
		ReadLine(serial, line, 20, 5);
		if (line != recognitionCode)
		{
			gLog.Write("Wrong recognition code: " + line + " should be: " + recognitionCode);
			throw std::runtime_error("Wrong recognition code");
		}
		gLog.Write("Communications ok. Initializing mouse input structure.");

		InitializeInputStruct();
		gLog.Write("Calculating mouse movement data.");
		CalculateMovementData();

		const int loggingInterval = 1;	//in seconds
		double timeSinceLog = 0;
		gTimer.GetCurrTime();
		std::istringstream stream;
		YawPitchRoll angle;
		bool buttonStates[3];
		bool prevButtonStates[3];
		double x, y, z;
		gLog.Write("Initializing MPU-6050");
		ReadLine(serial, line, 4, 10);
		if (line == "ok\r\n")
			gLog.Write("MPU-6050 initialization succesfull");
		else if (line == "err1\r\n")
		{
			ReadLine(serial, line, 100, 10);
			gLog.Write(line);
			throw std::runtime_error(line.c_str());
		}
		else
		{
			gLog.Write(line);
			throw std::runtime_error("Unknown data sent while waiting for MPU-6050 initialization.");
		}
		// Main loop:
		ch = 'r';
		serial.WriteData(&ch, 1);
		//WaitForCalm();
		double iInterval = GetInputInterval(serial);
		double tSinceLine, time, tSinceLogT;
		serial.Unbuffer();
		gTimer.GetCurrTime();
		Notification("Start using airMouse");
		gLog.Write("Starting main mouse movement loop");
		while (run())
		{
			if (!serial.IsConnected())
			{
				char str[] = "Connection with serial port dissapeared. Exiting";
				Notification(str);
				gLog.Write(str);
				SendMessage(gHwnd, WM_DESTROY, 0, 0);
				break;
			}

			Sleep(iInterval * 1000);
			ReadLine(serial, line, 400, 15);
			tSinceLine = gTimer.GetCurrTime();

			stream.str(line);
			char ch = stream.peek();
			//if (ch < 0 || ch > 9)		//if not digits (gyroscope data)
			//{
			//	gLog.Write(stream.str());
			//	throw std::runtime_error("Bad input from serial port.");
			//}

			stream >> angle.yaw >> angle.pitch >> angle.roll;
			//get button states
			for (int i = 0; i < 3; i++)
				stream >> buttonStates[i];
			if (!stream.good())
			{
				gLog.Write(stream.str());
				throw std::runtime_error("Bad input from serial port");
			}
			timeSinceLog += gTimer.GetCurrTime() - tSinceLogT;
			if (timeSinceLog > loggingInterval)
			{
				timeSinceLog = 0;
				gLog.Write(stream.str());
			}
			tSinceLogT = gTimer.GetCurrTime();

			//CHANGES//
			z += angle.pitch;
			x += angle.yaw * 30 * settings.xSpeed;
			y += angle.roll * -30 * settings.ySpeed;

			if (abs(x) >= 1 || abs(y) >= 1)
			{
				POINT pos;
				GetCursorPos(&pos);
				if (pos.x + x > xScreen)
					x = xScreen;
				else if (pos.x + x < 1)
					x = 1;
				if (pos.y + y > yScreen)
					y = yScreen;
				else if (pos.y + y < 1)
					y = 1;
				MoveCursor(x, y);
				if (abs(x) >= 1)
					x = 0.0;
				if (abs(y) >= 1)
					y = 0.0;
			}


			if (prevButtonStates[0] != buttonStates[0])
				LeftMouseBt(!buttonStates[0]);
			if (prevButtonStates[1] != buttonStates[1])
				RightMouseBt(!buttonStates[1]);

			for (int i = 0; i < 3; i++)
				prevButtonStates[i] = buttonStates[i];


		}
	}
	catch (AirSerial::WaitedTooLong & err)
	{
		Notification("Connection with arduino lost. Exiting...");
		Sleep(3000);
		gLog.Write(err.what());
		SendMessage(gHwnd, WM_DESTROY, 0, 0);
		run.Set(false);
		return;
	}
	catch (std::runtime_error & err)
	{
		MessageBox(NULL, err.what(), "runtime error", MB_ICONERROR);
		gLog.Write(err.what());
		SendMessage(gHwnd, WM_DESTROY, 0, 0);
		run.Set(false);
		return;
	}
	run.Set(false);
}

void WaitForCalm()
{
	gLog.Write("Waiting for calm data...");
	Notification("Please wait before airMouse becomes active...");
	Sleep(20000);
	Notification("airMouse active now!");
}

double GetInputInterval(AirSerial & serial)
{
	gLog.Write("Checking input intervals:");
	double sum = 0;
	std::string buff;
	std::ostringstream stream;
	
	for (int i = 0; i < 5; i++)
	{
		gTimer.GetCurrTime();
		ReadLine(serial, buff, 500, 10);
		double interval = gTimer.GetTimeEllapsed();
		sum += interval;
		stream << " " << interval;
		gLog.Write(stream.str());
		stream.str("");
	}

	double avg = sum / 5;
	stream << "Average input interval is: " << avg;
	gLog.Write(stream.str());
	return avg;
}

double GetBiggestInputInterval(AirSerial & serial)
{
	gLog.Write("Checking biggest input intervals:");
	double biggest = 0;
	std::string buff;
	std::ostringstream stream;

	for (int i = 0; i < 5; i++)
	{
		gTimer.GetCurrTime();
		ReadLine(serial, buff, 500, 10);
		double interval = gTimer.GetTimeEllapsed();
		stream << " " << interval;
		gLog.Write(stream.str());
		stream.str("");
		if (biggest < interval)
			biggest = interval;
	}

	stream << "Biggest input interval is: " << biggest;
	gLog.Write(stream.str());
	return biggest;
}


void LeftMouseBt(bool down)
{
	
	input.mi.dx = input.mi.dy = 0;
	if (down)
		input.mi.dwFlags = MOUSEEVENTF_LEFTDOWN;
	else
		input.mi.dwFlags = MOUSEEVENTF_LEFTUP;
	Input(&input);
}

void RightMouseBt(bool down)
{
	input.mi.dx = input.mi.dy = 0;
	if (down)
		input.mi.dwFlags = MOUSEEVENTF_RIGHTDOWN;
	else
		input.mi.dwFlags = MOUSEEVENTF_RIGHTUP;
	Input(&input);
}


int ReadLine(AirSerial & serial, std::string & buff, int chToRead, double waitTime)
{
	int chRead = serial.WaitForLine(buff, chToRead, waitTime);
	char ch = 'c';
	while (buff == "c\r\n")
	{
		serial.WriteData(&ch, 1);
		chRead = serial.WaitForLine(buff, chToRead, waitTime);
	}

	return chRead;
}