/*	Copyright (c) 2003-2016 Xsens Technologies B.V. or subsidiaries worldwide.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1.	Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2.	Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3.	Neither the names of the copyright holders nor the names of their contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

///////////////////////////////////////////////////////////////////////////
// Adapted by Leonardo Felipe L. S. dos Santos, 2019-2023 (@qleonardolp) //
///////////////////////////////////////////////////////////////////////////
#pragma optimize( "2", on )

#undef UNICODE

#define WIN32_LEAN_AND_MEAN

#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>

#pragma comment(lib, "Ws2_32.lib")

#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT "2324"
#define	TCP_ENABLE		0

#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <utility>
#include <string>
#include <list>
#include <set>

#include "AXIS.h"
#include "EPOS_NETWORK.h"
#include "generalheader.h"
#include "Controller.h"
#include "qASGD_KF.h"
#include "LowPassFilter2p.h"

#include "findClosestUpdateRate.h"
#include "mastercallback.h"
#include "mtwcallback.h"

#include <xsensdeviceapi.h> // The Xsens device API header
#include <xsens/xsmutex.h>
#include "xstypes.h"
#include <conio.h>
#include <thread>
#include <chrono>
#include <Eigen/Core>

#define XSENS_RATE 120          // Use 120 Hz update rate for MTw, 150 Hz usually crashes!
#define XSENS_FC 60             // IMU cutoff frequency
#define XSENS_CH 25             // Use radio channel 25 for wireless master.
#define CALIBRATION_PERIOD 3.0f // Gyroscope Bias integration period

using namespace Eigen;

void Habilita_Eixo(int ID);

void Desabilita_Eixo(int ID);

/*! \brief Stream insertion operator overload for XsPortInfo */
std::ostream &operator<<(std::ostream &out, XsPortInfo const &p)
{
  out << "Port: " << std::setw(2) << std::right << p.portNumber() << " (" << p.portName().toStdString() << ") @ "
      << std::setw(7) << p.baudrate() << " Bd"
      << ", "
      << "ID: " << p.deviceId().toString().toStdString();
  return out;
}

/*! \brief Stream insertion operator overload for XsDevice */
std::ostream &operator<<(std::ostream &out, XsDevice const &d)
{
  out << "ID: " << d.deviceId().toString().toStdString() << " (" << d.productCode().toStdString() << ")";
  return out;
}

int main(int argc, char **argv)
{

  WSADATA wsaData;
  int iResult;

  SOCKET ListenSocket = INVALID_SOCKET;
  SOCKET ClientSocket = INVALID_SOCKET;

  struct addrinfo *result = NULL;
  struct addrinfo hints;

  int iSendResult;
  char recvbuf[DEFAULT_BUFLEN];
  int recvbuflen = DEFAULT_BUFLEN;

  // Initialize Winsock
  iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
  if (iResult != 0)
  {
    printf("WSAStartup failed with error: %d\n", iResult);
    //return 1;
  }

  ZeroMemory(&hints, sizeof(hints));
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_protocol = IPPROTO_TCP;
  hints.ai_flags = AI_PASSIVE;

  // Resolve the server address and port
  iResult = getaddrinfo(NULL, DEFAULT_PORT, &hints, &result);
  if (iResult != 0)
  {
    printf("getaddrinfo failed with error: %d\n", iResult);
    WSACleanup();
    //return 1;
  }

  // Create a SOCKET for connecting to server
  ListenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
  if (ListenSocket == INVALID_SOCKET)
  {
    printf("socket failed with error: %ld\n", WSAGetLastError());
    freeaddrinfo(result);
    WSACleanup();
    //return 1;
  }

#if TCP_ENABLE
  // Setup the TCP listening socket
  iResult = bind(ListenSocket, result->ai_addr, (int)result->ai_addrlen);
  if (iResult == SOCKET_ERROR)
  {
    printf("bind failed with error: %d\n", WSAGetLastError());
    freeaddrinfo(result);
    closesocket(ListenSocket);
    WSACleanup();
    //return 1;
  }

  BOOL bOptVal = TRUE;
  int  bOptLen = sizeof(BOOL);

  iResult = setsockopt(ListenSocket, IPPROTO_TCP, TCP_NODELAY, (char*) &bOptVal, bOptLen);
  if (iResult == SOCKET_ERROR)
  {
    printf("setsockopt failed with error: %d\n", WSAGetLastError());
    closesocket(ListenSocket);
    WSACleanup();
    //return 1;
  }

  freeaddrinfo(result);

  iResult = listen(ListenSocket, SOMAXCONN);
  if (iResult == SOCKET_ERROR)
  {
    printf("listen failed with error: %d\n", WSAGetLastError());
    closesocket(ListenSocket);
    WSACleanup();
    //return 1;
  }

  // Accept a client socket
  ClientSocket = accept(ListenSocket, NULL, NULL);
  if (ClientSocket == INVALID_SOCKET)
  {
    printf("accept failed with error: %d\n", WSAGetLastError());
    closesocket(ListenSocket);
    WSACleanup();
    //return 1;
  }

  // No longer need server socket
  closesocket(ListenSocket);
#endif

  QueryPerformanceFrequency(&TICKS_PER_SECOND);
  ticksSampleTime = TICKS_PER_SECOND.QuadPart * SAMPLE_TIME;

  //START DE TRANSMISS�O DA REDE CAN
  epos.StartPDOS(1);
  epos.StartPDOS(2);
  epos.StartPDOS(3);
  epos.StartPDOS(4);
  epos.StartPDOS(5);
  epos.StartPDOS(1);
  epos.StartPDOS(2);
  epos.StartPDOS(3);
  epos.StartPDOS(4);
  epos.StartPDOS(5);

  std::cout << "INICIALIZANDO COMUNICACAO CANOpen COM AS EPOS" << std::endl;

  //Inicializando a comunica��o com os eixos
  for (int i = 0; i < 10; i++)
  {
    //Aguarda tempo
    endwait = clock() + 1 * CLOCKS_PER_SEC;
    while (clock() < endwait)
    {
    }

    //Sincroniza as epos
    epos.sync();

    eixo_out.ReadPDO01();
    eixo_in.ReadPDO01();

    printf(".");
  }

  std::cout << "Resetando Falhas " << std::endl;

  //EPOS 01
  eixo_out.PDOsetControlWord_FaultReset(true);
  eixo_out.WritePDO01();

  //printf("\nResetando as falhas.");

  endwait = clock() + 2 * CLOCKS_PER_SEC;
  while (clock() < endwait)
  {
  }

  std::cout << "..";

  //EPOS 01
  eixo_out.PDOsetControlWord_FaultReset(false);
  eixo_in.WritePDO01();

  std::cout << "..";

  endwait = clock() + 2 * CLOCKS_PER_SEC;
  while (clock() < endwait)
  {
  }

  printf("..");

  //EPOS 02
  eixo_in.PDOsetControlWord_FaultReset(true);
  eixo_in.WritePDO01();

  std::cout << "..";

  endwait = clock() + 2 * CLOCKS_PER_SEC;
  while (clock() < endwait)
  {
  }

  std::cout << "..";

  //EPOS 02
  eixo_in.PDOsetControlWord_FaultReset(false);
  eixo_in.WritePDO01();

  std::cout << "..";

  endwait = clock() + 2 * CLOCKS_PER_SEC;
  while (clock() < endwait)
  {
  }

  std::cout << "OK" << std::endl;

  // ---------------------------------------------  Xsens Awinda Station management  ---------------------------------------------- //

  /*

  | MTw  | desiredUpdateRate (max) |
  |------|------------------------|
  |  1   |           150 Hz        |
  |  2   |           120 Hz        |
  |  4   |           100 Hz        |
  |  6   |            75 Hz        |
  |  12  |            50 Hz        |
  |  18  |            40 Hz        |

  */

  const int desiredUpdateRate = XSENS_RATE;
  const int desiredRadioChannel = XSENS_CH;

  WirelessMasterCallback wirelessMasterCallback; // Callback for wireless master
  std::vector<MtwCallback *> mtwCallbacks;       // Callbacks for mtw devices

  //std::cout << "Constructing XsControl..." << std::endl;
  XsControl *control = XsControl::construct();
  if (control == 0)
  {
    std::cout << "Failed to construct XsControl instance." << std::endl;
  }

  try
  {
    //std::cout << "Scanning ports..." << std::endl;
    XsPortInfoArray detectedDevices = XsScanner::scanPorts();

    //std::cout << "Finding wireless master..." << std::endl;
    XsPortInfoArray::const_iterator wirelessMasterPort = detectedDevices.begin();
    while (wirelessMasterPort != detectedDevices.end() && !wirelessMasterPort->deviceId().isWirelessMaster())
    {
      ++wirelessMasterPort;
    }
    if (wirelessMasterPort == detectedDevices.end())
    {
      throw std::runtime_error("No wireless masters found");
    }
    std::cout << "Wireless master found @ " << *wirelessMasterPort << std::endl;

    std::cout << "Opening port..." << std::endl;
    if (!control->openPort(wirelessMasterPort->portName().toStdString(), wirelessMasterPort->baudrate()))
    {
      std::ostringstream error;
      error << "Failed to open port " << *wirelessMasterPort;
      throw std::runtime_error(error.str());
    }

    //std::cout << "Getting XsDevice instance for wireless master..." << std::endl;
    XsDevicePtr wirelessMasterDevice = control->device(wirelessMasterPort->deviceId());
    if (wirelessMasterDevice == 0)
    {
      std::ostringstream error;
      error << "Failed to construct XsDevice instance: " << *wirelessMasterPort;
      throw std::runtime_error(error.str());
    }

    std::cout << "XsDevice instance created @ " << *wirelessMasterDevice << std::endl;

    //std::cout << "Setting config mode..." << std::endl;
    if (!wirelessMasterDevice->gotoConfig())
    {
      std::ostringstream error;
      error << "Failed to goto config mode: " << *wirelessMasterDevice;
      throw std::runtime_error(error.str());
    }

    //std::cout << "Attaching callback handler..." << std::endl;
    wirelessMasterDevice->addCallbackHandler(&wirelessMasterCallback);

    std::cout << "Getting the list of the supported update rates..." << std::endl;
    const XsIntArray supportedUpdateRates = wirelessMasterDevice->supportedUpdateRates();

    std::cout << "Supported update rates: ";
    for (XsIntArray::const_iterator itUpRate = supportedUpdateRates.begin(); itUpRate != supportedUpdateRates.end(); ++itUpRate)
    {
      std::cout << *itUpRate << " ";
    }
    std::cout << std::endl;

    const int newUpdateRate = findClosestUpdateRate(supportedUpdateRates, desiredUpdateRate);

    std::cout << "Setting update rate to " << newUpdateRate << " Hz..." << std::endl;
    if (!wirelessMasterDevice->setUpdateRate(newUpdateRate))
    {
      std::ostringstream error;
      error << "Failed to set update rate: " << *wirelessMasterDevice;
      throw std::runtime_error(error.str());
    }

    std::cout << "Disabling radio channel if previously enabled..." << std::endl;
    if (wirelessMasterDevice->isRadioEnabled())
    {
      if (!wirelessMasterDevice->disableRadio())
      {
        std::ostringstream error;
        error << "Failed to disable radio channel: " << *wirelessMasterDevice;
        throw std::runtime_error(error.str());
      }
    }

    std::cout << "Setting radio channel to " << desiredRadioChannel << " and enabling radio..." << std::endl;
    if (!wirelessMasterDevice->enableRadio(desiredRadioChannel))
    {
      std::ostringstream error;
      error << "Failed to set radio channel: " << *wirelessMasterDevice;
      throw std::runtime_error(error.str());
    }

    std::cout << "Waiting for MTW to wirelessly connect...\n"
              << std::endl;

    bool quitOnMTw = false;
    bool waitForConnections = true;

    size_t connectedMTWCount = wirelessMasterCallback.getWirelessMTWs().size();
    do
    {
      XsTime::msleep(100);

      while (true)
      {
        size_t nextCount = wirelessMasterCallback.getWirelessMTWs().size();
        if (nextCount != connectedMTWCount)
        {
          std::cout << "Number of connected MTWs: " << nextCount << ". Press 'y' to start measurement or 'q' to quit \n";
          connectedMTWCount = nextCount;
        }
        else
        {
          break;
        }
      }
      if (_kbhit())
      {
        char keypressed = _getch();
        if ('y' == keypressed)
          waitForConnections = false;
        if ('q' == keypressed)
        {
          quitOnMTw = true;
          waitForConnections = false;
        }
      }
    } while (waitForConnections);

    if (quitOnMTw)
    {
      wirelessMasterDevice->gotoConfig();
      wirelessMasterDevice->disableRadio();
      throw std::runtime_error("quit by user request");
    }

    Mode control_mode;
    int log_time;

    printf("Choose the control mode:\n[0] MTC\n[1] ATC\n[2] ITC\n[3] STC\n");
    scanf("%d", &control_mode);
    while (control_mode > 3 || control_mode < 0)
    {
      printf("CHOOSE A PROPER CONTROL MODE:\n");
      scanf("%d", &control_mode);
    }

    printf("How long (sec) do you want to record this run? Zero (0) to do not record: ");
    scanf("%d", &log_time);

    std::cout << "Starting measurement..." << std::endl;
    if (!wirelessMasterDevice->gotoMeasurement())
    {
      std::ostringstream error;
      error << "Failed to goto measurement mode: " << *wirelessMasterDevice;
      throw std::runtime_error(error.str());
    }

    //std::cout << "Getting XsDevice instances for all MTWs..." << std::endl;
    XsDeviceIdArray allDeviceIds = control->deviceIds();
    XsDeviceIdArray mtwDeviceIds;
    for (XsDeviceIdArray::const_iterator i = allDeviceIds.begin(); i != allDeviceIds.end(); ++i)
    {
      if (i->isMtw())
      {
        mtwDeviceIds.push_back(*i);
      }
    }
    XsDevicePtrArray mtwDevices;
    for (XsDeviceIdArray::const_iterator i = mtwDeviceIds.begin(); i != mtwDeviceIds.end(); ++i)
    {
      XsDevicePtr mtwDevice = control->device(*i);
      if (mtwDevice != 0)
      {
        mtwDevices.push_back(mtwDevice);
      }
      else
      {
        throw std::runtime_error("Failed to create an MTW XsDevice instance");
      }
    }

    //std::cout << "Attaching callback handlers to MTWs..." << std::endl;
    mtwCallbacks.resize(mtwDevices.size());
    for (int i = 0; i < (int)mtwDevices.size(); ++i)
    {
      mtwCallbacks[i] = new MtwCallback(i, mtwDevices[i]);
      mtwDevices[i]->addCallbackHandler(mtwCallbacks[i]);
    }

    for (int i = 0; i < (int)mtwDevices.size(); ++i)
    {
      if (i == 0)
      {
        std::cout << "MTw Hum: " << mtwDevices[i]->deviceId().toString().toStdString();
      }
      if (i == 1)
      {
        std::cout << " MTw Exo: " << mtwDevices[i]->deviceId().toString().toStdString() << std::endl;
      }
    }

    std::this_thread::sleep_for(std::chrono::seconds(5));

    std::vector<XsVector> accData(mtwCallbacks.size());
    std::vector<XsVector> gyroData(mtwCallbacks.size());

    wirelessMasterCallback.mtw_event.clear();

    // Gyroscopes Bias Calibration (Trapezoial Integration)
    std::cout << "Calculating Gyroscope Bias, do not move the IMUs!";

    float integration_time(0.000f);
    float imus_ybias[2] = {0, 0};
    float gyro_y_last[2] = {0, 0};
    float deltaT = 0;
    clock_t bias_startpoint = clock();
    clock_t last_clk = clock();
    while (integration_time <= CALIBRATION_PERIOD)
    {
      XsTime::msleep(4);

      for (size_t i = 0; i < mtwCallbacks.size(); ++i)
      {
        if (mtwCallbacks[i]->dataAvailable())
        {
          XsDataPacket const *packet = mtwCallbacks[i]->getOldestPacket();

          gyroData[i] = packet->calibratedGyroscopeData();

          imus_ybias[i] += 0.5 * (gyro_y_last[i] + (float)gyroData[i].value(2)) * deltaT;
          gyro_y_last[i] = (float)gyroData[i].value(2);

          mtwCallbacks[i]->deleteOldestPacket();
        }
      }
      auto sys_clk = clock();
      deltaT = (float)(sys_clk - last_clk) / CLOCKS_PER_SEC;
      integration_time = (float)(sys_clk - bias_startpoint) / CLOCKS_PER_SEC;
      last_clk = sys_clk;
      // std::cout << "dt " << deltaT << " t_int " << integration_time << std::endl;
    }
    imus_ybias[0] /= CALIBRATION_PERIOD;
    imus_ybias[1] /= CALIBRATION_PERIOD;
    //debug
    printf("Bias: %.6f, %.6f\n", imus_ybias[0], imus_ybias[1]);

    //Sincroniza as epos
    epos.sync();

    endwait = clock() + 1 * CLOCKS_PER_SEC;
    while (clock() < endwait)
    {
    }

    eixo_out.ReadPDO01();
    eixo_in.ReadPDO01();

    Habilita_Eixo(2);

    // Loop MTw + Controle EPOS:
    epos.sync();
    eixo_out.ReadPDO01();
    eixo_in.ReadPDO01();
    accBasedControl xsens2Eposcan(&epos, &eixo_in, &eixo_out, control_mode, log_time);
    qASGDKF ahrs(log_time);

    std::cout << "Loop de Controle, pressione qualquer tecla para interromper!" << std::endl;

    float delay;
    int printer = 0;
    int scan_file = 0;

    std::chrono::system_clock::time_point mtw_data_stamp;
    clock_t beginning = 0;
    clock_t loop_duration;
    float freq;

    float mtw_hum = 0;
    float mtw_exo = 0;
    float mtw_hum_raw = 0;
    float mtw_exo_raw = 0;
    LowPassFilter2pFloat mtwHumFiltered(XSENS_RATE, XSENS_FC);
    LowPassFilter2pFloat mtwExoFiltered(XSENS_RATE, XSENS_FC);
    LowPassFilter2pFloat Filt[12];
    for (int i = 0; i < sizeof(Filt)/sizeof(LowPassFilter2pFloat); i++)
    {
      Filt[i].set_cutoff_frequency(XSENS_RATE, 30);
    }

    std::vector<float> gyros(mtwCallbacks.size());
    std::vector<float> imus(12);
    std::thread controller_t;
    std::condition_variable Cv;
    std::mutex Mtx;

/*
    switch (control_mode)
    {
    case MTC:
      controller_t = std::thread(&accBasedControl::accBasedController, &xsens2Eposcan, std::ref(gyros), std::ref(Cv), std::ref(Mtx));
      break;
    case ATC:
      controller_t = std::thread(&accBasedControl::CAdmittanceControl, &xsens2Eposcan, std::ref(gyros), std::ref(Cv), std::ref(Mtx));
      break;
    case ITC:
      controller_t = std::thread(&accBasedControl::ImpedanceControl, &xsens2Eposcan, std::ref(gyros), std::ref(Cv), std::ref(Mtx));
      break;
    case STC:
      controller_t = std::thread(&accBasedControl::SeaFeedbackControl, &xsens2Eposcan, std::ref(gyros), std::ref(Cv), std::ref(Mtx));
      break;
    }
*/
    controller_t = std::thread(&accBasedControl::Controller, &xsens2Eposcan, std::ref(gyros), std::ref(imus), std::ref(Cv), std::ref(Mtx));

    xsens2Eposcan.set_timestamp_begin(std::chrono::system_clock::now());

    while (!_kbhit())
    {
      XsTime::msleep(4);

      bool newDataAvailable = false;
      mtw_data_stamp = std::chrono::system_clock::now();

      for (size_t i = 0; i < mtwCallbacks.size(); ++i)
      {
        if (mtwCallbacks[i]->dataAvailable())
        {
          newDataAvailable = true;
          XsDataPacket const *packet = mtwCallbacks[i]->getOldestPacket();

          accData[i] = packet->calibratedAcceleration();
          gyroData[i] = packet->calibratedGyroscopeData();

          mtwCallbacks[i]->deleteOldestPacket();
        }
      }

      if (newDataAvailable)
      {
        std::unique_lock<std::mutex> Lck(Mtx);
        mtw_hum_raw = -(float)(gyroData[0].value(2) - imus_ybias[0]);
        mtw_hum = mtwHumFiltered.apply(mtw_hum_raw);
        gyros[0] = mtw_hum;

        // Para orientacao com o nome/led da IMU para fora da perna usar (2), (1), (0)...
        imus[0] = Filt[0].apply(accData[0].value(0));
        imus[1] = Filt[1].apply(accData[0].value(1));
        imus[2] = Filt[2].apply(accData[0].value(2));
        imus[3] = Filt[3].apply(gyroData[0].value(0));
        imus[4] = Filt[4].apply(gyroData[0].value(1));
        imus[5] = Filt[5].apply(gyroData[0].value(2));

        if (mtwCallbacks.size() == 2)
        {
          mtw_exo_raw = (float)(gyroData[1].value(2) - imus_ybias[1]);
          mtw_exo = mtwExoFiltered.apply(mtw_exo_raw);
          gyros[1] = mtw_exo;

          imus[6] = Filt[6].apply(accData[1].value(0));
          imus[7] = Filt[7].apply(accData[1].value(1));
          imus[8] = Filt[8].apply(accData[1].value(2));
          imus[9] = Filt[9].apply(gyroData[1].value(0));
          imus[10] = Filt[10].apply(gyroData[1].value(1));
          imus[11] = Filt[11].apply(gyroData[1].value(2));
        }

		    Vector3f acc;
		    Vector3f gyro;
		    acc << imus[0], imus[1], imus[2];
		    gyro << imus[3], imus[4], imus[5];
		    ahrs.updateqASGD1Kalman(gyro, acc, (1/freq));
		    acc << imus[6], imus[7], imus[8];
		    gyro << imus[9], imus[10], imus[11];
		    ahrs.updateqASGD2Kalman(gyro, acc, (1/freq));
        ahrs.Recorder();

        Cv.notify_one();
        Cv.wait(Lck);

        auto control_stamp = std::chrono::system_clock::now();
        delay = std::chrono::duration_cast<std::chrono::microseconds>(control_stamp - mtw_data_stamp).count();
        delay = 1e-3 * delay;

        printer++;
        scan_file++;

        loop_duration = clock() - beginning;
        beginning = clock();
        freq = (float)CLOCKS_PER_SEC / loop_duration;
      }

      if (scan_file == (int)RATE * 6) // every 6s reads the gains_values.txt
      {
        xsens2Eposcan.GainScan();
        ahrs.GainScan();
        scan_file = 0;
      }

#if TCP_ENABLE
      auto tcp_msg = xsens2Eposcan.TCPMessage();
      iSendResult = send(ClientSocket, tcp_msg, strlen(tcp_msg), 0);
#endif

      if (printer == (int)RATE / 4) // printing the status @ 4Hz
      {
        system("cls");
        xsens2Eposcan.UpdateControlStatus();
        Vector3f euler = ahrs.quat2euler(2)*(180 / MY_PI);
        std::cout << xsens2Eposcan.ctrl_word;
        float roll = euler(0);
        float pitch = euler(1);
        float yaw = euler(2);
        printf("\n Roll: %.4f, Pitch: %.4f, Yaw: %.4f\n", euler(0), euler(1), euler(2));
        printf(" MTw Rate: %4.2f Hz\n delay %2.2f ms\n\n MasterCallback:", freq, delay);
        // display MTW events, showing if one of the IMUs got disconnected:
        std::cout << wirelessMasterCallback.mtw_event << std::endl;
        //std::cout << " " << iSendResult << std::endl;
        printer = 0;
      }
    }
    (void)_getch();

    xsens2Eposcan.StopCtrlThread();
    Cv.notify_all();
    controller_t.join();

    //Zera o comando do motor
    xsens2Eposcan.~accBasedControl();

    ahrs.~qASGDKF();
    //Desabilita o Eixo
    Desabilita_Eixo(0);

    std::cout << "Setting config mode..." << std::endl;
    if (!wirelessMasterDevice->gotoConfig())
    {
      std::ostringstream error;
      error << "Failed to goto config mode: " << *wirelessMasterDevice;
      throw std::runtime_error(error.str());
    }

    std::cout << "Disabling radio... " << std::endl;
    if (!wirelessMasterDevice->disableRadio())
    {
      std::ostringstream error;
      error << "Failed to disable radio: " << *wirelessMasterDevice;
      throw std::runtime_error(error.str());
    }
  }
  catch (std::exception const &ex)
  {
    std::cout << ex.what() << std::endl;
    std::cout << "****ABORT****" << std::endl;
  }
  catch (...)
  {
    std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
    std::cout << "****ABORT****" << std::endl;
  }

  std::cout << "Closing XsControl..." << std::endl;
  control->close();

  std::cout << "Deleting mtw callbacks..." << std::endl;
  for (std::vector<MtwCallback *>::iterator i = mtwCallbacks.begin(); i != mtwCallbacks.end(); ++i)
  {
    delete (*i);
  }

  //FINALIZA A COMUNICA��O COM AS EPOS
  epos.StopPDOS(1);

  endwait = clock() + 2 * CLOCKS_PER_SEC;
  while (clock() < endwait)
  {
  }

  std::cout << "Successful exit." << std::endl;

  // shutdown the connection since we're done
  iResult = shutdown(ClientSocket, SD_SEND);
  if (iResult == SOCKET_ERROR)
  {
    printf("Socket shutdown failed with error: %d\n", WSAGetLastError());
    closesocket(ClientSocket);
    WSACleanup();
    return 1;
  }

  // cleanup
  closesocket(ClientSocket);
  WSACleanup();

  std::cout << "Press [ENTER] to continue." << std::endl;
  std::cin.get();

  return 0;
}

/* EPOS FUNCTIONS */

void Habilita_Eixo(int ID)
{

  if ((ID == 2) | (ID == 0))
  {

    eixo_in.PDOsetControlWord_SwitchOn(false);
    eixo_in.PDOsetControlWord_EnableVoltage(true);
    eixo_in.PDOsetControlWord_QuickStop(true);
    eixo_in.PDOsetControlWord_EnableOperation(false);
    eixo_in.WritePDO01();

    printf("\nENERGIZANDO O MOTOR 2 E HABILITANDO O CONTROLE");

    endwait = clock() + 0.5 * CLOCKS_PER_SEC;
    while (clock() < endwait)
    {
    }

    eixo_in.PDOsetControlWord_SwitchOn(true);
    eixo_in.PDOsetControlWord_EnableVoltage(true);
    eixo_in.PDOsetControlWord_QuickStop(true);
    eixo_in.PDOsetControlWord_EnableOperation(false);
    eixo_in.WritePDO01();

    endwait = clock() + 0.5 * CLOCKS_PER_SEC;
    while (clock() < endwait)
    {
    }

    eixo_in.PDOsetControlWord_SwitchOn(true);
    eixo_in.PDOsetControlWord_EnableVoltage(true);
    eixo_in.PDOsetControlWord_QuickStop(true);
    eixo_in.PDOsetControlWord_EnableOperation(true);
    eixo_in.WritePDO01();
  }
}

void Desabilita_Eixo(int ID)
{

  if ((ID == 2) | (ID == 0))
  {
    printf("\nDESABILITANDO O MOTOR E CONTROLE\n\n");

    eixo_in.PDOsetControlWord_SwitchOn(true);
    eixo_in.PDOsetControlWord_EnableVoltage(true);
    eixo_in.PDOsetControlWord_QuickStop(true);
    eixo_in.PDOsetControlWord_EnableOperation(false);
    eixo_in.WritePDO01();

    endwait = clock() + 0.5 * CLOCKS_PER_SEC;
    while (clock() < endwait)
    {
    }

    eixo_in.PDOsetControlWord_SwitchOn(false);
    eixo_in.PDOsetControlWord_EnableVoltage(true);
    eixo_in.PDOsetControlWord_QuickStop(true);
    eixo_in.PDOsetControlWord_EnableOperation(false);
    eixo_in.WritePDO01();
  }
}
