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

//#include <WinSock2.h>

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

#include "findClosestUpdateRate.h"
#include "mastercallback.h"
#include "mtwcallback.h"

//#include "qcustomplot\qcustomplot.h"
#include <xsensdeviceapi.h> // The Xsens device API header 
#include <xsens/xsmutex.h>
#include "xstypes.h"
#include <conio.h>
#include <thread>
#include <chrono>

void Habilita_Eixo(int ID);

void Desabilita_Eixo(int ID);


/*! \brief Stream insertion operator overload for XsPortInfo */
std::ostream& operator << (std::ostream& out, XsPortInfo const & p)
{
  out << "Port: " << std::setw(2) << std::right << p.portNumber() << " (" << p.portName().toStdString() << ") @ "
    << std::setw(7) << p.baudrate() << " Bd"
    << ", " << "ID: " << p.deviceId().toString().toStdString()
    ;
  return out;
}

/*! \brief Stream insertion operator overload for XsDevice */
std::ostream& operator << (std::ostream& out, XsDevice const & d)
{
  out << "ID: " << d.deviceId().toString().toStdString() << " (" << d.productCode().toStdString() << ")";
  return out;
}



int main(int argc, char** argv)
{
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
  while (clock() < endwait) {}

  std::cout << "..";

  //EPOS 01
  eixo_out.PDOsetControlWord_FaultReset(false);
  eixo_in.WritePDO01();

  std::cout << "..";

  endwait = clock() + 2 * CLOCKS_PER_SEC;
  while (clock() < endwait) {}

  printf("..");

  //EPOS 02
  eixo_in.PDOsetControlWord_FaultReset(true);
  eixo_in.WritePDO01();

  std::cout << "..";

  endwait = clock() + 2 * CLOCKS_PER_SEC;
  while (clock() < endwait) {}

  std::cout << "..";

  //EPOS 02
  eixo_in.PDOsetControlWord_FaultReset(false);
  eixo_in.WritePDO01();

  std::cout << "..";

  endwait = clock() + 2 * CLOCKS_PER_SEC;
  while (clock() < endwait) {}

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

  const int desiredUpdateRate = 120;						// Use 120 Hz update rate for MTw, 150 Hz usually crashes!
  const int desiredRadioChannel = 25;						// Use radio channel 25 for wireless master.

  WirelessMasterCallback wirelessMasterCallback;			// Callback for wireless master
  std::vector<MtwCallback*> mtwCallbacks;					// Callbacks for mtw devices

  //std::cout << "Constructing XsControl..." << std::endl;
  XsControl* control = XsControl::construct();
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

    std::cout << "Waiting for MTW to wirelessly connect...\n" << std::endl;

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

    char control_mode;
    int log_time;

    printf("Choose the control mode:\n[p] Position\n[s] Speed\n[k] CACuKF\n[a] CAC\n[u] CACu\n");
    scanf("%c", &control_mode);
	while (control_mode != 'p' && control_mode != 's' && control_mode != 'k' && control_mode != 'a' && control_mode != 'u')
	{
		printf("CHOOSE A PROPER CONTROL MODE: [p]  [s]  [k]  [a]  [u]\n");
		scanf("%c", &control_mode);
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

    std::vector<XsUShortVector> accData(mtwCallbacks.size());
    std::vector<XsUShortVector> gyroData(mtwCallbacks.size());

    wirelessMasterCallback.mtw_event.clear();

    // Gyroscopes Bias Calibration (Trapezoial Integration)
    std::cout << "Calculating Gyroscope Bias, do not move the IMUs!";

    float integration_time = 0;
    float imus_ybias[2] = {0,0};
    float gyro_y_last[2] = {0,0};
    clock_t integration_dt = CLOCKS_PER_SEC*clock();
    clock_t bias_startpoint = CLOCKS_PER_SEC*clock();
    while (integration_time <= 40.000f)
    {
      XsTime::msleep(4);

      for (size_t i = 0; i < mtwCallbacks.size(); ++i)
      {
        if (mtwCallbacks[i]->dataAvailable())
        {
          XsDataPacket const * packet = mtwCallbacks[i]->getOldestPacket();

          gyroData[i] = packet->rawGyroscopeData();

          mtwCallbacks[i]->deleteOldestPacket();

          imus_ybias[i] += 0.5*(gyro_y_last[i] + gyroData[i].at(2))*integration_dt;
          gyro_y_last[i] = gyroData[i].at(2);
        }
      }
      auto sys_clk = clock();
      integration_dt = CLOCKS_PER_SEC*(sys_clk - integration_dt);
      integration_time = (float) CLOCKS_PER_SEC*(sys_clk - bias_startpoint);
      // debug:
      std::cout << "dt " << integration_dt << " t_int " << integration_time;
    }
    std::cout << std::endl;

    //Sincroniza as epos
    epos.sync();

    endwait = clock() + 1 * CLOCKS_PER_SEC;
    while (clock() < endwait) {}

    eixo_out.ReadPDO01();
    eixo_in.ReadPDO01();

    Habilita_Eixo(2);

    // Loop MTw + Controle EPOS:
    epos.sync();
    eixo_out.ReadPDO01();
    eixo_in.ReadPDO01();
    accBasedControl xsens2Eposcan(&epos, &eixo_in, &eixo_out, control_mode, log_time);

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
    std::vector<float> gyros(mtwCallbacks.size());
    std::thread controller_t;
    std::condition_variable Cv;
    std::mutex Mtx;

    if(control_mode == 'k')
      controller_t = std::thread(&accBasedControl::CACurrentKF, &xsens2Eposcan, std::ref(mtw_hum), std::ref(Cv), std::ref(Mtx));
    else if(control_mode == 's')
      controller_t = std::thread(&accBasedControl::CAdmittanceControl, &xsens2Eposcan, std::ref(gyros), std::ref(Cv), std::ref(Mtx));
    else if (control_mode == 'p')
      controller_t = std::thread(&accBasedControl::accBasedController, &xsens2Eposcan, std::ref(gyros), std::ref(Cv), std::ref(Mtx));
    else if(control_mode == 'a')
      controller_t = std::thread(&accBasedControl::CAdmittanceControlKF, &xsens2Eposcan, std::ref(mtw_hum), std::ref(Cv), std::ref(Mtx));
    else if(control_mode == 'u')
      controller_t = std::thread(&accBasedControl::CACurrent, &xsens2Eposcan, std::ref(mtw_hum), std::ref(Cv), std::ref(Mtx));

	xsens2Eposcan.set_timestamp_begin(std::chrono::steady_clock::now());

    while (!_kbhit())
    {		
      XsTime::msleep(4);

      bool newDataAvailable = false;
      mtw_data_stamp = std::chrono::steady_clock::now();

      for (size_t i = 0; i < mtwCallbacks.size(); ++i)
      {
        if (mtwCallbacks[i]->dataAvailable())
        {
          newDataAvailable = true;
          XsDataPacket const * packet = mtwCallbacks[i]->getOldestPacket();

          accData[i] = packet->rawAcceleration();
          gyroData[i] = packet->rawGyroscopeData();

          mtwCallbacks[i]->deleteOldestPacket();
        }
      }

      if (newDataAvailable)
      {
        std::unique_lock<std::mutex> Lck(Mtx);
        gyros[0] = mtw_hum = -(float) (gyroData[0].at(2) - imus_ybias[0]);

        // Put a LowPassFilter2p here...

		// For accBasedPosition and OmegaControlKF
		if (mtwCallbacks.size() == 2 && (control_mode == 'p' || control_mode == 's'))
			gyros[1] = mtw_exo =  (float) (gyroData[1].at(2) - imus_ybias[1]);

        Cv.notify_one();
        Cv.wait(Lck);

		auto control_stamp = std::chrono::steady_clock::now();
		delay = std::chrono::duration_cast<std::chrono::microseconds>(control_stamp - mtw_data_stamp).count();
		delay = 1e-3*delay;

        printer++;
        scan_file++;

        loop_duration = clock() - beginning;
        beginning = clock();
        freq = (float)CLOCKS_PER_SEC / loop_duration;
      }

      if (scan_file == (int)RATE * 6)  // every 6s reads the gains_values.txt 
      {
		  xsens2Eposcan.GainScan();
		  scan_file = 0;
      }

      if (printer == (int)RATE / 4)   // printing the status @ 4Hz
      {
        system("cls");
		xsens2Eposcan.UpdateControlStatus();
        std::cout << xsens2Eposcan.ctrl_word;
        printf(" MTw Rate: %4.2f Hz\n delay %2.2f ms\n\n MasterCallback:", freq, delay);
        // display MTW events, showing if one of the IMUs got disconnected:
        std::cout << wirelessMasterCallback.mtw_event << std::endl; 
        printer = 0;
      }
    }
    (void)_getch();

    xsens2Eposcan.StopCtrlThread();
    Cv.notify_all();
    controller_t.join();

    //Zera o comando do motor
    xsens2Eposcan.~accBasedControl();
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
  catch (std::exception const & ex)
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
  for (std::vector<MtwCallback*>::iterator i = mtwCallbacks.begin(); i != mtwCallbacks.end(); ++i)
  {
    delete (*i);
  }

  //FINALIZA A COMUNICA��O COM AS EPOS
  epos.StopPDOS(1);

  endwait = clock() + 2 * CLOCKS_PER_SEC;
  while (clock() < endwait) {}

  std::cout << "Successful exit." << std::endl;
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
    while (clock() < endwait) {}

    eixo_in.PDOsetControlWord_SwitchOn(true);
    eixo_in.PDOsetControlWord_EnableVoltage(true);
    eixo_in.PDOsetControlWord_QuickStop(true);
    eixo_in.PDOsetControlWord_EnableOperation(false);
    eixo_in.WritePDO01();

    endwait = clock() + 0.5 * CLOCKS_PER_SEC;
    while (clock() < endwait) {}

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
    while (clock() < endwait) {}

    eixo_in.PDOsetControlWord_SwitchOn(false);
    eixo_in.PDOsetControlWord_EnableVoltage(true);
    eixo_in.PDOsetControlWord_QuickStop(true);
    eixo_in.PDOsetControlWord_EnableOperation(false);
    eixo_in.WritePDO01();

  }

}

