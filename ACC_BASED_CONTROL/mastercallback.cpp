#include "mastercallback.h"
#include <iostream>

//----------------------------------------------------------------------
// Callback handler for wireless master
//----------------------------------------------------------------------


XsDeviceSet WirelessMasterCallback::getWirelessMTWs() const
{        
	XsMutexLocker lock(m_mutex);
	return m_connectedMTWs;
}

void WirelessMasterCallback::onConnectivityChanged(XsDevice* dev, XsConnectivityState newState)
{
	XsMutexLocker lock(m_mutex);
	switch (newState)
	{
	case XCS_Disconnected:		/*!< Device has disconnected, only limited informational functionality is available. */
		std::cout << "\nEVENT: MTW Disconnected -> " << dev->deviceId().toString().toStdString() << std::endl;
    mtw_event = " EVENT: MTW Disconnected -> " + dev->deviceId().toString().toStdString();
		m_connectedMTWs.erase(dev);
		break;
	case XCS_Rejected:			/*!< Device has been rejected and is disconnected, only limited informational functionality is available. */
		std::cout << "\nEVENT: MTW Rejected -> " << dev->deviceId().toString().toStdString() << std::endl;
    mtw_event = " EVENT: MTW Rejected -> " + dev->deviceId().toString().toStdString();
		m_connectedMTWs.erase(dev);
		break;
	case XCS_PluggedIn:			/*!< Device is connected through a cable. */
		std::cout << "\nEVENT: MTW PluggedIn -> " << dev->deviceId().toString().toStdString() << std::endl;
    mtw_event = " EVENT: MTW PluggedIn -> " + dev->deviceId().toString().toStdString();
		m_connectedMTWs.erase(dev);
		break;
	case XCS_Wireless:			/*!< Device is connected wirelessly. */
		std::cout << "\nEVENT: MTW Connected -> " << dev->deviceId().toString().toStdString() << std::endl;
    mtw_event = " EVENT: MTW Connected -> " + dev->deviceId().toString().toStdString();
		m_connectedMTWs.insert(dev);
		break;
	case XCS_File:				/*!< Device is reading from a file. */
		std::cout << "\nEVENT: MTW File -> " << dev->deviceId().toString().toStdString() << std::endl;
    mtw_event = " EVENT: MTW File -> " + dev->deviceId().toString().toStdString();
		m_connectedMTWs.erase(dev);
		break;
	case XCS_Unknown:			/*!< Device is in an unknown state. */
		std::cout << "\nEVENT: MTW Unknown -> " << dev->deviceId().toString().toStdString() << std::endl;
    mtw_event = " EVENT: MTW Unknown -> " + dev->deviceId().toString().toStdString();
		m_connectedMTWs.erase(dev);
		break;
	default:
		std::cout << "\nEVENT: MTW Error -> " << dev->deviceId().toString().toStdString() << std::endl;
    mtw_event = " EVENT: MTW Error -> " + dev->deviceId().toString().toStdString();
		m_connectedMTWs.erase(dev);
		break;
	}
}
