#ifndef MASTERCALLBACK_H
#define MASTERCALLBACK_H

#include <xsensdeviceapi.h> // The Xsens device API header 
#include <xstypes.h>
#include <iomanip>
#include <set>

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


typedef std::set<XsDevice*> XsDeviceSet;

class WirelessMasterCallback : public XsCallback
{
public:
	XsDeviceSet getWirelessMTWs() const;
  std::string mtw_event;

protected:
	virtual void onConnectivityChanged(XsDevice* dev, XsConnectivityState newState);

private:
	mutable XsMutex m_mutex;
	XsDeviceSet m_connectedMTWs;
};


#endif /* MASTERCALLBACK_H */