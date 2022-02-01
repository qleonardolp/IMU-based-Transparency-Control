#ifndef DECLARATIONS_XSENS
#define DECLARATIONS_XSENS

//---------------------------------------//
// Headers for XSens
#include "include/xsensdeviceapi.h" // The Xsens device API header
#include "conio.h"                  // For non ANSI _kbhit() and _getch()

#include <string>
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <set>
#include <list>
#include <utility>

#include "xsens/xsmutex.h"
//---------------------------------------//

/*! \brief Stream insertion operator overload for XsPortInfo */
std::ostream &operator<<(std::ostream &out, XsPortInfo const &p);

/*! \brief Stream insertion operator overload for XsDevice */
std::ostream &operator<<(std::ostream &out, XsDevice const &d);

/*! \brief Given a list of update rates and a desired update rate, returns the closest update rate to the desired one */
int findClosestUpdateRate(const XsIntArray &supportedUpdateRates, const int desiredUpdateRate);

//----------------------------------------------------------------------
// Callback handler for wireless master
//----------------------------------------------------------------------
class WirelessMasterCallback : public XsCallback
{
public:
    typedef std::set<XsDevice *> XsDeviceSet;

    XsDeviceSet getWirelessMTWs() const;

protected:
    virtual void onConnectivityChanged(XsDevice *dev, XsConnectivityState newState);

private:
    mutable XsMutex m_mutex;
    XsDeviceSet m_connectedMTWs;
};

//----------------------------------------------------------------------
// Callback handler for MTw
// Handles onDataAvailable callbacks for MTW devices
//----------------------------------------------------------------------
class MtwCallback : public XsCallback
{
public:
    MtwCallback(int mtwIndex, XsDevice *device);

    bool dataAvailable() const;

    XsDataPacket const *getOldestPacket() const;

    void deleteOldestPacket();

    int getMtwIndex() const;

    XsDevice const &device() const;

protected:
    virtual void onLiveDataAvailable(XsDevice *, const XsDataPacket *packet);

private:
    mutable XsMutex m_mutex;
    std::list<XsDataPacket> m_packetBuffer;
    int m_mtwIndex;
    XsDevice *m_device;
};

#endif /* DECLARATIONS_XSENS */