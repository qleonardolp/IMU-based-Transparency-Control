#include "declarations_xsens.h"

int findClosestUpdateRate(const XsIntArray &supportedUpdateRates, const int desiredUpdateRate)
{
    if (supportedUpdateRates.empty())
    {
        return 0;
    }

    if (supportedUpdateRates.size() == 1)
    {
        return supportedUpdateRates[0];
    }

    int uRateDist = -1;
    int closestUpdateRate = -1;
    for (XsIntArray::const_iterator itUpRate = supportedUpdateRates.begin(); itUpRate != supportedUpdateRates.end(); ++itUpRate)
    {
        const int currDist = std::abs(*itUpRate - desiredUpdateRate);

        if ((uRateDist == -1) || (currDist < uRateDist))
        {
            uRateDist = currDist;
            closestUpdateRate = *itUpRate;
        }
    }
    return closestUpdateRate;
}

/*! \brief Stream insertion operator overload for XsPortInfo */
std::ostream &
operator<<(std::ostream &out, XsPortInfo const &p)
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

//----------------------------------------------------------------------
// Callback handler for wireless master
//----------------------------------------------------------------------
typedef std::set<XsDevice *> XsDeviceSet;

XsDeviceSet WirelessMasterCallback::getWirelessMTWs() const
{
    XsMutexLocker lock(m_mutex);
    return m_connectedMTWs;
}

void WirelessMasterCallback::onConnectivityChanged(XsDevice *dev, XsConnectivityState newState)
{
    XsMutexLocker lock(m_mutex);
    switch (newState)
    {
    case XCS_Disconnected: /*!< Device has disconnected, only limited informational functionality is available. */
        std::cout << "\nEVENT: MTW Disconnected -> " << *dev << std::endl;
        m_connectedMTWs.erase(dev);
        break;
    case XCS_Rejected: /*!< Device has been rejected and is disconnected, only limited informational functionality is available. */
        std::cout << "\nEVENT: MTW Rejected -> " << *dev << std::endl;
        m_connectedMTWs.erase(dev);
        break;
    case XCS_PluggedIn: /*!< Device is connected through a cable. */
        std::cout << "\nEVENT: MTW PluggedIn -> " << *dev << std::endl;
        m_connectedMTWs.erase(dev);
        break;
    case XCS_Wireless: /*!< Device is connected wirelessly. */
        std::cout << "\nEVENT: MTW Connected -> " << *dev << std::endl;
        m_connectedMTWs.insert(dev);
        break;
    case XCS_File: /*!< Device is reading from a file. */
        std::cout << "\nEVENT: MTW File -> " << *dev << std::endl;
        m_connectedMTWs.erase(dev);
        break;
    case XCS_Unknown: /*!< Device is in an unknown state. */
        std::cout << "\nEVENT: MTW Unknown -> " << *dev << std::endl;
        m_connectedMTWs.erase(dev);
        break;
    default:
        std::cout << "\nEVENT: MTW Error -> " << *dev << std::endl;
        m_connectedMTWs.erase(dev);
        break;
    }
}

//----------------------------------------------------------------------
// Callback handler for MTw
// Handles onDataAvailable callbacks for MTW devices
//----------------------------------------------------------------------

MtwCallback::MtwCallback(int mtwIndex, XsDevice *device)
    : m_mtwIndex(mtwIndex), m_device(device)
{
}

bool MtwCallback::dataAvailable() const
{
    XsMutexLocker lock(m_mutex);
    return !m_packetBuffer.empty();
}

XsDataPacket const *MtwCallback::getOldestPacket() const
{
    XsMutexLocker lock(m_mutex);
    XsDataPacket const *packet = &m_packetBuffer.front();
    return packet;
}

void MtwCallback::deleteOldestPacket()
{
    XsMutexLocker lock(m_mutex);
    m_packetBuffer.pop_front();
}

int MtwCallback::getMtwIndex() const
{
    return m_mtwIndex;
}

XsDevice const &MtwCallback::device() const
{
    assert(m_device != 0);
    return *m_device;
}

void MtwCallback::onLiveDataAvailable(XsDevice *, const XsDataPacket *packet)
{
    XsMutexLocker lock(m_mutex);
    // NOTE: Processing of packets should not be done in this thread.

    m_packetBuffer.push_back(*packet);
    if (m_packetBuffer.size() > 300)
    {
        std::cout << std::endl;
        deleteOldestPacket();
    }
}