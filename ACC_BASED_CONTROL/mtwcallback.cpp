#include "mtwcallback.h"
#include <xsens/xsdevice.h>
#include <xsens/xsdatapacket.h>

//----------------------------------------------------------------------
// Callback handler for MTw
// Handles onDataAvailable callbacks for MTW devices
//----------------------------------------------------------------------

MtwCallback::MtwCallback(int mtwIndex, XsDevice* device)
	:m_mtwIndex(mtwIndex)
	,m_device(device)
	{
	}

bool MtwCallback::dataAvailable() const
	{
		XsMutexLocker lock(m_mutex);
		return !m_packetBuffer.empty();
	}

XsDataPacket const * MtwCallback::getOldestPacket() const
	{
		XsMutexLocker lock(m_mutex);
		XsDataPacket const * packet = &m_packetBuffer.front();
		return packet;
	}

void MtwCallback::deleteOldestPacket()
	{
		XsMutexLocker lock(m_mutex);
		m_packetBuffer.pop_front();
	}

XsDataPacket MtwCallback::fetchOldestPacket()
	{
		XsMutexLocker lock(m_mutex);
		XsDataPacket packet = m_packetBuffer.front(); // read
		m_packetBuffer.pop_front();					  // delete
		return packet;
	}

int MtwCallback::getMtwIndex() const
	{
		return m_mtwIndex;
	}

XsDevice const & MtwCallback::device() const
	{
		assert(m_device != 0);
		return *m_device;
	}

void MtwCallback::onLiveDataAvailable(XsDevice*, const XsDataPacket* packet)
	{
		XsMutexLocker lock(m_mutex);
		// NOTE: Processing of packets should not be done in this thread.

		m_packetBuffer.push_back(*packet);
		if (m_packetBuffer.size() > 300)
		{
			//std::cout << std::endl;
			deleteOldestPacket();
		}
	}