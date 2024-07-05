#ifndef MTWCALLBACK_H
#define MTWCALLBACK_H

#include <xsens/xscallback.h>
#include <xsens/xsmutex.h>
#include <list>


//----------------------------------------------------------------------
// Callback handler for MTw
// Handles onDataAvailable callbacks for MTW devices
//----------------------------------------------------------------------

class MtwCallback : public XsCallback
{
public:

	MtwCallback(int mtwIndex, XsDevice* device);

	bool dataAvailable() const;

	XsDataPacket const * getOldestPacket() const;

	XsDataPacket fetchOldestPacket();

	void deleteOldestPacket();

	int getMtwIndex() const;

	XsDevice const & device() const;

protected:

	virtual void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet);

private:
	mutable XsMutex m_mutex;
	std::list<XsDataPacket> m_packetBuffer;
	int m_mtwIndex;
	XsDevice* m_device;
};


#endif /* MTWCALLBACK_H */