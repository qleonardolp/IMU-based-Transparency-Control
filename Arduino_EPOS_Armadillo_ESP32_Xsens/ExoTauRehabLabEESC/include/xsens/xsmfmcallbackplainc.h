/*	WARNING: COPYRIGHT (C) 2016 XSENS TECHNOLOGIES OR SUBSIDIARIES WORLDWIDE. ALL RIGHTS RESERVED.
	THIS FILE AND THE SOURCE CODE IT CONTAINS (AND/OR THE BINARY CODE FILES FOUND IN THE SAME
	FOLDER THAT CONTAINS THIS FILE) AND ALL RELATED SOFTWARE (COLLECTIVELY, "CODE") ARE SUBJECT
	TO A RESTRICTED LICENSE AGREEMENT ("AGREEMENT") BETWEEN XSENS AS LICENSOR AND THE AUTHORIZED
	LICENSEE UNDER THE AGREEMENT. THE CODE MUST BE USED SOLELY WITH XSENS PRODUCTS INCORPORATED
	INTO LICENSEE PRODUCTS IN ACCORDANCE WITH THE AGREEMENT. ANY USE, MODIFICATION, COPYING OR
	DISTRIBUTION OF THE CODE IS STRICTLY PROHIBITED UNLESS EXPRESSLY AUTHORIZED BY THE AGREEMENT.
	IF YOU ARE NOT AN AUTHORIZED USER OF THE CODE IN ACCORDANCE WITH THE AGREEMENT, YOU MUST STOP
	USING OR VIEWING THE CODE NOW, REMOVE ANY COPIES OF THE CODE FROM YOUR COMPUTER AND NOTIFY
	XSENS IMMEDIATELY BY EMAIL TO INFO@XSENS.COM. ANY COPIES OR DERIVATIVES OF THE CODE (IN WHOLE
	OR IN PART) IN SOURCE CODE FORM THAT ARE PERMITTED BY THE AGREEMENT MUST RETAIN THE ABOVE
	COPYRIGHT NOTICE AND THIS PARAGRAPH IN ITS ENTIRETY, AS REQUIRED BY THE AGREEMENT.
*/

#ifndef XSMFMCALLBACKPLAINC_H
#define XSMFMCALLBACKPLAINC_H

#include <xsens/pstdint.h>
#include <xsens/xsresultvalue.h>
#include <xsens/xsdeviceidarray.h>

#ifndef __cplusplus
#define XSMFMCALLBACK_INITIALIZER		{ 0, 0, 0 }
#endif

struct XsString;

/*! \brief Structure that contains callback functions for the Magnetic Field Mapper
	\details When using C++, please use the overloaded class XsMfMCallback instead.

	This structure contains pointers to functions that will be called by
	the magnetic field mapper library when certain events occur. To use it
	in C, set any callback you do not wish to use to 0 and put a valid
	function pointer in the others. Then pass the object to the XsMfm
	object's addCallbackHandler function.

	\note XsMfm does not copy the structure contents and does not take
	ownership of it. So make sure it is allocated on the heap or at least
	removed from wherever it was added by calling removeCallbackHandler
	before it is destroyed.
*/
typedef struct XsMfMCallbackPlainC
{
/*! \defgroup Callbacks Callback functions.
	\addtogroup Callbacks
	@{
*/
	/*! \brief Called when a device scan is done
		\param devices The list of deviceids that were detected during the scan
	*/
	void (*m_onScanDone)(struct XsMfMCallbackPlainC* thisPtr, const XsDeviceIdArray* devices);

	/*! \brief Called when the magfield mapping has completed
		\param dev The deviceid of which the magfieldmapping as completed
		\param result The result of the magfield mapping
	*/
	void (*m_onMfmDone)(struct XsMfMCallbackPlainC* thisPtr, XsDeviceId dev, int result);

	/*! \brief Called when an error has occurred while handling incoming data
		\param dev The deviceid that generated the error message
		\param error The error code that specifies exactly what problem occurred
		\param description A string with more details of the specific problem. May be empty, but not NULL.
	*/
	void (*m_onMfmError)(struct XsMfMCallbackPlainC* thisPtr, XsDeviceId dev, XsResultValue error, const struct XsString* description);

//! @}

#ifdef __cplusplus
	// Make sure that this struct is not used in C++ (except as base class for XsCallback)
	friend class XsMfmCallback;
	XsMfMCallbackPlainC() {}
	~XsMfMCallbackPlainC() throw() {}
private:
	XsMfMCallbackPlainC(XsMfMCallbackPlainC const &);
	XsMfMCallbackPlainC& operator = (XsMfMCallbackPlainC const &);

#endif

} XsMfMCallbackPlainC;

#endif // file guard
