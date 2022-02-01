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

#ifndef XSMFMCALLBACK_H
#define XSMFMCALLBACK_H

#include "xsmfmcallbackplainc.h"

#ifdef __cplusplus

/*! \brief Structure that contains callback functions for the Xsens Device API
	\details When programming in C++, simply overload the callback that you want to use and supply
	your XsCallbackPlainC-overloaded class to one of the setCallback functions

	When programming in C, create an XsCallbackPlainC structure and initialize each function pointer to
	the right function to call. The supplied first parameter is the address of the XsCallbackPlainC
	object that you supplied to the setCallback function. If you do not wish to receive a specific
	callback in C, set the function pointer to 0.

	In both cases, the calling application remains in control of the XsCallbackPlainC object and thus
	remains responsible for cleaning it up when the it is no longer necessary.

	\note Any callback function in %XsCallback that is not overloaded will only be called once to
	minimize callback overhead.
*/
class XsMfmCallback : public XsMfMCallbackPlainC
{
public:
	/*! \brief Constructor */
	XsMfmCallback()
	{
		m_onScanDone = sonScanDone;
		m_onMfmDone = sonMfmDone;
		m_onMfmError = sonMfmError;
	}

	/*! \brief Destructor
		\note Make sure that the object is removed from callback generating objects before destroying it!
	*/
	virtual ~XsMfmCallback() {}

// Swig needs these functions to be protected, not private, otherwise they are ignored.
protected:
/*! \protectedsection
	\addtogroup Callbacks
	@{
*/
	//! \copydoc m_onScanDone
	virtual void onScanDone(const XsDeviceIdArray* devices)
	{ (void) devices; }
	//! \copydoc m_onMfmDone
	virtual void onMfmDone(XsDeviceId dev, int result)
	{ (void) dev; (void) result; }
	//! \copydoc m_onMfmError
	virtual void onMfmError(XsDeviceId dev, XsResultValue error, const XsString* description)
	{ (void) dev, (void) error; (void) description; }
//! @}

private:
/*! \privatesection */
	static void sonScanDone(XsMfMCallbackPlainC* cb, const XsDeviceIdArray* devices) { ((XsMfmCallback*)cb)->onScanDone(devices); }
	static void sonMfmDone(XsMfMCallbackPlainC* cb, XsDeviceId dev, int result) { ((XsMfmCallback*)cb)->onMfmDone(dev, result); }
	static void sonMfmError(XsMfMCallbackPlainC* cb, XsDeviceId dev, XsResultValue error, const XsString* description) { ((XsMfmCallback*)cb)->onMfmError(dev, error, description); }
};

#endif

#endif // file guard
