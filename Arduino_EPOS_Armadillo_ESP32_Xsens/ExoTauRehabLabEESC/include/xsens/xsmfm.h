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

#ifndef GUARD_XSMFM_H_TMP
#define GUARD_XSMFM_H_TMP
#include <xsens/pstdint.h>
#include <xsens/xstypedefs.h>
#include <xsens/xsdeviceidarray.h>
#include <xsens/xsstringarray.h>
#include <xsens/xsportinfo.h>
#include <xsens/xsmatrix.h>
#include <xsens/xsvector.h>
#include <xsens/xsversion.h>
#include <xsens/xsmfmcallbackplainc.h>
#include "mfmconfig.h"
#ifdef __cplusplus
extern "C" {
#endif
/*! \addtogroup cinterface C Interface
	@{ */
struct XsMfm;
typedef struct XsMfm XsMfm;
MFM_DLL_API struct XsMfm* XsMfm_construct(void);/*!< \copydoc XsMfm::XsMfm()
\returns The newly constructed object*/
MFM_DLL_API void XsMfm_destruct(struct XsMfm* thisPtr);/*!< \copydoc XsMfm::~XsMfm()*/
MFM_DLL_API XsVersion* XsMfm_version(XsVersion* returnValue);/*!< \copydoc XsMfm::version()*/
MFM_DLL_API void XsMfm_reset(struct XsMfm* thisPtr);/*!< \copydoc XsMfm::reset()*/
MFM_DLL_API int XsMfm_scanMfmDevices(struct XsMfm* thisPtr);/*!< \copydoc XsMfm::scanMfmDevices()*/
MFM_DLL_API int XsMfm_scanMfmDevice(struct XsMfm* thisPtr, const XsPortInfo* portInfo);/*!< \copydoc XsMfm::scanMfmDevice(const XsPortInfo&)*/
MFM_DLL_API XsResultValue* XsMfm_loadInputFile(struct XsMfm* thisPtr, XsResultValue* returnValue, const XsString* inputFile, const XsDeviceIdArray* deviceIds, XsDeviceIdArray* detectedDeviceIds);/*!< \copydoc XsMfm::loadInputFile(const XsString&,const XsDeviceIdArray&,XsDeviceIdArray&)*/
MFM_DLL_API XsDeviceId* XsMfm_loadedDeviceId(const struct XsMfm* thisPtr, XsDeviceId* returnValue);/*!< \copydoc XsMfm::loadedDeviceId() const*/
MFM_DLL_API XsDeviceIdArray* XsMfm_loadedDeviceIds(const struct XsMfm* thisPtr, XsDeviceIdArray* returnValue);/*!< \copydoc XsMfm::loadedDeviceIds() const*/
MFM_DLL_API XsDeviceId* XsMfm_master(const struct XsMfm* thisPtr, XsDeviceId* returnValue, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::master(const XsDeviceId&) const*/
MFM_DLL_API int XsMfm_startLogging(struct XsMfm* thisPtr, const XsDeviceId* deviceId, const XsString* logfilename);/*!< \copydoc XsMfm::startLogging(const XsDeviceId&,const XsString&)*/
MFM_DLL_API int XsMfm_startLogging_1(struct XsMfm* thisPtr, const XsDeviceIdArray* deviceIdArray, const XsStringArray* logfilenameArray, XsStringArray* finalFilenameArray);/*!< \copydoc XsMfm::startLogging(const XsDeviceIdArray&,const XsStringArray&,XsStringArray&)*/
MFM_DLL_API int XsMfm_stopLogging(struct XsMfm* thisPtr, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::stopLogging(const XsDeviceId&)*/
MFM_DLL_API int XsMfm_stopLogging_1(struct XsMfm* thisPtr, const XsDeviceIdArray* deviceIdArray);/*!< \copydoc XsMfm::stopLogging(const XsDeviceIdArray&)*/
MFM_DLL_API int XsMfm_startProcessing(struct XsMfm* thisPtr, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::startProcessing(const XsDeviceId&)*/
MFM_DLL_API int XsMfm_startProcessing_1(struct XsMfm* thisPtr, const XsDeviceIdArray* deviceIdArray);/*!< \copydoc XsMfm::startProcessing(const XsDeviceIdArray&)*/
MFM_DLL_API int XsMfm_writeResultToMt(const struct XsMfm* thisPtr, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::writeResultToMt(const XsDeviceId&) const*/
MFM_DLL_API int XsMfm_writeResultToMt_1(const struct XsMfm* thisPtr, const XsDeviceIdArray* deviceIdArray);/*!< \copydoc XsMfm::writeResultToMt(const XsDeviceIdArray&) const*/
MFM_DLL_API XsString* XsMfm_writeResultToString(const struct XsMfm* thisPtr, XsString* returnValue, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::writeResultToString(const XsDeviceId&) const*/
MFM_DLL_API int XsMfm_writeResultToFile(const struct XsMfm* thisPtr, const XsDeviceId* deviceId, const XsString* filename);/*!< \copydoc XsMfm::writeResultToFile(const XsDeviceId&,const XsString&) const*/
MFM_DLL_API int XsMfm_writeResultToFile_1(const struct XsMfm* thisPtr, const XsDeviceIdArray* deviceIdArray, const XsStringArray* filenameArray);/*!< \copydoc XsMfm::writeResultToFile(const XsDeviceIdArray&,const XsStringArray&) const*/
MFM_DLL_API void XsMfm_clearCallbackHandlers(struct XsMfm* thisPtr, int chain);/*!< \copydoc XsMfm::clearCallbackHandlers(bool)*/
MFM_DLL_API void XsMfm_addCallbackHandler(struct XsMfm* thisPtr, XsMfMCallbackPlainC* cb, int chain);/*!< \copydoc XsMfm::addCallbackHandler(XsMfMCallbackPlainC*,bool)*/
MFM_DLL_API void XsMfm_removeCallbackHandler(struct XsMfm* thisPtr, XsMfMCallbackPlainC* cb, int chain);/*!< \copydoc XsMfm::removeCallbackHandler(XsMfMCallbackPlainC*,bool)*/
MFM_DLL_API int XsMfm_isValid(const struct XsMfm* thisPtr, XsDeviceId deviceId);/*!< \copydoc XsMfm::isValid(XsDeviceId) const*/
MFM_DLL_API int XsMfm_isMap3D(const struct XsMfm* thisPtr, XsDeviceId deviceId);/*!< \copydoc XsMfm::isMap3D(XsDeviceId) const*/
MFM_DLL_API XsVersion* XsMfm_getVersion(const struct XsMfm* thisPtr, XsVersion* returnValue, XsDeviceId deviceId);/*!< \copydoc XsMfm::getVersion(XsDeviceId) const*/
MFM_DLL_API XsMatrix* XsMfm_getMagFieldMeas(const struct XsMfm* thisPtr, XsMatrix* returnValue, XsDeviceId deviceId);/*!< \copydoc XsMfm::getMagFieldMeas(XsDeviceId) const*/
MFM_DLL_API XsMatrix* XsMfm_getVerticalMeas(const struct XsMfm* thisPtr, XsMatrix* returnValue, XsDeviceId deviceId);/*!< \copydoc XsMfm::getVerticalMeas(XsDeviceId) const*/
MFM_DLL_API XsMatrix* XsMfm_getMagFieldMfm(const struct XsMfm* thisPtr, XsMatrix* returnValue, XsDeviceId deviceId);/*!< \copydoc XsMfm::getMagFieldMfm(XsDeviceId) const*/
MFM_DLL_API XsMatrix* XsMfm_getGeoSelMagFieldMfmModel(const struct XsMfm* thisPtr, XsMatrix* returnValue, XsDeviceId deviceId);/*!< \copydoc XsMfm::getGeoSelMagFieldMfmModel(XsDeviceId) const*/
MFM_DLL_API XsVector* XsMfm_getHardIronCompensation(const struct XsMfm* thisPtr, XsVector* returnValue, XsDeviceId deviceId);/*!< \copydoc XsMfm::getHardIronCompensation(XsDeviceId) const*/
MFM_DLL_API XsMatrix* XsMfm_getSoftIronCompensation(const struct XsMfm* thisPtr, XsMatrix* returnValue, XsDeviceId deviceId);/*!< \copydoc XsMfm::getSoftIronCompensation(XsDeviceId) const*/
MFM_DLL_API XsVector* XsMfm_getGeoSelMfm(const struct XsMfm* thisPtr, XsVector* returnValue, XsDeviceId deviceId);/*!< \copydoc XsMfm::getGeoSelMfm(XsDeviceId) const*/
MFM_DLL_API XsMatrix* XsMfm_getGeoSelMagFieldMeas(const struct XsMfm* thisPtr, XsMatrix* returnValue, XsDeviceId deviceId);/*!< \copydoc XsMfm::getGeoSelMagFieldMeas(XsDeviceId) const*/
MFM_DLL_API XsMatrix* XsMfm_getGeoSelMagFieldMfm(const struct XsMfm* thisPtr, XsMatrix* returnValue, XsDeviceId deviceId);/*!< \copydoc XsMfm::getGeoSelMagFieldMfm(XsDeviceId) const*/
MFM_DLL_API XsVector* XsMfm_getNormMagFieldMeas(const struct XsMfm* thisPtr, XsVector* returnValue, XsDeviceId deviceId);/*!< \copydoc XsMfm::getNormMagFieldMeas(XsDeviceId) const*/
MFM_DLL_API XsVector* XsMfm_getNormGeoSelMagFieldMeas(const struct XsMfm* thisPtr, XsVector* returnValue, XsDeviceId deviceId);/*!< \copydoc XsMfm::getNormGeoSelMagFieldMeas(XsDeviceId) const*/
MFM_DLL_API XsVector* XsMfm_getNormMagFieldMfm(const struct XsMfm* thisPtr, XsVector* returnValue, XsDeviceId deviceId);/*!< \copydoc XsMfm::getNormMagFieldMfm(XsDeviceId) const*/
MFM_DLL_API XsVector* XsMfm_getHistResidualsModel(const struct XsMfm* thisPtr, XsVector* returnValue);/*!< \copydoc XsMfm::getHistResidualsModel() const*/
MFM_DLL_API XsVector* XsMfm_getNormalizedHistResidualVertical(const struct XsMfm* thisPtr, XsVector* returnValue, XsDeviceId deviceId);/*!< \copydoc XsMfm::getNormalizedHistResidualVertical(XsDeviceId) const*/
MFM_DLL_API XsVector* XsMfm_getNormalizedHistResidualDipAngle(const struct XsMfm* thisPtr, XsVector* returnValue, XsDeviceId deviceId);/*!< \copydoc XsMfm::getNormalizedHistResidualDipAngle(XsDeviceId) const*/
MFM_DLL_API XsVector* XsMfm_getNormalizedHistResidualMagnetic(const struct XsMfm* thisPtr, XsVector* returnValue, XsDeviceId deviceId);/*!< \copydoc XsMfm::getNormalizedHistResidualMagnetic(XsDeviceId) const*/
MFM_DLL_API XsVector* XsMfm_getHistResidualsModelBins(const struct XsMfm* thisPtr, XsVector* returnValue);/*!< \copydoc XsMfm::getHistResidualsModelBins() const*/
MFM_DLL_API XsVector* XsMfm_getNormalizedHistResidualBins(const struct XsMfm* thisPtr, XsVector* returnValue);/*!< \copydoc XsMfm::getNormalizedHistResidualBins() const*/
/*! @} */
#ifdef __cplusplus
} // extern "C"
struct XsMfm {
	//! \brief Construct a new XsMfm* object. Clean it up with the destruct() function or delete the object
	inline static XsMfm* construct(void)
	{
		return XsMfm_construct();
	}

	//! \brief Destruct a XsMfm object and free all memory allocated for it
	inline void destruct(void)
	{
		XsMfm_destruct(this);
	}

	inline static XsVersion version(void)
	{
		XsVersion returnValue;
		return *XsMfm_version(&returnValue);
	}

	inline void reset(void)
	{
		XsMfm_reset(this);
	}

	inline bool scanMfmDevices(void)
	{
		return 0 != XsMfm_scanMfmDevices(this);
	}

	inline bool scanMfmDevice(const XsPortInfo& portInfo)
	{
		return 0 != XsMfm_scanMfmDevice(this, &portInfo);
	}

	inline XsResultValue loadInputFile(const XsString& inputFile, const XsDeviceIdArray& deviceIds, XsDeviceIdArray& detectedDeviceIds)
	{
		XsResultValue returnValue;
		return *XsMfm_loadInputFile(this, &returnValue, &inputFile, &deviceIds, &detectedDeviceIds);
	}

	inline XsDeviceId loadedDeviceId(void) const
	{
		XsDeviceId returnValue;
		return *XsMfm_loadedDeviceId(this, &returnValue);
	}

	inline XsDeviceIdArray loadedDeviceIds(void) const
	{
		XsDeviceIdArray returnValue;
		return *XsMfm_loadedDeviceIds(this, &returnValue);
	}

	inline XsDeviceId master(const XsDeviceId& deviceId) const
	{
		XsDeviceId returnValue;
		return *XsMfm_master(this, &returnValue, &deviceId);
	}

	inline bool startLogging(const XsDeviceId& deviceId, const XsString& logfilename)
	{
		return 0 != XsMfm_startLogging(this, &deviceId, &logfilename);
	}

	inline bool startLogging(const XsDeviceIdArray& deviceIdArray, const XsStringArray& logfilenameArray, XsStringArray& finalFilenameArray)
	{
		return 0 != XsMfm_startLogging_1(this, &deviceIdArray, &logfilenameArray, &finalFilenameArray);
	}

	inline bool stopLogging(const XsDeviceId& deviceId)
	{
		return 0 != XsMfm_stopLogging(this, &deviceId);
	}

	inline bool stopLogging(const XsDeviceIdArray& deviceIdArray)
	{
		return 0 != XsMfm_stopLogging_1(this, &deviceIdArray);
	}

	inline bool startProcessing(const XsDeviceId& deviceId)
	{
		return 0 != XsMfm_startProcessing(this, &deviceId);
	}

	inline bool startProcessing(const XsDeviceIdArray& deviceIdArray)
	{
		return 0 != XsMfm_startProcessing_1(this, &deviceIdArray);
	}

	inline bool writeResultToMt(const XsDeviceId& deviceId) const
	{
		return 0 != XsMfm_writeResultToMt(this, &deviceId);
	}

	inline bool writeResultToMt(const XsDeviceIdArray& deviceIdArray) const
	{
		return 0 != XsMfm_writeResultToMt_1(this, &deviceIdArray);
	}

	inline XsString writeResultToString(const XsDeviceId& deviceId) const
	{
		XsString returnValue;
		return *XsMfm_writeResultToString(this, &returnValue, &deviceId);
	}

	inline bool writeResultToFile(const XsDeviceId& deviceId, const XsString& filename) const
	{
		return 0 != XsMfm_writeResultToFile(this, &deviceId, &filename);
	}

	inline bool writeResultToFile(const XsDeviceIdArray& deviceIdArray, const XsStringArray& filenameArray) const
	{
		return 0 != XsMfm_writeResultToFile_1(this, &deviceIdArray, &filenameArray);
	}

	inline void clearCallbackHandlers(bool chain = true)
	{
		XsMfm_clearCallbackHandlers(this, chain);
	}

	inline void addCallbackHandler(XsMfMCallbackPlainC* cb, bool chain = true)
	{
		XsMfm_addCallbackHandler(this, cb, chain);
	}

	inline void removeCallbackHandler(XsMfMCallbackPlainC* cb, bool chain = true)
	{
		XsMfm_removeCallbackHandler(this, cb, chain);
	}

	inline bool isValid(XsDeviceId deviceId) const
	{
		return 0 != XsMfm_isValid(this, deviceId);
	}

	inline bool isMap3D(XsDeviceId deviceId) const
	{
		return 0 != XsMfm_isMap3D(this, deviceId);
	}

	inline XsVersion getVersion(XsDeviceId deviceId) const
	{
		XsVersion returnValue;
		return *XsMfm_getVersion(this, &returnValue, deviceId);
	}

	inline XsMatrix getMagFieldMeas(XsDeviceId deviceId) const
	{
		XsMatrix returnValue;
		return *XsMfm_getMagFieldMeas(this, &returnValue, deviceId);
	}

	inline XsMatrix getVerticalMeas(XsDeviceId deviceId) const
	{
		XsMatrix returnValue;
		return *XsMfm_getVerticalMeas(this, &returnValue, deviceId);
	}

	inline XsMatrix getMagFieldMfm(XsDeviceId deviceId) const
	{
		XsMatrix returnValue;
		return *XsMfm_getMagFieldMfm(this, &returnValue, deviceId);
	}

	inline XsMatrix getGeoSelMagFieldMfmModel(XsDeviceId deviceId) const
	{
		XsMatrix returnValue;
		return *XsMfm_getGeoSelMagFieldMfmModel(this, &returnValue, deviceId);
	}

	inline XsVector getHardIronCompensation(XsDeviceId deviceId) const
	{
		XsVector returnValue;
		return *XsMfm_getHardIronCompensation(this, &returnValue, deviceId);
	}

	inline XsMatrix getSoftIronCompensation(XsDeviceId deviceId) const
	{
		XsMatrix returnValue;
		return *XsMfm_getSoftIronCompensation(this, &returnValue, deviceId);
	}

	inline XsVector getGeoSelMfm(XsDeviceId deviceId) const
	{
		XsVector returnValue;
		return *XsMfm_getGeoSelMfm(this, &returnValue, deviceId);
	}

	inline XsMatrix getGeoSelMagFieldMeas(XsDeviceId deviceId) const
	{
		XsMatrix returnValue;
		return *XsMfm_getGeoSelMagFieldMeas(this, &returnValue, deviceId);
	}

	inline XsMatrix getGeoSelMagFieldMfm(XsDeviceId deviceId) const
	{
		XsMatrix returnValue;
		return *XsMfm_getGeoSelMagFieldMfm(this, &returnValue, deviceId);
	}

	inline XsVector getNormMagFieldMeas(XsDeviceId deviceId) const
	{
		XsVector returnValue;
		return *XsMfm_getNormMagFieldMeas(this, &returnValue, deviceId);
	}

	inline XsVector getNormGeoSelMagFieldMeas(XsDeviceId deviceId) const
	{
		XsVector returnValue;
		return *XsMfm_getNormGeoSelMagFieldMeas(this, &returnValue, deviceId);
	}

	inline XsVector getNormMagFieldMfm(XsDeviceId deviceId) const
	{
		XsVector returnValue;
		return *XsMfm_getNormMagFieldMfm(this, &returnValue, deviceId);
	}

	inline XsVector getHistResidualsModel(void) const
	{
		XsVector returnValue;
		return *XsMfm_getHistResidualsModel(this, &returnValue);
	}

	inline XsVector getNormalizedHistResidualVertical(XsDeviceId deviceId) const
	{
		XsVector returnValue;
		return *XsMfm_getNormalizedHistResidualVertical(this, &returnValue, deviceId);
	}

	inline XsVector getNormalizedHistResidualDipAngle(XsDeviceId deviceId) const
	{
		XsVector returnValue;
		return *XsMfm_getNormalizedHistResidualDipAngle(this, &returnValue, deviceId);
	}

	inline XsVector getNormalizedHistResidualMagnetic(XsDeviceId deviceId) const
	{
		XsVector returnValue;
		return *XsMfm_getNormalizedHistResidualMagnetic(this, &returnValue, deviceId);
	}

	inline XsVector getHistResidualsModelBins(void) const
	{
		XsVector returnValue;
		return *XsMfm_getHistResidualsModelBins(this, &returnValue);
	}

	inline XsVector getNormalizedHistResidualBins(void) const
	{
		XsVector returnValue;
		return *XsMfm_getNormalizedHistResidualBins(this, &returnValue);
	}

	//! \brief Destructor, calls destruct() function to clean up object
	~XsMfm()
	{
		XsMfm_destruct(this);
	}

	//! \brief overloaded delete operator to allow user to use delete instead of calling destruct() function
	void operator delete (void*)
	{
	}

private:
	XsMfm(); //!< \brief Default constructor not implemented to prevent faulty memory allocation, use construct() function instead
#ifndef SWIG
	void* operator new (size_t); //!< \brief new operator not implemented to prevent faulty memory allocation by user, use construct() function instead
	void* operator new[] (size_t); //!< \brief array new operator not implemented to prevent faulty memory allocation by user, use construct() function instead
	void operator delete[] (void*); //!< \brief array delete operator not implemented to prevent faulty memory deallocation by user, use destruct() function instead
#endif
};
#endif // __cplusplus
#endif // GUARD_XSMFM_H_TMP
