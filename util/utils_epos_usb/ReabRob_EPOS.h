#if !defined(EposCommandLibraryDefinitions)
    #include<DefinitionsEPOS.h>
#endif



#if !defined(REABROB_EPOS_H)
    #define REABROB_EPOS_H

class USB_Network{
    public:
        HANDLE epos;
        char* deviceName;
        char* protocolStackName;
        char* interfaceName;
        char* portName;
        DWORD errorCode;
        WORD nodeId;
        std::mutex mtx;

        USB_Network(HANDLE epos,char* deviceName,char* protocolStackName){
            this->epos = VCS_OpenSubDevice(epos,deviceName,protocolStackName,&errorCode);
            this->deviceName = deviceName;
            this->protocolStackName = protocolStackName;
        }

        USB_Network(char* deviceName,char* protocolStackName,char* interfaceName,char* portName){
            mtx.lock();
            this->deviceName = deviceName;
            this->protocolStackName = protocolStackName;
            this->interfaceName = interfaceName;
            this->portName = portName;
            this->nodeId = 32;
            mtx.unlock();
        }
        bool connect(){
            this->epos = VCS_OpenDevice(deviceName,protocolStackName,interfaceName,portName,&errorCode);
            return(this->epos>0);
        }
        void setNodeId(WORD id){
            mtx.lock();
            this->nodeId = id;
            mtx.unlock();
        }
        
        void resetErrors(){
            VCS_ClearFault(epos,nodeId,&errorCode);
        }

        void close(){
            mtx.lock();
            
            VCS_CloseDevice(epos,&errorCode);
            mtx.unlock();

        }

        bool configureInputs(WORD nodeID){
            mtx.lock();
            VCS_AnalogInputConfiguration(epos,nodeID,1,AIC_GENERAL_PURPOSE_A,true,&errorCode);
            VCS_AnalogInputConfiguration(epos,nodeID,2,AIC_GENERAL_PURPOSE_B,true,&errorCode);
            mtx.unlock();
            return true;
        }

        bool setState(WORD state){
            return VCS_SetState(epos,nodeId,state,&errorCode);
        }

        bool setState(WORD id,WORD state){
            mtx.lock();
            VCS_SetState(epos,id,state,&errorCode);
            mtx.unlock();
            return true;
        }

        bool configure(){
            resetErrors();
             if(VCS_SetProtocolStackSettings(epos,1000000,50,&errorCode)>0)
             if(VCS_ActivateVelocityMode(epos,nodeId,&errorCode))
             if(VCS_SetMaxProfileVelocity(epos,nodeId,1000,&errorCode))
             return true;
             return false;
        }  

        bool configure(WORD id){
            mtx.lock();
            resetErrors();
             if(VCS_SetProtocolStackSettings(epos,1000000,50,&errorCode)>0)
             if(VCS_ActivateVelocityMode(epos,id,&errorCode))
             if(VCS_SetMaxProfileVelocity(epos,id,1000,&errorCode)){
                mtx.unlock();
                 return true; 

             }
                mtx.unlock();
             return false;
        }     

        bool setVelocity(int val){
            return VCS_SetVelocityMust(epos,nodeId,val,&errorCode);
        }

        bool setVelocity(WORD id,int val){
            mtx.lock();
            bool res = VCS_SetVelocityMust(epos,id,val,&errorCode);
            mtx.unlock();
            return res;
        }

        float analogRead(WORD port){
            long A_input;
            mtx.lock();
            VCS_GetAnalogInputVoltage(this->epos,nodeId,port,&A_input,&errorCode);
            mtx.unlock();
            return A_input/1000.0f;
        }
        float analogRead(WORD id,WORD port){
            long A_input;
            mtx.lock();
            // VCS_AnalogInputConfiguration
            VCS_GetAnalogInputVoltage(this->epos,id,port,&A_input,&errorCode);
            mtx.unlock();
            return float(A_input);
        }

        float positionRead(){
            long pos;
            VCS_GetPositionIs(epos,nodeId,&pos,&errorCode);
            return pos*1.0f;
        }   
        float positionRead(WORD id){
            mtx.lock();
            long pos;
            VCS_GetPositionIs(epos,id,&pos,&errorCode);
            mtx.unlock();
            return (float) pos*1.0f;
        }

};

#endif