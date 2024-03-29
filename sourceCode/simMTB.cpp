#include "simMTB.h"
#include <simLib/simLib.h>
#include <simLib/scriptFunctionData.h>
#include <simLib/socketOutConnection.h>
#include <iostream>
#include <boost/lexical_cast.hpp>

#ifdef QT_COMPIL
    #include <QString>
    #include <QProcess>
#endif

#define PLUGIN_VERSION 12
                // version 8 is after CoppeliaSim 3.3.0. Using stacks to exchange data with scripts.
                // version 9 is after CoppeliaSim 3.4.0. Using the new API notation.
                // version 10 is after CoppeliaSim 4.2.0.
                // version 11 is after CoppeliaSim 4.3.0.
                // version 12 is after CoppeliaSim 4.5.1.

LIBRARY simLib;

struct sMtbServer
{
    CSocketOutConnection* connection;
    int mtbServerHandle;
    int scriptHandle;
    float jointPositions[4];
    unsigned char robotInput[4];
    unsigned char robotOutput[4];
    std::vector<int> inputOutputConnections; // indicates which input buffer bit is linked with what output buffer bit:
                                            // value[0+4*n] = input bit number (1-32) of this robot 
                                            // value[1+4*n] = handle of the server robot on which we want to connect an output
                                            // value[2+4*n] = output bit number (1-32) of the other robot (see above) 
                                            // value[3+4*n] = connection type (0=same, 1=inverted) 
};

std::vector<sMtbServer> allMtbServers;
int nextMtbServerHandle=0;
std::string currentDirAndPath;

int getServerIndexFromServerHandle(int serverHandle)
{
    for (unsigned int i=0;i<allMtbServers.size();i++)
    {
        if (allMtbServers[i].mtbServerHandle==serverHandle)
            return(i);
    }
    return(-1);
}

int getServerIndexFromScriptHandle(int h)
{
    for (unsigned int i=0;i<allMtbServers.size();i++)
    {
        if (allMtbServers[i].scriptHandle==h)
            return(i);
    }
    return(-1);
}

void removeOutputInputConnection(int inputIndex,int inputBit)
{
    if ((inputIndex>=0)&&(inputIndex<int(allMtbServers.size())))
    {
        if (inputBit==0)
            allMtbServers[inputIndex].inputOutputConnections.clear();
        else
        {
            for (unsigned int i=0;i<allMtbServers[inputIndex].inputOutputConnections.size()/4;i++)
            {
                if (allMtbServers[inputIndex].inputOutputConnections[4*i+0]==inputBit)
                {
                    allMtbServers[inputIndex].inputOutputConnections.erase(allMtbServers[inputIndex].inputOutputConnections.begin()+4*i+0,allMtbServers[inputIndex].inputOutputConnections.begin()+4*i+4);
                    break;
                }
            }
        }
    }
}

void updateInputsFromConnections(int inputIndex)
{
    if ((inputIndex>=0)&&(inputIndex<int(allMtbServers.size())))
    {
        for (unsigned int i=0;i<allMtbServers[inputIndex].inputOutputConnections.size()/4;i++)
        {
            int outputIndex=allMtbServers[inputIndex].inputOutputConnections[4*i+1];
            if ((outputIndex>=0)&&(outputIndex<int(allMtbServers.size())))
            {
                unsigned char otherRobotOutputData[4];
                for (int j=0;j<4;j++)
                    otherRobotOutputData[j]=allMtbServers[outputIndex].robotOutput[j];

                int inBytePos=(allMtbServers[inputIndex].inputOutputConnections[4*i+0]-1)/8; // 0-3
                int inBitPos=(allMtbServers[inputIndex].inputOutputConnections[4*i+0]-1)%8; //0-7

                int outBytePos=(allMtbServers[inputIndex].inputOutputConnections[4*i+2]-1)/8; // 0-3
                int outBitPos=(allMtbServers[inputIndex].inputOutputConnections[4*i+2]-1)%8; //0-7

                bool setInputBit=false;
                if (otherRobotOutputData[outBytePos]&(1<<outBitPos))
                    setInputBit=((1-allMtbServers[inputIndex].inputOutputConnections[4*i+3])!=0);
                else
                    setInputBit=(allMtbServers[inputIndex].inputOutputConnections[4*i+3]!=0);

                if (setInputBit)
                    allMtbServers[inputIndex].robotInput[inBytePos]|=(1<<inBitPos);
                else
                    allMtbServers[inputIndex].robotInput[inBytePos]&=255-(1<<inBitPos);
            }
            else
            { // the other server robot was removed. We have to remove this output-input connection:
                allMtbServers[inputIndex].inputOutputConnections.erase(allMtbServers[inputIndex].inputOutputConnections.begin()+4*i+0,allMtbServers[inputIndex].inputOutputConnections.begin()+4*i+4);
                i--; // we have to reprocess this position in the for-loop!
            }
        }
    }
}

void updateAllInputs()
{ // Since each output can influence each input, and this cascaded, we call the update n*n times, where n is the number of servers
    for (unsigned int j=0;j<allMtbServers.size();j++)
    {
        for (unsigned int i=0;i<allMtbServers.size();i++)
            updateInputsFromConnections(i);
    }
}

bool addOutputInputConnection(int inputIndex,int inputBit,int outputIndex,int outputBit,int connectionType)
{
    if ((inputIndex<0)||(inputIndex>=int(allMtbServers.size())))
        return(false);
    if ((outputIndex<0)||(outputIndex>=int(allMtbServers.size())))
        return(false);
    if ( (inputBit<1)||(inputBit>32)||(outputBit<1)||(outputBit>32)||(connectionType<0)||(connectionType>1) )
        return(false);
    removeOutputInputConnection(inputIndex,inputBit);
    //4. Add the connection:
    allMtbServers[inputIndex].inputOutputConnections.push_back(inputBit);
    allMtbServers[inputIndex].inputOutputConnections.push_back(outputIndex);
    allMtbServers[inputIndex].inputOutputConnections.push_back(outputBit);
    allMtbServers[inputIndex].inputOutputConnections.push_back(connectionType);
    return(true);
}


// --------------------------------------------------------------------------------------
// simMTB.startServer
// --------------------------------------------------------------------------------------
const int inArgs_START_SERVER[]={
    5,
    sim_script_arg_string,0,
    sim_script_arg_int32,0,
    sim_script_arg_charbuff,0,
    sim_script_arg_float|sim_script_arg_table,4,
    sim_script_arg_float|sim_script_arg_table,2,
};

void LUA_START_SERVER_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    int handle=-1; // means error
    std::string msg("Problem launching or communicating with the MTB server.");
    if (D.readDataFromStack(p->stackID,inArgs_START_SERVER,inArgs_START_SERVER[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        std::string mtbServerExecutableName(inData->at(0).stringData[0]);
        int portNumber=inData->at(1).int32Data[0];
        std::string arguments(boost::lexical_cast<std::string>(portNumber));
        std::string program(inData->at(2).stringData[0]);

        #ifdef QT_COMPIL
            QString cmd;
            if (mtbServerExecutableName.find('/')==std::string::npos)
                cmd="./";
            cmd+=QString(mtbServerExecutableName.c_str());
            QStringList strList;
            strList << QString(arguments.c_str());
            QProcess::startDetached(cmd,strList,QString(currentDirAndPath.c_str()),NULL);
            #ifdef _WIN32
                Sleep(1000);
            #else
                sleep(1);
            #endif
        #else
            #ifdef _WIN32
                ShellExecuteA(NULL,"open",mtbServerExecutableName.c_str(),arguments.c_str(),NULL,SW_SHOWDEFAULT);
                Sleep(1000);
            #else
                pid_t pid=fork();
                if (pid==0)
                {
                    execl(mtbServerExecutableName.c_str(),mtbServerExecutableName.c_str(),inData->at(0).stringData[0].c_str(),inData->at(1).stringData[0].c_str(),boost::lexical_cast<std::string>(inData->at(2).int32Data[0]).c_str(), (char*) 0);
                    exit(0);
                }
                sleep(1);
            #endif
        #endif
        
        CSocketOutConnection* connection=new CSocketOutConnection("127.0.0.1",portNumber);
        if (connection->connectToServer()==1)
        { // we could connect!
            // Now send the program and initial values to the server:
            std::string dat(25,0);
            dat[0]=0; // compile command
            for (unsigned int i=0;i<4;i++)
                ((float*)(&dat[1+i*4]))[0]=inData->at(3).floatData[i];
            for (unsigned int i=0;i<2;i++)
                ((float*)(&dat[1+16+i*4]))[0]=inData->at(4).floatData[i];
            dat.insert(dat.end(),program.begin(),program.end());
            if (connection->sendData(&dat[0],int(dat.size())))
            { // send was successful
                // Now wait for the status reply:
                int dataSize;
                char* data=connection->receiveReplyData(dataSize);
                if (dataSize>0)
                { // data reception was ok!
                    if (dataSize==2)
                    { // ok, the program was correctly compiled
                        handle=nextMtbServerHandle++;
                        sMtbServer server;
                        server.connection=connection;
                        server.mtbServerHandle=handle;
                        server.scriptHandle=p->scriptID;
                        for (unsigned int i=0;i<4;i++)
                            server.jointPositions[i]=inData->at(3).floatData[i];
                        for (unsigned int i=0;i<4;i++)
                        {
                            server.robotInput[i]=0;
                            server.robotOutput[i]=0;
                        }
                        allMtbServers.push_back(server);
                        updateAllInputs();
                    }
                    else
                    { // there was a problem. Return the error string:
                        msg=std::string(data+1);
                        delete connection;
                    }
                    delete[] data;
                }
                else
                {
                    delete connection;
                    simSetLastError(nullptr,"Failed to receive data from the MTB server.");
                }
            }
            else
            {
                delete connection;
                simSetLastError(nullptr,"Failed to send data to the MTB server.");
            }
        }
        else
        {
            delete connection;
            simSetLastError(nullptr,"Failed to start or connect to MTB server.");
        }
            
    }
    else
        msg="Wrong or missing arguments.";
    D.pushOutData(CScriptFunctionDataItem(handle));
    if (handle<0)
        D.pushOutData(CScriptFunctionDataItem(msg));
    else
        D.pushOutData(CScriptFunctionDataItem(""));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------
// simMTB.stopServer
// --------------------------------------------------------------------------------------
const int inArgs_STOP_SERVER[]={
    1,
    sim_script_arg_int32,0,
};

void LUA_STOP_SERVER_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    bool success=false;
    if (D.readDataFromStack(p->stackID,inArgs_STOP_SERVER,inArgs_STOP_SERVER[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int serverHandle=inData->at(0).int32Data[0];
        int index=getServerIndexFromServerHandle(serverHandle);
        if (index!=-1)
        {
            delete allMtbServers[index].connection;
            allMtbServers.erase(allMtbServers.begin()+index);
            updateAllInputs();
            success=true;
        }
        else
            simSetLastError(nullptr,"Invalid MTB server handle.");
    }
    D.pushOutData(CScriptFunctionDataItem(success));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simMTB.step
// --------------------------------------------------------------------------------------
const int inArgs_STEP[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_float,0,
};

void LUA_STEP_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    std::string msg("Invalid MTB server handle, or problem communicating with the MTB server.");
    int result=-1;
    if (D.readDataFromStack(p->stackID,inArgs_STEP,inArgs_STEP[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int serverHandle=inData->at(0).int32Data[0];
        float timeStep=inData->at(1).floatData[0];
        int index=getServerIndexFromServerHandle(serverHandle);
        if (index!=-1)
        {
            // Now send the data to the server:
            std::string dat(9,0);
            dat[0]=1; // step command
            dat[1]=allMtbServers[index].robotInput[0];
            dat[2]=allMtbServers[index].robotInput[1];
            dat[3]=allMtbServers[index].robotInput[2];
            dat[4]=allMtbServers[index].robotInput[3];
            ((float*)&dat[5])[0]=timeStep;
            if (allMtbServers[index].connection->sendData(&dat[0],int(dat.size())))
            { // send was successful
                // Now wait for the status reply:
                int dataSize;
                char* data=allMtbServers[index].connection->receiveReplyData(dataSize);
                if (dataSize>=22)
                { // data reception was ok!
                    for (unsigned int i=0;i<4;i++)
                        allMtbServers[index].jointPositions[i]=((float*)(data+1))[i];
                    for (unsigned int i=0;i<4;i++)
                        allMtbServers[index].robotOutput[i]=data[1+16+i];
                    updateAllInputs();
                    msg=std::string(data+1+16+4);
                    if (msg.length()!=0)
                        result=0; // program not yet finished
                    else
                        result=1; // program finished

                    delete[] data;
                }
                else
                    simSetLastError(nullptr,"Failed to receive data from the MTB server.");
            }
            else
                simSetLastError(nullptr,"Failed to send data to the MTB server.");
        }
        else
            simSetLastError(nullptr,"Invalid MTB server handle.");
    }
    else
        msg="Wrong or missing arguments.";
    D.pushOutData(CScriptFunctionDataItem(result));
    D.pushOutData(CScriptFunctionDataItem(msg));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simMTB.getJoints
// --------------------------------------------------------------------------------------
const int inArgs_GET_JOINTS[]={
    1,
    sim_script_arg_int32,0,
};

void LUA_GET_JOINTS_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    std::vector<float> joints;
    if (D.readDataFromStack(p->stackID,inArgs_GET_JOINTS,inArgs_GET_JOINTS[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int serverHandle=inData->at(0).int32Data[0];
        int index=getServerIndexFromServerHandle(serverHandle);
        if (index!=-1)
        {
            for (unsigned int i=0;i<4;i++)
                joints.push_back(allMtbServers[index].jointPositions[i]);
        }
        else
            simSetLastError(nullptr,"Invalid MTB server handle.");
    }
    if (joints.size()!=0)
        D.pushOutData(CScriptFunctionDataItem(joints));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simMTB.getOutput
// --------------------------------------------------------------------------------------
const int inArgs_GET_OUTPUT[]={
    1,
    sim_script_arg_int32,0,
};

void LUA_GET_OUTPUT_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    std::vector<int> output;
    if (D.readDataFromStack(p->stackID,inArgs_GET_OUTPUT,inArgs_GET_OUTPUT[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int serverHandle=inData->at(0).int32Data[0];
        int index=getServerIndexFromServerHandle(serverHandle);
        if (index!=-1)
        {
            for (unsigned int i=0;i<4;i++)
                output.push_back(allMtbServers[index].robotOutput[i]);
        }
        else
            simSetLastError(nullptr,"Invalid MTB server handle.");
    }
    if (output.size()!=0)
        D.pushOutData(CScriptFunctionDataItem(output));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simMTB.getInput
// --------------------------------------------------------------------------------------
const int inArgs_GET_INPUT[]={
    1,
    sim_script_arg_int32,0,
};

void LUA_GET_INPUT_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    std::vector<int> input;
    if (D.readDataFromStack(p->stackID,inArgs_GET_INPUT,inArgs_GET_INPUT[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int serverHandle=inData->at(0).int32Data[0];
        int index=getServerIndexFromServerHandle(serverHandle);
        if (index!=-1)
        {
            for (unsigned int i=0;i<4;i++)
                input.push_back(allMtbServers[index].robotInput[i]);
        }
        else
            simSetLastError(nullptr,"Invalid MTB server handle.");
    }
    if (input.size()!=0)
        D.pushOutData(CScriptFunctionDataItem(input));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simMTB.setInput
// --------------------------------------------------------------------------------------
const int inArgs_SET_INPUT[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_int32|sim_script_arg_table,4,
};

void LUA_SET_INPUT_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_SET_INPUT,inArgs_SET_INPUT[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int serverHandle=inData->at(0).int32Data[0];
        int index=getServerIndexFromServerHandle(serverHandle);
        if (index!=-1)
        {
            for (unsigned int i=0;i<4;i++)
                allMtbServers[index].robotInput[i]=(unsigned char)inData->at(1).int32Data[i];
            updateAllInputs();
            result=true;
        }
        else
            simSetLastError(nullptr,"Invalid MTB server handle.");
    }
    D.pushOutData(CScriptFunctionDataItem(result));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simMTB.connectInput
// --------------------------------------------------------------------------------------
const int inArgs_CONNECT_INPUT[]={
    5,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_CONNECT_INPUT_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_CONNECT_INPUT,inArgs_CONNECT_INPUT[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int inputServerHandle=inData->at(0).int32Data[0];
        int inputIndex=getServerIndexFromServerHandle(inputServerHandle);
        int inputBits=inData->at(1).int32Data[0];
        int outputServerHandle=inData->at(2).int32Data[0];
        int outputIndex=getServerIndexFromServerHandle(outputServerHandle);
        int outputBits=inData->at(3).int32Data[0];
        int connectionType=inData->at(4).int32Data[0];
        if ((inputIndex!=-1)&&(outputIndex!=-1))
        {
            result=addOutputInputConnection(inputIndex,inputBits,outputIndex,outputBits,connectionType);        
            updateAllInputs();
        }
        else
            simSetLastError(nullptr,"Invalid MTB server handle.");
    }
    D.pushOutData(CScriptFunctionDataItem(result));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simMTB.disconnectInput
// --------------------------------------------------------------------------------------
const int inArgs_DISCONNECT_INPUT[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
};

void LUA_DISCONNECT_INPUT_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    bool result=false;
    if (D.readDataFromStack(p->stackID,inArgs_DISCONNECT_INPUT,inArgs_DISCONNECT_INPUT[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int inputServerHandle=inData->at(0).int32Data[0];
        int inputIndex=getServerIndexFromServerHandle(inputServerHandle);
        int inputBits=inData->at(1).int32Data[0];
        if (inputIndex!=-1)
        {
            removeOutputInputConnection(inputIndex,inputBits);
            updateAllInputs();
            result=true;
        }
        else
            simSetLastError(nullptr,"Invalid MTB server handle.");
    }
    D.pushOutData(CScriptFunctionDataItem(result));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

SIM_DLLEXPORT int simInit(SSimInit* info)
{
    simLib=loadSimLibrary(info->coppeliaSimLibPath);
    if (simLib==NULL)
    {
        simAddLog(info->pluginName,sim_verbosity_errors,"could not find or correctly load the CoppeliaSim library. Cannot start the plugin.");
        return(0); 
    }
    if (getSimProcAddresses(simLib)==0)
    {
        simAddLog(info->pluginName,sim_verbosity_errors,"could not find all required functions in the CoppeliaSim library. Cannot start the plugin.");
        unloadSimLibrary(simLib);
        return(0);
    }

    // Register the new functions:
    simRegisterScriptCallbackFunction("startServer",nullptr,LUA_START_SERVER_CALLBACK);
    simRegisterScriptCallbackFunction("stopServer",nullptr,LUA_STOP_SERVER_CALLBACK);
    simRegisterScriptCallbackFunction("step",nullptr,LUA_STEP_CALLBACK);
    simRegisterScriptCallbackFunction("getJoints",nullptr,LUA_GET_JOINTS_CALLBACK);
    simRegisterScriptCallbackFunction("getOutput",nullptr,LUA_GET_OUTPUT_CALLBACK);
    simRegisterScriptCallbackFunction("getInput",nullptr,LUA_GET_INPUT_CALLBACK);
    simRegisterScriptCallbackFunction("setInput",nullptr,LUA_SET_INPUT_CALLBACK);
    simRegisterScriptCallbackFunction("connectInput",nullptr,LUA_CONNECT_INPUT_CALLBACK);
    simRegisterScriptCallbackFunction("disconnectInput",nullptr,LUA_DISCONNECT_INPUT_CALLBACK);

    return(PLUGIN_VERSION);
}

SIM_DLLEXPORT void simCleanup()
{
    unloadSimLibrary(simLib);
}

SIM_DLLEXPORT void simMsg(SSimMsg* info)
{
    if (info->msgId==sim_message_eventcallback_scriptstatedestroyed)
    { // script state was destroyed. Destroy all associated server instances:
        int index=getServerIndexFromScriptHandle(info->auxData[0]);
        while (index>=0)
        {
            allMtbServers.erase(allMtbServers.begin()+index);
            index=getServerIndexFromScriptHandle(info->auxData[0]);
        }
    }
}

