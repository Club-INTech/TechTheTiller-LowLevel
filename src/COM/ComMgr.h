//
// Created by asphox on 24/04/18.
//

#ifndef TECHTHETOWN_LOWLEVEL_COMMGR_H
#define TECHTHETOWN_LOWLEVEL_COMMGR_H

#include "Interfaces/SerialInterface.h"
//
#include "Utils/Singleton.hpp"
#include "Utils/Median.h"
#include "Config/ComOptions.h"
#include <map>
#include <array>

class ComMgr : public Singleton<ComMgr>
{
public:

    ComMgr() = default;

    void init() override;

    template<typename T>
    bool read(T data){
        static bool r1=false,r2=false;
        if( com_options & COM_OPTIONS::SERIAL_R && !r1 )
            r2 = serial.read(data);
        return r1||r2;
    }

    /* ENVOI */

    void printfln(Header header,const char*, ...) __attribute__((format(printf, 3, 4)));
    void printf(Header header,const char*,...) __attribute__((format(printf, 3, 4)));
    void printOnSerial(const char*);

private:
    SerialInterface serial;

};


#endif //TECHTHETOWN_LOWLEVEL_COMMGR_H
