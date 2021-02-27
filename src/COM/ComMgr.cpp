//
// Created by asphox on 24/04/18.
//

#include "ComMgr.h"

void ComMgr::init() {
    if(com_options & SERIAL_W) {
        while(!Serial);
        Serial.begin(115200);
    }
}

void ComMgr::printfln(Header header,const char * data,...)
{
    va_list args;
    va_start(args,data);

    char tmp[HEADER_LENGTH + MAX_MESSAGE_LENGTH];
    memcpy(tmp,header,HEADER_LENGTH);
    tmp[HEADER_LENGTH] = '\0';

    strcat(tmp+HEADER_LENGTH, data);
    char formatted[HEADER_LENGTH + MAX_MESSAGE_LENGTH];
    vsnprintf(formatted, HEADER_LENGTH+MAX_MESSAGE_LENGTH, tmp, args);

    if( com_options & COM_OPTIONS::SERIAL_W ) {
        if(header != POSITION_HEADER) { // pas besoin de spammer la série avec les positions
            serial.printfln(formatted);
        }
    }
    if( memcmp(header,DEBUG_HEADER,HEADER_LENGTH) ) {
        if(header != POSITION_HEADER) { // pas besoin de spammer la série avec les positions
            printfln(DEBUG_HEADER,formatted);
        }
    }

    va_end(args);
}

void ComMgr::printf(Header header, const char *data, ...)
{
    va_list args;
    va_start(args,data);

    char tmp[HEADER_LENGTH + 64];
    memcpy(tmp,header,HEADER_LENGTH);
    tmp[HEADER_LENGTH] = '\0';

    strcat(tmp+HEADER_LENGTH, data);
    char formatted[HEADER_LENGTH + 64];
    vsnprintf(formatted, HEADER_LENGTH+64, tmp, args);

    if( com_options & COM_OPTIONS::SERIAL_W )
        serial.printf(formatted);

    if( memcmp(header,DEBUG_HEADER,HEADER_LENGTH) )
        printfln(DEBUG_HEADER,formatted);

    va_end(args);
}

void ComMgr::printOnSerial(const char* str)
{
    Serial.print(str);
}
