#! /bin/bash

green=`tput setaf 2`
red=`tput setaf 1`

cd ../src/MotionControlSystem/Encoders/test

if [ $# -eq 1 ] && [ $1 = "recompile" ] 
then
    if [ ! -f "test.cpp" ]; then
        echo -n "${red}"
        ls
        echo " test.cpp does not exist"
        exit 1
    fi

    g++ -std=c++11 -D TEST -Wall ../PinMaskDefines.h test.cpp -o test

    ./test

    exit 0

else

    if [ ! -f "test" ]; then
        if [ ! -f "test.cpp" ]; then
            exit 1
        fi

        g++ -std=c++11 -D TEST -Wall ../PinMaskDefines.h test.cpp -o test

        echo "${green}tests begin"
        ./test

        exit 0
    fi

    echo "${green}tests begin"
    echo "+++++++++++++++++++++++++++++"
    echo -n "${red}"
    ./test

    if [ $? -eq 0 ]; then
        echo "${green}test passed"
    fi

    exit 0

fi