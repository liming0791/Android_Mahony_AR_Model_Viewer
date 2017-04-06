#ifndef TIMER_H
#define TIMER_H

#include "stdio.h"
#include "stdlib.h"

#include <iostream>
#include <chrono>

using namespace std;

class Timer
{

    public:
        Timer() = default;

        static chrono::steady_clock::time_point t1, t2;

        static void Begin() { t1 = chrono::steady_clock::now(); };
        static void End(const string& logTAG) {
            t2 = chrono::steady_clock::now();        
            chrono::duration<double> time_used = chrono::duration_cast< chrono::duration< double > > (t2 - t1);
            cout << logTAG << " time: " << time_used.count() << " seconds." << endl;
        } ;

};

#define TIME_BEGIN() { chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
#define TIME_END(TAG) chrono::steady_clock::time_point t2 = chrono::steady_clock::now(); \
    chrono::duration<double> time_used = chrono::duration_cast< chrono::duration< double > > (t2 - t1); \
    cout << TAG << " time: " << time_used.count() << " seconds." << endl; }

#endif
