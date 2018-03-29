#ifndef RTK_ZHD_H_
#define RTK_ZHD_H_

#include "gps_helper.h"
#include "../../definitions.h"


#define RTK_ZHD_SYNC1 0xaa
#define RTK_ZHD_SYNC2 0x33


typedef enum {
    RTK_ZHD_DECODE_UNINIT,
} rtk_zhd_decode_state_t;


typedef struct {
        unsigned char head[2];                 //2 bytes deviation 0
        unsigned short int Version;            //2 bytes deviation 2
        unsigned short int Length;             //2 bytes deviation 4
        unsigned short int freq;               //2 bytes deviation 6
        float Time_utc;                        //4 bytes deviation 8
        unsigned short int Year_utc;           //2 bytes deviation 12
        unsigned short int Month_utc;          //2 bytes deviation 14
        unsigned short int Day_utc;            //2 bytes deviation 16
        unsigned short int Hour_utc;           //2 bytes deviation 18
        unsigned short int Min_utc;            //2 bytes deviation 20
        unsigned short int Sec_utc;            //2 bytes deviation 22
        double Latitude;                       //8 bytes deviation 24
        double Longitude;                      //8 bytes deviation 32
        double Altitude;                       //8 bytes deviation 40
        float Eph;                             //4 bytes deviation 48
        float Epv;                             //4 bytes deviation 52
        float Vel_earth;                       //4 bytes deviation 56
        float Angle_TrackTrue;                 //4 bytes deviation 60
        float Angle_Heading;                   //4 bytes deviation 64
        float Angle_Pitch;                     //4 bytes deviation 68
        double Vel_n;                          //8 bytes deviation 72
        double Vel_e;                          //8 bytes deviation 80
        double Vel_u;                          //8 bytes deviation 88
        unsigned short int Satellites_used;    //2 bytes deviation 96
        unsigned short int Satellites_track;   //2 bytes deviation 98
        float vel_ned_valid;                   //4 bytes deviation 100
        unsigned short int Fix_type;           //2 bytes deviation 104
        float Angle_PosType;                   //4 bytes deviation 106
        float Head_deviation;                  //4 bytes deviation 110
        unsigned short int INS_state;          //2 bytes deviation 114
        double GNSS_Alt_delta ;                //8 bytes deviation 116
        double Ellipsoidal_H;                  //8 bytes deviation 124
        unsigned char reserve[2];              //4 bytes deviation 132
        unsigned short int Checksum;           //2 bytes deviation 136
} GPSINSData_dev;                              //total 138 bytes


class rtk_zhd
{
public:
    rtk_zhd();
};

#endif // RTK_ZHD_H
