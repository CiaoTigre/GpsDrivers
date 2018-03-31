#ifndef RTK_ZHD_H_
#define RTK_ZHD_H_

#include "gps_helper.h"
#include "../../definitions.h"


#define RTK_ZHD_SYNC1 0xaa
#define RTK_ZHD_SYNC2 0x33

#define RTK_ZHD_UNLOGALL		"unlogall com1\r\n"
#define	RTK_ZHD_CUSTOM_PROTOCOL		"zhd log com1 gpsdata ontime 0.2\r\n"
#define RTK_ZHD_SAVECONFIG		"saveconfig\r\n"

#define RTK_ZHD_BAUDRATE 115200
#define RTK_ZHD_TIMEOUT_5HZ 400


typedef enum {
	RTK_ZHD_DECODE_UNINIT,
	RTK_ZHD_DECODE_GOT_HEAD1,
	RTK_ZHD_DECODE_GOT_HEAD2,
} rtk_zhd_decode_state_t;

#pragma pack(push, 1)

typedef struct {
	unsigned char head[2];                 //2 bytes deviation 0
	unsigned short int version;            //2 bytes deviation 2
	unsigned short int length;             //2 bytes deviation 4
	unsigned short int freq;               //2 bytes deviation 6
	float time_utc;                        //4 bytes deviation 8
	unsigned short int year_utc;           //2 bytes deviation 12
	unsigned short int month_utc;          //2 bytes deviation 14
	unsigned short int day_utc;            //2 bytes deviation 16
	unsigned short int hour_utc;           //2 bytes deviation 18
	unsigned short int min_utc;            //2 bytes deviation 20
	unsigned short int sec_utc;            //2 bytes deviation 22
	double latitude;                       //8 bytes deviation 24
	double longitude;                      //8 bytes deviation 32
	double altitude;                       //8 bytes deviation 40
	float eph;                             //4 bytes deviation 48
	float epv;                             //4 bytes deviation 52
	float vel_earth;                       //4 bytes deviation 56
	float angle_tracktrue;                 //4 bytes deviation 60
	float angle_heading;                   //4 bytes deviation 64
	float angle_pitch;                     //4 bytes deviation 68
	double vel_n;                          //8 bytes deviation 72
	double vel_e;                          //8 bytes deviation 80
	double vel_u;                          //8 bytes deviation 88
	unsigned short int satellites_used;    //2 bytes deviation 96
	unsigned short int satellites_track;   //2 bytes deviation 98
	float vel_ned_valid;                   //4 bytes deviation 100
	unsigned short int fix_type;           //2 bytes deviation 104
	float angle_postype;                   //4 bytes deviation 106
	float head_deviation;                  //4 bytes deviation 110
	unsigned short int ins_state;          //2 bytes deviation 114
	double gnss_alt_delta ;                //8 bytes deviation 116
	double ellipsoidal_h;                  //8 bytes deviation 124
	unsigned short int diff_age;           //2 bytes deviation 132
	unsigned char reserve[2];              //2 bytes deviation 134
	unsigned short int checksum;           //2 bytes deviation 136
} gps_rtk_zhd_packet_t;                        //total 138 bytes

#pragma pack(pop)

class GPSDriverRTKZHD : public GPSHelper
{
public:
	GPSDriverRTKZHD(GPSCallbackPtr callback, void *callback_user, struct vehicle_gps_position_s *gps_position);
	virtual ~GPSDriverRTKZHD() = default;

	int receive(unsigned timeout);
	int configure(unsigned &baudrate, OutputMode output_mode);

private:
	/**
	 * Parse the binary rtk_zhd packet
	 */
	int parseChar(uint8_t b, gps_rtk_zhd_packet_t &packet);

	/**
	 * Handle the package once it has arrived
	 */
	void handleMessage(gps_rtk_zhd_packet_t &packet);

	/**
	 * Reset the parse state machine for a fresh start
	 */
	void decodeInit();

	/**
	 * While parsing add every byte (except the sync bytes) to the checksum
	 */
	void addByteToChecksum(uint8_t);

	struct vehicle_gps_position_s *_gps_position {nullptr};
	rtk_zhd_decode_state_t _decode_state{RTK_ZHD_DECODE_UNINIT};
	uint8_t _rtk_zhd_revision{0};
	unsigned _rx_count{};
	uint16_t _rx_ck_xor;
};

#endif // RTK_ZHD_H
