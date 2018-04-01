

#include "rtk_zhd.h"

#include <string.h>

GPSDriverRTKZHD::GPSDriverRTKZHD(GPSCallbackPtr callback, void *callback_user, struct vehicle_gps_position_s *gps_position) :
	GPSHelper(callback, callback_user),
	_gps_position(gps_position)
{
	decodeInit();
}

void
GPSDriverRTKZHD::decodeInit()
{
	_rx_count = 0;
	_rx_ck_xor = 0;
	_decode_state = RTK_ZHD_DECODE_UNINIT;
}

int
GPSDriverRTKZHD::configure(unsigned &baudrate, OutputMode output_mode)
{
	/* set baudrate first */
	if (GPSHelper::setBaudrate(RTK_ZHD_BAUDRATE) != 0) {
		return -1;
	}

	baudrate = RTK_ZHD_BAUDRATE;

	/* Write config messages, don't wait for an answer */
	if (strlen(RTK_ZHD_UNLOGALL) != write(RTK_ZHD_UNLOGALL, strlen(RTK_ZHD_UNLOGALL))) {
		goto errout;
	}

	usleep(100000);

	if (strlen(RTK_ZHD_CUSTOM_5HZ) != write(RTK_ZHD_CUSTOM_5HZ, strlen(RTK_ZHD_CUSTOM_5HZ))) {
		goto errout;
	}

	usleep(100000);

	if (strlen(RTK_ZHD_SAVECONFIG) != write(RTK_ZHD_SAVECONFIG, strlen(RTK_ZHD_SAVECONFIG))) {
		goto errout;
	}

	return 0;

errout:
	GPS_WARN("rtk_zhd: config write failed");
	return -1;
}

int
GPSDriverRTKZHD::receive(unsigned timeout)
{
	uint8_t buf[GPS_READ_BUFFER_SIZE];
	gps_rtk_zhd_packet_t packet{};

	/* timeout additional to poll */
	gps_abstime time_started = gps_absolute_time();

	int j = 0;

	while (true) {

		int ret = read(buf, sizeof(buf), timeout);

		if (ret > 0) {
			/* first read whatever is left */
			if (j < ret) {
				/* pass received bytes to the packet decoder */
				while (j < ret) {
					if (parseChar(buf[j], packet) > 0) {
						handleMessage(packet);
						return 1;
					}

					j++;
				}

				/* everything is read */
				j = 0;
			}

		} else {
			usleep(20000);
		}

		/* in case we keep trying but only get crap from GPS */
		if (time_started + timeout * 1000 < gps_absolute_time()) {
			return -1;
		}
	}
}

int
GPSDriverRTKZHD::parseChar(uint8_t b, gps_rtk_zhd_packet_t &packet)
{
	int ret = 0;

	if (_decode_state == RTK_ZHD_DECODE_UNINIT) {

		if (b == RTK_ZHD_SYNC1) {
			_decode_state = RTK_ZHD_DECODE_GOT_HEAD1;
			addByteToChecksum(b);
			((uint8_t*)(&packet))[_rx_count] = b;
			_rx_count++;
		}

	} else if (_decode_state == RTK_ZHD_DECODE_GOT_HEAD1) {

		if (b == RTK_ZHD_SYNC2) {
			_decode_state = RTK_ZHD_DECODE_GOT_HEAD2;
			addByteToChecksum(b);
			((uint8_t*)(&packet))[_rx_count] = b;
			_rx_count++;
		} else {
			// Second start symbol was wrong, reset state machine
			decodeInit();
		}

	} else if (_decode_state == RTK_ZHD_DECODE_GOT_HEAD2) {

		//Add to checksum
		if (_rx_count <= (sizeof(packet) - 2 - 1)) {
			addByteToChecksum(b);
		}

		((uint8_t*)(&packet))[_rx_count] = b;
		_rx_count++;

		if (_rx_count >= sizeof(packet)) {

			/* Compare checksum */
			if (_rx_ck_xor == packet.checksum) {

				ret = 1;

			} else {

				ret = -1;
			}

			//Reset state mackine to decode next packet
			decodeInit();
		}
	}

	return ret;
}

void
GPSDriverRTKZHD::handleMessage(gps_rtk_zhd_packet_t &packet)
{
	//gps position info
	_gps_position->lat = packet.latitude * 10000000; //double to 1E-7 degrees
	_gps_position->lon = packet.longitude * 10000000; //double to 1E-7 degrees
	_gps_position->alt = packet.altitude * 1000; //double to 1E-3 meters
	_gps_position->alt_ellipsoid = packet.ellipsoidal_h * 1000; //double to 1E-3 meters

	_gps_position->s_variance_m_s = 0.02f; // m/s
	_gps_position->c_variance_rad = 0.1f * M_DEG_TO_RAD_F; //rad

	_gps_position->fix_type = packet.fix_type;

	_gps_position->eph = packet.eph;
	_gps_position->epv = packet.epv;

	_gps_position->hdop = packet.eph;
	_gps_position->vdop = packet.epv;

	//gps velocity info
	_gps_position->vel_m_s = packet.vel_ground_m_s;
	_gps_position->vel_n_m_s = packet.vel_n_m_s;
	_gps_position->vel_e_m_s = packet.vel_e_m_s;
	_gps_position->vel_d_m_s = -packet.vel_u_m_s; //from up velocity to down velocity
	_gps_position->cog_rad = packet.angle_tracktrue; // TODO don't know is rad or deg?
	_gps_position->vel_ned_valid = (unsigned)packet.vel_ned_valid;

	_gps_position->satellites_used = packet.satellites_used;

	/* convert time and date information to unix timestamp */
	struct tm timeinfo = {};

	timeinfo.tm_mday = packet.day_utc;
	timeinfo.tm_mon = packet.month_utc;
	timeinfo.tm_year = packet.year_utc;

	timeinfo.tm_hour = packet.hour_utc;
	timeinfo.tm_min = packet.min_utc;
	timeinfo.tm_sec = packet.sec_utc / 100; //10ms to second

	timeinfo.tm_isdst = 0;

	time_t epoch = mktime(&timeinfo);

	if (epoch > GPS_EPOCH_SECS) {
		// FMUv2+ boards have a hardware RTC, but GPS helps us to configure it
		// and control its drift. Since we rely on the HRT for our monotonic
		// clock, updating it from time to time is safe.

		timespec ts{};
		ts.tv_sec = epoch;
		ts.tv_nsec = packet.sec_utc * 10 * 1000000ULL;

		setClock(ts);

		_gps_position->time_utc_usec = static_cast<uint64_t>(epoch) * 1000000ULL;
		_gps_position->time_utc_usec += (packet.sec_utc % 100) * 10 * 1000ULL;

	} else {
		_gps_position->time_utc_usec = 0;
	}

	_gps_position->timestamp = gps_absolute_time();
	_gps_position->timestamp_time_relative = 0;

//	PX4_INFO("UTC_time: %f, month: %d, day: %d",
//		 (double)packet.time_utc, packet.month_utc, packet.day_utc);
//	PX4_INFO("vel_ned_valid: %.6f, vel_n: %.6f, vel_e: %.6f, vel_u: %.6f",
//		 (double)packet.vel_ned_valid, packet.vel_n_m_s, packet.vel_e_m_s, packet.vel_u_m_s);
}

void
GPSDriverRTKZHD::addByteToChecksum(uint8_t b)
{
	_rx_ck_xor ^= b;
}
