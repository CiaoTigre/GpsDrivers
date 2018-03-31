

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

	if (strlen(RTK_ZHD_CUSTOM_PROTOCOL) != write(RTK_ZHD_CUSTOM_PROTOCOL, strlen(RTK_ZHD_CUSTOM_PROTOCOL))) {
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
		PX4_INFO("total: ret = %d", ret);

		if (ret > 0) {
			/* first read whatever is left */
			if (j < ret) {
				/* pass received bytes to the packet decoder */
				while (j < ret) {
					PX4_INFO("idx: %d, %02x, check_xor: %x", j, buf[j], _rx_ck_xor);
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
			PX4_INFO("time out");
			return 1;
			//return -1;
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
				PX4_INFO("Checksum equal!!");
				ret = 1;
			} else {
				PX4_INFO("Checksum not equal");
				ret = -1;
			}
			ret = 1;

			PX4_INFO("Checksum_receive: 0x%x", packet.checksum);
			PX4_INFO("Checksum_my: 0x%x", _rx_ck_xor);

			//Reset state mackine to decode next packet
			decodeInit();
		}
	}

	return ret;
}


void
GPSDriverRTKZHD::handleMessage(gps_rtk_zhd_packet_t &packet)
{
	PX4_INFO("UTC: year: %d, month: %d, day: %d",
		 packet.year_utc, packet.month_utc, packet.day_utc);
	PX4_INFO("vel_ned_valid: %.6f, vel_n: %.6f, vel_e: %.6f, vel_u: %.6f",
		 (double)packet.vel_ned_valid, packet.vel_n, packet.vel_e, packet.vel_u);
}

void
GPSDriverRTKZHD::addByteToChecksum(uint8_t b)
{
	_rx_ck_xor = _rx_ck_xor ^ b;
}

