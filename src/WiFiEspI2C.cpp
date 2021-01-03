/*--------------------------------------------------------------------
This file is part of the Arduino WiFiEsp library.

The Arduino WiFiEsp library is free software: you can redistribute it
and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

The Arduino WiFiEsp library is distributed in the hope that it will be
useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with The Arduino WiFiEsp library.  If not, see
<http://www.gnu.org/licenses/>.
--------------------------------------------------------------------*/

#include "WiFiEspI2C.h"
#include <Wire.h>
#include "printf.h"

int16_t 	WiFiEspClass::_state[MAX_SOCK_NUM] = { NA_STATE, NA_STATE, NA_STATE, NA_STATE };
uint16_t 	WiFiEspClass::_server_port[MAX_SOCK_NUM] = { 0, 0, 0, 0 };


uint8_t WiFiEspClass::espMode = 0;

esp32_i2c_aps_list_t *aps_list;


/*
 * I2C
 */
TwoWire& _wire = Wire;
int8_t _slaveAddress;

extern "C" uint8_t esp32_i2c_rw(uint8_t data)
{
	_wire.requestFrom(_slaveAddress, 1);
	return _wire.read();
}

extern "C" void esp32_i2c_rw_len(uint8_t *send, uint8_t *recv, uint32_t len)
{
    if (send == NULL && recv == NULL)
    {
        printf(" buffer is null\r\n");
        return;
    }

	if (send != NULL)
	{
//		_wire.writeTransmission(SLAVE_ADDRESS, send, len, true);
		uint32_t txindex = 0;
		while (txindex < len) {
			_wire.beginTransmission(_slaveAddress);
			for (int i = 0; i < 32; i++)
			{
				if (txindex > len)
					break;
				_wire.write(send[txindex]);
				txindex++;
			}
			_wire.endTransmission();
			msleep(1);
		}
	}
	else
	{
//		_wire.readTransmission(SLAVE_ADDRESS, recv, len, true);
		_wire.requestFrom(_slaveAddress, len);
		int i = 0;
		while (_wire.available() > 0)
		{
			recv[i] = _wire.read();
			i++;
			if (i >= len)
				break;
		}
	}
}



WiFiEspClass::WiFiEspClass()
{

}

void WiFiEspClass::init(TwoWire& espI2C, uint8_t slaveAddress)
{
    LOGINFO(F("Initializing ESP module"));
	_wire = espI2C;
	_slaveAddress = slaveAddress;
    esp32_i2c_init();
}


char* WiFiEspClass::firmwareVersion()
{
//	return EspDrv::getFwVersion();
    char version[32];
    return esp32_i2c_firmware_version(version);
}


int WiFiEspClass::begin(const char* ssid, const char* passphrase)
{
    espMode = 1;
	if (esp32_i2c_connect_AP((uint8_t *)ssid, (uint8_t *)passphrase, 5) == 0)
		return WL_CONNECTED;

	return WL_CONNECT_FAILED;
}


int WiFiEspClass::beginAP(const char* ssid, uint8_t channel, const char* pwd, uint8_t enc, bool apOnly)
{
	if(apOnly)
        espMode = 2;
    else
        espMode = 3;
    
    if (esp32_i2c_ap_pass_phrase((uint8_t *)ssid, (uint8_t *)pwd, channel) == 0)
		return WL_CONNECTED;

	return WL_CONNECT_FAILED;
}

int WiFiEspClass::beginAP(const char* ssid)
{
	return beginAP(ssid, 10, "", 0);
}

int WiFiEspClass::beginAP(const char* ssid, uint8_t channel)
{
	return beginAP(ssid, channel, "", 0);
}


void WiFiEspClass::config(IPAddress ip)
{
	uint8_t _ip[4];
	_ip[0] = ip[0];
	_ip[1] = ip[1];
	_ip[2] = ip[2];
	_ip[3] = ip[3];
	esp32_i2c_ip_address(_ip);
}

void WiFiEspClass::configAP(IPAddress ip)
{
	config(ip);
}



int WiFiEspClass::disconnect()
{
	return esp32_i2c_disconnect_from_AP();
}

uint8_t* WiFiEspClass::macAddress(uint8_t* mac)
{
	// TODO we don't need _mac variable
	uint8_t* _mac = esp32_i2c_MAC_address();
	memcpy(mac, _mac, WL_MAC_ADDR_LENGTH);
    return mac;
}

IPAddress WiFiEspClass::localIP()
{
	IPAddress ret;
	esp32_i2c_net_t *net = esp32_i2c_get_network_data();
	ret = net->localIp;
	return ret;
}

IPAddress WiFiEspClass::subnetMask()
{
	IPAddress mask;
	esp32_i2c_net_t *net = esp32_i2c_get_network_data();
	mask = net->subnetMask;
	return mask;
}

IPAddress WiFiEspClass::gatewayIP()
{
	IPAddress gw;
	esp32_i2c_net_t *net = esp32_i2c_get_network_data();
	gw = net->gatewayIp;
	return gw;
}


char* WiFiEspClass::SSID()
{
	return esp32_i2c_get_ssid();
}

uint8_t* WiFiEspClass::BSSID(uint8_t* bssid)
{
	// TODO we don't need _bssid
	uint8_t _bssid[6] = { 0, 0, 0, 0, 0, 0 };
	memcpy(bssid, _bssid, WL_MAC_ADDR_LENGTH);
    return bssid;
}

int32_t WiFiEspClass::RSSI()
{
	return esp32_i2c_get_rssi();
}


int8_t WiFiEspClass::scanNetworks()
{
	aps_list = esp32_i2c_scan_networks();
	return aps_list->aps_num;
}

char* WiFiEspClass::SSID(uint8_t networkItem)
{
	esp32_i2c_ap_t **aps = aps_list[networkItem].aps;
	return (char *)(*aps)->ssid;
}

int32_t WiFiEspClass::RSSI(uint8_t networkItem)
{
	esp32_i2c_ap_t **aps = aps_list[networkItem].aps;
	return (*aps)->rssi;
}

uint8_t WiFiEspClass::encryptionType(uint8_t networkItem)
{
	esp32_i2c_ap_t **aps = aps_list[networkItem].aps;
	return (*aps)->encr;
}


uint8_t WiFiEspClass::status()
{
	return esp32_i2c_status();
}



////////////////////////////////////////////////////////////////////////////
// Non standard methods
////////////////////////////////////////////////////////////////////////////

void WiFiEspClass::reset(void)
{
    esp32_i2c_init();
}


bool WiFiEspClass::ping(const char *host)
{
	return (esp32_i2c_ping((uint8_t *)host, 1, 1) != -1);
}

uint8_t WiFiEspClass::getFreeSocket()
{
  // ESP Module assigns socket numbers in ascending order, so we will assign them in descending order
    for (int i = MAX_SOCK_NUM - 1; i >= 0; i--)
	{
      if (_state[i] == NA_STATE)
      {
          return i;
      }
    }
    return SOCK_NOT_AVAIL;
}

void WiFiEspClass::allocateSocket(uint8_t sock)
{
  _state[sock] = sock;
}

void WiFiEspClass::releaseSocket(uint8_t sock)
{
  _state[sock] = NA_STATE;
}


WiFiEspClass WiFi;

