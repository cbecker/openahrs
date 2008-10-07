/*
 *  Network utilities
 *
 *  Copyright (c) by Carlos Becker	http://github.com/cbecker 
 *
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
*/

#ifndef __ahrs_net_h_
#define	__ahrs_net_h_

#include <iostream>
#include <fstream>
#include <string>

using namespace std;

USING_PART_OF_NAMESPACE_EIGEN

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>

namespace openAHRS { namespace util
{
	class	UDPConnection
	{
		private:
			int	udp_socket;
			struct sockaddr_in	dest_addr;

		public:
			UDPConnection( const char *addr, int port ) {
				udp_socket	= socket( AF_INET, SOCK_DGRAM, IPPROTO_UDP  );

				/** enable broadcasting */
				int	optval	= 1;
				setsockopt( udp_socket, SOL_SOCKET, SO_BROADCAST, &optval, sizeof(optval) );

				dest_addr.sin_family	= AF_INET;
				dest_addr.sin_port		= htons(port);
				inet_aton( addr, &dest_addr.sin_addr );
				memset( &dest_addr.sin_zero, 0, sizeof(dest_addr.sin_zero) );
			};

			int Send( const void *data, int length ) {
				return sendto( udp_socket, data, length, 0, (const sockaddr *)&dest_addr, sizeof(dest_addr) );
			}
	};

}};

#endif	/* __ahrs_net_h_ */

