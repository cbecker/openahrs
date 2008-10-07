/*
 *  mmap abstraction
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



#ifndef __mmapc_h_
#define	__mmapc_h_

namespace input {
	/**
	 * Request memory mapping.
	 *
	 * @param IOM_BASE	base address to request
	 * @param IOM_SIZE	size we need, starting from IOM_BASE
	 *
	 * @return	a pointer to the mmaped memory or NULL on error
	 */
	void *DoMMap( unsigned int IOM_BASE, unsigned int IOM_SIZE );
};

#endif	/* __mmapc_h_ */

