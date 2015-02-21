/*
 * SerialUARTImpl.h
 *
 *  Created on: Sep 16, 2014
 *      Author: bgouveia
 */

#ifndef SERIALUARTIMPL_H_
#define SERIALUARTIMPL_H_

#include "SerialAbstract.h"


/**
 * UART0 Implementation of the Serial Class
 * \author Bruno Duarte Gouveia
 * \date Sep 16, 2014
 */
class SerialUARTImpl: public SerialAbstract {
public:
	SerialUARTImpl();
	virtual ~SerialUARTImpl();

	unsigned int available();

	char read();
	void write(const char * buffer, unsigned int length);
	void println(const char * string);

};

#endif /* SERIALUARTIMPL_H_ */
