/*
 * SerialUARTImpl.h
 *
 *  Created on: Sep 16, 2014
 *      Author: bgouveia
 */

#ifndef SERIALUARTIMPL_H_
#define SERIALUARTIMPL_H_

#include "SerialAbstract.h"


/*!
 * \class SerialUARTImpl
 *
 * \brief UART0 Implementation of the Serial Class
 *
 * \author Bruno Duarte Gouveia
 * \date
 */
class SerialUARTImpl: public SerialAbstract {
public:
	SerialUARTImpl();
	virtual ~SerialUARTImpl();

	unsigned int available();
	char read();
	void println(char * string);

};

#endif /* SERIALUARTIMPL_H_ */
