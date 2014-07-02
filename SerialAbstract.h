/*
 * SerialAbstract.h
 *
 *  Created on: Jul 2, 2014
 *      Author: bgouveia
 */

#ifndef SERIALABSTRACT_H_
#define SERIALABSTRACT_H_

class SerialAbstract {
public:
	virtual unsigned int available();
	virtual char read();
	virtual void println(char * string);
	virtual ~SerialAbstract();
};

#endif /* SERIALABSTRACT_H_ */
