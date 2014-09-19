/*
 * SerialAbstract.h
 *
 *  Created on: Jul 2, 2014
 *      Author: bgouveia
 */

#ifndef SERIALABSTRACT_H_
#define SERIALABSTRACT_H_

/**
 * Abstract Class to communicate with the robot with the necessary interface to use different communication protocols
 *
 * \author Bruno Duarte Gouveia
 * \date Sep 16, 2014
 */
class SerialAbstract {
public:

	/**
	 * returns the number of bytes available to read
	 * \return returns the number of bytes available
	 */
	virtual unsigned int available()= 0;

	/**
	 * read a single byte
	 * \return char from the communication device
	 */
	virtual char read()= 0;

	/**
	 * write a string to the communication device
	 * \param string string to be written to the communication device
	 */
	virtual void println(char * string)= 0;

	virtual ~SerialAbstract()= 0;
};

#endif /* SERIALABSTRACT_H_ */
