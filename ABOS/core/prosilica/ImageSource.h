#ifndef _IMAGE_SOURCE_HPP
#define _IMAGE_SOURCE_HPP

#include <stdint.h>
#include <QObject>

#include "Image.h"

class QWidget;

/** a single image source that an image device can create */
class ImageSource
{
	public:
		virtual ~ImageSource() {};

		/** open the image source */
		virtual void open() = 0;
		
		/** close the image source */
		virtual void close() = 0;

		/** get the next image from the source */
		virtual const Image* grabFrame() = 0;

		// return the current width and height of the image source */
		unsigned int width() const { return _width; }
		unsigned int height() const { return _height; }

	protected:
		ImageSource() : _width(0), _height(0) {};

		unsigned int _width, _height;
		
	private:
		ImageSource(ImageSource&);
		ImageSource& operator=(ImageSource&);
};

#endif /* _IMAGE_SOURCE_HPP */
