#ifndef _IMAGE_HPP
#define _IMAGE_HPP

#include <stdint.h>

class Image
{
	/// types ///
	public:
		typedef enum
		{
			RGB,
			YUV
		} Format;

	/// methods ///
	public:
		Image(const unsigned int width = 0, const unsigned int height = 0, const unsigned int bpp = 4);
		~Image();
		const Image &operator=(const Image &other);

		uint8_t* data() const { return _buffer; }

		unsigned int width() const { return _width; }
		unsigned int height() const { return _height; }

		unsigned int bytes() const { return _width * _height * _bpp; }
		
	private:
		uint8_t* _buffer;

		unsigned int _width, _height, _bpp;
};

#endif /* _IMAGE_HPP */
