#include "Image.h"
#include <memory>
#include <string.h>

Image::Image(const unsigned int width, const unsigned int height, const unsigned int bpp) :
	_width(width), _height(height), _bpp(bpp)
{
	_buffer = 0;

	if (_width && _height && _bpp)
	{
		_buffer = new uint8_t[bytes()];
	}
}

Image::~Image()
{
	if (_buffer)
	{
		delete[] _buffer;
		_buffer = 0;
	}
}

const Image& Image::operator=(const Image &other)
{
	const bool same = _width == other._width && _height == other._height && _bpp == other._bpp;
	
	_width = other._width;
	_height = other._height;
	_bpp = other._bpp;
	
	//copy data
	if (_width && _height && _bpp)
	{
		if (!same)
		{
			if (_buffer)
			{
				delete[] _buffer;
				_buffer = 0;
			}
			
			_buffer = new uint8_t[bytes()];
		}
		
		memcpy(_buffer, other._buffer, bytes());
	}
	
	return *this;
}
