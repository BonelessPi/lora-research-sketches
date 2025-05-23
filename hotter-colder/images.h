#ifndef IMAGES_H
#define IMAGES_H

#define logo_width 128
#define logo_height 64
// You can convert an image to these bytes using image2cpp (online) (make sure to check the swap bits box)
const unsigned char logo_bits [] PROGMEM = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff, 0x07, 0x00, 0x00, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff, 0x1f, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff, 0x3f, 0x00, 0x00, 0xf8, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x00, 0x00, 0xf8, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0x01, 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0x01, 0x00, 0xe0, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x03, 0x00, 0xe0, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x03, 0x00, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x07, 0x00, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x07, 0x00, 0x80, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x07, 0x00, 0x80, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x07, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x0f, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x0f, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x0f, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x0f, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x07, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x07, 0x00, 0x80, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x07, 0x00, 0x80, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x07, 0x00, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x03, 0x00, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x03, 0x00, 0xe0, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0x01, 0x00, 0xe0, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01, 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x00, 0x00, 0xf8, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff, 0x3f, 0x00, 0x00, 0xf8, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff, 0x1f, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff, 0x07, 0x00, 0x00, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};


#endif
