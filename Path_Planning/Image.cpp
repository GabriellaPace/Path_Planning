#include "Image.h"

Color::Color()
	: R(0), G(0), B(0)
{
}

Color::Color(float R, float G, float B)
	: R(R), G(G), B(B)
{
}

Color::~Color()
{
}

Image::Image()
{
}

Image::Image(int width, int height)
	: m_width(width), m_height(height), m_colors(std::vector<Color>(width * height))
{
}

Image::~Image()
{
}

Color Image::GetColor(int x, int y) const
{
	return m_colors[y*m_width + x]; //we read left to right and then jump up to next row (y = how many rows up we are). Every row has m_width amount of pixels.
}

void Image::SetColor(const Color & color, int x, int y)
{
	m_colors[y*m_width + x].R = color.R;
	m_colors[y*m_width + x].G = color.G;
	m_colors[y*m_width + x].B = color.B;
}

//Parts REQUIRED in a bitmap file:
/*	- File header: general information
	- Information header (DIB header): detailed information
	- Pixel array: color data for each pixel
	there could be more, but they are not mandatory.

Every row must occupy memory that is multiple of 4 Bytes!
Every pixel uses 3B (1 for every color), so we might need/have a padding at the end of each row (all set to 0)
	Bytes of padding = [4 - ( NumberOfPixelsInARow * 3 / 4 )] / 4		(the last /4 is to obtain 0 in case we have 4-0)
*/

void Image::Read(const char * path)
{
	std::ifstream f;
	f.open(path, std::ios::in | std::ios::binary);	//reading (::in) a file in binary

	if (!f.is_open()) { // = if the opening was NOT successful
		std::cout << "ERROR: File could not be opened.\n";
		return;
	}

	const int fileHeaderSize = 14;	//fixed
	const int informationHeaderSize = 40;	//fixed

	// reinterpret_cast:  doesn't modify the data in any way, just interprets it in a different way
	// static_cast:       keeps the value and changes the type
	unsigned char fileHeader[fileHeaderSize];
	f.read(reinterpret_cast<char*>(fileHeader), fileHeaderSize);

	if (fileHeader[0] != 'B' || fileHeader[1] != 'M') {		//check that the file we are reading is actually a bitmap:
		std::cout << "ERROR: The specified path does not correspond to a bitmap image.\n";
		f.close();
		return;
	}

	unsigned char informationHeader[informationHeaderSize];
	f.read(reinterpret_cast<char*>(informationHeader), informationHeaderSize);

	int fileSize = fileHeader[2] + (fileHeader[3] << 8) + (fileHeader[4] << 16) + (fileHeader[5] << 24);	//joining bytes
	m_width = informationHeader[4] + (informationHeader[5] << 8) + (informationHeader[6] << 16) + (informationHeader[7] << 24);
	m_height = informationHeader[8] + (informationHeader[9] << 8) + (informationHeader[10] << 16) + (informationHeader[11] << 24);

	m_colors.resize(m_width * m_height);

	const int paddingAmount = ((4 - (m_width * 3) % 4) % 4);	//calculating how much padding there is

	for (int y = 0; y < m_height; ++y) {	//rows
		for (int x = 0; x < m_width; ++x) {	//coloumns
			unsigned char color[3];

			f.read(reinterpret_cast<char*>(color), 3);	//reading the colors of 1 pixel (need 3B)

			//in .bmp the order is (B,G,R), so we start from last one
			m_colors[y*m_width + x].R = static_cast<float>(color[2]) / 255.0f;
			m_colors[y*m_width + x].G = static_cast<float>(color[1]) / 255.0f;
			m_colors[y*m_width + x].B = static_cast<float>(color[0]) / 255.0f;
		}

		f.ignore(paddingAmount);
	}

	f.close();
	std::cout << "File read.\n";
}



void Image::Export(const char * path) const
{
	std::ofstream f;
	f.open(path, std::ios::out | std::ios::binary);	//writing (::out) a file in binary

	if (!f.is_open()) { // = if the opening was NOT successful
		std::cout << "ERROR: File could not be opened.\n";
		return;
	}

	unsigned char bmpPad[3] = {0,0,0};	//padding can never be more than 3
	const int paddingAmount = ((4 - (m_width * 3) % 4) % 4);	//calculating how much padding we actually need

	const int fileHeaderSize = 14;	//fixed
	const int informationHeaderSize = 40;	//fixed
	const int fileSize = fileHeaderSize + informationHeaderSize + (m_width * m_height * 3) + (paddingAmount * m_height);
																	// ^ color data			// ^ multiplied for number of rows

	unsigned char fileHeader[fileHeaderSize];

	// File type
	fileHeader[0] = 'B';
	fileHeader[1] = 'M';
	// FIle size	//we need more space to be able to write an 'int' inside an 'unsigned char'
	fileHeader[2] = fileSize;
	fileHeader[3] = fileSize >> 8;
	fileHeader[4] = fileSize >> 16;
	fileHeader[5] = fileSize >> 24;
	// Reserved 1 (Not used)
	fileHeader[6] = 0;
	fileHeader[7] = 0;
	// Reserved 2 (Not used)
	fileHeader[8] = 0;
	fileHeader[9] = 0;
	// Pixel data offset	//(headers combined)
	fileHeader[10] = fileHeaderSize + informationHeaderSize;
	fileHeader[11] = 0;
	fileHeader[12] = 0;
	fileHeader[13] = 0;

	unsigned char informationHeader[informationHeaderSize];

	// Header size
	informationHeader[0] = informationHeaderSize;
	informationHeader[1] = 0;
	informationHeader[2] = 0;
	informationHeader[3] = 0;
	// Image width
	informationHeader[4] = m_width;
	informationHeader[5] = m_width >> 8;
	informationHeader[6] = m_width >> 16;
	informationHeader[7] = m_width >> 24;
	// Image width
	informationHeader[8] = m_height;
	informationHeader[9] = m_height >> 8;
	informationHeader[10] = m_height >> 16;
	informationHeader[11] = m_height >> 24;
	// Planes
	informationHeader[12] = 1;
	informationHeader[13] = 0;
	// Bits per pixel (RGB)
	informationHeader[14] = 8;	// we need 1B (=8bits) for every color  ->  RGB = 24b,  Greyscale=8b
	informationHeader[15] = 0;
	// Compression (No compression)
	informationHeader[16] = 0;
	informationHeader[17] = 0;
	informationHeader[18] = 0;
	informationHeader[19] = 0;
	// Image size (No compression)
	informationHeader[20] = 0;
	informationHeader[21] = 0;
	informationHeader[22] = 0;
	informationHeader[23] = 0;
	// X pixels per meter (Not specified)
	informationHeader[24] = 0;
	informationHeader[25] = 0;
	informationHeader[26] = 0;
	informationHeader[27] = 0;
	// Y pixels per meter (Not specified)
	informationHeader[28] = 0;
	informationHeader[29] = 0;
	informationHeader[30] = 0;
	informationHeader[31] = 0;
	// Total colors (Color palette not used)
	informationHeader[32] = 0;
	informationHeader[33] = 0;
	informationHeader[34] = 0;
	informationHeader[35] = 0;
	// Important colors (Generally ignored)
	informationHeader[36] = 0;
	informationHeader[37] = 0;
	informationHeader[38] = 0;
	informationHeader[39] = 0;

	f.write(reinterpret_cast<char*>(fileHeader), fileHeaderSize);
	f.write(reinterpret_cast<char*>(informationHeader), informationHeaderSize);

	// reinterpret_cast:  doesn't modify the data in any way, just interprets it in a different way
	// static_cast:       keeps the value and changes the type

	for (int y = 0; y < m_height; ++y) {	//rows
		for (int x = 0; x < m_width; ++x) {	//coloumns
			unsigned char R = static_cast<unsigned char>(GetColor(x, y).R * 225.0f);
			unsigned char G = static_cast<unsigned char>(GetColor(x, y).G * 225.0f);
			unsigned char B = static_cast<unsigned char>(GetColor(x, y).B * 225.0f);

			unsigned char color[] = { B, G, R };	//inverted order, that's how -bmp are wrote

			f.write(reinterpret_cast<char*>(color), 3);		// color needs 3B
		}

		f.write(reinterpret_cast<char*>(bmpPad), paddingAmount);		// adding the padding
	}

	f.close();
	std::cout << "File created.\n";
}