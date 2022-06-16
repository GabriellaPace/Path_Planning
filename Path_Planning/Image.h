#pragma once
#include <vector>

struct Color {
	float R, G, B;

	Color();
	Color(float R, float G, float B);
	~Color();
};


class Image {
public:
	Image();
	Image(int width, int height);
	~Image();

	Color GetColor(int x, int y) const;
	void SetColor(const Color& color, int x, int y);
	Color GetColor(int x, int y) const;
	void SetColor(const Color& color, int x, int y);

	void Read(const char* path);
	void Export(const char* path) const;

private:
	int m_width;
	int m_height;
	std::vector<Color> m_colors;
};

