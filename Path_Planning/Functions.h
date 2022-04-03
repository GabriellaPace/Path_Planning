#pragma once

float nonDom(float g, float rhs) {
	if (g >= rhs) {
		std::cout << " g " << std::endl;
		return g;
	}
	else {
		std::cout << " rhs " << std::endl;
		return rhs;
	}
}