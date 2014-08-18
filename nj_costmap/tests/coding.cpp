#include <cstdio>
#include <cstdint>
#include <cmath>

using std::printf;

void encodePoint(const double x, const double y, uint32_t& code)
{
	if (((x * 100) < (((double) INT16_MIN) + 1)) ||
			((x * 100) > (((double) INT16_MAX) - 1)) ||
			((y * 100) < (((double) INT16_MIN) + 1)) ||
			((y * 100) > (((double) INT16_MAX) - 1)))
	{
		fprintf(stderr, "Point (%f, %f) cannot be encoded\n", x, y);
	}
	int16_t xint = (int16_t) std::lround(x * 100);
	int16_t yint = (int16_t) std::lround(y * 100);
	code = (xint << 16);
	code |= (yint & 0x0000FFFF);
}

void decodePoint(const uint32_t code, double& x, double& y)
{
	int16_t xint = (int16_t) (code >> 16);
	int16_t yint = (int16_t) code;
	x = ((double) xint) / 100;
	y = ((double) yint) / 100;
}

int main(int argc, char** argv)
{
	double xin = 2.546;
	double yin = 1.20;
	double xout;
	double yout;
	uint32_t code;

	encodePoint(xin, yin, code);
	decodePoint(code, xout, yout);
	printf("encoded x: %f, y: %f\n", xin, yin);
	printf("decoded x: %f, y: %f\n", xout, yout);
	printf("\n");

	xin = -0.02;
	encodePoint(xin, yin, code);
	decodePoint(code, xout, yout);
	printf("encoded x: %f, y: %f\n", xin, yin);
	printf("decoded x: %f, y: %f\n", xout, yout);
	printf("\n");
	
	yin = -57.893;
	encodePoint(xin, yin, code);
	decodePoint(code, xout, yout);
	printf("encoded x: %f, y: %f\n", xin, yin);
	printf("decoded x: %f, y: %f\n", xout, yout);
	printf("\n");

	xin = -1.984;
	encodePoint(xin, yin, code);
	decodePoint(code, xout, yout);
	printf("encoded x: %f, y: %f\n", xin, yin);
	printf("decoded x: %f, y: %f\n", xout, yout);
	printf("\n");

	xin = -700.984;
	encodePoint(xin, yin, code);
	decodePoint(code, xout, yout);
	printf("encoded x: %f, y: %f\n", xin, yin);
	printf("decoded x: %f, y: %f\n", xout, yout);
	printf("\n");

	xin = 0.184421;
	yin = 2.10794;
	encodePoint(xin, yin, code);
	decodePoint(code, xout, yout);
	printf("encoded x: %f, y: %f\n", xin, yin);
	printf("code: %d (%#010x)\n", code, code);
	printf("decoded x: %f, y: %f\n", xout, yout);
	printf("\n");

}
