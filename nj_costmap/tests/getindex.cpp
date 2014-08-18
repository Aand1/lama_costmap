#include <iostream>
#include <vector>

/* Return the index of the first value in an array greater than a value
 *
 * array[in] input array, must be partitioned
 * a[in] value for which a close value in array is searched
 */
int getIndex(const std::vector<double> &array, const double a)
{
    int count;
    count = array.size();
	int step;
 
	int i = 0;
	int i_smaller = 0;
    while (count > 0) {
        i = i_smaller; 
        step = count / 2; 
        i += step;
        if (!(a < array[i]))
		{
            i_smaller = ++i;
            count -= step + 1;
        }
		else
		{
		   	count = step;
		}
    }
	return i;
}

int main(int argc, char** argv)
{
	std::vector<double> v{0, 1, 2, 3, 4, 5};

	double a = -1;
	std::cout << "With " << a << ": " << getIndex(v, a) << std::endl;
	a = 0;
	std::cout << "With " << a << ": " << getIndex(v, a) << std::endl;
	a = 0.5;
	std::cout << "With " << a << ": " << getIndex(v, a) << std::endl;
	a = 4.5;
	std::cout << "With " << a << ": " << getIndex(v, a) << std::endl;
	a = 5.0;
	std::cout << "With " << a << ": " << getIndex(v, a) << std::endl;
	a = 6;
	std::cout << "With " << a << ": " << getIndex(v, a) << std::endl;
}
