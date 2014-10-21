#include <stdio.h>
#include <stdlib.h>
main()
{
	int first = 1;
	printf("const unsigned char dma_code[] PROGMEM = {");
	for (;;) {
		int c = getchar();
		if (c < 0)
			break;
		if (first) {
			first = 0;
			printf("0x%02x", c);
		} else {
			printf(",0x%02x", c);
		}
	}
	printf("};\nconst int dma_size = sizeof(dma_code)\n;");
	exit(0);
}
