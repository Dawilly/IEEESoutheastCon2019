#include <stdio.h>
#include "logger.h"

void addFileContent(FILE*, Logger*);

int main() {
	FILE* fp;
	FILE* writer = fopen("test.txt", "w");
	Logger* logger;
	int status;
	int running = 1;
	char cmd;
	char str[50];

	printf("Testing logger...\n");
	logger = createLogger(writer);
	printf("Logger created. Starting thread.\n");
	status = runLogger(logger);
	printf("Detach!\n");

	while (running) {
		fscanf(stdin, "%s", str);

		cmd = str[0];

		switch (cmd) {
			case 'o':
				fprintf(stdout, "File name: ");
				fscanf(stdin, "%s", str);
				fprintf(stdout, "Opening file... ");
				fp = fopen(str, "r");
				if (!fp) {
					fprintf(stdout, "Failed!\n");
				} else {
					addFileContent(fp, logger);
				}
				fprintf(stdout, "Done!\n");
				break;
			case 'a':
				fprintf(stdout, "Input message : ");
				fscanf(stdin, "%s", &str);
				addMessage(logger, str, Info);
				break;
			case 'q':
				running = 0;
				break;
			default:
				fprintf(stdout, "Invalid input.\n");
		}
	}

	return 0;
}

void addFileContent(FILE* fp, Logger* logger) {
	char str[100];

	fscanf(fp, "%s", str);
	while (!feof(fp)) {
		addMessage(logger, str, Info);
		fscanf(fp, "%s", str);
	}
	fclose(fp);
	return;
}