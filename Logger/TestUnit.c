#include <stdio.h>
#include "logger.h"

void addFileContent(FILE*, Logger*);

int main() {
	FILE* fp;
	Logger* logger;
	int status;
	int running = 1;
	char cmd;
	char str[50];

	printf("Testing logger...");
	logger = createLogger(stdin);
	printf("Logger created. Starting thread.");
	status = runLogger(logger);
	printf("Detach!");

	while (running) {
		fscanf(stdin, "%c", &cmd);

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