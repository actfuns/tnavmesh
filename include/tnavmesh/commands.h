#ifndef COMMANDS_H
#define COMMANDS_H

int cmd_build(int argc, char** argv);
int cmd_query(int argc, char** argv);
int cmd_render(int argc, char** argv);
int cmd_inspect(int argc, char** argv);

void printUsage();

#endif
